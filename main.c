/*
 * BT Ring Tower 2
 * ---------------
 * Urządzenie łączy się z telefonem przez Bluetooth HFP (Hands-Free Profile)
 * i sygnalizuje stan połączenia oraz przychodzące połączenia za pomocą:
 *   - wieży sygnalizacyjnej RYG (GPIO 27/26/25)
 *   - wyjścia przekaźnika/buzzera RING_OUT (GPIO 33)
 *   - wyświetlacza OLED 128x32 (I2C, SDA=21, SCL=22)
 *   - przycisku (GPIO 0)
 *
 * Sprzęt: ESP32, ESP-IDF v5.5.1, Classic BT only (BLE zwolnione).
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"  /* flagi do bezpiecznej komunikacji callback→task */

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_hf_client_api.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

static const char *TAG = "BT_RING_TOWER";

/* ======================================================================
 * KONFIGURACJA PINÓW
 * ====================================================================== */

#define PIN_TOWER_RED      GPIO_NUM_27
#define PIN_TOWER_YELLOW   GPIO_NUM_26
#define PIN_TOWER_GREEN    GPIO_NUM_25
#define PIN_RING_OUT       GPIO_NUM_33
#define PIN_BUTTON         GPIO_NUM_0

/* ======================================================================
 * STANY APLIKACJI
 * ====================================================================== */

typedef enum {
    APP_STATE_IDLE = 0,
    APP_STATE_PAIRING,
    APP_STATE_CONNECTED,
    APP_STATE_RINGING,
    APP_STATE_IN_CALL
} app_state_t;

static app_state_t app_state = APP_STATE_IDLE;

/* ======================================================================
 * ZMIENNE GLOBALNE – STAN BT
 * ====================================================================== */

static esp_bd_addr_t peer_addr    = {0};
static bool          have_peer_addr = false;
static int g_hfp_connection_state = ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED;
static SemaphoreHandle_t g_bt_state_mutex = NULL;

/* Stan połączenia SCO – używany przez hf_audio_data_cb */
static esp_hf_sync_conn_hdl_t g_sco_handle    = 0;
static uint16_t               g_sco_frame_size = 60; /* domyślnie CVSD 8kHz 7.5ms */

/*
 * Flagi zdarzeń – jedyny bezpieczny mechanizm komunikacji z callbacków BT
 * do tasków aplikacyjnych. Callbacki BT NIE mogą:
 *   - wykonywać operacji flash (NVS) – blokują wątek BT, powodują timeout SCO
 *   - wywoływać innych API BT (ryzyko zakleszczenia wewnętrznych locków stosu)
 * Zamiast tego ustawiają bit w EventGroup; bt_reconnect_task obsługuje te żądania.
 */
static EventGroupHandle_t g_bt_events = NULL;
#define EVT_SAVE_PEER_ADDR     (1 << 0)  /* zapisz peer_addr do NVS    */
#define EVT_CLEAR_PEER_ADDR    (1 << 1)  /* skasuj peer_addr z NVS     */
#define EVT_SET_DISCOVERABLE   (1 << 2)  /* włącz tryb parowania (GAP) */
#define EVT_SET_CONNECTABLE    (1 << 3)  /* wyłącz discoverable (GAP)  */

/* ======================================================================
 * POMOCNICZE FUNKCJE INLINE
 * ====================================================================== */

/* Wątkobezpieczny odczyt stanu HFP przez mutex */
static inline int bt_state_get(void)
{
    int state;
    xSemaphoreTake(g_bt_state_mutex, portMAX_DELAY);
    state = g_hfp_connection_state;
    xSemaphoreGive(g_bt_state_mutex);
    return state;
}

/* Wątkobezpieczny zapis stanu HFP przez mutex – wywoływać tylko z callbacku HFP */
static inline void bt_state_set(int new_state)
{
    xSemaphoreTake(g_bt_state_mutex, portMAX_DELAY);
    g_hfp_connection_state = new_state;
    xSemaphoreGive(g_bt_state_mutex);
}

/* Czas systemowy w ms (bez dryfu, uint64_t µs → uint32_t ms) */
static inline uint32_t get_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/* ======================================================================
 * NVS – ZAPIS / ODCZYT ADRESU TELEFONU
 * UWAGA: wywoływać TYLKO z bt_reconnect_task, nigdy z callbacków BT!
 * ====================================================================== */

/*
 * save_peer_addr_to_nvs – zapisuje adres MAC sparowanego telefonu do flash.
 * Operacja flash zajmuje kilka ms – blokowanie wątku BT w tym czasie
 * powoduje timeout SCO i drop połączenia. Dlatego jest wywoływana z tasku,
 * nie z callbacku.
 */
static void save_peer_addr_to_nvs(const esp_bd_addr_t addr)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("bt_cfg", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, "peer_addr", addr, sizeof(esp_bd_addr_t));
        if (err == ESP_OK) {
            nvs_commit(nvs);
            ESP_LOGI(TAG, "Zapisalem peer_addr do NVS");
        } else {
            ESP_LOGE(TAG, "Blad nvs_set_blob: %s", esp_err_to_name(err));
        }
        nvs_close(nvs);
    } else {
        ESP_LOGE(TAG, "Blad nvs_open: %s", esp_err_to_name(err));
    }
}

/*
 * load_peer_addr_from_nvs – wczytuje adres telefonu z flash przy starcie.
 * Wywoływana z app_main, przed uruchomieniem stosu BT, więc bezpieczna.
 */
static bool load_peer_addr_from_nvs(esp_bd_addr_t addr_out)
{
    nvs_handle_t nvs;
    size_t size = sizeof(esp_bd_addr_t);
    esp_err_t err = nvs_open("bt_cfg", NVS_READONLY, &nvs);
    if (err != ESP_OK) return false;
    err = nvs_get_blob(nvs, "peer_addr", addr_out, &size);
    nvs_close(nvs);
    return (err == ESP_OK && size == sizeof(esp_bd_addr_t));
}

/*
 * clear_peer_addr_from_nvs – kasuje zapisany adres telefonu z flash.
 * Tak samo jak save: tylko z tasku, nie z callbacku.
 */
static void clear_peer_addr_from_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("bt_cfg", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_erase_key(nvs, "peer_addr");
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI(TAG, "Usunieto peer_addr z NVS");
    }
    have_peer_addr = false;
}

/* ======================================================================
 * STEROWANIE WIEŻĄ RYG
 * ====================================================================== */

static void set_tower_outputs(bool red, bool yellow, bool green)
{
    gpio_set_level(PIN_TOWER_RED,    red    ? 1 : 0);
    gpio_set_level(PIN_TOWER_YELLOW, yellow ? 1 : 0);
    gpio_set_level(PIN_TOWER_GREEN,  green  ? 1 : 0);
}

/* ======================================================================
 * OLED 128x32 – STEROWNIK SSD1306 (I2C)
 * ====================================================================== */

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_21
#define I2C_SCL_PIN     GPIO_NUM_22
#define OLED_I2C_ADDR   0x3C
#define OLED_WIDTH      128
#define OLED_HEIGHT     32
#define OLED_PAGES      (OLED_HEIGHT / 8)

static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];

typedef struct {
    char    c;
    uint8_t rows[7];
} glyph_t;

static const glyph_t font5x7[] = {
    { ' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00} },
    { '0', {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E} },
    { '1', {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E} },
    { '2', {0x0E,0x11,0x10,0x08,0x04,0x02,0x1F} },
    { '3', {0x1F,0x08,0x04,0x08,0x10,0x11,0x0E} },
    { '4', {0x08,0x0C,0x0A,0x09,0x1F,0x08,0x08} },
    { '5', {0x1F,0x01,0x0F,0x10,0x10,0x11,0x0E} },
    { '6', {0x0C,0x02,0x01,0x0F,0x11,0x11,0x0E} },
    { '7', {0x1F,0x10,0x08,0x04,0x02,0x02,0x02} },
    { '8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E} },
    { '9', {0x0E,0x11,0x11,0x1E,0x10,0x08,0x06} },
    { ':', {0x00,0x04,0x04,0x00,0x04,0x04,0x00} },
    { 'A', {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11} },
    { 'B', {0x0F,0x11,0x11,0x0F,0x11,0x11,0x0F} },
    { 'C', {0x0E,0x11,0x01,0x01,0x01,0x11,0x0E} },
    { 'D', {0x0F,0x11,0x11,0x11,0x11,0x11,0x0F} },
    { 'E', {0x1F,0x01,0x01,0x0F,0x01,0x01,0x1F} },
    { 'F', {0x1F,0x01,0x01,0x0F,0x01,0x01,0x01} },
    { 'G', {0x0E,0x11,0x01,0x1D,0x11,0x11,0x0E} },
    { 'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11} },
    { 'I', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E} },
    { 'J', {0x1C,0x08,0x08,0x08,0x08,0x09,0x06} },
    { 'K', {0x11,0x09,0x05,0x03,0x05,0x09,0x11} },
    { 'L', {0x01,0x01,0x01,0x01,0x01,0x01,0x1F} },
    { 'M', {0x11,0x1B,0x15,0x11,0x11,0x11,0x11} },
    { 'N', {0x11,0x11,0x13,0x15,0x19,0x11,0x11} },
    { 'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E} },
    { 'P', {0x1E,0x11,0x11,0x1E,0x01,0x01,0x01} },
    { 'Q', {0x0E,0x11,0x11,0x11,0x15,0x09,0x16} },
    { 'R', {0x0F,0x11,0x11,0x0F,0x0A,0x11,0x11} },
    { 'S', {0x0E,0x11,0x01,0x0E,0x10,0x11,0x0E} },
    { 'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04} },
    { 'U', {0x11,0x11,0x11,0x11,0x11,0x11,0x0E} },
    { 'V', {0x11,0x11,0x11,0x11,0x11,0x0A,0x04} },
    { 'W', {0x11,0x11,0x11,0x11,0x15,0x1B,0x11} },
    { 'X', {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11} },
    { 'Y', {0x11,0x11,0x0A,0x04,0x04,0x04,0x04} },
    { 'Z', {0x1F,0x10,0x08,0x04,0x02,0x01,0x1F} },
};

static const size_t FONT_GLYPHS_COUNT = sizeof(font5x7) / sizeof(font5x7[0]);

static const glyph_t *oled_find_glyph(char c)
{
    if (c >= 'a' && c <= 'z') c -= 32;
    for (size_t i = 0; i < FONT_GLYPHS_COUNT; ++i) {
        if (font5x7[i].c == c) return &font5x7[i];
    }
    return &font5x7[0];
}

static esp_err_t oled_write_command(uint8_t cmd)
{
    uint8_t buf[2] = {0x00, cmd};
    return i2c_master_write_to_device(I2C_PORT, OLED_I2C_ADDR,
                                      buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t oled_write_data(const uint8_t *data, size_t len)
{
    esp_err_t err = ESP_OK;
    uint8_t buf[1 + 16];
    size_t offset = 0;
    while (offset < len && err == ESP_OK) {
        size_t chunk = len - offset;
        if (chunk > 16) chunk = 16;
        buf[0] = 0x40;
        memcpy(&buf[1], data + offset, chunk);
        err = i2c_master_write_to_device(I2C_PORT, OLED_I2C_ADDR,
                                         buf, chunk + 1, pdMS_TO_TICKS(100));
        offset += chunk;
    }
    return err;
}

static void oled_update_full(void)
{
    for (int page = 0; page < OLED_PAGES; ++page) {
        oled_write_command(0xB0 | page);
        oled_write_command(0x00);
        oled_write_command(0x10);
        oled_write_data(&oled_buffer[page * OLED_WIDTH], OLED_WIDTH);
    }
}

static void oled_clear_buffer(void)
{
    memset(oled_buffer, 0x00, sizeof(oled_buffer));
}

static void oled_clear(void)
{
    oled_clear_buffer();
    oled_update_full();
}

static void oled_set_pixel(int x, int y, bool on)
{
    if (x < 0 || x >= OLED_WIDTH || y < 0 || y >= OLED_HEIGHT) return;
    int page = y / 8;
    int bit  = y % 8;
    uint8_t *byte = &oled_buffer[page * OLED_WIDTH + x];
    if (on) *byte |=  (1 << bit);
    else    *byte &= ~(1 << bit);
}

static void init_oled(void)
{
    i2c_config_t conf = {
        .mode          = I2C_MODE_MASTER,
        .sda_io_num    = I2C_SDA_PIN,
        .scl_io_num    = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    oled_write_command(0xAE);
    oled_write_command(0x20); oled_write_command(0x00);
    oled_write_command(0xB0);
    oled_write_command(0xC8);
    oled_write_command(0x00); oled_write_command(0x10);
    oled_write_command(0x40);
    oled_write_command(0x81); oled_write_command(0x7F);
    oled_write_command(0xA1);
    oled_write_command(0xA6);
    oled_write_command(0xA8); oled_write_command(0x1F);  /* 128x32: multiplex = 31 */
    oled_write_command(0xD3); oled_write_command(0x00);
    oled_write_command(0xD5); oled_write_command(0x80);
    oled_write_command(0xD9); oled_write_command(0xF1);
    oled_write_command(0xDA); oled_write_command(0x02);  /* 128x32: COM pins sekwencyjne */
    oled_write_command(0xDB); oled_write_command(0x40);
    oled_write_command(0x8D); oled_write_command(0x14);
    oled_write_command(0xAF);

    oled_clear();
}

static void oled_draw_char_big(int x, int base_y, char c)
{
    const glyph_t *g = oled_find_glyph(c);
    for (int row = 0; row < 7; ++row) {
        uint8_t bits = g->rows[row];
        for (int col = 0; col < 5; ++col) {
            if ((bits >> col) & 0x01) {
                int px = x + col * 2;
                int py = base_y + row * 2;
                oled_set_pixel(px,     py,     true);
                oled_set_pixel(px + 1, py,     true);
                oled_set_pixel(px,     py + 1, true);
                oled_set_pixel(px + 1, py + 1, true);
            }
        }
    }
}

static void oled_draw_text_big(int line, const char *text)
{
    if (!text) return;
    int base_y = (line == 0) ? 0 : 16;
    int x = 0;
    while (*text && x < OLED_WIDTH) {
        oled_draw_char_big(x, base_y, *text);
        x += 11;
        text++;
    }
}

static void oled_show_2lines(const char *line1, const char *line2)
{
    oled_clear_buffer();
    if (line1) oled_draw_text_big(0, line1);
    if (line2) oled_draw_text_big(1, line2);
    oled_update_full();
}

/* ======================================================================
 * STEROWANIE ALARMEM I UI
 * ====================================================================== */

static void start_ringing_alarm(void);
static void stop_ringing_alarm(void);

/*
 * update_ui_by_state – aktualizuje LEDy, RING_OUT i OLED co 50 ms.
 *
 * Logika lampek:
 *   ŻÓŁTY  – miga 0,5 s gdy BT rozłączone
 *   ZIELONY – świeci stale gdy połączone, bez aktywności
 *   CZERWONY + RING_OUT – migają 0,5 s gdy dzwoni telefon (tylko gdy BT aktywne)
 *
 * Zabezpieczenie race condition: jeśli BT rozłączone a stan aplikacji
 * wskazuje aktywną sesję – wymuszamy IDLE (patrz komentarz przy warunku).
 */
static void update_ui_by_state(void)
{
    uint32_t t = get_ms();

    bool t_red    = false;
    bool t_yellow = false;
    bool t_green  = false;
    bool ring_out = false;

    int cur_bt_state = bt_state_get();

    bool bt_connected =
        (cur_bt_state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED) ||
        (cur_bt_state == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED);

    /*
     * Race condition: callback BT ustawia g_hfp_connection_state (przez mutex)
     * zanim zdąży zmienić app_state. Gdyby ui_task trafił w ten przedział,
     * widziałby bt_connected=false przy app_state=RINGING – żółty+czerwony razem.
     * Rozwiązanie: przy braku BT zawsze wracamy do IDLE.
     */
    if (!bt_connected &&
        (app_state == APP_STATE_RINGING  ||
         app_state == APP_STATE_IN_CALL  ||
         app_state == APP_STATE_CONNECTED)) {
        app_state = APP_STATE_IDLE;
    }

    static app_state_t last_state = (app_state_t)0xFF;

    if (!bt_connected) {
        t_yellow = ((t / 500) % 2) != 0;
    }
    if (app_state == APP_STATE_CONNECTED && bt_connected) {
        t_green = true;
    }
    if (app_state == APP_STATE_RINGING && bt_connected) {
        bool blink = ((t / 500) % 2) != 0;
        t_red    = blink;
        ring_out = blink;
    }

    switch (app_state) {
    case APP_STATE_IDLE:
        if (last_state != APP_STATE_IDLE)
            oled_show_2lines("BT BRAK", "POLACZENIA");
        break;
    case APP_STATE_PAIRING:
        if (last_state != APP_STATE_PAIRING)
            oled_show_2lines("TRYB PAR", "PARUJ W TEL");
        break;
    case APP_STATE_CONNECTED:
        if (last_state != APP_STATE_CONNECTED)
            oled_show_2lines("BT OK", "OCZEKIWANIE");
        break;
    case APP_STATE_RINGING:
        if (last_state != APP_STATE_RINGING)
            oled_show_2lines("DZWONI TEL", "ODB W TEL");
        break;
    case APP_STATE_IN_CALL:
        if (last_state != APP_STATE_IN_CALL)
            oled_show_2lines("ROZMOWA", "W TOKU");
        break;
    default:
        break;
    }

    set_tower_outputs(t_red, t_yellow, t_green);
    gpio_set_level(PIN_RING_OUT, ring_out ? 1 : 0);

    last_state = app_state;
}

static void start_ringing_alarm(void)
{
    ESP_LOGI(TAG, "RING: start alarm");
    app_state = APP_STATE_RINGING;
}

static void stop_ringing_alarm(void)
{
    ESP_LOGI(TAG, "STOP: koniec alarmu");
    int cur = bt_state_get();
    if (cur == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED ||
        cur == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED) {
        app_state = APP_STATE_CONNECTED;
    } else {
        app_state = APP_STATE_IDLE;
    }
}

/* ======================================================================
 * CALLBACK GAP – PAROWANIE I AUTORYZACJA
 *
 * WAŻNE: callback GAP działa w kontekście wątku Bluedroid.
 * Dozwolone: zmiana app_state, ustawienie bitów w EventGroup (ISR-safe).
 * ZABRONIONE: operacje flash (NVS), długie blokowania.
 * ====================================================================== */

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "GAP AUTH OK: device='%s' addr=%02x:%02x:%02x:%02x:%02x:%02x",
                     param->auth_cmpl.device_name,
                     param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                     param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                     param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
        } else {
            ESP_LOGW(TAG, "GAP AUTH FAIL: status=%d addr=%02x:%02x:%02x:%02x:%02x:%02x",
                     param->auth_cmpl.stat,
                     param->auth_cmpl.bda[0], param->auth_cmpl.bda[1],
                     param->auth_cmpl.bda[2], param->auth_cmpl.bda[3],
                     param->auth_cmpl.bda[4], param->auth_cmpl.bda[5]);
            have_peer_addr = false;
            app_state = APP_STATE_PAIRING;
            xEventGroupSetBits(g_bt_events, EVT_CLEAR_PEER_ADDR | EVT_SET_DISCOVERABLE);
        }
        break;

    case ESP_BT_GAP_PIN_REQ_EVT:
        ESP_LOGI(TAG, "GAP PIN_REQ addr=%02x:%02x:%02x:%02x:%02x:%02x min_16=%d",
                 param->pin_req.bda[0], param->pin_req.bda[1],
                 param->pin_req.bda[2], param->pin_req.bda[3],
                 param->pin_req.bda[4], param->pin_req.bda[5],
                 param->pin_req.min_16_digit);
        {
            esp_bt_pin_code_t pin = {'0','0','0','0'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        }
        break;

    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "GAP SSP CFM_REQ addr=%02x:%02x:%02x:%02x:%02x:%02x passkey=%lu",
                 param->cfm_req.bda[0], param->cfm_req.bda[1],
                 param->cfm_req.bda[2], param->cfm_req.bda[3],
                 param->cfm_req.bda[4], param->cfm_req.bda[5],
                 (unsigned long)param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;

    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "GAP SSP KEY_NOTIF passkey=%lu", (unsigned long)param->key_notif.passkey);
        break;

    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "GAP SSP KEY_REQ addr=%02x:%02x:%02x:%02x:%02x:%02x",
                 param->key_req.bda[0], param->key_req.bda[1],
                 param->key_req.bda[2], param->key_req.bda[3],
                 param->key_req.bda[4], param->key_req.bda[5]);
        break;

    case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
        ESP_LOGI(TAG, "GAP RSSI: delta=%d status=%d",
                 param->read_rssi_delta.rssi_delta, param->read_rssi_delta.stat);
        break;

    case ESP_BT_GAP_CONFIG_EIR_DATA_EVT:
        ESP_LOGI(TAG, "GAP EIR config: stat=%d", param->config_eir_data.stat);
        break;

    case ESP_BT_GAP_SET_AFH_CHANNELS_EVT:
        ESP_LOGI(TAG, "GAP AFH channels: stat=%d", param->set_afh_channels.stat);
        break;

    case ESP_BT_GAP_READ_REMOTE_NAME_EVT:
        ESP_LOGI(TAG, "GAP remote name: stat=%d name='%s'",
                 param->read_rmt_name.stat,
                 param->read_rmt_name.stat == ESP_BT_STATUS_SUCCESS
                     ? (char*)param->read_rmt_name.rmt_name : "(err)");
        break;

    case ESP_BT_GAP_REMOVE_BOND_DEV_COMPLETE_EVT:
        ESP_LOGI(TAG, "GAP REMOVE_BOND: status=%d addr=%02x:%02x:%02x:%02x:%02x:%02x",
                 param->remove_bond_dev_cmpl.status,
                 param->remove_bond_dev_cmpl.bda[0], param->remove_bond_dev_cmpl.bda[1],
                 param->remove_bond_dev_cmpl.bda[2], param->remove_bond_dev_cmpl.bda[3],
                 param->remove_bond_dev_cmpl.bda[4], param->remove_bond_dev_cmpl.bda[5]);
        break;

    case ESP_BT_GAP_QOS_CMPL_EVT:
        ESP_LOGI(TAG, "GAP QOS: status=%d", param->qos_cmpl.stat);
        break;

    default:
        ESP_LOGD(TAG, "GAP event=%d (unhandled)", event);
        break;
    }
}

/* ======================================================================
 * CALLBACK HFP CLIENT – ZDARZENIA PROFILU HANDS-FREE
 *
 * WAŻNE: callback HFP działa w kontekście wątku Bluedroid.
 * Zasady bezpieczeństwa:
 *   1. NIE wywołuj funkcji BT API (gap_set_scan_mode, itp.) – ryzyko zakleszczenia
 *      wewnętrznych locków Bluedroid podczas gdy callback jeszcze trwa.
 *   2. NIE wykonuj operacji flash (NVS) – blokuje wątek BT nawet o kilka ms,
 *      co powoduje timeout SCO i drop połączenia podczas dzwonienia.
 *   3. Używaj EventGroup do zlecania operacji do bt_reconnect_task.
 *   4. bt_state_set() przez mutex jest bezpieczne (krótkie blokowanie).
 *   5. Zmiana app_state jest bezpieczna (zmienna atomowa na ESP32).
 * ====================================================================== */

static void hf_client_cb(esp_hf_client_cb_event_t event,
                         esp_hf_client_cb_param_t *param)
{
    switch (event) {

    case ESP_HF_CLIENT_CONNECTION_STATE_EVT: {
        bt_state_set(param->conn_stat.state);
        int new_state = param->conn_stat.state;

        /* Nazwy stanów dla czytelnych logów */
        const char *state_names[] = {
            "DISCONNECTED", "CONNECTING", "CONNECTED", "SLC_CONNECTED", "DISCONNECTING"
        };
        const char *state_str = (new_state >= 0 && new_state < 5)
                                ? state_names[new_state] : "UNKNOWN";

        ESP_LOGI(TAG, "HFP CONN STATE: %s (%d) addr=%02x:%02x:%02x:%02x:%02x:%02x",
                 state_str, new_state,
                 param->conn_stat.remote_bda[0], param->conn_stat.remote_bda[1],
                 param->conn_stat.remote_bda[2], param->conn_stat.remote_bda[3],
                 param->conn_stat.remote_bda[4], param->conn_stat.remote_bda[5]);

        switch (new_state) {
        case ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED:
            ESP_LOGW(TAG, "HFP DISCONNECTED – app_state->IDLE");
            app_state = APP_STATE_IDLE;
            xEventGroupSetBits(g_bt_events, EVT_SET_CONNECTABLE);
            break;

        case ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING:
            ESP_LOGI(TAG, "HFP CONNECTING...");
            break;

        case ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED:
        case ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED:
            ESP_LOGI(TAG, "HFP CONNECTED (state=%s) peer_features=0x%04x",
                     state_str, param->conn_stat.peer_feat);
            app_state = APP_STATE_CONNECTED;
            xEventGroupSetBits(g_bt_events, EVT_SET_CONNECTABLE);

            if (!have_peer_addr ||
                memcmp(peer_addr, param->conn_stat.remote_bda, sizeof(esp_bd_addr_t)) != 0) {
                ESP_LOGI(TAG, "Nowy peer_addr – zapisuje do NVS");
                memcpy(peer_addr, param->conn_stat.remote_bda, sizeof(esp_bd_addr_t));
                have_peer_addr = true;
                xEventGroupSetBits(g_bt_events, EVT_SAVE_PEER_ADDR);
            }
            break;

        case ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTING:
            ESP_LOGW(TAG, "HFP DISCONNECTING...");
            break;

        default:
            ESP_LOGW(TAG, "HFP CONN unknown state=%d", new_state);
            break;
        }
        break;
    }

    case ESP_HF_CLIENT_RING_IND_EVT:
        ESP_LOGI(TAG, "HFP RING_IND");
        start_ringing_alarm();
        break;

    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        ESP_LOGI(TAG, "HFP CALL_SETUP: %d (0=idle,1=incoming,2=outdialing,3=outalerting)",
                 param->call_setup.status);
        if (param->call_setup.status == ESP_HF_CALL_SETUP_STATUS_INCOMING) {
            start_ringing_alarm();
        } else {
            stop_ringing_alarm();
        }
        break;

    case ESP_HF_CLIENT_CIND_CALL_EVT:
        ESP_LOGI(TAG, "HFP CALL: %d (0=none,1=active)", param->call.status);
        if (param->call.status == 0) {
            stop_ringing_alarm();
        } else {
            app_state = APP_STATE_IN_CALL;
        }
        break;

    case ESP_HF_CLIENT_AUDIO_STATE_EVT: {
        int audio_state = param->audio_stat.state;
        const char *audio_names[] = {
            "DISCONNECTED", "CONNECTING", "CONNECTED", "CONNECTED_MSBC"
        };
        const char *astr = (audio_state >= 0 && audio_state < 4)
                           ? audio_names[audio_state] : "UNKNOWN";
        ESP_LOGI(TAG, "HFP AUDIO STATE: %s (%d) handle=%lu frame_size=%u addr=%02x:%02x:%02x:%02x:%02x:%02x",
                 astr, audio_state,
                 (unsigned long)param->audio_stat.sync_conn_handle,
                 param->audio_stat.preferred_frame_size,
                 param->audio_stat.remote_bda[0], param->audio_stat.remote_bda[1],
                 param->audio_stat.remote_bda[2], param->audio_stat.remote_bda[3],
                 param->audio_stat.remote_bda[4], param->audio_stat.remote_bda[5]);

        if (audio_state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
            audio_state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            g_sco_handle = param->audio_stat.sync_conn_handle;
            if (param->audio_stat.preferred_frame_size > 0) {
                g_sco_frame_size = param->audio_stat.preferred_frame_size;
            }
            ESP_LOGI(TAG, "SCO zestawione: handle=%lu frame_size=%u",
                     (unsigned long)g_sco_handle, g_sco_frame_size);
        } else if (audio_state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
            ESP_LOGW(TAG, "SCO rozlaczone (handle bylo %lu)", (unsigned long)g_sco_handle);
            g_sco_handle = 0;
        }
        break;
    }

    case ESP_HF_CLIENT_VOLUME_CONTROL_EVT:
        ESP_LOGI(TAG, "HFP VOLUME: type=%d vol=%d",
                 param->volume_control.type, param->volume_control.volume);
        break;

    case ESP_HF_CLIENT_CIND_SIGNAL_STRENGTH_EVT:
        ESP_LOGD(TAG, "HFP SIGNAL: %d", param->signal_strength.value);
        break;

    case ESP_HF_CLIENT_CIND_ROAMING_STATUS_EVT:
        ESP_LOGD(TAG, "HFP ROAMING: %d", param->roaming.status);
        break;

    case ESP_HF_CLIENT_CIND_BATTERY_LEVEL_EVT:
        ESP_LOGD(TAG, "HFP BATTERY: %d", param->battery_level.value);
        break;

    case ESP_HF_CLIENT_BSIR_EVT:
        ESP_LOGI(TAG, "HFP BSIR (in-band ringtone): %d", param->bsir.state);
        break;

    case ESP_HF_CLIENT_CLIP_EVT:
        /* CLIP – numer dzwoniącego (Calling Line Identification) */
        ESP_LOGI(TAG, "HFP CLIP (caller id): %s",
                 param->clip.number ? param->clip.number : "(null)");
        break;

    case ESP_HF_CLIENT_CCWA_EVT:
        /* CCWA – oczekujące połączenie (Call Waiting) */
        ESP_LOGI(TAG, "HFP CCWA (call waiting): %s",
                 param->ccwa.number ? param->ccwa.number : "(null)");
        break;

    case ESP_HF_CLIENT_BINP_EVT:
        ESP_LOGI(TAG, "HFP BINP (last voice tag): %s",
                 param->binp.number ? param->binp.number : "(null)");
        break;

    case ESP_HF_CLIENT_COPS_CURRENT_OPERATOR_EVT:
        ESP_LOGI(TAG, "HFP COPS (operator): %s",
                 param->cops.name ? param->cops.name : "(null)");
        break;

    case ESP_HF_CLIENT_CLCC_EVT:
        ESP_LOGI(TAG, "HFP CLCC: idx=%d dir=%d status=%d mpty=%d num=%s",
                 param->clcc.idx, param->clcc.dir,
                 param->clcc.status, param->clcc.mpty,
                 param->clcc.number ? param->clcc.number : "(null)");
        break;

    case ESP_HF_CLIENT_CIND_CALL_HELD_EVT:
        ESP_LOGI(TAG, "HFP CALL_HELD: %d", param->call_held.status);
        break;

    case ESP_HF_CLIENT_AT_RESPONSE_EVT:
        ESP_LOGI(TAG, "HFP AT_RESPONSE: cme=%d code=%d",
                 param->at_response.cme, param->at_response.code);
        break;

    case ESP_HF_CLIENT_CIND_SERVICE_AVAILABILITY_EVT:
        ESP_LOGD(TAG, "HFP SERVICE: %d", param->service_availability.status);
        break;

    case ESP_HF_CLIENT_PKT_STAT_NUMS_GET_EVT:
        ESP_LOGI(TAG, "HFP PKT STATS: rx_total=%lu rx_correct=%lu rx_err=%lu rx_none=%lu tx_total=%lu tx_discarded=%lu",
                 (unsigned long)param->pkt_nums.rx_total,
                 (unsigned long)param->pkt_nums.rx_correct,
                 (unsigned long)param->pkt_nums.rx_err,
                 (unsigned long)param->pkt_nums.rx_none,
                 (unsigned long)param->pkt_nums.tx_total,
                 (unsigned long)param->pkt_nums.tx_discarded);
        break;

    case ESP_HF_CLIENT_PROF_STATE_EVT:
        ESP_LOGI(TAG, "HFP PROF_STATE: %d", param->prof_stat.state);
        break;

    default:
        ESP_LOGW(TAG, "HFP event=%d (unhandled)", event);
        break;
    }
}

/* ======================================================================
 * INICJALIZACJA GPIO
 * ====================================================================== */

static void init_gpio(void)
{
    gpio_config_t io_conf = {0};
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
        (1ULL << PIN_TOWER_RED)    |
        (1ULL << PIN_TOWER_YELLOW) |
        (1ULL << PIN_TOWER_GREEN)  |
        (1ULL << PIN_RING_OUT);
    gpio_config(&io_conf);

    gpio_config_t btn_conf = {0};
    btn_conf.intr_type    = GPIO_INTR_DISABLE;
    btn_conf.mode         = GPIO_MODE_INPUT;
    btn_conf.pin_bit_mask = (1ULL << PIN_BUTTON);
    btn_conf.pull_up_en   = 1;
    gpio_config(&btn_conf);

    set_tower_outputs(false, false, false);
    gpio_set_level(PIN_RING_OUT, 0);
}

/* ======================================================================
 * PEŁNE WYŁĄCZENIE STOSU BT
 *
 * MUSI być wywołane przed nvs_flash_erase() lub esp_restart() w handlerach
 * przycisku. Jeśli Bluedroid działa w chwili kasowania NVS, jego task
 * (wyższy priorytet, drugi rdzeń) może nadpisać skasowane klucze bondów
 * w ciągu kilkuset ms — restart wczyta te dane z powrotem i bond "wraca".
 * Wyłączenie stosu zatrzymuje wszystkie wątki BT przed operacjami na flash.
 * ====================================================================== */
static void bt_full_shutdown(void)
{
    esp_hf_client_deinit();
    vTaskDelay(pdMS_TO_TICKS(300));  /* czas na obsługę callbacków deinit */
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}

/* ======================================================================
 * AUDIO HCI SCO – zewnętrzny kodek (CONFIG_BT_HFP_USE_EXTERNAL_CODEC=y)
 *
 * ESP-IDF v5.5.1 wymaga nowego API audio gdy HCI + USE_EXTERNAL_CODEC.
 * Stary legacy esp_hf_client_register_data_callback używał wewnętrznego
 * kodeka mSBC, który wg dokumentacji ESP-IDF jest niestabilny i ma być
 * usunięty. Po N połączeniach wewnętrzny kodek gubił stan i SCO padało.
 *
 * Nowe API: jedna funkcja zwrotna wywoływana dla każdej ramki SCO.
 * Odbiera bufor z dźwiękiem z telefonu (odrzucamy – brak głośnika),
 * a następnie wysyła bufor z ciszą jako "mikrofon".
 *
 * CVSD cisza = 0x55 (01010101 w bitstream).
 * 0x00 to NIE jest cisza w CVSD – to sygnał piłokształtny (głośny dźwięk),
 * który powoduje że iPhone uznaje mikrofon za uszkodzony i po kilku
 * połączeniach odmawia zestawienia SCO.
 * ====================================================================== */

/* g_sco_handle i g_sco_frame_size zadeklarowane w sekcji ZMIENNE GLOBALNE */

static void hf_audio_data_cb(esp_hf_sync_conn_hdl_t hdl,
                              esp_hf_audio_buff_t   *in_buf,
                              bool                   is_bad_frame)
{
    (void)hdl;
    (void)is_bad_frame;

    /* Zwalniamy odebrany bufor – brak głośnika, dane z telefonu odrzucamy */
    esp_hf_client_audio_buff_free(in_buf);

    /* Alokujemy bufor ciszy do wysłania jako "mikrofon" */
    esp_hf_audio_buff_t *out = esp_hf_client_audio_buff_alloc(g_sco_frame_size);
    if (!out) return;

    /*
     * 0x55 = bitstream 01010101... w CVSD = cisza (amplituda 0).
     * Stos przejmuje własność bufora po udanym send – nie free'ujemy.
     */
    memset(out->data, 0x55, out->data_len);

    if (esp_hf_client_audio_data_send(g_sco_handle, out) != ESP_OK) {
        esp_hf_client_audio_buff_free(out);
    }
}

/* ======================================================================
 * INICJALIZACJA BLUETOOTH HFP
 * ====================================================================== */

/*
 * init_bluetooth – inicjalizuje stos BT w trybie Classic BT only.
 *
 * Kolejność wymagana przez ESP-IDF:
 *   1. esp_bt_mem_release(BLE) – przed init kontrolera, odzyskuje ~30 kB RAM
 *   2. esp_bt_controller_init/enable – sprzętowy kontroler BT
 *   3. esp_bluedroid_init/enable – stos protokołów
 *   4. GAP: callback, nazwa, SSP (IO_CAP_NONE = Just Works)
 *   5. Scan mode: discoverable na starcie, connectable-only po połączeniu
 *   6. esp_hf_client_init – rejestracja profilu HFP
 *
 * Scan mode zarządzany wyłącznie przez bt_reconnect_task (przez EventGroup),
 * nigdy bezpośrednio z callbacków BT.
 */
static void init_bluetooth(void)
{
    esp_err_t ret;

    /* Zwalniamy pamięć BLE – MUSI być przed esp_bt_controller_init() */
    esp_bt_mem_release(ESP_BT_MODE_BLE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }

    esp_bt_gap_register_callback(gap_cb);
    esp_bt_gap_set_device_name("BT_RING_TOWER");

    /* SSP Just Works – nowoczesne telefony nie potrzebują PINu */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t   iocap      = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(esp_bt_io_cap_t));

    /* Discoverable na starcie – konieczne przy pierwszym parowaniu */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    esp_hf_client_register_callback(hf_client_cb);
    esp_hf_client_init();
    /*
     * W trybie PCM stos BT obsługuje audio sprzętowo – brak callbacków audio.
     * HCI audio (audio_data_cb + alloc/free co 7.5ms) powodował Interrupt WDT:
     * operacje heap wyłączają przerwania, blokując I2C ISR (OLED 50ms).
     */

    ESP_LOGI(TAG, "Bluetooth HFP zainicjalizowany");
}

/* ======================================================================
 * TASK RECONNECT I OBSŁUGA ZDARZEŃ BT
 *
 * Ten task pełni dwie role:
 *   1. Przetwarza żądania z callbacków BT (EventGroup) – NVS, scan mode
 *   2. Co RECONNECT_INTERVAL_MS próbuje połączyć się z telefonem
 *
 * Dlaczego operacje BT API są tutaj, nie w callbackach?
 *   Callbacki BT działają w kontekście wątku Bluedroid. Wywołanie z tego
 *   wątku funkcji, które wewnętrznie próbują zająć ten sam lock (np.
 *   esp_bt_gap_set_scan_mode), może prowadzić do zakleszczenia.
 *   Operacje NVS (flash) zajmują kilka ms i blokują wątek BT – w tym czasie
 *   telefon nie dostaje odpowiedzi na SCO setup i rozłącza HFP.
 * ====================================================================== */

#define BT_RECONNECT_INTERVAL_MS   12000
#define BT_CONNECTING_TIMEOUT_MS   15000

static void bt_reconnect_task(void *arg)
{
    /* Czas na pełną inicjalizację stosu BT przed pierwszą próbą */
    vTaskDelay(pdMS_TO_TICKS(6000));

    uint32_t connecting_since_ms = 0;

    while (1) {
        /*
         * Sprawdzamy EventGroup z timeout 0 (nieblokujące) – obsługujemy
         * zlecenia z callbacków BT (NVS, scan mode) natychmiast gdy się pojawią.
         * xEventGroupWaitBits z timeout=1 tick pozwala na pętlę polling.
         */
        EventBits_t bits = xEventGroupWaitBits(
            g_bt_events,
            EVT_SAVE_PEER_ADDR | EVT_CLEAR_PEER_ADDR |
            EVT_SET_DISCOVERABLE | EVT_SET_CONNECTABLE,
            pdTRUE,   /* kasuj bity po odczycie */
            pdFALSE,  /* OR – dowolny bit wystarczy */
            pdMS_TO_TICKS(BT_RECONNECT_INTERVAL_MS)  /* timeout = interwał reconnect */
        );

        /* --- Obsługa zleceń z callbacków BT --- */

        if (bits & EVT_SAVE_PEER_ADDR) {
            /* Zapis adresu do flash – bezpieczny tu, bo nie jesteśmy w kontekście BT */
            save_peer_addr_to_nvs(peer_addr);
        }

        if (bits & EVT_CLEAR_PEER_ADDR) {
            /* Kasowanie bonda z Bluedroid i adresu z flash */
            if (have_peer_addr) {
                esp_bt_gap_remove_bond_device(peer_addr);
            }
            clear_peer_addr_from_nvs();
        }

        if (bits & EVT_SET_DISCOVERABLE) {
            /* Włączamy odkrywalność – parowanie lub auth failure */
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }

        if (bits & EVT_SET_CONNECTABLE) {
            /* Wyłączamy odkrywalność – połączeni lub rozłączeni ale sparowani */
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        }

        /* --- Aktywny reconnect --- */

        int state = bt_state_get();

        /* Detekcja i obsługa utknięcia w stanie CONNECTING */
        if (state == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTING) {
            if (connecting_since_ms == 0) {
                connecting_since_ms = get_ms();
            } else if ((get_ms() - connecting_since_ms) > BT_CONNECTING_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Timeout CONNECTING – anuluje");
                esp_hf_client_disconnect(peer_addr);
                connecting_since_ms = 0;
            }
        } else {
            connecting_since_ms = 0;
        }

        /* Próba połączenia tylko gdy mamy adres i jesteśmy rozłączeni */
        if (have_peer_addr &&
            state == ESP_HF_CLIENT_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "Probuje polaczyc sie z telefonem...");
            esp_err_t err = esp_hf_client_connect(peer_addr);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "esp_hf_client_connect: %s", esp_err_to_name(err));
            }
        }
    }
}

/* ======================================================================
 * OBSŁUGA PRZYCISKU
 * ====================================================================== */

typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SHORT,     /* 100–700 ms   – brak akcji                  */
    BTN_EVENT_MEDIUM,    /* 700–3000 ms  – toggle trybu parowania       */
    BTN_EVENT_LONG,      /* 3–6 s        – skasuj sparowany telefon     */
    BTN_EVENT_VERY_LONG  /* >6 s         – restart urządzenia           */
} button_event_t;

/* Polling przycisku – wywoływany co 50 ms z ui_task */
static button_event_t button_poll(void)
{
    static bool     last_pressed  = false;
    static uint32_t press_time_ms = 0;

    bool pressed = (gpio_get_level(PIN_BUTTON) == 0);
    uint32_t now = get_ms();

    if (pressed && !last_pressed) {
        press_time_ms = now;
    } else if (!pressed && last_pressed) {
        uint32_t dt = now - press_time_ms;
        if      (dt >  100 && dt <  700) return BTN_EVENT_SHORT;
        else if (dt >= 700 && dt < 3000) return BTN_EVENT_MEDIUM;
        else if (dt >= 3000 && dt < 6000) return BTN_EVENT_LONG;
        else if (dt >= 6000)              return BTN_EVENT_VERY_LONG;
    }

    last_pressed = pressed;
    return BTN_EVENT_NONE;
}

/*
 * handle_button – wykonuje akcję przypisaną do zdarzenia przycisku.
 * Zmiany scan mode są zlecane przez EventGroup (tak samo jak z callbacków BT),
 * żeby cała logika GAP była w jednym miejscu (bt_reconnect_task).
 */
static void handle_button(button_event_t ev)
{
    switch (ev) {
    case BTN_EVENT_SHORT:
        ESP_LOGI(TAG, "Krotkie wcisniecie (brak akcji)");
        break;

    case BTN_EVENT_MEDIUM:
        if (app_state != APP_STATE_PAIRING) {
            ESP_LOGI(TAG, "Wejscie w tryb parowania");
            app_state = APP_STATE_PAIRING;
            xEventGroupSetBits(g_bt_events, EVT_SET_DISCOVERABLE);
        } else {
            ESP_LOGI(TAG, "Wyjscie z trybu parowania");
            int cur = bt_state_get();
            app_state = (cur == ESP_HF_CLIENT_CONNECTION_STATE_CONNECTED ||
                         cur == ESP_HF_CLIENT_CONNECTION_STATE_SLC_CONNECTED)
                        ? APP_STATE_CONNECTED : APP_STATE_IDLE;
            xEventGroupSetBits(g_bt_events, EVT_SET_CONNECTABLE);
        }
        break;

    case BTN_EVENT_LONG: {
        /*
         * Kasowanie WSZYSTKICH bondów + restart ESP32.
         *
         * Kolejność jest krytyczna:
         *   1. Usuń bondy z Bluedroid (pока działa – może zapisać do NVS)
         *   2. Odczekaj aż Bluedroid potwierdzi usunięcie (zapis do NVS)
         *   3. WYŁĄCZ cały stos BT (bt_full_shutdown) – zatrzymuje wątki BT
         *   4. Skasuj peer_addr z NVS – stos nie działa, nie nadpisze
         *   5. Restart
         *
         * Bez kroku 3: Bluedroid (wyższy priorytet, drugi rdzeń) mógłby
         * zapisać klucze bondów z powrotem do NVS podczas opóźnienia przed
         * restartem, a restart wczytałby je ponownie – bond "wraca".
         */
        ESP_LOGI(TAG, "Kasuje wszystkie bondy i restartuję");
        oled_show_2lines("KASUJE", "BONDY...");

        int bond_num = esp_bt_gap_get_bond_device_num();
        if (bond_num > 0) {
            esp_bd_addr_t *bond_list = malloc(bond_num * sizeof(esp_bd_addr_t));
            if (bond_list) {
                esp_bt_gap_get_bond_device_list(&bond_num, bond_list);
                for (int i = 0; i < bond_num; i++) {
                    esp_bt_gap_remove_bond_device(bond_list[i]);
                }
                free(bond_list);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));  /* czas na zapis usunięcia bondów do NVS */
        bt_full_shutdown();              /* zatrzymaj BT – nie nadpisze NVS po kasowaniu */
        clear_peer_addr_from_nvs();
        esp_restart();
        break;
    }

    case BTN_EVENT_VERY_LONG:
        /*
         * Reset fabryczny – kasuje CAŁE NVS (bondy Bluedroid + peer_addr).
         *
         * bt_full_shutdown() MUSI być przed nvs_flash_erase().
         * Bez niego: Bluedroid (wątek wyższy priorytet) może zapisać klucze
         * bondów z powrotem do NVS po kasowaniu, a przed restartem.
         * Efekt: restart wczytuje "skasowane" bondy – reset nie działa.
         */
        ESP_LOGW(TAG, "Reset fabryczny");
        oled_show_2lines("RESET", "FABRYCZNY");
        bt_full_shutdown();   /* najpierw stop BT, potem kasuj NVS */
        nvs_flash_erase();
        esp_restart();
        break;

    default:
        break;
    }
}

/* ======================================================================
 * UI TASK – WIEŻA + OLED + PRZYCISK (co 50 ms)
 * ====================================================================== */

static void ui_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        button_event_t ev = button_poll();
        if (ev != BTN_EVENT_NONE) handle_button(ev);
        update_ui_by_state();
        /* vTaskDelayUntil zapewnia dokładny okres 50 ms niezależnie od czasu I2C */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50));
    }
}

/* ======================================================================
 * APP_MAIN
 * ====================================================================== */

void app_main(void)
{
    esp_err_t ret;

    /* NVS – wymagane przez Bluedroid (bond keys) i nasz peer_addr */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS uszkodzony – kasuje i inicjalizuje od nowa");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Mutex BT – musi istnieć przed jakimkolwiek użyciem bt_state_get/set */
    g_bt_state_mutex = xSemaphoreCreateMutex();
    if (!g_bt_state_mutex) {
        ESP_LOGE(TAG, "Brak mutexa BT – restart");
        esp_restart();
    }

    /*
     * EventGroup – kanał komunikacji callback→task.
     * Musi istnieć przed init_bluetooth(), bo callbacki mogą odpalić
     * natychmiast po rejestracji (Bluedroid może wysłać zdarzenia z historii).
     */
    g_bt_events = xEventGroupCreate();
    if (!g_bt_events) {
        ESP_LOGE(TAG, "Brak EventGroup – restart");
        esp_restart();
    }

    init_gpio();
    init_oled();

    if (load_peer_addr_from_nvs(peer_addr)) {
        have_peer_addr = true;
        ESP_LOGI(TAG, "Zaladowano peer_addr z NVS");
    } else {
        ESP_LOGI(TAG, "Brak peer_addr – czekam na parowanie");
    }

    init_bluetooth();

    /*
     * Stosy 6144 bajtów zamiast 4096:
     *   bt_reconnect_task – wywołuje BT API (connect, gap_set_scan_mode) i NVS
     *   ui_task           – wywołuje I2C OLED + pełną logikę UI
     * Większy stos eliminuje trudne do reprodukcji crashe przy przepełnieniu.
     */
    xTaskCreate(bt_reconnect_task, "bt_reconnect", 6144, NULL, 5, NULL);
    xTaskCreate(ui_task,           "ui_task",      6144, NULL, 5, NULL);

    ESP_LOGI(TAG, "System wystartowal");
}
