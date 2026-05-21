# BT Ring Tower 2

## Cel

Urządzenie na ESP32, które łączy się z telefonem przez Bluetooth HFP (Hands-Free Profile) i sygnalizuje przychodzące połączenia za pomocą wieży sygnalizacyjnej, buzzera/przekaźnika, wyświetlacza OLED i przycisku.

**Sprzęt:** ESP32, ESP-IDF v5.5.1, Classic BT only (BLE zwolnione)

### Piny

| Pin | GPIO | Funkcja |
|-----|------|---------|
| PIN_TOWER_RED | 27 | Wieża — czerwony |
| PIN_TOWER_YELLOW | 26 | Wieża — żółty |
| PIN_TOWER_GREEN | 25 | Wieża — zielony |
| PIN_RING_OUT | 33 | Przekaźnik / buzzer |
| PIN_BUTTON | 0 | Przycisk |
| I2C SDA | 21 | OLED 128×32 |
| I2C SCL | 22 | OLED 128×32 |

---

## Architektura — taski FreeRTOS

Program składa się z dwóch tasków użytkownika plus callbacków BT działających w wątku Bluedroid:

```
app_main()
  ├── bt_reconnect_task  — reconnect, NVS, GAP scan mode
  └── ui_task            — przycisk + OLED + wieża co 50ms

Wątek Bluedroid (wewnętrzny):
  ├── gap_cb()           — callback parowania/autoryzacji
  └── hf_client_cb()     — callback zdarzeń HFP
```

---

## Stany aplikacji (`app_state_t`)

| Stan | Znaczenie |
|------|-----------|
| `APP_STATE_IDLE` | Brak połączenia BT |
| `APP_STATE_PAIRING` | Tryb parowania aktywny |
| `APP_STATE_CONNECTED` | Połączony, oczekiwanie |
| `APP_STATE_RINGING` | Telefon dzwoni |
| `APP_STATE_IN_CALL` | Rozmowa w toku |

---

## Opis każdej funkcji

### Pomocnicze

**`bt_state_get()` / `bt_state_set()`**
Wątkobezpieczny odczyt i zapis stanu połączenia HFP przez mutex. Używane ponieważ `g_hfp_connection_state` jest zapisywane z wątku Bluedroid, a czytane z `ui_task`.

**`get_ms()`**
Zwraca czas systemowy w milisekundach na podstawie `esp_timer_get_time()`. Używany do migania LEDów i timeoutów.

---

### NVS — pamięć flash

**`save_peer_addr_to_nvs(addr)`**
Zapisuje adres MAC sparowanego telefonu do pamięci flash (namespace `bt_cfg`, klucz `peer_addr`). Wywoływana **tylko z `bt_reconnect_task`** — nigdy z callbacków BT, bo operacje flash blokują wątek BT na kilka ms i powodują timeout SCO.

**`load_peer_addr_from_nvs(addr_out)`**
Wczytuje zapisany adres telefonu przy starcie urządzenia. Wywołanie z `app_main` przed inicjalizacją BT — bezpieczne.

**`clear_peer_addr_from_nvs()`**
Kasuje adres telefonu z flash. Wywoływana z `bt_reconnect_task` lub z handlera przycisku (po wcześniejszym `bt_full_shutdown()`).

---

### Wieża RYG

**`set_tower_outputs(red, yellow, green)`**
Ustawia stany wyjść GPIO 27/26/25. Prosta funkcja opakowująca trzy wywołania `gpio_set_level`.

---

### OLED 128×32 (SSD1306 I2C)

**`oled_write_command(cmd)`**
Wysyła jeden bajt komendy do sterownika SSD1306 przez I2C (prefiks `0x00`).

**`oled_write_data(data, len)`**
Wysyła dane graficzne do OLED w kawałkach po 16 bajtów (prefiks `0x40`). Chunking wymagany przez ograniczenia I2C bufora ESP-IDF.

**`oled_update_full()`**
Przepisuje cały bufor `oled_buffer` (512 bajtów) na wyświetlacz, strona po stronie (4 strony × 128 kolumn).

**`oled_clear_buffer()` / `oled_clear()`**
`clear_buffer` — zeruje bufor RAM. `clear` — zeruje bufor i wysyła na wyświetlacz.

**`oled_set_pixel(x, y, on)`**
Ustawia lub kasuje pojedynczy piksel w buforze. Przelicza współrzędne (x,y) na stronę i bit w tablicy `oled_buffer`.

**`oled_find_glyph(c)`**
Wyszukuje literę w tablicy czcionki `font5x7`. Konwertuje małe litery na wielkie. Dla nieznanych znaków zwraca spację.

**`oled_draw_char_big(x, base_y, c)`**
Rysuje jeden znak czcionką 5×7 pikseli, skalowaną 2× (każdy piksel → 2×2 piksele), wynikowy rozmiar znaku to ~10×14 pikseli.

**`oled_draw_text_big(line, text)`**
Rysuje ciąg znaków w linii 0 (y=0) lub linii 1 (y=16). Każdy znak zajmuje 11 pikseli szerokości.

**`oled_show_2lines(line1, line2)`**
Czyści bufor, rysuje dwie linie tekstu i wysyła na wyświetlacz. Główna funkcja do wyświetlania komunikatów.

**`init_oled()`**
Inicjalizuje magistralę I2C (400 kHz, SDA=21, SCL=22) i wysyła sekwencję inicjalizacyjną SSD1306 dla wyświetlacza 128×32 (multiplex=31, COM pins sequential).

---

### Logika UI i alarmu

**`update_ui_by_state()`**
Wywoływana co 50 ms z `ui_task`. Na podstawie `app_state` i stanu BT:
- Bez BT → żółty miga co 500 ms
- Połączony → zielony świeci stale
- Dzwoni → czerwony + RING_OUT migają co 500 ms
- Przy każdej **zmianie** stanu → aktualizuje OLED (guard `last_state` zapobiega zapisowi I2C co 50 ms)
- Race condition guard: jeśli BT rozłączone a stan = RINGING/IN_CALL → wymusza IDLE

**`start_ringing_alarm()`**
Ustawia `app_state = APP_STATE_RINGING`. `update_ui_by_state` zajmuje się resztą.

**`stop_ringing_alarm()`**
Cofa stan do `APP_STATE_CONNECTED` (jeśli BT wciąż aktywne) lub `APP_STATE_IDLE`.

---

### Callback GAP (`gap_cb`)

Obsługuje zdarzenia parowania i autoryzacji Bluetooth:

| Zdarzenie | Działanie |
|-----------|-----------|
| `AUTH_CMPL` sukces | Loguje adres i nazwę urządzenia |
| `AUTH_CMPL` błąd | Kasuje peer_addr, wchodzi w tryb parowania |
| `PIN_REQ` | Odpowiada PINem `0000` (urządzenia starszego typu) |
| `CFM_REQ` | Automatycznie akceptuje SSP Just Works (nowoczesne telefony) |
| `KEY_NOTIF` | Loguje passkey |
| `REMOVE_BOND_COMPLETE` | Loguje wynik usunięcia bonda |
| Pozostałe | Loguje ID zdarzenia |

Wszystkie zmiany scan mode są zlecane przez EventGroup do `bt_reconnect_task` — nigdy wywołane bezpośrednio z callbacku.

---

### Callback HFP (`hf_client_cb`)

Główny callback zdarzeń protokołu Hands-Free:

| Zdarzenie | Działanie |
|-----------|-----------|
| `CONNECTION_STATE_EVT` | Aktualizuje `g_hfp_connection_state` przez mutex; przy CONNECTED zapisuje peer_addr przez EventGroup; przy DISCONNECTED → IDLE |
| `RING_IND_EVT` | Uruchamia alarm dzwonienia |
| `CIND_CALL_SETUP_EVT` | Start/stop alarmu zależnie od statusu (incoming/idle) |
| `CIND_CALL_EVT` | Aktywna rozmowa → `IN_CALL`, koniec → stop alarmu |
| `AUDIO_STATE_EVT` | Zapisuje uchwyt SCO i rozmiar ramki; loguje stan połączenia audio (CVSD/mSBC) |
| `CLIP_EVT` | Loguje numer dzwoniącego (Caller ID) |
| `CCWA_EVT` | Loguje oczekujące połączenie (Call Waiting) |
| `PKT_STAT_NUMS_GET_EVT` | Loguje statystyki pakietów SCO (rx/tx, błędy) |
| `VOLUME_CONTROL_EVT` | Loguje zmianę głośności |
| Pozostałe | Loguje ID zdarzenia |

---

### GPIO

**`init_gpio()`**
Konfiguruje GPIO:
- Wyjścia (bez przerwań): PIN_TOWER_RED/YELLOW/GREEN (27/26/25), PIN_RING_OUT (33)
- Wejście z pull-up: PIN_BUTTON (GPIO 0)

Wszystkie wyjścia startują w stanie LOW.

---

### Bluetooth

**`bt_full_shutdown()`**
Pełne wyłączenie stosu BT w prawidłowej kolejności: deinit HFP → czekaj 300 ms → disable/deinit Bluedroid → disable/deinit kontroler. **Musi być wywołana przed `nvs_flash_erase()` lub kasowaniem bondów**, bo inaczej wątek Bluedroid może nadpisać NVS po kasowaniu.

**`hf_audio_data_cb(hdl, in_buf, is_bad_frame)`**
Callback SCO audio (używany tylko w trybie HCI — w aktualnej konfiguracji PCM nieaktywny). Zwalnia odebrany bufor, alokuje bufor ciszy wypełniony `0x55` (cisza CVSD) i odsyła jako "mikrofon". `0x55` = wzorzec 01010101 w bitstream CVSD = amplituda 0. `0x00` to NIE jest cisza — to sygnał piłokształtny powodujący, że iPhone po kilku połączeniach odmawia zestawienia SCO.

**`init_bluetooth()`**
Inicjalizuje Classic BT:
1. Zwalnia pamięć BLE (~30 kB RAM)
2. Inicjalizuje i uruchamia kontroler BT
3. Inicjalizuje i uruchamia Bluedroid
4. Rejestruje callback GAP, ustawia nazwę `BT_RING_TOWER`, konfiguruje SSP Just Works (IO_CAP_NONE)
5. Ustawia tryb discoverable
6. Rejestruje callback HFP i inicjalizuje profil

W trybie PCM nie rejestruje audio callback — stos BT obsługuje audio sprzętowo bez angażowania CPU.

---

### Przycisk

**`button_poll()`**
Polling co 50 ms (z `ui_task`). Mierzy czas trzymania przycisku i zwraca zdarzenie przy puszczeniu:

| Czas | Zdarzenie |
|------|-----------|
| 100–700 ms | `BTN_EVENT_SHORT` |
| 700–3000 ms | `BTN_EVENT_MEDIUM` |
| 3–6 s | `BTN_EVENT_LONG` |
| >6 s | `BTN_EVENT_VERY_LONG` |

**`handle_button(ev)`**
Wykonuje akcje:
- **SHORT** — brak akcji (loguje)
- **MEDIUM** — toggle trybu parowania (discoverable ON/OFF)
- **LONG** — kasuje wszystkie bondy BT + peer_addr z NVS + restart
- **VERY_LONG** — reset fabryczny (`nvs_flash_erase` + restart)

---

### Taski

**`bt_reconnect_task(void*)`**
Czeka 6 s przy starcie, potem w pętli co 12 s:
1. Sprawdza EventGroup — obsługuje zlecenia z callbacków (NVS, scan mode)
2. Wykrywa timeout w stanie CONNECTING (>15 s) — wywołuje disconnect
3. Jeśli ma adres telefonu i stan = DISCONNECTED → próbuje połączyć

**`ui_task(void*)`**
Pętla co dokładnie 50 ms (`vTaskDelayUntil`):
1. Odpytuje przycisk (`button_poll`)
2. Obsługuje zdarzenie przycisku (`handle_button`)
3. Aktualizuje LEDy, RING_OUT i OLED (`update_ui_by_state`)

---

### Punkt wejścia

**`app_main()`**
Sekwencja startowa:
1. Inicjalizacja NVS (z auto-naprawą przy uszkodzeniu)
2. Tworzenie mutexa BT i EventGroup — muszą istnieć przed init BT
3. `init_gpio()` + `init_oled()`
4. Wczytanie peer_addr z NVS
5. `init_bluetooth()`
6. Start tasków `bt_reconnect_task` i `ui_task` (stos 6144 bajtów każdy)
