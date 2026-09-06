// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "display.h"
#include "settings.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "display";

// ---------------------------------------------------------------------------
// Hardware constants — Display1 in Kehrschleifenbaustein.kicad_sch
// ---------------------------------------------------------------------------

#define OLED_SDA      GPIO_NUM_21
#define OLED_SCL      GPIO_NUM_22
#define OLED_ADDR     0x3C
#define OLED_HZ       400000
#define OLED_TIMEOUT_MS   20   // one page is ~3 ms at 400 kHz, so this is generous
#define OLED_W        128
#define OLED_PAGES    8    // 64 px / 8 px per page
#define CHAR_W        6    // 5 px glyph + 1 px gap
#define COLS          (OLED_W / CHAR_W)  // 21 characters per row

// ---------------------------------------------------------------------------
// 5x8 font — ASCII 0x20 to 0x7E, 5 column bytes per glyph.
// Each byte is a column, bit 0 = top pixel.
// Source: classic Adafruit GFX / public domain embedded font.
// ---------------------------------------------------------------------------

static const uint8_t font5x8[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // 0x20  ' '
    {0x00,0x00,0x5F,0x00,0x00}, // 0x21  '!'
    {0x00,0x07,0x00,0x07,0x00}, // 0x22  '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // 0x23  '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // 0x24  '$'
    {0x23,0x13,0x08,0x64,0x62}, // 0x25  '%'
    {0x36,0x49,0x55,0x22,0x50}, // 0x26  '&'
    {0x00,0x05,0x03,0x00,0x00}, // 0x27  '''
    {0x00,0x1C,0x22,0x41,0x00}, // 0x28  '('
    {0x00,0x41,0x22,0x1C,0x00}, // 0x29  ')'
    {0x14,0x08,0x3E,0x08,0x14}, // 0x2A  '*'
    {0x08,0x08,0x3E,0x08,0x08}, // 0x2B  '+'
    {0x00,0x50,0x30,0x00,0x00}, // 0x2C  ','
    {0x08,0x08,0x08,0x08,0x08}, // 0x2D  '-'
    {0x00,0x60,0x60,0x00,0x00}, // 0x2E  '.'
    {0x20,0x10,0x08,0x04,0x02}, // 0x2F  '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // 0x30  '0'
    {0x00,0x42,0x7F,0x40,0x00}, // 0x31  '1'
    {0x42,0x61,0x51,0x49,0x46}, // 0x32  '2'
    {0x21,0x41,0x45,0x4B,0x31}, // 0x33  '3'
    {0x18,0x14,0x12,0x7F,0x10}, // 0x34  '4'
    {0x27,0x45,0x45,0x45,0x39}, // 0x35  '5'
    {0x3C,0x4A,0x49,0x49,0x30}, // 0x36  '6'
    {0x01,0x71,0x09,0x05,0x03}, // 0x37  '7'
    {0x36,0x49,0x49,0x49,0x36}, // 0x38  '8'
    {0x06,0x49,0x49,0x29,0x1E}, // 0x39  '9'
    {0x00,0x36,0x36,0x00,0x00}, // 0x3A  ':'
    {0x00,0x56,0x36,0x00,0x00}, // 0x3B  ';'
    {0x08,0x14,0x22,0x41,0x00}, // 0x3C  '<'
    {0x14,0x14,0x14,0x14,0x14}, // 0x3D  '='
    {0x00,0x41,0x22,0x14,0x08}, // 0x3E  '>'
    {0x02,0x01,0x51,0x09,0x06}, // 0x3F  '?'
    {0x32,0x49,0x79,0x41,0x3E}, // 0x40  '@'
    {0x7E,0x11,0x11,0x11,0x7E}, // 0x41  'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 0x42  'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 0x43  'C'
    {0x7F,0x41,0x41,0x22,0x1C}, // 0x44  'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 0x45  'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 0x46  'F'
    {0x3E,0x41,0x49,0x49,0x7A}, // 0x47  'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 0x48  'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 0x49  'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 0x4A  'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 0x4B  'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 0x4C  'L'
    {0x7F,0x02,0x0C,0x02,0x7F}, // 0x4D  'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 0x4E  'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 0x4F  'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 0x50  'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 0x51  'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 0x52  'R'
    {0x46,0x49,0x49,0x49,0x31}, // 0x53  'S'
    {0x01,0x01,0x7F,0x01,0x01}, // 0x54  'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 0x55  'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 0x56  'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 0x57  'W'
    {0x63,0x14,0x08,0x14,0x63}, // 0x58  'X'
    {0x07,0x08,0x70,0x08,0x07}, // 0x59  'Y'
    {0x61,0x51,0x49,0x45,0x43}, // 0x5A  'Z'
    {0x00,0x7F,0x41,0x41,0x00}, // 0x5B  '['
    {0x02,0x04,0x08,0x10,0x20}, // 0x5C  '\'
    {0x00,0x41,0x41,0x7F,0x00}, // 0x5D  ']'
    {0x04,0x02,0x01,0x02,0x04}, // 0x5E  '^'
    {0x40,0x40,0x40,0x40,0x40}, // 0x5F  '_'
    {0x00,0x01,0x02,0x04,0x00}, // 0x60  '`'
    {0x20,0x54,0x54,0x54,0x78}, // 0x61  'a'
    {0x7F,0x48,0x44,0x44,0x38}, // 0x62  'b'
    {0x38,0x44,0x44,0x44,0x20}, // 0x63  'c'
    {0x38,0x44,0x44,0x48,0x7F}, // 0x64  'd'
    {0x38,0x54,0x54,0x54,0x18}, // 0x65  'e'
    {0x08,0x7E,0x09,0x01,0x02}, // 0x66  'f'
    {0x0C,0x52,0x52,0x52,0x3E}, // 0x67  'g'
    {0x7F,0x08,0x04,0x04,0x78}, // 0x68  'h'
    {0x00,0x44,0x7D,0x40,0x00}, // 0x69  'i'
    {0x20,0x40,0x44,0x3D,0x00}, // 0x6A  'j'
    {0x7F,0x10,0x28,0x44,0x00}, // 0x6B  'k'
    {0x00,0x41,0x7F,0x40,0x00}, // 0x6C  'l'
    {0x7C,0x04,0x18,0x04,0x78}, // 0x6D  'm'
    {0x7C,0x08,0x04,0x04,0x78}, // 0x6E  'n'
    {0x38,0x44,0x44,0x44,0x38}, // 0x6F  'o'
    {0x7C,0x14,0x14,0x14,0x08}, // 0x70  'p'
    {0x08,0x14,0x14,0x18,0x7C}, // 0x71  'q'
    {0x7C,0x08,0x04,0x04,0x08}, // 0x72  'r'
    {0x48,0x54,0x54,0x54,0x20}, // 0x73  's'
    {0x04,0x3F,0x44,0x40,0x20}, // 0x74  't'
    {0x3C,0x40,0x40,0x20,0x7C}, // 0x75  'u'
    {0x1C,0x20,0x40,0x20,0x1C}, // 0x76  'v'
    {0x3C,0x40,0x30,0x40,0x3C}, // 0x77  'w'
    {0x44,0x28,0x10,0x28,0x44}, // 0x78  'x'
    {0x0C,0x50,0x50,0x50,0x3C}, // 0x79  'y'
    {0x44,0x64,0x54,0x4C,0x44}, // 0x7A  'z'
    {0x00,0x08,0x36,0x41,0x00}, // 0x7B  '{'
    {0x00,0x00,0x7F,0x00,0x00}, // 0x7C  '|'
    {0x00,0x41,0x36,0x08,0x00}, // 0x7D  '}'
    {0x10,0x08,0x08,0x10,0x08}, // 0x7E  '~'
};

// ---------------------------------------------------------------------------
// I2C / SSD1306 internals
//
// The panel is driven through a RAM framebuffer instead of writing every
// glyph straight out: KB_Loop() redraws on every detector edge and every
// 250 ms tick, and a per-character I2C transaction would stall the 10 ms
// sensing tick for far too long. Drawing into `fb` costs nothing, and
// oled_flush() pushes only the pages that really changed — usually none.
// ---------------------------------------------------------------------------

static i2c_master_bus_handle_t s_bus = nullptr;
static i2c_master_dev_handle_t s_dev = nullptr;
static bool s_ok = false;

// KB_Loop() draws from the main task, KB_ExtA()/KB_ExtB() draw from the MQTT
// event task. Both would otherwise interleave in the framebuffer and clear
// each other's dirty flags mid-frame.
static SemaphoreHandle_t s_lock = nullptr;

static uint8_t fb[OLED_PAGES][OLED_W];      // 1 bit per pixel, bit 0 = top of page
static bool    fb_dirty[OLED_PAGES];

static esp_err_t oled_write(const uint8_t *data, size_t len) {
    return i2c_master_transmit(s_dev, data, len, OLED_TIMEOUT_MS);
}

// Set the write cursor to (page, col); page addressing mode.
static esp_err_t oled_set_pos(uint8_t page, uint8_t col) {
    const uint8_t buf[4] = {
        0x00,                                   // control byte: command stream
        (uint8_t)(0xB0 | (page & 0x07)),        // page address
        (uint8_t)(0x00 | (col & 0x0F)),         // lower nibble of col
        (uint8_t)(0x10 | ((col >> 4) & 0x0F)),  // upper nibble of col
    };
    return oled_write(buf, sizeof(buf));
}

// Push every page whose content changed since the last flush. A panel that
// stops answering takes the display out of service instead of making every
// later redraw wait for the I2C timeout — the loop control must keep its
// 10 ms tick with or without a display.
static void oled_flush(void) {
    uint8_t buf[1 + OLED_W];
    buf[0] = 0x40;                              // control byte: data stream
    for (uint8_t p = 0; p < OLED_PAGES; p++) {
        if (!fb_dirty[p]) continue;
        memcpy(buf + 1, fb[p], OLED_W);
        if (oled_set_pos(p, 0) != ESP_OK || oled_write(buf, sizeof(buf)) != ESP_OK) {
            ESP_LOGE(TAG, "display write failed, disabling the display");
            s_ok = false;
            return;
        }
        fb_dirty[p] = false;
    }
}

static void fb_clear(void) {
    memset(fb, 0, sizeof(fb));
    memset(fb_dirty, true, sizeof(fb_dirty));
}

// Draw one character into the framebuffer at (row, col in pixels).
static void fb_draw_char(uint8_t row, uint8_t col, char c) {
    if (row >= OLED_PAGES || col + CHAR_W > OLED_W) return;
    if (c < 0x20 || c > 0x7E) c = ' ';
    const uint8_t *glyph = font5x8[c - 0x20];
    for (uint8_t i = 0; i < 5; i++) {
        if (fb[row][col + i] != glyph[i]) {
            fb[row][col + i] = glyph[i];
            fb_dirty[row] = true;
        }
    }
    if (fb[row][col + 5] != 0x00) {             // 1 px gap after the glyph
        fb[row][col + 5] = 0x00;
        fb_dirty[row] = true;
    }
}

// Draw a string at (row, col in pixels). Returns the column after the last
// character.
static uint8_t fb_draw_str(uint8_t row, uint8_t col, const char *s) {
    while (*s && col + CHAR_W <= OLED_W) {
        fb_draw_char(row, col, *s++);
        col += CHAR_W;
    }
    return col;
}

// Draw a string and blank the rest of the row, so a shorter value never
// leaves fragments of the previous one behind.
static void fb_draw_line(uint8_t row, uint8_t col, const char *s) {
    col = fb_draw_str(row, col, s);
    for (; col < OLED_W; col++) {
        if (fb[row][col] != 0x00) {
            fb[row][col] = 0x00;
            fb_dirty[row] = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Screens
// ---------------------------------------------------------------------------

typedef enum {
    SCREEN_LOOP    = 0,   // state machine, polarity and detectors
    SCREEN_NETWORK = 1,   // WiFi / IP / broker / client id
    SCREEN_SENSORS = 2,   // raw ADC averages, for setting the thresholds
    SCREEN_COUNT   = 3,
} display_screen_t;

static const char *screen_title[SCREEN_COUNT] = {"Loop", "Network", "Sensors"};

static display_screen_t s_screen = SCREEN_LOOP;

// Cached by display_set_network(), owned by main.cpp's event handlers.
static bool s_wifi_ok = false;
static bool s_mqtt_ok = false;
static char s_ip_str[16] = "---";

// Short names for the states of calculateKBStatus(), at most 21 characters
// so they fit on one row.
static const char *state_name(uint8_t status) {
    switch (status) {
        case 0:  return "Off";
        case 1:  return "About to enter A";
        case 2:  return "Entered from A";
        case 3:  return "Leaving through B";
        case 4:  return "About to enter B";
        case 5:  return "Entered from B";
        case 6:  return "Leaving through A";
        case 7:  return "FAIL both sides";
        case 8:  return "FAIL low voltage";
        case 9:  return "FAIL stray voltage";
        case 10: return "FAIL train too long";
        default: return "?";
    }
}

// Track polarity at the two gaps: A for states 1, 2, 6 - B for 3, 4, 5.
// The loop is unpowered in every other state, so it has no polarity.
static char state_polarity(uint8_t status) {
    switch (status) {
        case 1: case 2: case 6: return 'A';
        case 3: case 4: case 5: return 'B';
        default:                return '-';
    }
}

// The loop is powered in states 1 to 6 and switched off in 0 and 7 to 10.
static bool state_powered(uint8_t status) {
    return status >= 1 && status <= 6;
}

// "16.2V" from millivolts, without pulling float formatting into printf.
static void format_volts(char *buf, size_t len, uint32_t mv) {
    snprintf(buf, len, "%lu.%luV", (unsigned long)(mv / 1000),
                                   (unsigned long)((mv % 1000) / 100));
}

// Every row is composed as one full line and drawn with fb_draw_line(), which
// blanks the rest of the row. Nothing from the previous screen can survive in
// a gap that way, and because fb_draw_*() only mark a page dirty on a real
// change, redrawing an unchanged row still costs no I2C traffic.

static void draw_loop_screen(const kb_state_t *kb) {
    char buf[32];
    char volts[12];

    // Row 2: state number, the resulting polarity, and the lost-train flag.
    snprintf(buf, sizeof(buf), "State %-2u  Pol %c%s",
             kb->status, state_polarity(kb->status), kb->trainLost ? " LOST" : "");
    fb_draw_line(2, 0, buf);

    // Row 3: what that state means, in words.
    fb_draw_line(3, 0, state_name(kb->status));

    // Row 4: is the loop powered, and what the ADC measures inside it.
    format_volts(volts, sizeof(volts), kb->voltage_mv);
    snprintf(buf, sizeof(buf), "Loop %-3s  U %s",
             state_powered(kb->status) ? "ON" : "OFF", volts);
    fb_draw_line(4, 0, buf);

    fb_draw_line(5, 0, "");

    // Rows 6 and 7: the five detectors in track order, ExtA on the left.
    //   Ea Ia Im Ib Eb
    //   #  #  .  .  .
    fb_draw_line(6, 0, "Ea Ia Im Ib Eb");
    snprintf(buf, sizeof(buf), "%c  %c  %c  %c  %c",
             kb->ExtA ? '#' : '.', kb->IntA ? '#' : '.', kb->IntM ? '#' : '.',
             kb->IntB ? '#' : '.', kb->ExtB ? '#' : '.');
    fb_draw_line(7, 0, buf);
}

static void draw_network_screen(void) {
    char buf[32];
    // The label takes 6 of the 21 columns, so the value is cut to the 15 that
    // are left rather than silently overflowing the row.

    snprintf(buf, sizeof(buf), "WiFi: %s", s_wifi_ok ? "OK" : "--");
    fb_draw_line(2, 0, buf);

    snprintf(buf, sizeof(buf), "IP:   %s", s_ip_str);
    fb_draw_line(3, 0, buf);

    snprintf(buf, sizeof(buf), "MQTT: %.15s", s_mqtt_ok ? mqtt_server : "--");
    fb_draw_line(4, 0, buf);

    snprintf(buf, sizeof(buf), "ID:   %.15s", client_name);
    fb_draw_line(5, 0, buf);

    fb_draw_line(6, 0, "");
    fb_draw_line(7, 0, "");
}

// Raw ADC averages as the state machine sees them — the numbers to look at
// when the occupancy threshold or the voltage divider need checking.
static void draw_sensors_screen(const kb_state_t *kb) {
    char buf[32];
    char volts[12];
    static const char *sense_label[3] = {"IntA", "IntM", "IntB"};

    snprintf(buf, sizeof(buf), "ADC raw    thr %u", kb->senseThreshold);
    fb_draw_line(2, 0, buf);

    for (uint8_t i = 0; i < 3; i++) {
        snprintf(buf, sizeof(buf), "%s: %4u", sense_label[i], kb->senseRaw[i]);
        fb_draw_line(3 + i, 0, buf);
    }

    format_volts(volts, sizeof(volts), kb->voltage_mv);
    snprintf(buf, sizeof(buf), "Ubus: %4u  %s", kb->voltageRaw, volts);
    fb_draw_line(6, 0, buf);

    fb_draw_line(7, 0, "");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void display_init(void) {
    s_lock = xSemaphoreCreateMutex();
    if (!s_lock) {
        ESP_LOGE(TAG, "could not create the display mutex");
        return;
    }

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port                     = I2C_NUM_0;
    bus_cfg.sda_io_num                   = OLED_SDA;
    bus_cfg.scl_io_num                   = OLED_SCL;
    bus_cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt            = 7;
    bus_cfg.flags.enable_internal_pullup = true;   // the OLED module carries the real pull-ups

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus() failed: %s", esp_err_to_name(err));
        return;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = OLED_ADDR;
    dev_cfg.scl_speed_hz    = OLED_HZ;

    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device() failed: %s", esp_err_to_name(err));
        return;
    }

    // No panel on the bus is not fatal — the board runs the loop just fine
    // without it, so log it and leave the display disabled.
    err = i2c_master_probe(s_bus, OLED_ADDR, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no SSD1306 at 0x%02X: %s", OLED_ADDR, esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    static const uint8_t init_cmds[] = {
        0x00,        // control byte: command stream
        0xAE,        // display OFF
        0xD5, 0x80,  // clock divide / oscillator frequency
        0xA8, 0x3F,  // multiplex ratio = 64 rows
        0xD3, 0x00,  // display offset = 0
        0x40,        // display start line = 0
        0x8D, 0x14,  // charge pump enable
        0x20, 0x02,  // page addressing mode
        0xA1,        // segment remap: col 127 -> SEG0
        0xC8,        // COM output scan: remapped (top->bottom)
        0xDA, 0x12,  // COM pin config: alternative
        0x81, 0xCF,  // contrast = 207
        0xD9, 0xF1,  // pre-charge period
        0xDB, 0x40,  // VCOMH deselect level
        0xA4,        // follow display RAM (not all-on)
        0xA6,        // normal display (not inverted)
        0xAF,        // display ON
    };
    err = oled_write(init_cmds, sizeof(init_cmds));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 init sequence failed: %s", esp_err_to_name(err));
        return;
    }

    s_ok = true;                    // oled_flush() clears this again if the panel drops out
    fb_clear();
    fb_draw_str(0, 0, "Kehrschleife");
    oled_flush();

    if (s_ok) {
        ESP_LOGI(TAG, "SSD1306 initialised");
    }
}

void display_next_screen(void) {
    if (!s_ok) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_screen = (display_screen_t)((s_screen + 1) % SCREEN_COUNT);
    ESP_LOGI(TAG, "screen %d/%d", s_screen + 1, (int)SCREEN_COUNT);
    xSemaphoreGive(s_lock);
}

void display_set_network(bool wifi_ok, bool mqtt_ok, const char *ip_str) {
    if (!s_lock) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_wifi_ok = wifi_ok;
    s_mqtt_ok = mqtt_ok;
    if (ip_str) {
        snprintf(s_ip_str, sizeof(s_ip_str), "%s", ip_str);
    }
    xSemaphoreGive(s_lock);
}

void display_update(const kb_state_t *kb) {
    if (!s_ok || !kb) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);

    // Rows 0 and 1 are the same on every screen: what this board is and
    // whether it is talking to anyone, then which page the button is on.
    char head[48];
    snprintf(head, sizeof(head), "%-15s%s", "Kehrschleife",
             s_wifi_ok ? (s_mqtt_ok ? "MQTT" : "WiFi") : "----");
    fb_draw_line(0, 0, head);

    snprintf(head, sizeof(head), "%-17.17s%u/%u", screen_title[s_screen],
             (unsigned)(s_screen + 1), (unsigned)SCREEN_COUNT);
    fb_draw_line(1, 0, head);

    switch (s_screen) {
        case SCREEN_LOOP:    draw_loop_screen(kb);    break;
        case SCREEN_NETWORK: draw_network_screen();   break;
        case SCREEN_SENSORS: draw_sensors_screen(kb); break;
        default: break;
    }

    oled_flush();
    xSemaphoreGive(s_lock);
}
