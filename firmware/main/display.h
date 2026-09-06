// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <stdbool.h>
#include "kb.h"

// Initialise I2C and the SSD1306 128x64 OLED (Display1, addr 0x3C,
// SDA=GPIO21, SCL=GPIO22). Safe to call before WiFi is up; if the panel does
// not answer, every other call below turns into a no-op.
void display_init(void);

// Advance to the next screen (driven by the button SW2 on buttonPin).
void display_next_screen(void);

// Cache the network state shown on the network screen. Called from the WiFi
// and MQTT event handlers in main.cpp, which are the only places that know it.
void display_set_network(bool wifi_ok, bool mqtt_ok, const char *ip_str);

// Redraw the current screen from `kb`. Only the display pages that actually
// changed are pushed over I2C, so calling this on every state change and on
// the periodic refresh is cheap.
void display_update(const kb_state_t *kb);
