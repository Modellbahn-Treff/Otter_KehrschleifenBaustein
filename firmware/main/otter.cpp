// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "otter.h"

const uint8_t sensePin[3] = {36, 39, 32};
const uint8_t voltagePin = 33;
const uint8_t dipPin[4] = {4, 13, 16, 17};
const uint8_t buttonPin = 27;
const uint8_t swapRelayPin[2] = {25, 26};
const uint8_t switchRelayPin[2] = {18, 19};
const char *MqttRefresh = "otter/Refresh";
