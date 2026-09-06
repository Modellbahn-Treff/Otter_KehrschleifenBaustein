// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <stdint.h>
#include <stdbool.h>

// Kehrschleifen-Steuerung: sensing, loop-voltage measurement, the state
// machine and the K1/K2 relay switching in kb.cpp.

// Number of local occupancy sense pins (IntA, IntM, IntB). Shared with
// kb_state_t::senseRaw below so the two can never drift apart.
#define SENSE_PIN_COUNT 3

// Snapshot of everything the state machine works with, for the display.
typedef struct {
    uint8_t  status;            // 0..11, see calculateKBStatus() in kb.cpp
    bool     ExtA, IntA, IntM, IntB, ExtB;  // detectors, in track order
    bool     trainLost;         // a train vanished inside the loop, waiting for it to reappear
    bool     voltagePresent;    // measured loop voltage is above the threshold
    uint32_t voltage_mv;        // measured loop voltage in mV
    uint16_t voltageRaw;        // averaged raw ADC reading behind voltage_mv
    uint16_t senseRaw[SENSE_PIN_COUNT];  // averaged raw ADC readings of IntA, IntM, IntB
    uint16_t senseThreshold;    // raw value above which a sense input counts as occupied
} kb_state_t;

void KB_Start(void);
void KB_ExtA(const char *msg);
void KB_ExtB(const char *msg);
void KB_Loop(void);

// Forces the state machine to `status` (0..11) and switches the relays to
// match, bypassing the normal sensor-driven transitions.
void KB_SetStatus(uint8_t status);

// Fills `out` with the current state. Used by the display.
void KB_GetState(kb_state_t *out);
