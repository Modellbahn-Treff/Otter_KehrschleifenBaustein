// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <stdint.h>
#include <stdbool.h>

// Kehrschleifen-Steuerung — placeholder module.
// TODO: implement relay switching (K1/K2) based on relais.kicad_sch.

// Snapshot of everything the state machine works with, for the display.
typedef struct {
    uint8_t  status;            // 0..10, see calculateKBStatus() in kb.cpp
    bool     ExtA, IntA, IntM, IntB, ExtB;  // detectors, in track order
    bool     trainLost;         // a train vanished inside the loop, waiting for it to reappear
    bool     voltagePresent;    // measured loop voltage is above the threshold
    uint32_t voltage_mv;        // measured loop voltage in mV
    uint16_t voltageRaw;        // averaged raw ADC reading behind voltage_mv
    uint16_t senseRaw[3];       // averaged raw ADC readings of IntA, IntM, IntB
    uint16_t senseThreshold;    // raw value above which a sense input counts as occupied
} kb_state_t;

void KB_Start(void);
void KB_ExtA(const char *msg);
void KB_ExtB(const char *msg);
void KB_Loop(void);

// Fills `out` with the current state. Used by the display.
void KB_GetState(kb_state_t *out);
