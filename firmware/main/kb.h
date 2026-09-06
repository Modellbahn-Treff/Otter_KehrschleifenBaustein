// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Kehrschleifen-Steuerung — placeholder module.
// TODO: implement relay switching (K1/K2) and Melder (detection) evaluation
// based on relais.kicad_sch / Melder.kicad_sch.

void KB_Start(void);
void KB_ExtA(const char *msg);
void KB_ExtB(const char *msg);
void KB_Loop(void);
