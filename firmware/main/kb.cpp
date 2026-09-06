// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "kb.h"
#include "otter.h"
#include "display.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "KB";

static void calculateKBStatus(bool isrecursive = false);
static void updateDisplay(void);

#define ADC_RING_SIZE   20

#define ADC_VREF_MV            3300  // nominal ADC1 full-scale at ADC_ATTEN_DB_12 (uncalibrated)
#define ADC_MAX_RAW            4095  // 12-bit resolution
#define VOLTAGE_DIVIDER_TOP_K    75  // R41
#define VOLTAGE_DIVIDER_BOT_K    10  // R42
#define VOLTAGE_THRESHOLD_MV  10000  // 10 V
#define SENSE_THRESHOLD_RAW      35  // averaged raw ADC value above which a Melder counts as occupied
#define SENSE_HOLD_MS           500  // debounce: keep "occupied" this long after the last detection
#define LOST_TRAIN_TIMEOUT_MS 60000  // keep the loop powered this long after a train vanished inside
#define DISPLAY_REFRESH_MS      250  // redraw interval for the live values (voltage, raw ADC)
#define BUTTON_DEBOUNCE_MS       50  // ignore further edges on the screen button for this long

static uint8_t  kbStatus = 0;                                     // 0=off, 1=Train is about to enter from A, 2=Train entered from A, 3=Train leaving through B, 4=Train is about to enter from B, 5= Train entered from B, 6=Train leaving through A, 7=Fail-safe because of trains on both sides, 8=Fail-safe because of low voltage, 9=Fail-safe because of voltage during off state, 10=Fail-safe because of too long train in the loop, 11=Fail-safe because of unknown reason (e.g. two trains inside the loop at the same time)

// calculateKBStatus() runs on the main task (KB_Loop -> updateVoltage /
// sense pin handling) but is also invoked from the MQTT event task
// (KB_ExtA/KB_ExtB), and KB_SetStatus() (also MQTT task) writes kbStatus and
// the edge-tracking statics directly. This recursive mutex serializes all
// three entry points so a forced status via MQTT can never interleave with
// an in-progress sensor-driven transition. Recursive because
// calculateKBStatus() calls itself to settle multi-step transitions.
static SemaphoreHandle_t kbMutex = nullptr;

// Inputs as seen at the previous calculateKBStatus() call, used to detect
// edges. File-scope (rather than local statics) so KB_SetStatus() can
// resync them when forcing a status, keeping them consistent with the
// current sensor state instead of going stale.
static bool lastExtA = false, lastIntA = false, lastIntM = false, lastIntB = false, lastExtB = false;

static float    voltage = 0.0f;                                   // voltage in the loop, in mV
static uint16_t voltageRaw = 0;                                   // peak raw ADC reading behind `voltage`
static bool     voltagePresent = false;                           // true if voltage is present in the loop (above VOLTAGE_THRESHOLD_MV)

static bool     ExtA = false;                                     // true if ExtA is occupied
static bool     IntA = false;                                     // true if IntA is occupied
static bool     IntM = false;                                     // true if IntM is occupied
static bool     IntB = false;                                     // true if IntB is occupied
static bool     ExtB = false;                                     // true if ExtB is occupied

static bool     trainLost   = false;                              // true while a train vanished inside the loop without passing ExtA/ExtB
static uint32_t trainLostAt = 0;                                  // millis() when the train vanished

static bool     relaisJustSwitched = false;                       // true for 100ms after a relay pulse, to avoid misreading the Voltage sensor
static uint32_t relaisSwitchedAt = 0;                             // millis() when the relais pulse ended

static uint8_t  sensePinStatus[SENSE_PIN_COUNT];
static uint8_t  sensePinLastStatus[SENSE_PIN_COUNT] = {2, 2, 2};  // 0=not occupied, 1=occupied, 2=unknown (force a publish on the first loop)
static uint16_t sensePinSamples[SENSE_PIN_COUNT][ADC_RING_SIZE];  // ring buffer of raw ADC samples (0..4095)
static uint8_t  sensePinIndex[SENSE_PIN_COUNT];                   // ring index per sensing pin
static uint32_t sensePinSums[SENSE_PIN_COUNT];                    // running sum per sensing pin
static uint32_t sensePinTimeOff[SENSE_PIN_COUNT];                 // debounce: keep "occupied" until this tick

static adc_oneshot_unit_handle_t adc1_handle = nullptr;
static adc_channel_t sensePinChannel[SENSE_PIN_COUNT];
static adc_channel_t voltagePinChannel;

static uint16_t voltagePinSamples[ADC_RING_SIZE];  // ring buffer of raw ADC samples (0..4095)
static uint8_t  voltagePinIndex = 0;                // ring index

// Non-blocking relay pulse queue: KB_Loop() runs on the main 10 ms tick, so
// calculateKBStatus() must never vTaskDelay() to time a relay coil pulse —
// that would stall sensing, voltage sampling and button polling for the
// duration. Instead, pulses are queued here and stepped once per tick by
// updateRelayQueue().
#define RELAY_PULSE_MS    100
#define RELAY_QUEUE_SIZE    8

static gpio_num_t relayQueue[RELAY_QUEUE_SIZE];
static uint8_t    relayQueueHead     = 0;      // pin currently pulsing / next to pulse
static uint8_t    relayQueueTail     = 0;      // one past the last queued pin
static bool       relayPulseOn       = false;  // true while the head pin's coil is energized
static uint32_t   relayPulseDeadline = 0;      // millis() at which to de-energize it

// relayEnqueue() is called from both the main task (KB_Loop -> calculateKBStatus)
// and the MQTT event task (KB_ExtA/KB_ExtB -> calculateKBStatus), and
// updateRelayQueue() (main task only) shares the same queue state. A spinlock
// keeps the two from corrupting relayQueueHead/Tail — critical sections here
// are a handful of instructions, so this never meaningfully delays the 10 ms
// tick the way a blocking mutex could.
static portMUX_TYPE relayMux = portMUX_INITIALIZER_UNLOCKED;

static void relayEnqueue(gpio_num_t pin) {
    portENTER_CRITICAL(&relayMux);
    uint8_t next = (relayQueueTail + 1) % RELAY_QUEUE_SIZE;
    if (next == relayQueueHead) {
        portEXIT_CRITICAL(&relayMux);
        ESP_LOGW(TAG, "relay queue full, dropping pulse");
        return;
    }
    relayQueue[relayQueueTail] = pin;
    relayQueueTail = next;
    portEXIT_CRITICAL(&relayMux);
}

// Energizes the relay pair matching `newStatus`, if it enters a polarity
// group (off/fail-safe, "A", "B") that `previousStatus` was not already in.
// Shared by calculateKBStatus() and KB_SetStatus() (forced status via MQTT).
static void applyRelayForStatus(uint8_t newStatus, uint8_t previousStatus) {
    if (newStatus == 0 || newStatus >= 7) {
        relayEnqueue((gpio_num_t)switchRelayPin[1]);  //turn off the loop
    } else if ((newStatus == 1 || newStatus == 2 || newStatus == 6) && (previousStatus != 1 && previousStatus != 2 && previousStatus != 6)) {
        relayEnqueue((gpio_num_t)swapRelayPin[1]);
        relayEnqueue((gpio_num_t)switchRelayPin[0]); //turn on the loop
    } else if ((newStatus == 3 || newStatus == 4 || newStatus == 5) && (previousStatus != 3 && previousStatus != 4 && previousStatus != 5)) {
        relayEnqueue((gpio_num_t)swapRelayPin[0]);
        relayEnqueue((gpio_num_t)switchRelayPin[0]);  //turn on the loop
    }
}

// Reads one ADC1 channel via the oneshot driver, configured in KB_Start().
static uint16_t analogReadRaw(adc_channel_t channel) {
    int raw = 0;
    esp_err_t err = adc_oneshot_read(adc1_handle, channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "adc_oneshot_read() failed: %s", esp_err_to_name(err));
        return 0;
    }
    return (uint16_t)raw;
}

// Reads Melder input `p` (index into sensePin[]/sensePinChannel[], not a
// GPIO number).
static uint16_t analogRead(uint8_t p) {
    return analogReadRaw(sensePinChannel[p]);
}

static uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Advances the relay pulse queue by at most one step: releases the head pin
// once it has been energized for RELAY_PULSE_MS, then energizes the next
// queued pin. Non-blocking — must be called once per KB_Loop() tick.
static void updateRelayQueue(void) {
    portENTER_CRITICAL(&relayMux);
    if (relayPulseOn) {
        if ((int32_t)(millis() - relayPulseDeadline) < 0) {
            portEXIT_CRITICAL(&relayMux);
            return;                      // still energized, not yet time to release
        }
        gpio_set_level(relayQueue[relayQueueHead], 0);
        relayQueueHead = (relayQueueHead + 1) % RELAY_QUEUE_SIZE;
        relayPulseOn = false;
        relaisJustSwitched = true;  // mark that a relay has just switched
        relaisSwitchedAt = millis();  // record when the relay switched
    }
    if (relayQueueHead != relayQueueTail) {
        gpio_set_level(relayQueue[relayQueueHead], 1);
        relayPulseDeadline = millis() + RELAY_PULSE_MS;
        relayPulseOn = true;
    }
    portEXIT_CRITICAL(&relayMux);
}

void KB_Start(void) {
    kbMutex = xSemaphoreCreateRecursiveMutex();

    adc_oneshot_unit_init_cfg_t init_cfg = {};
    init_cfg.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc1_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = ADC_ATTEN_DB_12;
    chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

    for (uint8_t p = 0; p < SENSE_PIN_COUNT; p++) {
        adc_unit_t unit;
        ESP_ERROR_CHECK(adc_oneshot_io_to_channel(sensePin[p], &unit, &sensePinChannel[p]));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, sensePinChannel[p], &chan_cfg));
    }

    adc_unit_t voltage_unit;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(voltagePin, &voltage_unit, &voltagePinChannel));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, voltagePinChannel, &chan_cfg));

    // SW2 shorts buttonPin to GND, so it needs the internal pull-up.
    gpio_config_t btn_cfg = {};
    btn_cfg.pin_bit_mask = (1ULL << buttonPin);
    btn_cfg.mode         = GPIO_MODE_INPUT;
    btn_cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
    btn_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));

    gpio_config_t relay_cfg = {};
    relay_cfg.pin_bit_mask = (1ULL << swapRelayPin[0])   | (1ULL << swapRelayPin[1]) |
                            (1ULL << switchRelayPin[0]) | (1ULL << switchRelayPin[1]);
    relay_cfg.mode         = GPIO_MODE_OUTPUT;
    relay_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    relay_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    relay_cfg.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&relay_cfg));

    gpio_set_level((gpio_num_t)swapRelayPin[0], 1);
    vTaskDelay(pdMS_TO_TICKS(100));  // wait for the relay to switch
    gpio_set_level((gpio_num_t)swapRelayPin[0], 0);
    gpio_set_level((gpio_num_t)switchRelayPin[1], 1);
    vTaskDelay(pdMS_TO_TICKS(100));  // wait for the relay to switch
    gpio_set_level((gpio_num_t)switchRelayPin[1], 0);
}

void KB_ExtA(const char *msg) {
    //convert the message to a number, if possible
    uint8_t value = atol(msg);
    if (value > 0) {
       ExtA = true;
       ESP_LOGI(TAG, "KB_ExtA(%s) occupied", msg);
    } else if (value == 0) {
       ExtA = false;
       ESP_LOGI(TAG, "KB_ExtA(%s) clear", msg);
    }
    calculateKBStatus();
    updateDisplay();
}

// TODO: react to a message on external input B.
void KB_ExtB(const char *msg) {
    //convert the message to a number, if possible
    uint8_t value = atol(msg);
    if (value > 0) {
       ExtB = true;
       ESP_LOGI(TAG, "KB_ExtB(%s) occupied", msg);
    } else if (value == 0) {
       ExtB = false;
       ESP_LOGI(TAG, "KB_ExtB(%s) clear", msg);
    }
    calculateKBStatus();
    updateDisplay();
}


// State machine of the reversing loop (Kehrschleife).
//
// Detector layout (left to right):  ExtA | IntA  IntM  IntB | ExtB
//   ExtA / ExtB are outside the loop (external inputs via KB_ExtA/KB_ExtB,
//   may arrive with some delay), IntA / IntM / IntB are inside the loop
//   (local sense pins). The "|" mark the two insulated gaps where the
//   polarity must match the outside track.
// The voltage sensor sits inside the loop, so it reads 0 V while the loop is
// switched off. Every state that expects the loop to be powered therefore
// treats "no voltage" as a fault (-> 8), and state 0 treats "voltage" as a
// fault (-> 9).
//
// Polarity per state: A for 1, 2, 6 - B for 3, 4, 5.
//
// A train may stop and reverse at any moment, in any state. The detectors do
// not tell the direction, so it is derived from edges (which detector just
// became occupied or clear) while the train is completely inside the loop,
// which is also the only moment where the polarity may be switched safely.
// This assumes the three inner detectors cover the loop without gaps.
// Known limit: a train that is long enough to occupy IntA and IntB at the
// same time produces no edges when it reverses and cannot be protected.
//
// A train that vanishes from the inner detectors without passing ExtA or
// ExtB (derailed, lifted off, detector dropout) does not switch the loop off
// right away: the current state is kept for LOST_TRAIN_TIMEOUT_MS so the
// train can reappear, then KB_Loop() calls this function again to shut down.
//
// Called whenever one of the inputs (ExtA, ExtB, IntA, IntM, IntB,
// voltagePresent) changes, and once more when the lost-train timeout expires.
static void calculateKBStatus(bool isrecursive) {
    xSemaphoreTakeRecursive(kbMutex, portMAX_DELAY);

    uint8_t previousStatus = kbStatus;

    bool extAFell = !ExtA && lastExtA;
    bool extBFell = !ExtB && lastExtB;
    bool intARose =  IntA && !lastIntA;
    bool intMRose =  IntM && !lastIntM;
    bool intBRose =  IntB && !lastIntB;

    // "Loop interior clear": nothing between the two gaps.
    bool loopClear  = !IntA && !IntM && !IntB;
    // "Everything clear": no detector inside or outside the loop is occupied.
    bool allClear   = loopClear && !ExtA && !ExtB;
    // "Inside only": a train is completely inside the loop and does not
    // straddle a gap, so the polarity may be switched right now.
    bool insideOnly = !loopClear && !ExtA && !ExtB;

    // Direction of a train that is completely inside the loop.
    bool headingA = insideOnly && (intARose                       // head just entered IntA coming from the middle
                                || (intMRose && IntB && !IntA)    // head just entered IntM coming from IntB
                                || extBFell);                     // train came back from ExtB into the loop
    bool headingB = insideOnly && (intBRose                       // head just entered IntB coming from the middle
                                || (intMRose && IntA && !IntB)    // head just entered IntM coming from IntA
                                || extAFell);                     // train came back from ExtA into the loop

    // Lost-train handling: everything is clear now, but at the previous call
    // only inner detectors were occupied, so the train did not leave through
    // ExtA or ExtB. Start the timeout instead of switching off.
    bool wasInsideOnly = (lastIntA || lastIntM || lastIntB) && !lastExtA && !lastExtB;
    bool loopInUse     = (kbStatus == 2 || kbStatus == 3 || kbStatus == 5 || kbStatus == 6);
    if (allClear && wasInsideOnly && loopInUse && !trainLost) {
        trainLost   = true;
        trainLostAt = millis();
        ESP_LOGW(TAG, "train vanished inside the loop, keeping status %u for %u s", kbStatus, LOST_TRAIN_TIMEOUT_MS / 1000);
    } else if (!allClear && trainLost) {
        trainLost = false;           // the train (or something) is detected again -> continue in the current state
        ESP_LOGI(TAG, "train detected again");
    }
    // "Left the loop": all clear and not waiting for a lost train, i.e. the
    // train really passed ExtA/ExtB or the lost-train timeout has expired.
    bool leftLoop = allClear && !trainLost;

    if (kbStatus == 0) {             // off: loop unpowered, nothing approaching
        if (voltagePresent && !relaisJustSwitched) {
            kbStatus = 9;            // 0 -> 9: voltage inside the loop although it should be off -> fail-safe
        } else if (ExtA && !ExtB) {
            kbStatus = 1;            // 0 -> 1: train approaching on side A only -> prepare entry from A
        } else if (ExtB && !ExtA) {
            kbStatus = 4;            // 0 -> 4: train approaching on side B only -> prepare entry from B
        }
        // ExtA && ExtB at the same time: stay off, nobody may enter.
    } else if (kbStatus == 1) {      // Train is about to enter from A (loop powered, polarity A)
        if (!voltagePresent && !relaisJustSwitched) {
            kbStatus = 8;            // 1 -> 8: loop should be powered but is not -> fail-safe
        } else if (ExtB) {
            kbStatus = 7;            // 1 -> 7: a second train shows up on side B -> fail-safe
        } else if (IntB) {
            kbStatus = 11;           // 1 -> 11: a second train shows up on IntB -> fail-safe
        } else if (IntA || IntM) {
            kbStatus = 2;            // 1 -> 2: head of the train reached IntA or IntM -> train is entering
        } else if (!ExtA) {
            kbStatus = 0;            // 1 -> 0: train reversed and backed away from side A without entering -> off
        }
    } else if (kbStatus == 2) {      // Train entered from A (polarity A, train on ExtA and/or IntA..IntM)
        if (!voltagePresent && !relaisJustSwitched) {
            kbStatus = 8;            // 2 -> 8: loop lost power while a train is inside -> fail-safe
        } else if (ExtB) {
            kbStatus = 7;            // 2 -> 7: another train shows up on side B -> fail-safe
        } else if (ExtA && IntB) {
            kbStatus = 10;           // 2 -> 10: head reached IntB while the tail is still on ExtA -> train too long -> fail-safe
        } else if (!ExtA && IntB) {
            kbStatus = 3;            // 2 -> 3: tail cleared ExtA and head reached IntB -> switch to polarity B, train may leave through B
        } else if (leftLoop) {
            kbStatus = 0;            // 2 -> 0: train backed out completely through A, or vanished and did not reappear within 60 s -> off
        } else if (loopClear && ExtA) {
            kbStatus = 1;            // 2 -> 1: train reversed and backed out onto ExtA -> about to enter from A again (same polarity)
        }
        // A train that reverses while still inside keeps polarity A; it can
        // only reach gap A, and reaching IntB switches to 3.
    } else if (kbStatus == 3) {      // Train leaving through B (polarity B, train fully inside or straddling gap B)
        if (!voltagePresent && !relaisJustSwitched) {
            kbStatus = 8;            // 3 -> 8: loop lost power while the train is leaving -> fail-safe
        } else if (headingA) {
            kbStatus = 6;            // 3 -> 6: train reversed and is heading back to A while fully inside -> switch to polarity A
        } else if (leftLoop) {
            kbStatus = 0;            // 3 -> 0: train has completely left through B, or vanished and did not reappear within 60 s -> off
        } else if (loopClear && ExtB && !ExtA) {
            kbStatus = 4;            // 3 -> 4: loop empty, ExtB still occupied (leaving train or next train) -> entry from B possible (same polarity)
        } else if (loopClear && ExtA && !ExtB) {
            kbStatus = 1;            // 3 -> 1: loop empty, next train waiting on side A -> switch to polarity A
        } else if (loopClear && ExtA && ExtB) {
            kbStatus = 0;            // 3 -> 0: loop empty, trains on both sides -> off, nobody may enter
        }
    } else if (kbStatus == 4) {      // Train is about to enter from B (loop powered, polarity B)
        if (!voltagePresent && !relaisJustSwitched) {
            kbStatus = 8;            // 4 -> 8: loop should be powered but is not -> fail-safe
        } else if (ExtA) {
            kbStatus = 7;            // 4 -> 7: a second train shows up on side A -> fail-safe
        } else if (IntA) {
            kbStatus = 11;           // 4 -> 11: a second train shows up on IntA -> fail-safe
        } else if (IntB || IntM) {
            kbStatus = 5;            // 4 -> 5: head of the train reached IntB or IntM -> train is entering
        } else if (!ExtB) {
            kbStatus = 0;            // 4 -> 0: train reversed and backed away from side B without entering -> off
        }
    } else if (kbStatus == 5) {      // Train entered from B (polarity B, train on ExtB and/or IntB..IntM)
        if (!voltagePresent && !relaisJustSwitched) {
            kbStatus = 8;            // 5 -> 8: loop lost power while a train is inside -> fail-safe
        } else if (ExtA) {
            kbStatus = 7;            // 5 -> 7: another train shows up on side A -> fail-safe
        } else if (ExtB && IntA) {
            kbStatus = 10;           // 5 -> 10: head reached IntA while the tail is still on ExtB -> train too long -> fail-safe
        } else if (!ExtB && IntA) {
            kbStatus = 6;            // 5 -> 6: tail cleared ExtB and head reached IntA -> switch to polarity A, train may leave through A
        } else if (leftLoop) {
            kbStatus = 0;            // 5 -> 0: train backed out completely through B, or vanished and did not reappear within 60 s -> off
        } else if (loopClear && ExtB) {
            kbStatus = 4;            // 5 -> 4: train reversed and backed out onto ExtB -> about to enter from B again (same polarity)
        }
        // A train that reverses while still inside keeps polarity B; it can
        // only reach gap B, and reaching IntA switches to 6.
    } else if (kbStatus == 6) {      // Train leaving through A (polarity A, train fully inside or straddling gap A)
        if (!voltagePresent && !relaisJustSwitched) {
            kbStatus = 8;            // 6 -> 8: loop lost power while the train is leaving -> fail-safe
        } else if (headingB) {
            kbStatus = 3;            // 6 -> 3: train reversed and is heading back to B while fully inside -> switch to polarity B
        } else if (leftLoop) {
            kbStatus = 0;            // 6 -> 0: train has completely left through A, or vanished and did not reappear within 60 s -> off
        } else if (loopClear && ExtA && !ExtB) {
            kbStatus = 1;            // 6 -> 1: loop empty, ExtA still occupied (leaving train or next train) -> entry from A possible (same polarity)
        } else if (loopClear && ExtB && !ExtA) {
            kbStatus = 4;            // 6 -> 4: loop empty, next train waiting on side B -> switch to polarity B
        } else if (loopClear && ExtA && ExtB) {
            kbStatus = 0;            // 6 -> 0: loop empty, trains on both sides -> off, nobody may enter
        }
    } else if (kbStatus == 7) {      // Fail-safe because of trains on both sides (loop switched off)
        if (allClear && !voltagePresent) {
            kbStatus = 0;            // 7 -> 0: both trains gone and loop unpowered -> back to off
        }
    } else if (kbStatus == 8) {      // Fail-safe because of low voltage (loop lost power while in use)
        if (allClear && !voltagePresent) {
            kbStatus = 0;            // 8 -> 0: all detectors clear and loop unpowered -> back to off
        }
    } else if (kbStatus == 9) {      // Fail-safe because of voltage during off state (relay/wiring fault)
        if (allClear && !voltagePresent) {
            kbStatus = 0;            // 9 -> 0: voltage gone and all detectors clear -> back to off
        }
    } else if (kbStatus == 10) {     // Fail-safe because of too long train in the loop (loop switched off)
        if (allClear && !voltagePresent) {
            kbStatus = 0;            // 10 -> 0: train removed and loop unpowered -> back to off
        }
    } else if (kbStatus == 11) {     // Fail-safe because of unknown reason (e.g. two trains inside the loop at the same time)
        if (allClear && !voltagePresent) {
            kbStatus = 0;            // 11 -> 0: all detectors clear and loop unpowered -> back to off
        }
    } else {
        ESP_LOGW(TAG, "unknown kbStatus %u, resetting to 11", kbStatus);
        kbStatus = 11;
    }

    lastExtA = ExtA;
    lastIntA = IntA;
    lastIntM = IntM;
    lastIntB = IntB;
    lastExtB = ExtB;

    if (kbStatus == 0 || kbStatus >= 7) {
        trainLost = false;           // off and fail-safe states do not wait for a lost train
    }

    if (kbStatus != previousStatus) {
        ESP_LOGI(TAG, "kbStatus: %u -> %u", previousStatus, kbStatus);

        applyRelayForStatus(kbStatus, previousStatus);

        bool offOrFailsafe = (kbStatus == 0 || kbStatus >= 7);
        bool enteredGroupA = (kbStatus == 1 || kbStatus == 2 || kbStatus == 6) && (previousStatus != 1 && previousStatus != 2 && previousStatus != 6);
        bool enteredGroupB = (kbStatus == 3 || kbStatus == 4 || kbStatus == 5) && (previousStatus != 3 && previousStatus != 4 && previousStatus != 5);
        if (!offOrFailsafe && !enteredGroupA && !enteredGroupB && !isrecursive) {
            calculateKBStatus(true);  // re-evaluate the state machine
        }
    }

    xSemaphoreGiveRecursive(kbMutex);
}

// Forces the state machine straight to `status` and switches the relays to
// match, bypassing the normal sensor-driven transitions. Used by the
// otter/KB/Status/set MQTT command. Subsequent sensor edges continue the
// state machine from this new status as usual.
void KB_SetStatus(uint8_t status) {
    if (status > 11) {
        ESP_LOGW(TAG, "KB_SetStatus: invalid status %u, ignoring", status);
        return;
    }
    xSemaphoreTakeRecursive(kbMutex, portMAX_DELAY);
    uint8_t previousStatus = kbStatus;
    kbStatus  = status;
    trainLost = false;
    // Resync the edge-tracking statics to the current sensor state, so the
    // next real calculateKBStatus() call does not compute a spurious edge
    // from whatever they were left at before this forced jump.
    lastExtA = ExtA;
    lastIntA = IntA;
    lastIntM = IntM;
    lastIntB = IntB;
    lastExtB = ExtB;
    if (kbStatus != previousStatus) {
        ESP_LOGI(TAG, "kbStatus (forced via MQTT): %u -> %u", previousStatus, kbStatus);
        applyRelayForStatus(kbStatus, previousStatus);
    }
    xSemaphoreGiveRecursive(kbMutex);
    updateDisplay();
}

void KB_GetState(kb_state_t *out) {
    if (!out) return;
    out->status         = kbStatus;
    out->ExtA           = ExtA;
    out->IntA           = IntA;
    out->IntM           = IntM;
    out->IntB           = IntB;
    out->ExtB           = ExtB;
    out->trainLost      = trainLost;
    out->voltagePresent = voltagePresent;
    out->voltage_mv     = (uint32_t)voltage;
    out->voltageRaw     = voltageRaw;
    for (uint8_t p = 0; p < SENSE_PIN_COUNT; p++) {
        out->senseRaw[p] = (uint16_t)(sensePinSums[p] / ADC_RING_SIZE);
    }
    out->senseThreshold = SENSE_THRESHOLD_RAW;
}

static void updateDisplay(void) {
    kb_state_t state;
    KB_GetState(&state);
    display_update(&state);
}

// Screen button (SW2 on buttonPin, shorts to GND when pressed): pages
// through the display screens. Polled from the 10 ms tick rather than run
// off an interrupt, so it needs no ISR service of its own.
static void updateButton(void) {
    static bool     lastLevel  = true;   // released, thanks to the internal pull-up
    static uint32_t lastChange = 0;

    bool     level = gpio_get_level((gpio_num_t)buttonPin) != 0;
    uint32_t now   = millis();

    if (level != lastLevel && (uint32_t)(now - lastChange) >= BUTTON_DEBOUNCE_MS) {
        lastChange = now;
        lastLevel  = level;
        if (!level) {                    // falling edge -> pressed
            display_next_screen();
            updateDisplay();
        }
    }
}

// Measures the loop voltage via voltagePin (75k/10k divider, R41/R42) and
// sets `voltage` to true once it exceeds VOLTAGE_THRESHOLD_MV (10 V).
static void updateVoltage(void) {
    uint16_t raw = analogReadRaw(voltagePinChannel);
    voltagePinSamples[voltagePinIndex] = raw;
    voltagePinIndex = (voltagePinIndex + 1) % ADC_RING_SIZE;

    // Peak-hold over the ring buffer rather than an average, so a brief
    // pulse (e.g. a short DCC/AC half-wave) isn't smoothed away.
    uint16_t max_raw = 0;
    for (uint8_t i = 0; i < ADC_RING_SIZE; i++) {
        if (voltagePinSamples[i] > max_raw) {
            max_raw = voltagePinSamples[i];
        }
    }

    uint32_t pin_mv   = ((uint32_t)max_raw * ADC_VREF_MV) / ADC_MAX_RAW;
    uint32_t loop_mv  = pin_mv * (VOLTAGE_DIVIDER_TOP_K + VOLTAGE_DIVIDER_BOT_K) / VOLTAGE_DIVIDER_BOT_K;

    // Kept up to date on every sample, not only on threshold crossings, so
    // the display shows the actual reading.
    voltage    = loop_mv;
    voltageRaw = max_raw;

    bool newVoltage = (loop_mv > VOLTAGE_THRESHOLD_MV);
    if (newVoltage != voltagePresent) {
        voltagePresent = newVoltage;
        ESP_LOGI(TAG, "voltage: %s (%lu mV)", voltagePresent ? "present" : "absent", (unsigned long)voltage);
        if (!relaisJustSwitched) {
            calculateKBStatus();
            updateDisplay();
        }
    }
}

// Called from the main 10 ms tick after the Melder inputs below have been
// sampled.
void KB_Loop(void) {
    updateRelayQueue();
    updateButton();
    updateVoltage();

    if (relaisJustSwitched && (uint32_t)(millis() - relaisSwitchedAt) >= 100) {
        relaisJustSwitched = false;
        calculateKBStatus();
    }
    // The state machine redraws on every change, but the voltage and the raw
    // ADC values move without one, so refresh them on a slow tick as well.
    // Unchanged display pages cost no I2C traffic.
    static uint32_t lastDisplayRefresh = 0;
    if ((uint32_t)(millis() - lastDisplayRefresh) >= DISPLAY_REFRESH_MS) {
        lastDisplayRefresh = millis();
        updateDisplay();
    }

    // Lost-train timeout expired: evaluate the state machine once more so it
    // can switch the loop off (-> 0) if the train has not reappeared.
    if (trainLost && (uint32_t)(millis() - trainLostAt) >= LOST_TRAIN_TIMEOUT_MS) {
        ESP_LOGW(TAG, "train did not reappear within %u s", LOST_TRAIN_TIMEOUT_MS / 1000);
        trainLost = false;
        calculateKBStatus();
        updateDisplay();
    }

    for (uint8_t p = 0; p < SENSE_PIN_COUNT; p++) {
        uint8_t i = sensePinIndex[p];
        // remove oldest sample from sum
        sensePinSums[p] -= sensePinSamples[p][i];
        // read newest sample
        uint16_t v = analogRead(p);
        // store & add to sum
        sensePinSamples[p][i] = v;
        sensePinSums[p] += v;
        // advance ring index
        sensePinIndex[p] = (i + 1) % ADC_RING_SIZE;

        if ((sensePinSums[p] / ADC_RING_SIZE) > SENSE_THRESHOLD_RAW) {
            sensePinTimeOff[p] = millis() + SENSE_HOLD_MS;
            sensePinStatus[p] = 1;
        } else {
            if (sensePinTimeOff[p] <= millis()) {
                sensePinStatus[p] = 0;
            }
        }

        // TODO: not finished yet — MqttMBM still needs wiring up.
        if (sensePinStatus[p] != sensePinLastStatus[p]) {
            if (p == 0) {
                IntA = (sensePinStatus[p] == 1);
                ESP_LOGI(TAG, "IntA: %s", IntA ? "occupied" : "clear");
            } else if (p == 1) {
                IntM = (sensePinStatus[p] == 1);
                ESP_LOGI(TAG, "IntM: %s", IntM ? "occupied" : "clear");
            } else if (p == 2) {
                IntB = (sensePinStatus[p] == 1);
                ESP_LOGI(TAG, "IntB: %s", IntB ? "occupied" : "clear");
            }
            sensePinLastStatus[p] = sensePinStatus[p];
            calculateKBStatus();
            updateDisplay();
        }
    }
}
