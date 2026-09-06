// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>

#include "kb.h"
#include "display.h"
#include "otter.h"
#include "settings.h"
#include "serial_config.h"

static const char *TAG = "main";

esp_mqtt_client_handle_t mqtt_client = nullptr;

// Mirrored onto the display's network screen; only the handlers below know it.
static bool s_wifi_ok = false;
static bool s_mqtt_ok = false;
static char s_ip_str[16] = "---";

// ---------------------------------------------------------------------------
// MQTT
// ---------------------------------------------------------------------------

static uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// Last values published on MqttOccupied/MqttStatus + "/state"/MqttVoltage,
// so KB_Loop -> mqtt_publish_kb_state() (called once per 10 ms tick) only
// enqueues a message when something actually changed.
static bool     s_last_occupied       = false;
static bool     s_last_occupied_valid = false;
static uint8_t  s_last_status         = 0;
static bool     s_last_status_valid   = false;
static float    s_last_voltage_v      = 0.0f;
static bool     s_last_voltage_valid  = false;
static uint32_t s_last_voltage_publish_ms = 0;

static void mqtt_publish_occupied(bool occupied) {
    esp_mqtt_client_enqueue(mqtt_client, MqttOccupied, occupied ? "1" : "0", 0, 0, 0, true);
}

static void mqtt_publish_voltage(float voltage_v) {
    char payload[16];
    snprintf(payload, sizeof(payload), "%.2f", voltage_v);
    esp_mqtt_client_enqueue(mqtt_client, MqttVoltage, payload, 0, 0, 0, true);
}

static void mqtt_publish_status(uint8_t status) {
    char topic[SETTINGS_TOPIC_LEN + 8];
    snprintf(topic, sizeof(topic), "%s/state", MqttStatus);
    char payload[4];
    snprintf(payload, sizeof(payload), "%u", status);
    esp_mqtt_client_enqueue(mqtt_client, topic, payload, 0, 0, 0, true);
}

// Publishes MqttOccupied, MqttVoltage and MqttStatus/state unconditionally,
// e.g. in response to otter/Refresh or right after (re)connecting.
static void mqtt_republish_all(void) {
    kb_state_t state;
    KB_GetState(&state);

    bool  occupied   = (state.status != 0);
    float voltage_v  = state.voltage_mv / 1000.0f;

    mqtt_publish_occupied(occupied);
    mqtt_publish_voltage(voltage_v);
    mqtt_publish_status(state.status);

    s_last_occupied           = occupied;
    s_last_occupied_valid     = true;
    s_last_status             = state.status;
    s_last_status_valid       = true;
    s_last_voltage_v          = voltage_v;
    s_last_voltage_valid      = true;
    s_last_voltage_publish_ms = millis();
}

// Called once per KB_Loop() tick: publishes MqttOccupied and MqttStatus/state
// whenever they change, and MqttVoltage at most once a second and only if it
// moved by more than 5% since the last publish.
static void mqtt_publish_kb_state(void) {
    kb_state_t state;
    KB_GetState(&state);

    bool occupied = (state.status != 0);
    if (!s_last_occupied_valid || occupied != s_last_occupied) {
        mqtt_publish_occupied(occupied);
        s_last_occupied       = occupied;
        s_last_occupied_valid = true;
    }

    if (!s_last_status_valid || state.status != s_last_status) {
        mqtt_publish_status(state.status);
        s_last_status       = state.status;
        s_last_status_valid = true;
    }

    float voltage_v = state.voltage_mv / 1000.0f;
    if (!s_last_voltage_valid) {
        mqtt_publish_voltage(voltage_v);
        s_last_voltage_v          = voltage_v;
        s_last_voltage_valid      = true;
        s_last_voltage_publish_ms = millis();
    } else if ((uint32_t)(millis() - s_last_voltage_publish_ms) >= 1000) {
        float delta = (voltage_v > s_last_voltage_v) ? (voltage_v - s_last_voltage_v) : (s_last_voltage_v - voltage_v);
        bool  changed = (s_last_voltage_v == 0.0f) ? (voltage_v != 0.0f) : (delta > 0.05f * s_last_voltage_v);
        if (changed) {
            mqtt_publish_voltage(voltage_v);
            s_last_voltage_v          = voltage_v;
            s_last_voltage_publish_ms = millis();
        }
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            s_mqtt_ok = true;
            display_set_network(s_wifi_ok, s_mqtt_ok, s_ip_str);
            esp_mqtt_client_subscribe(mqtt_client, MqttRefresh, 0);
            esp_mqtt_client_subscribe(mqtt_client, MqttSet, 0);
            esp_mqtt_client_subscribe(mqtt_client, MqttExtA, 0);
            esp_mqtt_client_subscribe(mqtt_client, MqttExtB, 0);
            {
                char status_set_topic[SETTINGS_TOPIC_LEN + 8];
                snprintf(status_set_topic, sizeof(status_set_topic), "%s/set", MqttStatus);
                esp_mqtt_client_subscribe(mqtt_client, status_set_topic, 0);
            }
            mqtt_republish_all();  // announce current state to freshly (re)connected subscribers
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            s_mqtt_ok = false;
            display_set_network(s_wifi_ok, s_mqtt_ok, s_ip_str);
            break;

        case MQTT_EVENT_DATA: {
            // esp_mqtt_event_t::topic/data are NOT null-terminated — copy to local buffers
            char topic[event->topic_len + 1];
            char msg[event->data_len + 1];
            memcpy(topic, event->topic, event->topic_len);
            topic[event->topic_len] = '\0';
            memcpy(msg, event->data, event->data_len);
            msg[event->data_len] = '\0';

            ESP_LOGI(TAG, "Message arrived on topic: %s. Message: %s", topic, msg);

            if (strcmp(topic, MqttRefresh) == 0) {
                ESP_LOGI(TAG, "Refresh angefragt");
                mqtt_republish_all();
            }
            if (strcmp(topic, MqttSet) == 0) {
                ESP_LOGI(TAG, "Settings update via MQTT");
                char tx[64];
                settings_result_t result = settings_apply_json(msg, tx, sizeof(tx));

                char ack_topic[SETTINGS_TOPIC_LEN + 5];
                snprintf(ack_topic, sizeof(ack_topic), "%s/ack", MqttSet);

                char ack_payload[160];
                if (result == SETTINGS_OK || result == SETTINGS_OK_REBOOT) {
                    snprintf(ack_payload, sizeof(ack_payload), "{\"tx\":\"%s\"}", tx);
                } else if (result == SETTINGS_ERR_NVS) {
                    ESP_LOGE(TAG, "Settings update failed: NVS write error");
                    snprintf(ack_payload, sizeof(ack_payload), "{\"tx\":\"%s\",\"error\":\"EEPROM write failed\"}", tx);
                } else {
                    ESP_LOGE(TAG, "Settings update failed: JSON parse error");
                    snprintf(ack_payload, sizeof(ack_payload), "{\"tx\":\"%s\",\"error\":\"JSON parse error\"}", tx);
                }
                esp_mqtt_client_enqueue(mqtt_client, ack_topic, ack_payload, 0, 0, 0, true);
                if (result == SETTINGS_OK_REBOOT) {
                    ESP_LOGI(TAG, "Reboot requested via MQTT");
                    xTaskCreate([](void *) {
                        vTaskDelay(pdMS_TO_TICKS(500));
                        esp_restart();
                    }, "reboot", 2048, nullptr, 5, nullptr);
                }
            }
            if (strcmp(topic, MqttExtA) == 0) {
                KB_ExtA(msg);
            }
            if (strcmp(topic, MqttExtB) == 0) {
                KB_ExtB(msg);
            }
            {
                char status_set_topic[SETTINGS_TOPIC_LEN + 8];
                snprintf(status_set_topic, sizeof(status_set_topic), "%s/set", MqttStatus);
                if (strcmp(topic, status_set_topic) == 0) {
                    ESP_LOGI(TAG, "Status forced via MQTT: %s", msg);
                    KB_SetStatus((uint8_t)atoi(msg));
                }
            }
            break;
        }

        default:
            break;
    }
}

static void setup_mqtt() {
    char uri[80];
    snprintf(uri, sizeof(uri), "mqtt://%s", mqtt_server);

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri       = uri;
    mqtt_cfg.credentials.client_id    = client_name;
    mqtt_cfg.buffer.size              = 2048;

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client,
                                   (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, nullptr);
    // MQTT client is started from the IP_EVENT_STA_GOT_IP handler, not here.
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");
        s_wifi_ok = false;
        s_mqtt_ok = false;
        snprintf(s_ip_str, sizeof(s_ip_str), "---");
        display_set_network(s_wifi_ok, s_mqtt_ok, s_ip_str);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&ev->ip_info.ip));
        ESP_LOGI(TAG, "WiFi connected, IP: %s", s_ip_str);
        s_wifi_ok = true;
        display_set_network(s_wifi_ok, s_mqtt_ok, s_ip_str);
        esp_mqtt_client_start(mqtt_client);
    }
}

static void setup_wifi() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    // Static IP — mirrors the original WiFi.config() call
    esp_netif_dhcpc_stop(sta_netif);
    esp_netif_ip_info_t ip_info = {};
    ip_info.ip.addr      = ESP_IP4TOADDR(networkByte1, networkByte2, AbschNummer,   KehrschleifenBaustein);
    ip_info.gw.addr      = ESP_IP4TOADDR(networkByte1, networkByte2, gatewayByte3,  gatewayByte4);
    ip_info.netmask.addr = ESP_IP4TOADDR(255, 255, 0, 0);
    esp_netif_set_ip_info(sta_netif, &ip_info);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        wifi_event_handler, nullptr, nullptr);
    esp_event_handler_instance_register(IP_EVENT,   IP_EVENT_STA_GOT_IP,
                                        wifi_event_handler, nullptr, nullptr);

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid,     ssid,     sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_LOGI(TAG, "Connecting to %s", ssid);
    esp_wifi_start();
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

extern "C" void app_main() {
    // NVS must be initialised before WiFi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Load settings from NVS (falls back to compiled-in defaults if not set)
    settings_load_from_nvs();

    // Initialise the OLED before the module, so its first state is drawn
    display_init();

    // Initialise GPIO for the Kehrschleifen module
    KB_Start();

    setup_mqtt();  // init client before WiFi so the handle exists when IP arrives
    setup_wifi();
    serial_config_start();

    // Main loop — 10 ms tick
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10));
        KB_Loop();
        mqtt_publish_kb_state();
    }
}
