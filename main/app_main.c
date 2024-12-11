
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

#include "dht11.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include <esp_system.h>

static const char *TAG = "ESP_MQTT_CLIENT";

// Global variables and initialization flags
bool Initialize = false;
esp_mqtt_client_handle_t client = NULL;

char topic[64]; // Buffer for MQTT subscription topic
char data[64]; // Buffer for MQTT received data

bool PompOn = false; // State of the water pump
bool initTimer = false; // Timer initialization flag
int64_t execution_time_gardener = 0;
int64_t end_time_gardener = 0;
int64_t start_time_gardener = 0;
int gardener_status = 1; // Status of the gardener process

// Gardener statuses
#define WATERING_PROCEDURE 0
#define WATERING_DONE 1
#define WATERING_ERROR -1

// User configuration variables
bool virtualBtn = false;
bool autoMode = false;
int64_t mqtt_send_interval = 1000000;

// Handles received MQTT messages based on topics
void subs_handler(void) {
    if (strcmp(topic, "VirtualBtn") == 0) {
        virtualBtn = strcmp(data, "true") == 0;
    }
    
    if (strcmp(topic, "AutoMode") == 0) {
        autoMode = strcmp(data, "true") == 0;
    }

    if (strcmp(topic, "TimeInterval") == 0) {
        mqtt_send_interval = atoi(data);
    }

    if (strcmp(topic, "ErrorReset") == 0 && strcmp(data, "true") == 0) {
        gardener_status = WATERING_DONE;
    }

    if (strcmp(topic, "SysReset") == 0 && strcmp(data, "true") == 0) {
        esp_restart(); // Reboot the system
    }
}

// Handles MQTT events
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT disconnected");
        break;

    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "Connecting...");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT Error");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT - received event");
        snprintf(topic, sizeof(topic), "%.*s", event->topic_len, event->topic);
        snprintf(data, sizeof(data), "%.*s", event->data_len, event->data);
        subs_handler();
        break;

    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

// Configures and starts the MQTT client
static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

// Initializes GPIO pins and peripherals
void pins_initialization(void) {
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT); // Water pump control
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_INPUT); // Manual button input
    gpio_set_pull_mode(GPIO_NUM_14, GPIO_PULLDOWN_ONLY);

    // Configure ADC channels for sensors
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);

    DHT11_init(GPIO_NUM_27); // Initialize DHT11 sensor

    ESP_LOGI("SYS", "Initialization complete.");
    Initialize = true;
}

// Gardener function for soil moisture monitoring and watering control
void Gardener(float *Moisture, bool *NeedWater, bool *autoMode, int *WaterVolume) {
    int64_t timeOutUS = 100000000;

    if (*Moisture < 55 && *autoMode && execution_time_gardener < timeOutUS && *WaterVolume > 10 && gardener_status != WATERING_ERROR) {
        if (!PompOn) initTimer = true;

        if (initTimer) {
            start_time_gardener = esp_timer_get_time();
            ESP_LOGW("GARDENER", "Pump is turned on!");
        }

        gpio_set_level(GPIO_NUM_25, true); // Turn on pump
        PompOn = true;
        initTimer = false;

        end_time_gardener = esp_timer_get_time();
        execution_time_gardener = end_time_gardener - start_time_gardener;
        gardener_status = WATERING_PROCEDURE;
    } else if (execution_time_gardener >= timeOutUS || gardener_status == WATERING_ERROR) {
        ESP_LOGE("GARDENER", "Timeout or error: No water or corrupted sensor data.");
        execution_time_gardener = 0;
        gpio_set_level(GPIO_NUM_25, false); // Turn off pump
        *NeedWater = false;
        PompOn = false;
        gardener_status = WATERING_ERROR;
    } else {
        execution_time_gardener = 0;
        gpio_set_level(GPIO_NUM_25, false); // Turn off pump
        *NeedWater = false;
        PompOn = false;
        gardener_status = WATERING_DONE;
    }
}

// Task for reading sensors and controlling the system
void ReadSensor(void *pvParameter) {
    if (!Initialize) pins_initialization();

    // Subscribe to relevant MQTT topics
    esp_mqtt_client_subscribe(client, "VirtualBtn", 0);
    esp_mqtt_client_subscribe(client, "AutoMode", 0);
    esp_mqtt_client_subscribe(client, "TimeInterval", 0);
    esp_mqtt_client_subscribe(client, "ErrorReset", 0);
    esp_mqtt_client_subscribe(client, "SysReset", 0);

    bool NeedWater = false;
    int64_t execution_time_mqtt = 0;
    int64_t start_time_mqtt = esp_timer_get_time();

    while (1) {
        // Read sensor values
        int RawSoilMoisture = adc1_get_raw(ADC1_CHANNEL_4);
        int RawExtTemp = adc1_get_raw(ADC1_CHANNEL_6);
        int RawWaterVolume = adc1_get_raw(ADC1_CHANNEL_5);
        int airHumidity = DHT11_read().humidity;
        int IntTemp = DHT11_read().temperature;
        bool manualBtn = gpio_get_level(GPIO_NUM_14);

        int WaterVolume = 100 * RawWaterVolume / 4096;
        float ExtTemp = 145.0 * (RawExtTemp / 4096.0) - 40.0;
        float PercentMoisture = 100 * (2500.0 - RawSoilMoisture) / (2500 - 815) + 3;

        // Format sensor values as strings
        char PercentMoistureChar[10];
        char airHumidityChar[10];
        char IntTempChar[10];
        char ExtTempChar[10];
        char WaterVolumeChar[10];

        sprintf(PercentMoistureChar, "%.2f", PercentMoisture);
        sprintf(airHumidityChar, "%d", airHumidity);
        sprintf(WaterVolumeChar, "%d", WaterVolume);
        sprintf(IntTempChar, "%d", IntTemp);
        sprintf(ExtTempChar, "%.2f", ExtTemp);

        // Check soil moisture and update water need status
        if (PercentMoisture < 40) NeedWater = true;

        if (NeedWater) Gardener(&PercentMoisture, &NeedWater, &autoMode, &WaterVolume);

        // Manual pump control
        if (manualBtn || virtualBtn) {
            gpio_set_level(GPIO_NUM_25, true);
        } else if (!(manualBtn || virtualBtn) && !PompOn) {
            gpio_set_level(GPIO_NUM_25, false);
        }

        // Send sensor data to MQTT periodically
        int64_t end_time_mqtt = esp_timer_get_time();
        execution_time_mqtt = end_time_mqtt - start_time_mqtt;
        if (execution_time_mqtt >= mqtt_send_interval) {
            esp_mqtt_client_publish(client, "IntTemp", IntTempChar, 0, 0, 0);
            esp_mqtt_client_publish(client, "Humidity", airHumidityChar, 0, 0, 0);
            esp_mqtt_client_publish(client, "ExtTemp", ExtTempChar, 0, 0, 0);
            esp_mqtt_client_publish(client, "Moisture", PercentMoistureChar, 0, 0, 0);
            esp_mqtt_client_publish(client, "WaterLevel", WaterVolumeChar, 0, 0, 0);
            start_time_mqtt = esp_timer_get_time();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Task delay for 100 ms
    }
}

// Main application entry point
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    mqtt_app_start();
    xTaskCreate(&ReadSensor, "Reader loop", 4096, NULL, 5, NULL);
}
