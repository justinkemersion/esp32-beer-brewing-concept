#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "onewire.h"
#include "ds18b20.h"
#include "bme280.h"
#include "config.h"

// Pin definitions
#define DS18B20_PIN GPIO_NUM_4
#define DOOR_PIN GPIO_NUM_27
#define RELAY_PIN GPIO_NUM_26
#define LED_PIN GPIO_NUM_2
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22

// I2C config
#define I2C_PORT I2C_NUM_0
#define I2C_FREQ_HZ 100000

// Event group bits
#define WIFI_CONNECTED_BIT BIT0
#define MQTT_CONNECTED_BIT BIT1

// Logging tag
static const char *TAG = "BeerMonitor";

// Global handles
static esp_mqtt_client_handle_t mqtt_client;
static bme280_dev_t bme280;
static EventGroupHandle_t connectivity_event_group;
static QueueHandle_t target_temp_queue;

// Function prototypes
void init_hw(void);
void init_wifi(void);
void init_mqtt(void);
float read_ds18b20(void);
void door_isr(void *arg);
void sensor_task(void *pvParameters);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

// Hardware initialization
void init_hw(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << DOOR_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(DOOR_PIN, door_isr, NULL);

    onewire_init(DS18B20_PIN);

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    i2c_param_config(I2C_PORT, &i2c_conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    bme280.params = BME280_DEFAULT_PARAMS;
    bme280.i2c_port = I2C_PORT;
    bme280.i2c_addr = BME280_I2C_ADDRESS_DEFAULT;
    if (bme280_init(&bme280) != ESP_OK) {
        ESP_LOGE(TAG, "BME280 init failed");
    }
}

// Wi-Fi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGW(TAG, "Wi-Fi disconnected, reconnecting...");
                xEventGroupClearBits(connectivity_event_group, WIFI_CONNECTED_BIT);
                esp_wifi_connect();
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(connectivity_event_group, WIFI_CONNECTED_BIT);
    }
}

// Wi-Fi initialization
void init_wifi(void) {
    connectivity_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    xEventGroupWaitBits(connectivity_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

// MQTT event handler
static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            xEventGroupSetBits(connectivity_event_group, MQTT_CONNECTED_BIT);
            esp_mqtt_client_subscribe(mqtt_client, "beer/fridge/set_temp", 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            xEventGroupClearBits(connectivity_event_group, MQTT_CONNECTED_BIT);
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, "beer/fridge/set_temp", event->topic_len) == 0) {
                float new_target;
                if (sscanf(event->data, "%f", &new_target) == 1) {
                    if (new_target >= 5.0 && new_target <= 25.0) { // Reasonable range for fermentation
                        xQueueSend(target_temp_queue, &new_target, portMAX_DELAY);
                        ESP_LOGI(TAG, "Target temp updated to %.2f°C", new_target);
                    } else {
                        ESP_LOGW(TAG, "Invalid target temp: %.2f°C", new_target);
                    }
                }
            }
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;
        default:
            break;
    }
}

// MQTT initialization
void init_mqtt(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER,
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_event_handler_register_with(esp_mqtt_client_get_event_loop(mqtt_client), ESP_EVENT_ANY_ID, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);

    xEventGroupWaitBits(connectivity_event_group, MQTT_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
}

// Read DS18B20 temperature with error handling
float read_ds18b20(void) {
    onewire_reset(DS18B20_PIN);
    onewire_write(DS18B20_PIN, 0xCC);
    onewire_write(DS18B20_PIN, 0x44);
    vTaskDelay(750 / portTICK_PERIOD_MS);
    onewire_reset(DS18B20_PIN);
    onewire_write(DS18B20_PIN, 0xCC);
    onewire_write(DS18B20_PIN, 0xBE);
    uint8_t temp_l = onewire_read(DS18B20_PIN);
    uint8_t temp_h = onewire_read(DS18B20_PIN);
    int16_t raw = (temp_h << 8) | temp_l;
    float temp = raw / 16.0;
    if (temp < -55.0 || temp > 125.0) { // DS18B20 range
        ESP_LOGE(TAG, "DS18B20 read error: %.2f°C", temp);
        return NAN; // Not-a-Number for error
    }
    return temp;
}

// Door interrupt handler
void door_isr(void *arg) {
    int state = gpio_get_level(DOOR_PIN);
    if (xEventGroupGetBits(connectivity_event_group) & MQTT_CONNECTED_BIT) {
        esp_mqtt_client_publish(mqtt_client, "beer/fridge/door", state ? "closed" : "open", 0, 1, 0);
        ESP_LOGI(TAG, "Door: %s", state ? "closed" : "open");
    }
}

// Temperature control (Hysteresis)
void control_temp_hysteresis(float beer_temp, float target_temp) {
    const float hysteresis = 1.0;
    if (beer_temp > target_temp + hysteresis) {
        gpio_set_level(RELAY_PIN, 1);
        ESP_LOGI(TAG, "Cooling ON: Beer Temp %.2f°C", beer_temp);
    } else if (beer_temp < target_temp - hysteresis) {
        gpio_set_level(RELAY_PIN, 0);
        ESP_LOGI(TAG, "Cooling OFF: Beer Temp %.2f°C", beer_temp);
    }
}

// PID control (commented out, for experimentation)
typedef struct {
    float target_temp;
    float last_error;
    float integral;
} pid_state_t;

void control_temp_pid(float beer_temp, pid_state_t *pid, float dt) {
    float error = pid->target_temp - beer_temp;
    pid->integral += error * dt;
    float derivative = (error - pid->last_error) / dt;
    float output = 5.0 * error + 0.1 * pid->integral + 1.0 * derivative;
    pid->last_error = error;

    if (output > 0.5) {
        gpio_set_level(RELAY_PIN, 1);
        ESP_LOGI(TAG, "Cooling ON: Beer Temp %.2f°C, PID Output %.2f", beer_temp, output);
    } else {
        gpio_set_level(RELAY_PIN, 0);
        ESP_LOGI(TAG, "Cooling OFF: Beer Temp %.2f°C, PID Output %.2f", beer_temp, output);
    }
}

// Sensor and control task
void sensor_task(void *pvParameters) {
    float target_temp = *((float *)pvParameters); // Initial target from task creation
    float beer_temp, last_valid_beer_temp = target_temp; // Fallback for errors

    while (1) {
        // Check for target temp updates from MQTT
        float new_target;
        if (xQueueReceive(target_temp_queue, &new_target, 0) == pdTRUE) {
            target_temp = new_target;
            ESP_LOGI(TAG, "Target temp updated in task: %.2f°C", target_temp);
        }

        // Blink LED
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);

        // Read sensors
        beer_temp = read_ds18b20();
        if (isnan(beer_temp)) {
            beer_temp = last_valid_beer_temp; // Use last valid temp on error
        } else {
            last_valid_beer_temp = beer_temp;
        }

        bme280_data_t bme_data;
        if (bme280_read_forced(&bme280, &bme_data) != ESP_OK) {
            ESP_LOGE(TAG, "BME280 read failed");
            continue; // Skip this cycle
        }

        // Publish to MQTT if connected
        if (xEventGroupGetBits(connectivity_event_group) & MQTT_CONNECTED_BIT) {
            char msg[50];
            snprintf(msg, 50, "%.2f", beer_temp);
            esp_mqtt_client_publish(mqtt_client, "beer/fridge/beer_temp", msg, 0, 1, 0);
            snprintf(msg, 50, "%.2f", bme_data.temperature);
            esp_mqtt_client_publish(mqtt_client, "beer/fridge/ambient_temp", msg, 0, 1, 0);
            snprintf(msg, 50, "%.2f", bme_data.humidity);
            esp_mqtt_client_publish(mqtt_client, "beer/fridge/humidity", msg, 0, 1, 0);
        }

        // Control temperature (Hysteresis default)
        control_temp_hysteresis(beer_temp, target_temp);
        // pid_state_t pid = { .target_temp = target_temp, .last_error = 0.0, .integral = 0.0 };
        // control_temp_pid(beer_temp, &pid, 5.0); // Uncomment to test PID

        ESP_LOGI(TAG, "Beer: %.2f°C, Ambient: %.2f°C, Humidity: %.2f%%, Target: %.2f°C", 
                 beer_temp, bme_data.temperature, bme_data.humidity, target_temp);

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

// Main entry point
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Create queue for target temp updates
    target_temp_queue = xQueueCreate(5, sizeof(float));
    if (target_temp_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create target temp queue");
        return;
    }

    ESP_LOGI(TAG, "Starting Lager/IPA Fermentation Monitor...");
    init_hw();
    init_wifi();
    init_mqtt();

    float initial_target_temp = 13.0; // Default 13°C for Lager/IPA hybrid
    xTaskCreate(sensor_task, "sensor_task", 4096, &initial_target_temp, 5, NULL);
}
