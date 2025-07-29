#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cJSON.h"

#include "esp_log.h"
#include "esp_https_server.h"
#include "esp_http_server.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

// Define missing error code for WebSocket client disconnect
#define ESP_ERR_HTTPD_WS_CLIENT_DISCONNECTED ESP_ERR_INVALID_STATE

static const char *TAG = "wss_server";
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

extern const uint8_t cert_start[] asm("_binary_cert_pem_start");
extern const uint8_t cert_end[]   asm("_binary_cert_pem_end");
extern const uint8_t key_start[]  asm("_binary_key_pem_start");
extern const uint8_t key_end[]    asm("_binary_key_pem_end");

#define LEDC_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL_LED    LEDC_CHANNEL_0
#define LEDC_CHANNEL_MOTOR1 LEDC_CHANNEL_1
#define LEDC_CHANNEL_MOTOR2 LEDC_CHANNEL_2
#define LEDC_TIMER          LEDC_TIMER_0

#define LED_GPIO            2
#define MOTOR1_PWM_GPIO     13
#define MOTOR1_IN4_GPIO     12
#define MOTOR2_PWM_GPIO     26
#define MOTOR2_IN4_GPIO     27

// Переменные для хранения значений яркости/скорости
static int led_bri = 0, motor1_bri = 0, motor2_bri = 0;
static int buf = 0; // Буфер для числового сообщения
static bool client_connected = false, new_message_received = false;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ip_event_got_ip_t* evt = event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Disconnected. Reconnecting...");
    }
}

void wifi_init(void) {
    wifi_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "WILHELM.TEL-2Z8VV9N3TN",
            .password = "45273402810274282608",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.capable = true, .required = false},
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

void pwm_init(void) {
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channels[] = {
        { .gpio_num = LED_GPIO,        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_LED,
          .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR1_PWM_GPIO, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_MOTOR1,
          .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .duty = 0, .hpoint = 0 },
        { .gpio_num = MOTOR2_PWM_GPIO, .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_MOTOR2,
          .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .duty = 0, .hpoint = 0 },
    };
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }

    gpio_reset_pin(MOTOR1_IN4_GPIO);
    gpio_set_direction(MOTOR1_IN4_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR1_IN4_GPIO, 0);
    gpio_reset_pin(MOTOR2_IN4_GPIO);
    gpio_set_direction(MOTOR2_IN4_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR2_IN4_GPIO, 0);
}

esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        client_connected = true;
        ESP_LOGI(TAG, "WS handshake OK");
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {0};
    uint8_t buf_ws[128] = {0};
    ws_pkt.payload = buf_ws;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, sizeof(buf_ws));
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "WS recv_frame error: %s", esp_err_to_name(ret));
        client_connected = false;
        led_bri = motor1_bri = motor2_bri = 0;
        ESP_LOGI(TAG, "WS client disconnected or error");
        return ESP_OK;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.len > 0) {
        buf_ws[ws_pkt.len] = '\0'; // Завершаем строку
        cJSON *json = cJSON_Parse((char*)buf_ws);
        if (json != NULL) {
            // Проверяем, является ли сообщение JSON-объектом
            if (cJSON_IsObject(json)) {
                cJSON *led = cJSON_GetObjectItem(json, "led");
                cJSON *motor1 = cJSON_GetObjectItem(json, "motor1");
                cJSON *motor2 = cJSON_GetObjectItem(json, "motor2");

                // Обработка значений, если они присутствуют и являются числами
                if (led && cJSON_IsNumber(led)) {
                    int val = led->valueint;
                    if (val >= 0 && val <= 255) {
                        led_bri = val;
                        new_message_received = true;
                        ESP_LOGI(TAG, "JSON: led = %d", val);
                    } else {
                        ESP_LOGW(TAG, "JSON: Invalid led value: %d", val);
                    }
                }
                if (motor1 && cJSON_IsNumber(motor1)) {
                    int val = motor1->valueint;
                    if (val >= 0 && val <= 255) {
                        motor1_bri = val;
                        new_message_received = true;
                        ESP_LOGI(TAG, "JSON: motor1 = %d", val);
                    } else {
                        ESP_LOGW(TAG, "JSON: Invalid motor1 value: %d", val);
                    }
                }
                if (motor2 && cJSON_IsNumber(motor2)) {
                    int val = motor2->valueint;
                    if (val >= 0 && val <= 255) {
                        motor2_bri = val;
                        new_message_received = true;
                        ESP_LOGI(TAG, "JSON: motor2 = %d", val);
                    } else {
                        ESP_LOGW(TAG, "JSON: Invalid motor2 value: %d", val);
                    }
                }
            } else {
                // Для обратной совместимости: пробуем интерпретировать как строку с числом
                char *endptr;
                long val = strtol((char*)ws_pkt.payload, &endptr, 10);
                if (*endptr == '\0' && val >= 0 && val <= 255) {
                    buf = (int)val;
                    new_message_received = true;
                    ESP_LOGI(TAG, "Valid numeric payload: %ld", val);
                } else {
                    ESP_LOGW(TAG, "Bad payload: %.*s", ws_pkt.len, ws_pkt.payload);
                }
            }
            cJSON_Delete(json);
        } else {
            // Если не JSON, пробуем как строку с числом
            char *endptr;
            long val = strtol((char*)ws_pkt.payload, &endptr, 10);
            if (*endptr == '\0' && val >= 0 && val <= 255) {
                buf = (int)val;
                new_message_received = true;
                ESP_LOGI(TAG, "Valid numeric payload: %ld", val);
            } else {
                ESP_LOGW(TAG, "Bad payload: %.*s", ws_pkt.len, ws_pkt.payload);
            }
        }
    } else if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
        ws_pkt.type = HTTPD_WS_TYPE_PONG;
        httpd_ws_send_frame_async(req->handle, httpd_req_to_sockfd(req), &ws_pkt);
    } else if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        client_connected = false;
        led_bri = motor1_bri = motor2_bri = 0;
        ESP_LOGI(TAG, "WS client sent CLOSE frame");
    }

    return ESP_OK;
}

httpd_handle_t start_wss_server(void) {
    size_t cert_len = cert_end - cert_start;
    size_t key_len = key_end - key_start;
    char *cert_buf = calloc(1, cert_len + 1);
    char *key_buf  = calloc(1, key_len + 1);
    memcpy(cert_buf, cert_start, cert_len);
    memcpy(key_buf , key_start , key_len );

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();
    conf.servercert = (const uint8_t*)cert_buf;
    conf.servercert_len = cert_len + 1;
    conf.prvtkey_pem   = (const uint8_t*)key_buf;
    conf.prvtkey_len   = key_len + 1;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.max_resp_headers = 16;
    config.max_open_sockets = 1;

    httpd_handle_t server = NULL;
    if (httpd_ssl_start(&server, &conf) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTPS server");
        free(cert_buf);
        free(key_buf);
        return NULL;
    }

    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ws_uri));
    ESP_LOGI(TAG, "WSS server running at /ws");

    return server;
}

void control_task(void *pv) {
    while (1) {
        if (new_message_received && buf != 0) {
            // Если пришло числовое сообщение, применяем его ко всем устройствам
            // Только если новое значение больше текущего
            if (buf > led_bri) {
                led_bri = buf;
                ESP_LOGI(TAG, "Updated led_bri to %d (numeric)", led_bri);
            }
            if (buf > motor1_bri) {
                motor1_bri = buf;
                ESP_LOGI(TAG, "Updated motor1_bri to %d (numeric)", motor1_bri);
            }
            if (buf > motor2_bri) {
                motor2_bri = buf;
                ESP_LOGI(TAG, "Updated motor2_bri to %d (numeric)", motor2_bri);
            }
            buf = 0; // Сбрасываем буфер
            new_message_received = false;
        } else if (client_connected) {
            // Плавное затухание, если клиент подключен, но новых сообщений нет
            int temp_led = led_bri - led_bri * 0.03f;
            int temp_motor1 = motor1_bri - motor1_bri * 0.03f;
            int temp_motor2 = motor2_bri - motor2_bri * 0.03f;

            // Принудительно устанавливаем 0, если значение становится меньше 1
            led_bri = temp_led < 1.0f ? 0 : roundf(temp_led);
            motor1_bri = temp_motor1 < 1.0f ? 0 : roundf(temp_motor1);
            motor2_bri = temp_motor2 < 1.0f ? 0 : roundf(temp_motor2);
        } else {
            // Сбрасываем все значения, если клиент отключен
            led_bri = motor1_bri = motor2_bri = 0;
        }

        // Применяем значения к каналам PWM
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_LED, led_bri));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_LED));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR1, motor1_bri));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR1));
        gpio_set_level(MOTOR1_IN4_GPIO, 0);

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR2, motor2_bri));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_MOTOR2));
        gpio_set_level(MOTOR2_IN4_GPIO, 0);

        vTaskDelay(pdMS_TO_TICKS

(10));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    pwm_init();
    wifi_init();

    ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);

    httpd_handle_t server = start_wss_server();
    if (!server) {
        ESP_LOGE(TAG, "Server startup failed");
        return;
    }

    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);
}