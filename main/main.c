#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "mdns.h"

// Log tag for debugging
static const char *TAG = "CARD_DEALER";

// html file pointer
extern const uint8_t steering_panel_html_start[] asm("_binary_SteeringPanel_html_start");
extern const uint8_t steering_panel_html_end[]   asm("_binary_SteeringPanel_html_end");

// ULN2003 stepper motor driver pins
#define STEP_IN1      GPIO_NUM_32
#define STEP_IN2      GPIO_NUM_33
#define STEP_IN3      GPIO_NUM_14
#define STEP_IN4      GPIO_NUM_13
#define STEPS_PER_REV 4096

// Speed ramp parameters
#define DELAY_MAX_US  2400
#define DELAY_MIN_US  1200
#define RAMP_STEPS     150

// Wifi access point paramaters
#define WIFI_SSID     "CardDealer-ESP32"
#define WIFI_PASS     "cardAdmin"

// Card dealer states
static volatile int g_current_step_pos = 0;
static volatile bool g_abort_flag = false;
static volatile bool g_is_busy = false;

// Stepper motor sequence
static const uint8_t half_step_seq[8][4] = 
{
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// GPIO pin configuration
static void init_stepper_pins(void) 
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << STEP_IN1) | (1ULL << STEP_IN2) |
                        (1ULL << STEP_IN3) | (1ULL << STEP_IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

// Stepper motor coil state control
static void set_coils(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4) 
{
    gpio_set_level(STEP_IN1, in1);
    gpio_set_level(STEP_IN2, in2);
    gpio_set_level(STEP_IN3, in3);
    gpio_set_level(STEP_IN4, in4);
}

// Stepper motor shutdown
static void stepper_release(void) 
{
    set_coils(0, 0, 0, 0);
}

// Main stepper motor movement function
static void stepper_move_to_target(int target_step) 
{
    // Normalization of target position and delta calculation for required movement
    target_step = ((target_step % STEPS_PER_REV) + STEPS_PER_REV) % STEPS_PER_REV;
    int diff = target_step - g_current_step_pos;

    // Shortest path calculation
    if (diff > (STEPS_PER_REV / 2)) 
    {
        diff -= STEPS_PER_REV;
    } 
    else if (diff < -(STEPS_PER_REV / 2)) 
    {
        diff += STEPS_PER_REV;
    }

    // Return if no movement needed
    int total_steps = abs(diff);
    if (total_steps == 0) 
    {
        return;
    }

    // Movement direction
    int dir = (diff >= 0) ? 1 : -1;

    static int seq_idx = 0;

    // Ramp adaptation for short movements
    int actual_ramp = RAMP_STEPS;
    if (total_steps < 2 * RAMP_STEPS) 
    {
        actual_ramp = total_steps / 2;
    }

    // Movement log
    ESP_LOGI(TAG, "Movement: %d -> %d (%d steps, direction: %s)", g_current_step_pos, target_step, total_steps, dir > 0 ? "CW" : "CCW");

    // Main movement loop
    for (int i = 0; i < total_steps; i++) 
    {
        // Loop interruption check
        if (g_abort_flag) 
        {
            ESP_LOGW(TAG, "Movement interrupted (STOP)!");
            break;
        }

        // Delay calculation for acceleration and deceleration phases (ramp function)
        uint32_t delay_us = DELAY_MIN_US;
        if (actual_ramp > 0) 
        {
            if (i < actual_ramp) 
            {
                // Acceleration
                delay_us = DELAY_MAX_US - ((DELAY_MAX_US - DELAY_MIN_US) * i) / actual_ramp;
            } 
            else if (i >= total_steps - actual_ramp) 
            {
                // Deceleration
                int decel_step = i - (total_steps - actual_ramp);
                delay_us = DELAY_MIN_US + ((DELAY_MAX_US - DELAY_MIN_US) * decel_step) / actual_ramp;
            }
        }

        // Index calculation and coil activation
        seq_idx = (dir > 0) ? (seq_idx + 1) % 8 : (seq_idx - 1 + 8) % 8;
        set_coils
        (
            half_step_seq[seq_idx][0],
            half_step_seq[seq_idx][1],
            half_step_seq[seq_idx][2],
            half_step_seq[seq_idx][3]
        );

        // Current piosition update and delay
        g_current_step_pos = ((g_current_step_pos + dir) % STEPS_PER_REV + STEPS_PER_REV) % STEPS_PER_REV;
        esp_rom_delay_us(delay_us);

        // Every 40 steps 1 tick (10ms) delay movement to allow other tasks to run (prevents watchdog reset)
        if (i % 40 == 0) 
        {
            vTaskDelay(1);
        }
    }

    // Turn off coils after movement
    stepper_release();
}

// Deal job structure definition
typedef struct 
{
    int total_targets;
    bool return_to_zero;
    struct 
    {
        int steps;
        int cards;
    } targets[32];
} deal_job_t;

// Global deal job instance
static deal_job_t g_deal_job;

// Main deal job function with pointer to deal parameters
static void deal_worker_task(void *pvParameters) 
{
    // Card dealer state update
    g_is_busy = true;
    g_abort_flag = false;

    ESP_LOGI(TAG, "=== DEALING SEQUENCE BEGIN (%d targets) ===", g_deal_job.total_targets);

    // Main loop for dealing to each target
    for (int i = 0; i < g_deal_job.total_targets && !g_abort_flag; i++) 
    {
        // Card deal count and validation
        int cards_to_give = g_deal_job.targets[i].cards;
        if (cards_to_give <= 0) 
        {
            continue;
        }

        // Rotation towards target position
        ESP_LOGI(TAG, "Target %d/%d -> rotation to position %d steps", i + 1, g_deal_job.total_targets, g_deal_job.targets[i].steps);
        stepper_move_to_target(g_deal_job.targets[i].steps);

        // Check for STOP command
        if (g_abort_flag) 
        {
            break;
        }

        // Dealing cards to target
        //DC MOTOR STEERING IS NOT IMPLEMENTED, SO THIS IS JUST A SIMULATION OF CARD DEALING
        ESP_LOGI(TAG, "-> Dealing %d cards to target %d...", cards_to_give, i + 1);

        for (int c = 0; c < cards_to_give && !g_abort_flag; c++) 
        {
            ESP_LOGI(TAG, "   [Card %d/%d]", c + 1, cards_to_give);
            // Delay (simulating card dealing time)
            vTaskDelay(pdMS_TO_TICKS(350));
        }

        // Delay before moving to the next target
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    // Return to position 0° if required and not interrupted
    if (g_deal_job.return_to_zero && !g_abort_flag) 
    {
        ESP_LOGI(TAG, "Returning to position 0°...");
        stepper_move_to_target(0);
    }

    // Finalization of the deal job and deletion of current task
    stepper_release();
    ESP_LOGI(TAG, "=== END OF DEALING ===");
    g_is_busy = false;
    vTaskDelete(NULL);
}

// HTTP CORS headers
static void set_cors_headers(httpd_req_t *req) 
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
}

// HTTP OPTIONS handler for CORS preflight requests
static esp_err_t options_handler(httpd_req_t *req) 
{
    set_cors_headers(req);
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// HTTP root path handler
static esp_err_t root_get_handler(httpd_req_t *req) 
{
    const size_t html_size = (steering_panel_html_end - steering_panel_html_start);
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, (const char *)steering_panel_html_start, html_size);
    return ESP_OK;
}

// HTTP handler for position calibration (zeroing)
static esp_err_t api_zero_handler(httpd_req_t *req) 
{
    set_cors_headers(req);
    // Type of response is JSON
    httpd_resp_set_type(req, "application/json");
    g_current_step_pos = 0;
    stepper_release();
    ESP_LOGI(TAG, "Calibrated 0°");
    // Send JSON acknowledgment (position calibration)
    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"current_step\":0}");
    return ESP_OK;
}

// HTTP handler for stop command
static esp_err_t api_stop_handler(httpd_req_t *req) {
    set_cors_headers(req);
    // Type of response is JSON
    httpd_resp_set_type(req, "application/json");
    g_abort_flag = true;
    stepper_release();
    ESP_LOGW(TAG, "STOP");
    // Send JSON acknowledgment (stop command)
    httpd_resp_sendstr(req, "{\"status\":\"stopped\"}");
    return ESP_OK;
}

// HTTP handler for movement command
static esp_err_t api_move_handler(httpd_req_t *req) 
{
    set_cors_headers(req);
    // Type of response is JSON
    httpd_resp_set_type(req, "application/json");

    // Receive JSON data from the request
    char buf[256];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    // Parse JSON and extract target steps
    cJSON *root = cJSON_Parse(buf);
    if (root != NULL) 
    {
        // Extract "target_steps" from JSON and initiate movement if valid
        cJSON *steps_item = cJSON_GetObjectItem(root, "target_steps");
        if (cJSON_IsNumber(steps_item) && !g_is_busy) 
        {
            g_abort_flag = false;
            stepper_move_to_target(steps_item->valueint);
        }
        cJSON_Delete(root);
    }

    // Send JSON acknowledgment (movement command)
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// HTTP handler for dealing command
static esp_err_t api_deal_handler(httpd_req_t *req) 
{
    set_cors_headers(req);
    // Type of response is JSON
    httpd_resp_set_type(req, "application/json");

    if (g_is_busy) 
    {
        // Send JSON acknowledgment (busy state)
        httpd_resp_sendstr(req, "{\"status\":\"busy\"}");
        return ESP_OK;
    }

    // Receive JSON data from the request
    char buf[1536];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) return ESP_FAIL;
    buf[ret] = '\0';

    // Parse JSON and extract deal job parameters
    cJSON *root = cJSON_Parse(buf);
    if (root != NULL) 
    {
        // Extract "return_to_zero" and "targets" from JSON
        cJSON *ret_zero = cJSON_GetObjectItem(root, "return_to_zero");
        cJSON *targets = cJSON_GetObjectItem(root, "targets");

        g_deal_job.return_to_zero = cJSON_IsBool(ret_zero) ? cJSON_IsTrue(ret_zero) : true;
        g_deal_job.total_targets = 0;

        // Validate and extract targets array
        if (cJSON_IsArray(targets)) 
        {
            int count = cJSON_GetArraySize(targets);
            if (count > 32) count = 32;
            g_deal_job.total_targets = count;

            for (int i = 0; i < count; i++) 
            {
                // Extract each target's steps and cards from the JSON array
                cJSON *item = cJSON_GetArrayItem(targets, i);
                cJSON *steps = cJSON_GetObjectItem(item, "steps");
                cJSON *cards = cJSON_GetObjectItem(item, "cards");

                g_deal_job.targets[i].steps = cJSON_IsNumber(steps) ? steps->valueint : 0;
                g_deal_job.targets[i].cards = cJSON_IsNumber(cards) ? cards->valueint : 1;
            }
        }
        cJSON_Delete(root);

        // Start the deal worker task if there are any valid targets left
        if (g_deal_job.total_targets > 0) 
        {
            xTaskCreate(deal_worker_task, "deal_worker", 4096, NULL, 5, NULL);
        }
    }

    // Send JSON acknowledgment (deal command)
    httpd_resp_sendstr(req, "{\"status\":\"started\"}");
    return ESP_OK;
}

// mDNS service initialization and configuration
static void start_mdns_service(void)
{
    // Initialize mDNS service
    esp_err_t err = mdns_init();
    if (err != ESP_OK) 
    {
        ESP_LOGE("MDNS", "Error initializing mDNS: %s", esp_err_to_name(err));
        return;
    }

    // Domain name and instance name configuration
    ESP_ERROR_CHECK(mdns_hostname_set("poker"));
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 Poker Dealer"));

    // mDNS service registration for HTTP on TCP port 80
    ESP_ERROR_CHECK(mdns_service_add("Poker-Server", "_http", "_tcp", 80, NULL, 0));

    ESP_LOGI("MDNS", "mDNS ready! Domain address updated to: http://poker.local");
}

// HTTP server initialization and URI handler registration
static void start_webserver(void) 
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    config.stack_size = 8192;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) 
    {
        // Register URI handlers for root and API endpoints
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_zero = { .uri = "/api/zero", .method = HTTP_POST, .handler = api_zero_handler };
        httpd_uri_t uri_stop = { .uri = "/api/stop", .method = HTTP_POST, .handler = api_stop_handler };
        httpd_uri_t uri_move = { .uri = "/api/move", .method = HTTP_POST, .handler = api_move_handler };
        httpd_uri_t uri_deal = { .uri = "/api/deal", .method = HTTP_POST, .handler = api_deal_handler };

        // Register URI handlers for OPTIONS method to handle CORS preflight requests
        httpd_uri_t uri_opt_zero = { .uri = "/api/zero", .method = HTTP_OPTIONS, .handler = options_handler };
        httpd_uri_t uri_opt_stop = { .uri = "/api/stop", .method = HTTP_OPTIONS, .handler = options_handler };
        httpd_uri_t uri_opt_move = { .uri = "/api/move", .method = HTTP_OPTIONS, .handler = options_handler };
        httpd_uri_t uri_opt_deal = { .uri = "/api/deal", .method = HTTP_OPTIONS, .handler = options_handler };

        // Register all URI handlers with the HTTP server
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_zero);
        httpd_register_uri_handler(server, &uri_stop);
        httpd_register_uri_handler(server, &uri_move);
        httpd_register_uri_handler(server, &uri_deal);
        httpd_register_uri_handler(server, &uri_opt_zero);
        httpd_register_uri_handler(server, &uri_opt_stop);
        httpd_register_uri_handler(server, &uri_opt_move);
        httpd_register_uri_handler(server, &uri_opt_deal);

        ESP_LOGI(TAG, "HTTP server ready on http://192.168.4.1");
    }
}

// Wi-Fi access point initialization and configuration
static void wifi_init_softap(void) 
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Wi-Fi access point configuration
    wifi_config_t wifi_config = 
    {
        .ap = 
        {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .password = WIFI_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi AP ready! SSID: %s", WIFI_SSID);
}

// Main application entry point
void app_main(void) 
{
    // Flash memory check
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Essential initialization (program startup)
    init_stepper_pins();
    stepper_release();

    wifi_init_softap();
    start_mdns_service();
    start_webserver();
}