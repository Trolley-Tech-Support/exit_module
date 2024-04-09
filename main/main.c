#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "esp_flash.h"
#include "esp_ghota.h"
#include "nvs_flash.h"
#include "inttypes.h"
#include "rc522.h"
#include "cJSON.h"
#include "soc/mcpwm_periph.h"
#include "esp_crt_bundle.h"
#include "certs.h"
#include "esp_ota_ops.h"
#include "esp_https_ota.h"

#define EXAMPLE_ESP_WIFI_SSID      "VM6193248_2.4GHz"
#define EXAMPLE_ESP_WIFI_PASS      "bpvoj9gvuuTyfmzv"
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2 * 1024

#define GPIO_PWM0A_OUT 13

#define SERVO_MIN_PULSEWIDTH 500
#define SERVO_MAX_PULSEWIDTH 2500 
#define SERVO_MAX_DEGREE 90

#define portTICK_RATE_MS 10

#define DO_BACKGROUND_UPDATE 1
#define DO_FOREGROUND_UPDATE 0
#define DO_MANUAL_CHECK_UPDATE 0

#define FIRMWARE_VERSION "4.2.0"

static const char* TAG = "EXIT_MODULE";
static rc522_handle_t scanner;

static EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

static bool get_payment_status(uint64_t trolley_id) {
    
    bool  payment_status = false;
    int content_length = 0;

    char *output_buffer = malloc(MAX_HTTP_OUTPUT_BUFFER);
    if (NULL == output_buffer) {
        ESP_LOGE(TAG, "can't alloc memory for output buffer");
        return payment_status;
    }

    char url[MAX_HTTP_RECV_BUFFER];
    snprintf(url, sizeof(url), "https://iot.api.pastav.com/item/payment?trolley_id=%lld", trolley_id);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .auth_type = HTTP_AUTH_TYPE_BASIC,
        .client_cert_pem = client_cert_pem,
        .client_key_pem = private_key_pem,
       .transport_type = HTTP_TRANSPORT_OVER_SSL,
       .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }

    content_length = esp_http_client_fetch_headers(client);
    if (content_length < 0) {
        ESP_LOGE(TAG, "HTTP client fetch headers failed");
        free(output_buffer);
        esp_http_client_cleanup(client);
        return false;
    }

    int status = esp_http_client_get_status_code(client);
    if ( status != 200) {
        ESP_LOGI(TAG, "Error in the Request Get Payment Status : %d", status);
        free(output_buffer);
        esp_http_client_cleanup(client);
        return false;
    }

    int data_read = esp_http_client_read_response(client, output_buffer, MAX_HTTP_OUTPUT_BUFFER);
    if (data_read >= 0) {
        //ESP_LOGI(TAG, " GET Response Received: %s", output_buffer);
        cJSON *json = cJSON_Parse(output_buffer);
        if (json != NULL) {
            cJSON *payment_status_json = cJSON_GetObjectItem(json, "payment_status");
            if (cJSON_IsBool(payment_status_json)) {
                payment_status = cJSON_IsTrue(payment_status_json);
            }
            cJSON_Delete(json);
        } else {
            ESP_LOGE(TAG, "Failed to parse JSON response");
        }
    } else {
        ESP_LOGE(TAG, "Failed to read response");
    }

    free(output_buffer);
    esp_http_client_cleanup(client);
    return payment_status;
}

static bool set_payment_status(uint64_t trolley_id) {

    char url[MAX_HTTP_RECV_BUFFER];
    snprintf(url, sizeof(url), "https://iot.api.pastav.com/payment/setfalse?trolley_id=%lld", trolley_id);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .auth_type = HTTP_AUTH_TYPE_BASIC,
        .client_cert_pem = client_cert_pem,
        .client_key_pem = private_key_pem,
       .transport_type = HTTP_TRANSPORT_OVER_SSL,
       .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Payment Status Set to false");
        return true;
    } else {
        ESP_LOGI(TAG, "Error in the Request Set Payment Status");
        for (int i = 0; i < 5; i++) {
            esp_err_t err = esp_http_client_perform(client);
            if(err == ESP_OK){
                ESP_LOGI(TAG, "Payment Status Set to false");
                return true;
            }
        }
        ESP_LOGE(TAG, "Maximum retries exceeded. Unable to set payment status.");
    }
    esp_http_client_cleanup(client);
    return false;
}

static void mcpwm_gpio_initialize(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


void servo_initialize() {
    mcpwm_gpio_initialize();

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
}

void open_gate() {
    uint32_t angle;
    angle = servo_per_degree_init(45);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void close_gate() {
    uint32_t angle;
    angle = servo_per_degree_init(0);
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void rc522_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    rc522_event_data_t* data = (rc522_event_data_t*) event_data;

    switch(event_id) {
        case RC522_EVENT_TAG_SCANNED: {
                rc522_tag_t* tag = (rc522_tag_t*) data->ptr;
                ESP_LOGI(TAG, "Tag scanned (sn: %" PRIu64 ")", tag->serial_number);
                bool payment_status = get_payment_status(tag->serial_number);
                if(payment_status){
                    bool set_status = set_payment_status(tag->serial_number);
                    if(set_status){
                        open_gate();
                        vTaskDelay(2000 / portTICK_RATE_MS);
                        close_gate();
                        vTaskDelay(2000 / portTICK_RATE_MS);
                    }
                } else {
                    ESP_LOGI(TAG, "Check Payment at your end !");
                    ESP_LOGI(TAG, "Payment in not Complete State in Server !");
                }
            }
            break;
    }
}

static esp_err_t validate_image_header(esp_app_desc_t *new_app_info)
{
    if (new_app_info == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "New Firmware Details:");
    ESP_LOGI(TAG, "Project name: %s", new_app_info->project_name);
    ESP_LOGI(TAG, "Firmware version: %s", new_app_info->version);
    ESP_LOGI(TAG, "Compiled time: %s %s", new_app_info->date, new_app_info->time);
    ESP_LOGI(TAG, "ESP-IDF: %s", new_app_info->idf_ver);
    ESP_LOGI(TAG, "SHA256:");
    ESP_LOG_BUFFER_HEX(TAG, new_app_info->app_elf_sha256, sizeof(new_app_info->app_elf_sha256));

    const esp_partition_t *running = esp_ota_get_running_partition();
    ESP_LOGD(TAG, "Current partition %s type %d subtype %d (offset 0x%08" PRIx32 ")",
             running->label, running->type, running->subtype, running->address);
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    ESP_LOGD(TAG, "Update partition %s type %d subtype %d (offset 0x%08" PRIx32 ")",
             update->label, update->type, update->subtype, update->address);

#ifdef CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
    /**
     * Secure version check from firmware image header prevents subsequent download and flash write of
     * entire firmware image. However this is optional because it is also taken care in API
     * esp_https_ota_finish at the end of OTA update procedure.
     */
    const uint32_t hw_sec_version = esp_efuse_read_secure_version();
    if (new_app_info->secure_version < hw_sec_version)
    {
        ESP_LOGW(TAG, "New firmware security version is less than eFuse programmed, %d < %d", new_app_info->secure_version, hw_sec_version);
        return ESP_FAIL;
    }
#endif

    return ESP_OK;
}

static esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client)
{
    esp_err_t err = ESP_OK;
    return err;
}

void advanced_ota_task(void *pvParameter)
{
    while(1) {

        ghota_config_t ghconfig = {
            .filenamematch = "exit_module-esp32.bin",
            .orgname = "Trolley-Tech-Support",
            .reponame = "exit_module",
            /* You should pick something larger than 1 minute practically */
            .updateInterval = 1,
        };

        ghota_client_handle_t *ghota_client = ghota_init(&ghconfig);
        if (ghota_client == NULL) {
            ESP_LOGW(TAG, "ghota_client_init failed");
            return;
        }

        esp_err_t ota_finish_err = ESP_OK;

        esp_http_client_config_t config = {
            .url = "https://iot.api.pastav.com/esp/update",
            .method = HTTP_METHOD_GET,
            .auth_type = HTTP_AUTH_TYPE_BASIC,
            .client_cert_pem = client_cert_pem,
            .client_key_pem = private_key_pem,
            .transport_type = HTTP_TRANSPORT_OVER_SSL,
            .crt_bundle_attach = esp_crt_bundle_attach,
        };

        esp_https_ota_config_t ota_config = {
            .http_config = &config,
            .http_client_init_cb = _http_client_init_cb,
        };

        esp_https_ota_handle_t https_ota_handle = NULL;
        esp_err_t err = esp_https_ota_begin(&ota_config, &https_ota_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ESP HTTPS OTA Begin failed");
            vTaskDelete(NULL);
        }

        esp_app_desc_t app_desc;
        err = esp_https_ota_get_img_desc(https_ota_handle, &app_desc);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "esp_https_ota_get_img_desc failed");
            goto ota_end;
        }

        err = validate_image_header(&app_desc);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "image header verification failed");
            goto ota_end;
        }

        ESP_ERROR_CHECK(ghota_check(ghota_client));
        semver_t *cur = ghota_get_current_version(ghota_client);
        if (cur) {
            ESP_LOGI(TAG, "Current version: %d.%d.%d", cur->major, cur->minor, cur->patch);
            semver_free(cur);
        }
    
        semver_t *new = ghota_get_latest_version(ghota_client);
        if (new) {
            ESP_LOGI(TAG, "New version: %d.%d.%d", new->major, new->minor, new->patch);
            semver_free(new);
        }

        if (semver_gt(*new, *cur) == 1) {
            ESP_LOGI(TAG, "New version is greater than current version");
        } else if (semver_eq(*new, *cur) == 1) {
            ESP_LOGI(TAG, "New version is equal to current version");
        } else {
            ESP_LOGI(TAG, "New version is less than current version");
        }

        if (semver_gt(*new, *cur) == 1) {
            int last_progress = -1;
            while (1)
            {
                err = esp_https_ota_perform(https_ota_handle);
                if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS)
                {
                    break;
                }
                int32_t dl = esp_https_ota_get_image_len_read(https_ota_handle);
                int32_t size = esp_https_ota_get_image_size(https_ota_handle);
                int progress = 100 * ((float)dl / (float)size);
                if ((progress % 5 == 0) && (progress != last_progress))
                {
                    ESP_ERROR_CHECK(esp_event_post(GHOTA_EVENTS, GHOTA_EVENT_FIRMWARE_UPDATE_PROGRESS, &progress, sizeof(progress), portMAX_DELAY));
                    ESP_LOGV(TAG, "Firmware Update Progress: %d%%", progress);
                    last_progress = progress;
                }
            }

            if (esp_https_ota_is_complete_data_received(https_ota_handle) != true) {
                // the OTA image was not completely received and user can customise the response to this situation.
                ESP_LOGW(TAG, "Complete data was not received.");
            } else {
                ota_finish_err = esp_https_ota_finish(https_ota_handle);
                if ((err == ESP_OK) && (ota_finish_err == ESP_OK)) {
                    ESP_LOGI(TAG, "ESP_HTTPS_OTA upgrade successful. Rebooting ...");
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                    esp_restart();
                } else {
                    if (ota_finish_err == ESP_ERR_OTA_VALIDATE_FAILED) {
                        ESP_LOGW(TAG, "Image validation failed, image is corrupted");
                    }
                    ESP_LOGW(TAG, "ESP_HTTPS_OTA upgrade failed 0x%x", ota_finish_err);
                }
            }
        
        }
        ota_end:
            esp_https_ota_abort(https_ota_handle);
            ESP_LOGI(TAG, "ESP_HTTPS_OTA update not done !");
        vTaskDelay(30000 / portTICK_PERIOD_MS);
    }    
    vTaskDelete(NULL);
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(ESP_HTTPS_OTA_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    xTaskCreate(&advanced_ota_task, "advanced_ota_task", 1024 * 8, NULL, 5, NULL);

    servo_initialize();
    rc522_config_t config = {
        .spi.host = VSPI_HOST,
        .spi.miso_gpio = 25,
        .spi.mosi_gpio = 23,
        .spi.sck_gpio = 19,
        .spi.sda_gpio = 22,
    };

    rc522_create(&config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_ANY, rc522_handler, NULL);
    rc522_start(scanner);
}