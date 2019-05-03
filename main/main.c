#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt.h"
#include "cJSON.h"
#include "esp_http_server.h"
#include "esp_https_server.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "soc/rtc_wdt.h"
#include "mqtt_client.h"
#include "main.h"

static EventGroupHandle_t wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client;
static esp_mqtt_client_config_t mqtt_cfg;
static httpd_handle_t http_server;
QueueHandle_t xQueue;

esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0xC8,
    .scan_window = 0x32,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE};

esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x64,
    .adv_int_max = 0x64,
    .adv_type = ADV_TYPE_SCAN_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static advname_t advname = {
    .name_length = 8,
    .field_type = 0x09,
    .ble_name = "bletest"};

static eddystone_tlm_t eddystone_tlm = {
    .length1 = 0x02,
    .type1 = 0x01,
    .flags1 = 0x06,
    .length2 = 0x03,
    .type2 = 0x03,
    .uuid2 = 0xFEAA,
    .length3 = 0x11,
    .type3 = 0x16,
    .uuid3 = 0xFEAA,
    .frame_type = 0x20,
    .version = 0};

uint32_t eddystone_tlm_counter = 0;
uint8_t temprature_sens_read();
void httpserver_start(void);

inline char atoc(uint8_t value)
{
    if (value < 10)
    {
        return (char)(value + 48);
    }
    else
    {
        return (char)(value + 55);
    }
}

char *array2hex(const uint8_t *hex, char *target, uint8_t len)
{
    int i, hop = 0;
    for (i = 0; i < len; i++)
    {
        target[hop] = atoc((hex[i] & 0xf0) >> 4);
        target[hop + 1] = atoc((hex[i] & 0x0f));
        hop += 2;
    }
    target[hop] = '\0';
    return (target);
}

char *mac2str(const uint8_t *mac, char *name)
{
    int i, hop = 0;
    for (i = 0; i < 5; i++)
    {
        name[hop] = atoc((((uint8_t)mac[i] & 0xf0) >> 4));
        name[hop + 1] = atoc(((uint8_t)mac[i] & 0x0f));
        name[hop + 2] = 0x3a;
        hop += 3;
    }
    name[hop] = atoc(((uint8_t)mac[5] & 0xf0) >> 4);
    name[hop + 1] = atoc(((uint8_t)mac[5] & 0x0f));
    name[hop + 2] = '\0';
    return (name);
}

void set_adv_eddystone_tlm_data(void)
{
    eddystone_tlm.voltage = swap_uint16((uint16_t)(4 * 1000));
    eddystone_tlm.temperature = (uint16_t)(((temprature_sens_read() - 32) / 1.8));

    eddystone_tlm_counter++;
    eddystone_tlm.adv_count = swap_uint32(eddystone_tlm_counter);
    uint32_t time_since_start = (uint32_t)(esp_timer_get_time() / 100000);
    eddystone_tlm.uptime = swap_uint32(time_since_start);

    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw((uint8_t *)&eddystone_tlm, sizeof(eddystone_tlm_t)));
}

void set_adv_scan_rsp_data(void)
{
    ESP_ERROR_CHECK(esp_ble_gap_config_scan_rsp_data_raw((uint8_t *)&advname, strlen(advname.ble_name) + 2));
}

void etlm_print(eddystone_tlm_t *eddystone_tlm, esp_bd_addr_t *device_mac, int8_t rssi, char *name)
{
    char mac_buf[MAC_ADDR_STR_LEN];
    uint16_t temp = swap_uint16(eddystone_tlm->temperature);
    float temperature = (float)(int8_t)((temp >> 8) & 0xff) + (temp & 0xff) / 256.0;
    ESP_LOGI(TAG, "Eddystone TLM: {MAC: %s, Name: %s, RSSI: %d dBm, Version: %d, Battery: %d mV, Temperature: %3.1f, Count: %d, Uptime: %9.1f s}",
             mac2str((uint8_t *)device_mac, (char *)&mac_buf),
             name,
             rssi,
             eddystone_tlm->version,
             swap_uint16(eddystone_tlm->voltage),
             temperature,
             swap_uint32(eddystone_tlm->adv_count),
             (float)(swap_uint32(eddystone_tlm->uptime) / 10));
}

bool is_eddystone_tlm_packet(uint8_t *adv_data, uint8_t adv_data_len)
{
    if ((adv_data != NULL) && (adv_data_len == 0x19))
    {
        if (!memcmp(adv_data, (uint8_t *)&eddystone_tlm, 12))
        {
            return true;
        }
    }
    return false;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
    {
        ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&ble_adv_params));
        break;
    }
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
    {
        set_adv_eddystone_tlm_data();
        break;
    }
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        ESP_ERROR_CHECK(esp_ble_gap_start_scanning(0));
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    {
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "BLE scan start failed: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "Scan started successfully");
        }
        break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Broadcast start failed: %s", esp_err_to_name(err));
            break;
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            //if Eddystone TLM
            if (is_eddystone_tlm_packet(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len))
            {
                char name[29];
                packet_t packet;
                memcpy(&name, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len + 2], (size_t)(scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len] - 1));
                name[(size_t)(scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len] - 1)] = '\0';
                etlm_print(
                    (eddystone_tlm_t *)(scan_result->scan_rst.ble_adv),
                    (esp_bd_addr_t *)&scan_result->scan_rst.bda,
                    (uint8_t)scan_result->scan_rst.rssi,
                    name);
                packet.rssi = (uint8_t)scan_result->scan_rst.rssi;
                memcpy(&packet.bda, (esp_bd_addr_t *)&scan_result->scan_rst.bda, sizeof(esp_bd_addr_t));
                memcpy(&packet.ble_adv, scan_result->scan_rst.ble_adv, ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX);
                xQueueSend(xQueue, (void *)&packet, sizeof(packet_t));
            }
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Scan stop failed: %s", esp_err_to_name(err));
        }

        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Broadcast stop failed: %s", esp_err_to_name(err));
        }
        set_adv_eddystone_tlm_data();
        break;
    default:
        break;
    }
}

static void init_ble_gap()
{
    ESP_LOGI(TAG, "Setting up BLE GAP ...");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_err_t status;
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(TAG, "BLE GAP callback register error: %s", esp_err_to_name(status));
        return;
    }
}

static void init_nvs()
{
    ESP_LOGI(TAG, "Initialize ESP32 NVS...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void send_task(void *pvParameters)
{
    packet_t packet;
    size_t count = 0;
    int i = 0;
    int msg_id;
    if (xQueue != 0)
    {
        while (1)
        {
            count = uxQueueMessagesWaiting(xQueue);
            for (i = 0; i < count; i++)
            {
                if (xQueueReceive(xQueue, &packet, (TickType_t)0))
                {
                    msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, (char *)&packet, sizeof(packet_t), 1, 0);
                    if (msg_id == 0)
                    {
                        ESP_LOGI(TAG, "MQTT msg_id=%d send error", msg_id);
                    }
                }
            }
            vTaskDelay(250 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        esp_mqtt_client_start(mqtt_client);
        httpserver_start();
        xTaskCreatePinnedToCore(send_task, "MQTT_SEND", 4096, NULL, 5, NULL, 1);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

typedef struct
{
    multi_heap_info_t internal_ram;
    multi_heap_info_t external_ram;
    multi_heap_info_t total_ram;
} free_mem_t;

char *freemem_to_json(void)
{
    cJSON *root, *internal, *external;

    free_mem_t *free_mem = pvPortMalloc(sizeof(free_mem_t));

    memset(free_mem, 0, sizeof(free_mem_t));
    heap_caps_get_info(&free_mem->internal_ram, MALLOC_CAP_INTERNAL);
    heap_caps_get_info(&free_mem->external_ram, MALLOC_CAP_SPIRAM);
    heap_caps_get_info(&free_mem->total_ram, MALLOC_CAP_DEFAULT);

    root = cJSON_CreateObject();
    cJSON_AddItemToObject(root, "Internal_RAM", internal = cJSON_CreateObject());
    cJSON_AddItemToObject(internal, "Total", cJSON_CreateNumber((float)(free_mem->internal_ram.total_free_bytes + free_mem->internal_ram.total_allocated_bytes)));
    cJSON_AddItemToObject(internal, "Free", cJSON_CreateNumber((float)free_mem->internal_ram.total_free_bytes));
    cJSON_AddItemToObject(internal, "Used", cJSON_CreateNumber((float)free_mem->internal_ram.total_allocated_bytes));
    cJSON_AddItemToObject(internal, "MinimumFree", cJSON_CreateNumber((float)free_mem->internal_ram.minimum_free_bytes));
    cJSON_AddItemToObject(internal, "LargestFreeBlock", cJSON_CreateNumber((float)free_mem->internal_ram.largest_free_block));

    cJSON_AddItemToObject(root, "External_RAM", external = cJSON_CreateObject());
    cJSON_AddItemToObject(external, "Total", cJSON_CreateNumber((float)(free_mem->external_ram.total_free_bytes + free_mem->external_ram.total_allocated_bytes)));
    cJSON_AddItemToObject(external, "Free", cJSON_CreateNumber((float)free_mem->external_ram.total_free_bytes));
    cJSON_AddItemToObject(external, "Used", cJSON_CreateNumber((float)free_mem->external_ram.total_allocated_bytes));
    cJSON_AddItemToObject(external, "MinimumFree", cJSON_CreateNumber((float)free_mem->external_ram.minimum_free_bytes));
    cJSON_AddItemToObject(external, "LargestFreeBlock", cJSON_CreateNumber((float)free_mem->external_ram.largest_free_block));

    char *freemem = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (free_mem)
    {
        vPortFree(free_mem);
        free_mem = NULL;
    }
    return freemem;
}

char *ps_to_json(void)
{
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    uint32_t ulTotalRunTime = 0;
    uint32_t ulStatsAsPercentage = 0;
    task_control_block_t *TCB = NULL;
    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    if (pxTaskStatusArray != NULL)
    {
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        cJSON *root = cJSON_CreateObject();
        cJSON *processes = cJSON_CreateArray();

        for (x = 0; x < uxArraySize; x++)
        {
            TCB = (task_control_block_t *)pxTaskStatusArray[x].xHandle;
            UBaseType_t stack_size = (UBaseType_t)(TCB->pxEndOfStack) - (UBaseType_t)(TCB->pxStack) + 4;
            UBaseType_t stack_used = (UBaseType_t)(TCB->pxEndOfStack) - (UBaseType_t)(TCB->pxTopOfStack) + 4;
            ulStatsAsPercentage = (pxTaskStatusArray[x].ulRunTimeCounter * 100) / (ulTotalRunTime * portNUM_PROCESSORS);
            cJSON *task = cJSON_CreateObject();
            cJSON_AddItemToObject(task, "TaskName", cJSON_CreateString(pxTaskStatusArray[x].pcTaskName));
            cJSON_AddItemToObject(task, "TaskNumber", cJSON_CreateNumber(pxTaskStatusArray[x].xTaskNumber));
            cJSON_AddItemToObject(task, "CurrentState", cJSON_CreateNumber(pxTaskStatusArray[x].eCurrentState));
            cJSON_AddItemToObject(task, "RunTimeCounter", cJSON_CreateNumber(pxTaskStatusArray[x].ulRunTimeCounter));

            if (pxTaskStatusArray[x].xCoreID != 0)
            {
                pxTaskStatusArray[x].xCoreID = 1;
            }
            cJSON_AddItemToObject(task, "CoreID", cJSON_CreateNumber(pxTaskStatusArray[x].xCoreID));
            cJSON_AddItemToObject(task, "BasePriority", cJSON_CreateNumber(pxTaskStatusArray[x].uxBasePriority));
            cJSON_AddItemToObject(task, "CurrentPriority", cJSON_CreateNumber(pxTaskStatusArray[x].uxCurrentPriority));
            cJSON_AddItemToObject(task, "StackSize", cJSON_CreateNumber(stack_size));
            cJSON_AddItemToObject(task, "StackUsed", cJSON_CreateNumber(stack_used));
            cJSON_AddItemToObject(task, "HighWaterMark", cJSON_CreateNumber(stack_size - pxTaskStatusArray[x].usStackHighWaterMark));
            cJSON_AddItemToObject(task, "CPUPercentage", cJSON_CreateNumber(ulStatsAsPercentage));
            cJSON_AddItemToObject(processes, "", task);
        }
        cJSON_AddItemToObject(root, "TotalRunTimeCounter", cJSON_CreateNumber(ulTotalRunTime));
        cJSON_AddItemToObject(root, "Processes", processes);
        char *ps = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);

        if (pxTaskStatusArray)
        {
            vPortFree(pxTaskStatusArray);
            pxTaskStatusArray = NULL;
        }
        return ps;
    }

    if (pxTaskStatusArray)
    {
        vPortFree(pxTaskStatusArray);
        pxTaskStatusArray = NULL;
    }

    return NULL;
}

esp_err_t getstats_handler(httpd_req_t *req)
{
    static char *stats = NULL;
    stats = freemem_to_json();

    if (stats)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_status(req, HTTPD_200);
        esp_err_t res = httpd_resp_send(req, stats, strlen(stats));
        vPortFree(stats);
        stats = NULL;
        return res;
    }
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

esp_err_t ps_handler(httpd_req_t *req)
{
    static char *ps = NULL;
    ps = ps_to_json();

    if (ps)
    {
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_status(req, HTTPD_200);
        esp_err_t res = httpd_resp_send(req, ps, strlen(ps));
        vPortFree(ps);
        ps = NULL;
        return res;
    }
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

const httpd_uri_t _uri_getstats = {
    .uri = "/getstats",
    .method = HTTP_GET,
    .handler = getstats_handler,
    .user_ctx = NULL};

const httpd_uri_t _uri_ps = {
    .uri = "/ps",
    .method = HTTP_GET,
    .handler = ps_handler,
    .user_ctx = NULL};


void httpserver_start(void)
{
    httpd_ssl_config_t https_srv_config = HTTPD_SSL_CONFIG_DEFAULT();
    https_srv_config.httpd.max_open_sockets = 12;
    https_srv_config.httpd.max_uri_handlers = 16;
    https_srv_config.httpd.stack_size = 4096;
    https_srv_config.transport_mode = HTTPD_SSL_TRANSPORT_INSECURE;
    https_srv_config.port_insecure = 80;
    https_srv_config.httpd.uri_match_fn = httpd_uri_match_wildcard;
    https_srv_config.httpd.core_id = 1;
    https_srv_config.httpd.task_priority = 6;
    httpd_ssl_start(&http_server, &https_srv_config);
    httpd_register_uri_handler(http_server, &_uri_getstats);
    httpd_register_uri_handler(http_server, &_uri_ps);
}

static void init_wifi(void)
{
    ESP_LOGI(TAG, "Initialize WiFi...");
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD}};

    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "rwble_bug");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi init finished!");
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT client connected");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT client disconnected");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT message published: msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
        break;
    }
    return ESP_OK;
}

void init_mqtt(void)
{
    mqtt_cfg.event_handle = mqtt_event_handler;
    mqtt_cfg.disable_clean_session = true;
    mqtt_cfg.disable_auto_reconnect = false;
    mqtt_cfg.keepalive = 10;
    mqtt_cfg.buffer_size = 1024;
    mqtt_cfg.username = MQTT_CRED;
    mqtt_cfg.password = MQTT_CRED;
    mqtt_cfg.client_id = MQTT_CRED;
    mqtt_cfg.host = MQTT_HOST;
    mqtt_cfg.port = 1883;
    mqtt_cfg.transport = MQTT_TRANSPORT_OVER_TCP,
    mqtt_cfg.task_stack = 4096;
    mqtt_cfg.task_prio = 5;
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    xQueue = xQueueCreate(100, 64);
    rtc_wdt_disable();
    init_nvs();
    init_mqtt();
    init_wifi();

    init_ble_gap();
    esp_ble_gap_set_scan_params(&ble_scan_params);
    set_adv_scan_rsp_data();

    vTaskDelay(250 / portTICK_PERIOD_MS);
    while (1)
    {
        esp_ble_gap_stop_advertising();
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}