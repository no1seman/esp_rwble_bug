#include <stdint.h>
#include "esp_gap_ble_api.h"

#define ESP_BLE_NAME_LEN 15
#define MAC_ADDR_LEN 6
#define MAC_ADDR_STR_LEN (MAC_ADDR_LEN * 2 + 5 + 1)
#define CONNECTED_BIT BIT0
#define TAG "rwble_bug"

#define WIFI_SSID "*"
#define WIFI_PASSWORD "*"
#define MQTT_CRED "*"
#define MQTT_TOPIC MQTT_CRED "/data"
#define MQTT_HOST "192.168.3.211"

typedef struct advname_s
{
    uint8_t name_length;
    uint8_t field_type;
    char ble_name[ESP_BLE_NAME_LEN];
} __attribute__((packed)) advname_t;

typedef struct eddystone_tlm_s
{
    //head
    uint8_t length1;
    uint8_t type1;
    uint8_t flags1;
    uint8_t length2;
    uint8_t type2;
    uint16_t uuid2;
    uint8_t length3;
    uint8_t type3;
    uint16_t uuid3;
    uint8_t frame_type;
    //vendor
    uint8_t version;
    uint16_t voltage;
    uint16_t temperature;
    uint32_t adv_count;
    uint32_t uptime;
} __attribute__((packed)) eddystone_tlm_t;

typedef struct packet_s
{
    uint8_t ble_adv[ESP_BLE_ADV_DATA_LEN_MAX + ESP_BLE_SCAN_RSP_DATA_LEN_MAX];
    esp_bd_addr_t bda;
    int rssi;
} packet_t;

inline uint16_t swap_uint16(uint16_t val)
{
    return (val << 8) | (val >> 8);
}

inline uint32_t swap_uint32(uint32_t val)
{
    val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0xFF00FF);
    return (val << 16) | (val >> 16);
}

inline int64_t swap_int64(int64_t val)
{
    val = ((val << 8) & 0xFF00FF00FF00FF00ULL) | ((val >> 8) & 0x00FF00FF00FF00FFULL);
    val = ((val << 16) & 0xFFFF0000FFFF0000ULL) | ((val >> 16) & 0x0000FFFF0000FFFFULL);
    return (val << 32) | ((val >> 32) & 0xFFFFFFFFULL);
}

typedef enum
{
    eNotWaitingNotification = 0,
    eWaitingNotification,
    eNotified
} eNotifyValue;

typedef struct
{
    volatile StackType_t *pxTopOfStack;

#if (portUSING_MPU_WRAPPERS == 1)
    xMPU_SETTINGS xMPUSettings;
#endif

    ListItem_t xGenericListItem;
    ListItem_t xEventListItem;
    UBaseType_t uxPriority;
    StackType_t *pxStack;
    char pcTaskName[configMAX_TASK_NAME_LEN];
    BaseType_t xCoreID;

#if (portSTACK_GROWTH > 0 || configENABLE_TASK_SNAPSHOT == 1)
    StackType_t *pxEndOfStack;
#endif

#if (portCRITICAL_NESTING_IN_TCB == 1)
    UBaseType_t uxCriticalNesting;
    uint32_t uxOldInterruptState;
#endif

#if (configUSE_TRACE_FACILITY == 1)
    UBaseType_t uxTCBNumber;
    UBaseType_t uxTaskNumber;
#endif

#if (configUSE_MUTEXES == 1)
    UBaseType_t uxBasePriority;
    UBaseType_t uxMutexesHeld;
#endif

#if (configUSE_APPLICATION_TASK_TAG == 1)
    TaskHookFunction_t pxTaskTag;
#endif

#if (configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0)
    void *pvThreadLocalStoragePointers[configNUM_THREAD_LOCAL_STORAGE_POINTERS];
#if (configTHREAD_LOCAL_STORAGE_DELETE_CALLBACKS)
    TlsDeleteCallbackFunction_t pvThreadLocalStoragePointersDelCallback[configNUM_THREAD_LOCAL_STORAGE_POINTERS];
#endif
#endif

#if (configGENERATE_RUN_TIME_STATS == 1)
    uint32_t ulRunTimeCounter;
#endif

#if (configUSE_NEWLIB_REENTRANT == 1)
    struct _reent xNewLib_reent;
#endif

#if (configUSE_TASK_NOTIFICATIONS == 1)
    volatile uint32_t ulNotifiedValue;
    volatile eNotifyValue eNotifyState;
#endif
#if (tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0)
    uint8_t ucStaticallyAllocated;
#endif
} task_control_block_t;