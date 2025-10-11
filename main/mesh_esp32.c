#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_mesh.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "portmacro.h"

#define CONFIG_MESH_ROUTER_SSID "WIFI_SSID"
#define CONFIG_MESH_ROUTER_PASSWD "WIFI_PASSWD"

#define CONFIG_MESH_AP_PASSWD "mesh1234"
#define CONFIG_MESH_AP_CONNECTIONS 6

static const char *TAG = "mesh_demo";
static const uint8_t MESH_ID[6] = {0x32, 0xAE, 0x11, 0xFF, 0xB5, 0xC0};


/* ---------------------- STRUCTS ---------------------- */

// esto lo envía cada nodo hijo al root de manera periódica
typedef struct {
    uint8_t parent_mac[6];
    uint8_t self_mac[6];
    int layer;
} node_packet_t;

/* ---------------------- SEND/RECEIVE TASKS ---------------------- */

void send_task() {
    const char* message = "Hello from CHILDREN NODE";

    mesh_data_t data;
    data.data = (uint8_t *)message;
    data.size = strlen(message) + 1;
    data.proto = MESH_PROTO_BIN;
    data.tos = MESH_TOS_P2P;

    esp_err_t err = esp_mesh_send(NULL, &data, MESH_DATA_TODS, NULL, 0);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Message SENT from children node");
    }
    else {
        ESP_LOGE(TAG, "FAILED to send message from children node");
    }
}

void receive_task() {
    mesh_addr_t from;
    mesh_data_t data;

    data.data = malloc(1500);
    data.size = 1500;
    int flag;

    while (true) {
        esp_err_t err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "RECEIVED from %02x:%02x:%02x:%02x:%02x:%02x: %s",
                     from.addr[0], from.addr[1], from.addr[2],
                     from.addr[3], from.addr[4], from.addr[5],
                     (char *)data.data);
        }
        else {
            ESP_LOGE(TAG, "ERROR on receive_task: %d", esp_err_to_name(err));
        }
    }
}

/* ---------------------- EVENT HANDLERS ---------------------- */

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void mesh_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    mesh_addr_t id = {0,};
    static uint8_t last_layer = 0;

    switch (event_id) {
        case MESH_EVENT_STARTED:
            esp_mesh_get_id(&id);
            break;

        case MESH_EVENT_STOPPED:
            ESP_LOGI(TAG, "<MESH_EVENT_STOPPED>");
            break;

        case MESH_EVENT_CHILD_CONNECTED:
            ESP_LOGI(TAG, "<MESH_EVENT_CHILD_CONNECTED>");
            receive_task();
            break;

        case MESH_EVENT_CHILD_DISCONNECTED:
            ESP_LOGI(TAG, "<MESH_EVENT_CHILD_DISCONNECTED>");
            break;

        case MESH_EVENT_PARENT_CONNECTED:
            esp_mesh_get_parent_bssid(&id);
            // send_task();
            ESP_LOGI(TAG, "<MESH_EVENT_PARENT_CONNECTED> to %02x:%02x:%02x:%02x:%02x:%02x",
                     id.addr[0], id.addr[1], id.addr[2], id.addr[3], id.addr[4], id.addr[5]);
            break;

        case MESH_EVENT_PARENT_DISCONNECTED:
            ESP_LOGW(TAG, "<MESH_EVENT_PARENT_DISCONNECTED>");
            break;

        case MESH_EVENT_LAYER_CHANGE:
            {
                mesh_event_layer_change_t *layer_change = (mesh_event_layer_change_t *)event_data;
                if (last_layer != layer_change->new_layer) {
                    last_layer = layer_change->new_layer;
                    ESP_LOGI(TAG, "<MESH_EVENT_LAYER_CHANGE> Layer %d", last_layer);
                }
            }
            break;

        case MESH_EVENT_ROOT_ADDRESS:
            ESP_LOGI(TAG, "<MESH_EVENT_ROOT_ADDRESS> root assigned");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled mesh event id: %ld", event_id);
            break;
    }
}



/* ---------------------- MAIN ---------------------- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Initializing Wi-Fi and Mesh...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID, &mesh_event_handler, NULL));

    /* Start Wi-Fi */
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi started");

    /* Initialize Mesh */
    ESP_ERROR_CHECK(esp_mesh_init());
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));
    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(10));

    /* Mesh configuration */
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();
    memcpy((uint8_t *)&cfg.mesh_id, MESH_ID, 6);
    cfg.channel = 0;  // 0 = auto
    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *)&cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *)&cfg.router.password, CONFIG_MESH_ROUTER_PASSWD, strlen(CONFIG_MESH_ROUTER_PASSWD));
    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *)&cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD, strlen(CONFIG_MESH_AP_PASSWD));

    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(TAG, "Mesh started successfully!");

    ESP_LOGI(TAG, "Waiting for mesh to form...");
}

