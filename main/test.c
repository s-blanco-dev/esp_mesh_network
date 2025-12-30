#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "driver/uart.h"
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
#include "dht.h"
#include "soc/gpio_num.h"

/* ---------- CONFIG WIFI / MESH ---------- */
#define CONFIG_MESH_ROUTER_SSID   "uuuu"
#define CONFIG_MESH_ROUTER_PASSWD "ahhh"

#define CONFIG_MESH_AP_PASSWD     "mesh1234"
#define CONFIG_MESH_AP_CONNECTIONS 6

/* ---------- UART PINS ---------- */
#define UART_NUM UART_NUM_1
#define TX_PIN   7
#define RX_PIN   6
#define DHT_GPIO  GPIO_NUM_10
#define DHT_TYPE DHT_TYPE_DHT22

/* ---------- MESH ID ---------- */
static const char *TAG = "mesh_demo";
static const uint8_t MESH_ID[6] = {0x32, 0xAE, 0x11, 0xFF, 0xB5, 0xC0};

/* Tareas para no crear múltiples instancias */
static TaskHandle_t send_task_handle = NULL;
static TaskHandle_t recv_task_handle = NULL;

/* ---------- STRUCT QUE VIAJA EN LA MALLA ---------- */
typedef struct {
    uint8_t parent_mac[6];
    uint8_t self_mac[6];
    float   temp;
    float   humidity;
    int     layer;
} node_packet_t;

/* ---------- HELPER: MAC - 1 (STA del parent) ---------- */
static void mac_decrement(const uint8_t in[6], uint8_t out[6])
{
    memcpy(out, in, 6);
    for (int i = 5; i >= 0; --i) {
        if (out[i] > 0) {
            out[i]--;
            break;
        } else {
            out[i] = 0xFF;
        }
    }
}

/* =========================================================
 *                  TASK: SEND (NO ROOT)
 * ========================================================= */
void send_task(void *arg)
{
    mesh_data_t data;
    node_packet_t pkt;
    mesh_addr_t parent;
    esp_err_t err;

    while (1) {
        if (!esp_mesh_is_root()) {
            /* MAC del AP del parent (bssid) */
            esp_mesh_get_parent_bssid(&parent);

            /* MAC propia STA */
            esp_wifi_get_mac(WIFI_IF_STA, pkt.self_mac);

            /* MAC STA del parent (AP - 1) */
            uint8_t parent_sta_mac[6];
            mac_decrement(parent.addr, parent_sta_mac);
            memcpy(pkt.parent_mac, parent_sta_mac, 6);

            float temp = 0.0f;
            float humidity = 0.0f;

            dht_read_float_data(DHT_TYPE_DHT11, GPIO_NUM_10, &humidity, &temp);

            pkt.layer    = esp_mesh_get_layer();
            pkt.temp     = temp;
            pkt.humidity = humidity;

            data.data = (uint8_t *)&pkt;
            data.size = sizeof(pkt);
            data.proto = MESH_PROTO_BIN;
            data.tos   = MESH_TOS_P2P;

            err = esp_mesh_send(NULL, &data, MESH_DATA_P2P, NULL, 0);

            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Sent packet (layer %d)", pkt.layer);
            } else {
                ESP_LOGW(TAG, "Send failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* =========================================================
 *          TASK: RECEIVE (ROOT, lee de la malla)
 * ========================================================= */
void receive_task(void *arg)
{
    mesh_addr_t from;
    mesh_data_t data;
    data.data = malloc(1500);
    data.size = 1500;

    while (1) {
        int flag = 0;
        esp_err_t err = esp_mesh_recv(&from, &data, portMAX_DELAY, &flag, NULL, 0);

        if (err == ESP_OK && data.size == sizeof(node_packet_t)) {
            node_packet_t *pkt = (node_packet_t *)data.data;

            ESP_LOGI(TAG, "PKT from %02x:%02x:%02x:%02x:%02x:%02x",
                     from.addr[0], from.addr[1], from.addr[2],
                     from.addr[3], from.addr[4], from.addr[5]);

            ESP_LOGI(TAG,
                     "Self: %02x:%02x:%02x:%02x:%02x:%02x | Parent: %02x:%02x:%02x:%02x:%02x:%02x | Layer: %d | Temp: %.1f Hum: %.1f",
                     pkt->self_mac[0], pkt->self_mac[1], pkt->self_mac[2],
                     pkt->self_mac[3], pkt->self_mac[4], pkt->self_mac[5],
                     pkt->parent_mac[0], pkt->parent_mac[1], pkt->parent_mac[2],
                     pkt->parent_mac[3], pkt->parent_mac[4], pkt->parent_mac[5],
                     pkt->layer, pkt->temp, pkt->humidity);

            /* Enviar el struct crudo por UART al otro ESP */
            uart_write_bytes(UART_NUM, (const char *)pkt, sizeof(node_packet_t));
        }
    }
}

void root_send() {
    node_packet_t pkt;
    mesh_addr_t parent;

    while (1) {

    if (esp_mesh_is_root()) {
        esp_mesh_get_parent_bssid(&parent);
        esp_wifi_get_mac(WIFI_IF_STA, pkt.self_mac);
        memcpy(pkt.parent_mac, parent.addr, 6);
        pkt.layer = esp_mesh_get_layer();
        pkt.temp = 20.0;
        pkt.humidity = 80.0;

        ESP_LOGI(TAG, "Sent INITIAL ROOT PACKET (layer %d)", pkt.layer);
        uart_write_bytes(UART_NUM, (const char*)&pkt, sizeof(node_packet_t));
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* =========================================================
 *                 START TASKS SEGÚN ROL
 * ========================================================= */
void startTask(void)
{
    if (esp_mesh_is_root()) {
        if (recv_task_handle == NULL) {
            xTaskCreate(root_send, "receive_task", 4096, NULL, 5, &recv_task_handle);
            xTaskCreate(receive_task, "receive_task", 4096, NULL, 5, &recv_task_handle);
            ESP_LOGI(TAG, "Started receive_task on ROOT");
        } else {
            ESP_LOGI(TAG, "receive_task already running on ROOT");
        }
    } else {
        if (send_task_handle == NULL) {
            xTaskCreate(send_task, "send_task", 4096, NULL, 5, &send_task_handle);
            ESP_LOGI(TAG, "Started send_task on CHILD");
        } else {
            ESP_LOGI(TAG, "send_task already running on CHILD");
        }
    }
}



/* =========================================================
 *                         UART
 * ========================================================= */
static void init_uart()
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM, &cfg);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 2048, 0, 0, NULL, 0);

    ESP_LOGI(TAG, "UART1 ready to transmit to receiver ESP");
}

/* =========================================================
 *                   EVENT HANDLERS
 * ========================================================= */
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
            ESP_LOGI(TAG, "<MESH_EVENT_STARTED>");
            break;

        case MESH_EVENT_STOPPED:
            ESP_LOGI(TAG, "<MESH_EVENT_STOPPED>");
            break;

        case MESH_EVENT_CHILD_CONNECTED:
            ESP_LOGI(TAG, "<MESH_EVENT_CHILD_CONNECTED>");
            /* Por si el root aún no había creado la tarea de receive */
            if (esp_mesh_is_root() && recv_task_handle == NULL) {
                xTaskCreate(receive_task, "receive_task", 4096, NULL, 5, &recv_task_handle);
                ESP_LOGI(TAG, "Created receive_task on CHILD_CONNECTED");
            }
            break;

        case MESH_EVENT_CHILD_DISCONNECTED:
            ESP_LOGI(TAG, "<MESH_EVENT_CHILD_DISCONNECTED>");
            break;

        case MESH_EVENT_PARENT_CONNECTED:
            esp_mesh_get_parent_bssid(&id);
            ESP_LOGI(TAG, "<MESH_EVENT_PARENT_CONNECTED> to %02x:%02x:%02x:%02x:%02x:%02x",
                     id.addr[0], id.addr[1], id.addr[2],
                     id.addr[3], id.addr[4], id.addr[5]);
            if (!esp_mesh_is_root() && send_task_handle == NULL) {
                xTaskCreate(send_task, "send_task", 4096, NULL, 5, &send_task_handle);
                ESP_LOGI(TAG, "Created send_task on PARENT_CONNECTED");
            }
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
            startTask();
            break;

        default:
            ESP_LOGD(TAG, "Unhandled mesh event id: %ld", event_id);
            break;
    }
}

// esp_err_t init_dht() {
//     esp_err_t init = dht_init(DHT_GPIO, DHT_TYPE);
//     if (init == ESP_OK) {
//         printf("DHT sensor initialized successfully.\n");
//     } else {
//         printf("Failed to initialize DHT sensor.\n");
//     }
//     return init;
// }

/* =========================================================
 *                         MAIN
 * ========================================================= */
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

    /* Handlers de eventos */
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(MESH_EVENT, ESP_EVENT_ANY_ID,
                                               &mesh_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi started");

    /* ---------- Init Mesh ---------- */
    ESP_ERROR_CHECK(esp_mesh_init());

    /* Max layer alto para permitir layer 3 */
    ESP_ERROR_CHECK(esp_mesh_set_max_layer(6));
    ESP_ERROR_CHECK(esp_mesh_set_vote_percentage(1));
    ESP_ERROR_CHECK(esp_mesh_set_xon_qsize(128));

    esp_mesh_set_topology(MESH_TOPO_TREE);

    ESP_ERROR_CHECK(esp_mesh_enable_ps());
    ESP_ERROR_CHECK(esp_mesh_set_ap_assoc_expire(60));

    /* ---------- Config Mesh ---------- */
    mesh_cfg_t cfg = MESH_INIT_CONFIG_DEFAULT();

    memcpy((uint8_t *)&cfg.mesh_id, MESH_ID, 6);

    cfg.channel = 0;  // auto: el root se alinea con el canal del router

    cfg.router.ssid_len = strlen(CONFIG_MESH_ROUTER_SSID);
    memcpy((uint8_t *)&cfg.router.ssid, CONFIG_MESH_ROUTER_SSID, cfg.router.ssid_len);
    memcpy((uint8_t *)&cfg.router.password, CONFIG_MESH_ROUTER_PASSWD,
           strlen(CONFIG_MESH_ROUTER_PASSWD));

    cfg.mesh_ap.max_connection = CONFIG_MESH_AP_CONNECTIONS;
    memcpy((uint8_t *)&cfg.mesh_ap.password, CONFIG_MESH_AP_PASSWD,
           strlen(CONFIG_MESH_AP_PASSWD));

    ESP_ERROR_CHECK(esp_mesh_set_config(&cfg));

    ESP_ERROR_CHECK(esp_mesh_start());
    ESP_LOGI(TAG, "Mesh started successfully!");

    ESP_LOGI(TAG, "Waiting for mesh to form...");
    init_uart();
}
