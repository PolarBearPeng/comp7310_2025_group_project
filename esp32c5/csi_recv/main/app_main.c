/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/**
 * In this file, the following code blocks are marked for customization.
 * Each block starts with the comment: "// YOUR CODE HERE" 
 * and ends with: "// END OF YOUR CODE".
 *
 * [1] Modify the CSI Buffer and FIFO Lengths:
 *     - Adjust the buffer configuration based on your system if necessary.
 *
 * [2] Implement Algorithms:
 *     - Develop algorithms for motion detection, breathing rate estimation, and MQTT message sending.
 *     - Implement them in their respective functions.
 *
 * [3] Modify Wi-Fi Configuration:
 *     - Modify the Wi-Fi settings–SSID and password to connect to your router.
 *
 * [4] Finish the function `csi_process()`:
 *     - Fill in the group information.
 *     - Process and analyze CSI data in the `csi_process` function.
 *     - Implement your algorithms in this function if on-board. (Task 2)
 *     - Return the results to the host or send the CSI data via MQTT. (Task 3)
 *
 * Feel free to modify these sections to suit your project requirements!
 * 
 * Have fun building!
 */


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "nvs_flash.h"
#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h" // Include the header for esp_timer_get_time
#include "mqtt_client.h"
#include "esp_netif.h"

bool wifi_connected = false;
const char *TAG = "csi_recv";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;

// [1] YOUR CODE HERE
#define CSI_BUFFER_LENGTH 800
#define CSI_FIFO_LENGTH 100
static int16_t CSI_Q[CSI_BUFFER_LENGTH];
static int CSI_Q_INDEX = 0; // CSI Buffer Index
// Enable/Disable CSI Buffering. 1: Enable, using buffer, 0: Disable, using serial output
static bool CSI_Q_ENABLE = 0; 
static void csi_process(const int8_t *csi_data, int length);
// [1] END OF YOUR CODE


// [2] YOUR CODE HERE
// Modify the following functions to implement your algorithms.
// NOTE: Please do not change the function names and return types.
bool motion_detection() {
    // TODO: Implement motion detection logic using CSI data in CSI_Q
    // 检测CSI信号中的变化来判断是否有运动
    if (CSI_Q_INDEX < 100) return false; // 数据不足，无法可靠检测
    
    int threshold = 50; // 运动检测阈值
    int sum_diff = 0;
    
    // 计算相邻样本的平均差值
    for (int i = 1; i < CSI_Q_INDEX && i < CSI_BUFFER_LENGTH; i++) {
        sum_diff += abs(CSI_Q[i] - CSI_Q[i-1]);
    }
    
    int avg_diff = sum_diff / (CSI_Q_INDEX - 1);
    ESP_LOGI(TAG, "Motion detection: avg_diff = %d, threshold = %d", avg_diff, threshold);
    
    return avg_diff > threshold;
}

int breathing_rate_estimation() {
    // TODO: Implement breathing rate estimation using CSI data in CSI_Q
    // 使用FFT或者波峰检测估计呼吸频率
    if (CSI_Q_INDEX < 600) return 0; // 至少需要10秒数据(假设采样率60Hz)
    
    // 简化的波峰检测算法
    int peaks = 0;
    int min_peak_distance = 30; // 约0.5秒
    int last_peak = -min_peak_distance;
    
    // 移动平均滤波
    int window_size = 5;
    int16_t smoothed[CSI_BUFFER_LENGTH];
    
    for (int i = 0; i < CSI_Q_INDEX; i++) {
        int sum = 0;
        int count = 0;
        for (int j = i - window_size/2; j <= i + window_size/2; j++) {
            if (j >= 0 && j < CSI_Q_INDEX) {
                sum += CSI_Q[j];
                count++;
            }
        }
        smoothed[i] = sum / count;
    }
    
    // 寻找波峰
    for (int i = 1; i < CSI_Q_INDEX - 1; i++) {
        if (smoothed[i] > smoothed[i-1] && smoothed[i] > smoothed[i+1] && (i - last_peak) > min_peak_distance) {
            peaks++;
            last_peak = i;
        }
    }
    
    // 计算呼吸率 (每分钟呼吸次数)
    // 假设数据采样率为60Hz
    float duration_seconds = CSI_Q_INDEX / 60.0f;
    int breaths_per_minute = (int)((peaks * 60) / duration_seconds);
    
    ESP_LOGI(TAG, "Breathing rate: %d breaths/minute (from %d peaks in %.1f seconds)", 
             breaths_per_minute, peaks, duration_seconds);
    
    return breaths_per_minute;
}

void mqtt_send() {
    // TODO: Implement MQTT message sending using CSI data or Results
    // NOTE: If you implement the algorithm on-board, you can return the results to the host, else send the CSI data.
    // 检查WiFi和MQTT连接状态
     if (!wifi_connected) {
        ESP_LOGE(TAG, "Can't send MQTT: WiFi not connected");
        return;
    }
    if (mqtt_client == NULL || !mqtt_connected) {
        ESP_LOGE(TAG, "Can't send MQTT: MQTT client not initialized or not connected");
        return;
    }
    // 检测运动和呼吸率
    bool motion_detected = motion_detection();
    int breathing_rate = breathing_rate_estimation();
    
    // 准备MQTT消息
    char mqtt_message[256];
    
    // 构建JSON格式消息
    snprintf(mqtt_message, sizeof(mqtt_message), 
             "{\"team_id\":\"Team1\",\"timestamp\":%lld,\"motion_detected\":%s,\"breathing_rate\":%d,\"samples\":%d}",
             esp_timer_get_time() / 1000, // 毫秒时间戳
             motion_detected ? "true" : "false",
             breathing_rate,
             CSI_Q_INDEX);
    
    ESP_LOGI(TAG, "MQTT message prepared: %s", mqtt_message);
    
    // 发布MQTT消息
    int msg_id = esp_mqtt_client_publish(mqtt_client, "csi/results", mqtt_message, 0, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish MQTT message");
    } else {
        ESP_LOGI(TAG, "MQTT message published successfully, msg_id=%d", msg_id);
    }

    // 定期清理缓冲区，保留最新的CSI_FIFO_LENGTH个样本
    if (CSI_Q_INDEX > CSI_FIFO_LENGTH * 1.5) {
        memmove(CSI_Q, CSI_Q + (CSI_Q_INDEX - CSI_FIFO_LENGTH), CSI_FIFO_LENGTH * sizeof(int16_t));
        CSI_Q_INDEX = CSI_FIFO_LENGTH;
        ESP_LOGI(TAG, "CSI buffer trimmed to %d samples", CSI_Q_INDEX);
    }
}
// [2] END OF YOUR CODE


#define CONFIG_LESS_INTERFERENCE_CHANNEL   40
#define CONFIG_WIFI_BAND_MODE   WIFI_BAND_MODE_5G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT20
#define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT20
#define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
#define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT20
#define CONFIG_ESP_NOW_RATE             WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_FORCE_GAIN                   1
#define CONFIG_GAIN_CONTROL                 CONFIG_FORCE_GAIN

// UPDATE: Define parameters for scan method
#if CONFIG_EXAMPLE_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_EXAMPLE_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_EXAMPLE_SCAN_METHOD*/
//

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
typedef struct
{
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 16; /**< reserved */
    unsigned fft_gain : 8;
    unsigned agc_gain : 8;
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;

#if CONFIG_FORCE_GAIN
    /**
     * @brief Enable/disable automatic fft gain control and set its value
     * @param[in] force_en true to disable automatic fft gain control
     * @param[in] force_value forced fft gain value
     */
    extern void phy_fft_scale_force(bool force_en, uint8_t force_value);

    /**
     * @brief Enable/disable automatic gain control and set its value
     * @param[in] force_en true to disable automatic gain control
     * @param[in] force_value forced gain value
     */
    extern void phy_force_rx_gain(int force_en, int force_value);
#endif

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data);

//------------------------------------------------------WiFi Initialize------------------------------------------------------
static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_got_ip));
    
    // [3] YOUR CODE HERE
    // You need to modify the ssid and password to match your Wi-Fi network.
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "vivo X100",         
            .password = "Pengziyu010702",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            // UPDATES: only use this scan method when you want to connect your mobile phone's hotpot
            .scan_method = DEFAULT_SCAN_METHOD,
            //
        
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    // [3] END OF YOUR CODE

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init finished.");
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT Message published successfully, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT Error occurred");
            break;
        default:
            ESP_LOGI(TAG, "Other MQTT event id:%d", event->event_id);
            break;
    }
}

//------------------------------------------------------WiFi Event Handler------------------------------------------------------
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                             int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Trying to connect to AP...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "Connection failed! Retrying...");
        wifi_connected = false;
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "Connected to AP - SSID: %s, Channel: %d, RSSI: %d",
                    ap_info.ssid, ap_info.primary, ap_info.rssi);
        }
    }
}

//------------------------------------------------------ESP-NOW Initialize------------------------------------------------------
static void wifi_esp_now_init(esp_now_peer_info_t peer) 
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE, 
        .rate = CONFIG_ESP_NOW_RATE,//  WIFI_PHY_RATE_MCS0_LGI,    
        .ersu = false,                     
        .dcm = false                       
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr,&rate_config));
    ESP_LOGI(TAG, "================ ESP NOW Ready ================");
    ESP_LOGI(TAG, "esp_now_init finished.");
}

//------------------------------------------------------MQTT Initialize------------------------------------------------------
static void mqtt_init(void)
{
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Cannot initialize MQTT: WiFi not connected");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing MQTT client");
    
    // 获取本机IP地址 - 使用新的esp_netif API
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(netif, &ip_info);
    char local_ip[16];
    sprintf(local_ip, IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Device IP Address: %s", local_ip);
    
    // 配置MQTT客户端 - 使用新的配置结构
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = "mqtt://192.168.89.246",
            .address.port = 1883,
        },
        .credentials = {
            .username = "peng",
            .authentication.password = "112233",
        },
        .session = {
            .keepalive = 15,
        },
        .task = {
            .stack_size = 8192,
        }
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    
    // 注册MQTT事件处理函数
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    // 启动MQTT客户端
    esp_mqtt_client_start(mqtt_client);
    ESP_LOGI(TAG, "MQTT client started");
}

//------------------------------------------------------CSI Callback------------------------------------------------------
static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;

    ESP_LOGI(TAG, "CSI callback triggered");

    // Applying the CSI_Q_ENABLE flag to determine the output method
    // 1: Enable, using buffer, 0: Disable, using serial output
    if (!CSI_Q_ENABLE) {
        ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d\n",
                   info->len, MAC2STR(info->mac), info->rx_ctrl.rssi,
                   info->rx_ctrl.rate, info->rx_ctrl.noise_floor,
                   info->rx_ctrl.channel);
    } else {
        csi_process(info->buf, info->len);
    }

    
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    ESP_LOGI(TAG, "Received MAC: "MACSTR", Expected MAC: "MACSTR, 
             MAC2STR(info->mac), MAC2STR(CONFIG_CSI_SEND_MAC));
    
    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) {
        ESP_LOGI(TAG, "MAC address doesn't match, skipping packet");
        return;
    }

    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;
    static int s_count = 0;

#if CONFIG_GAIN_CONTROL
    static uint16_t agc_gain_sum=0; 
    static uint16_t fft_gain_sum=0; 
    static uint8_t agc_gain_force_value=0; 
    static uint8_t fft_gain_force_value=0; 
    if (s_count<100) {
        agc_gain_sum += phy_info->agc_gain;
        fft_gain_sum += phy_info->fft_gain;
    }else if (s_count == 100) {
        agc_gain_force_value = agc_gain_sum/100;
        fft_gain_force_value = fft_gain_sum/100;
    #if CONFIG_FORCE_GAIN
        phy_fft_scale_force(1,fft_gain_force_value);
        phy_force_rx_gain(1,agc_gain_force_value);
    #endif
        ESP_LOGI(TAG,"fft_force %d, agc_force %d",fft_gain_force_value,agc_gain_force_value);
    }
#endif

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    if (CSI_Q_ENABLE == 0) {
        ESP_LOGI(TAG, "================ CSI RECV via Serial Port ================");
        ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
            s_count++, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
            rx_ctrl->noise_floor, phy_info->fft_gain, phy_info->agc_gain, rx_ctrl->channel,
            rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
        ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);

        for (int i = 1; i < info->len; i++) {
            ets_printf(",%d", info->buf[i]);
        }
        ets_printf("]\"\n");
    }

    else{
        ESP_LOGI(TAG, "================ CSI RECV via Buffer ================");
        csi_process(info->buf, info->len);
    }
}

//------------------------------------------------------CSI Processing & Algorithms------------------------------------------------------
static void csi_process(const int8_t *csi_data, int length)
{   
    if (CSI_Q_INDEX + length > CSI_BUFFER_LENGTH) {
        int shift_size = CSI_BUFFER_LENGTH - CSI_FIFO_LENGTH;
        memmove(CSI_Q, CSI_Q + CSI_FIFO_LENGTH, shift_size * sizeof(int16_t));
        CSI_Q_INDEX = shift_size;
    }    
    ESP_LOGI(TAG, "CSI Buffer Status: %d samples stored", CSI_Q_INDEX);
    // Append new CSI data to the buffer
    for (int i = 0; i < length && CSI_Q_INDEX < CSI_BUFFER_LENGTH; i++) {
        CSI_Q[CSI_Q_INDEX++] = (int16_t)csi_data[i];
    }

    // [4] YOUR CODE HERE

    // 1. Fill the information of your group members
    ESP_LOGI(TAG, "================ GROUP INFO ================");
    const char *TEAM_MEMBER[] = {"a", "b", "c", "d"};
    const char *TEAM_UID[] = {"1", "2", "3", "4"};
    ESP_LOGI(TAG, "TEAM_MEMBER: %s, %s, %s, %s | TEAM_UID: %s, %s, %s, %s",
                TEAM_MEMBER[0], TEAM_MEMBER[1], TEAM_MEMBER[2], TEAM_MEMBER[3],
                TEAM_UID[0], TEAM_UID[1], TEAM_UID[2], TEAM_UID[3]);
    ESP_LOGI(TAG, "================ END OF GROUP INFO ================");

    // 2. Call your algorithm functions here, e.g.: motion_detection(), breathing_rate_estimation(), and mqtt_send()
    // If you implement the algorithm on-board, you can return the results to the host, else send the CSI data.
    // motion_detection();
    // breathing_rate_estimation();
    // mqtt_send();
    if (CSI_Q_INDEX >= CSI_FIFO_LENGTH) {
    bool motion = motion_detection();
    int breathing = breathing_rate_estimation();
    
    // 输出检测结果
    ESP_LOGI(TAG, "================ DETECTION RESULTS ================");
    ESP_LOGI(TAG, "Motion detected: %s", motion ? "YES" : "NO");
    ESP_LOGI(TAG, "Breathing rate: %d breaths/minute", breathing);
    ESP_LOGI(TAG, "================ END OF RESULTS ================");
    
    // 每10次CSI处理发送一次MQTT消息
    static int process_count = 0;
    if (++process_count % 10 == 0) {
        mqtt_send();
    }
} else {
    ESP_LOGI(TAG, "Collecting data... %d/%d samples", CSI_Q_INDEX, CSI_FIFO_LENGTH);
}
    // [4] END YOUR CODE HERE
}


//------------------------------------------------------CSI Config Initialize------------------------------------------------------
static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    wifi_csi_config_t csi_config = {
        .enable                   = true,                           
        .acquire_csi_legacy       = false,               
        .acquire_csi_force_lltf   = false,           
        .acquire_csi_ht20         = true,                  
        .acquire_csi_ht40         = true,                  
        .acquire_csi_vht          = false,                  
        .acquire_csi_su           = false,                   
        .acquire_csi_mu           = false,                   
        .acquire_csi_dcm          = false,                  
        .acquire_csi_beamformed   = false,           
        .acquire_csi_he_stbc_mode = 2,                                                                                                                                                                                                                                                                               
        .val_scale_cfg            = 0,                    
        .dump_ack_en              = false,                      
        .reserved                 = false                         
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

//------------------------------------------------------Main Function------------------------------------------------------
void app_main()
{
    /**
     * @brief Initialize NVS
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /**
     * @brief Initialize Wi-Fi
     */
    wifi_init();

    // Get Device MAC Address
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "Device MAC Address: " MACSTR, MAC2STR(mac));

    // Try to connect to WiFi
    ESP_LOGI(TAG, "Connecting to WiFi...");

    // Wait for Wi-Fi connection
    int retry_count = 0;
    wifi_connected = false;
    while (!wifi_connected && retry_count < 20) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        retry_count++;
        ESP_LOGI(TAG, "Waiting for Wi-Fi connection... (%d/20)", retry_count);

        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            ESP_LOGI(TAG, "Connected to SSID: %s, RSSI: %d, Channel: %d",
                     ap_info.ssid, ap_info.rssi, ap_info.primary);
            wifi_connected = true;
        }
    }

    /**
     * @brief Initialize ESP-NOW
     */

    if (wifi_connected) {
        esp_now_peer_info_t peer = {
            .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
            .ifidx     = WIFI_IF_STA,
            .encrypt   = false,
            .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        };

        wifi_esp_now_init(peer); // Initialize ESP-NOW Communication
        wifi_csi_init(); // Initialize CSI Collection
        mqtt_init(); // Initialize MQTT Client

    } else {
        ESP_LOGI(TAG, "WiFi connection failed");
        return;
    }
}
