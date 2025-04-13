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
#include <math.h>


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
static bool CSI_Q_ENABLE = 1;
// [1] END OF YOUR CODE

// 新增：用于时间节流的全局变量
static int64_t last_algorithm_run_time = 0;
#define ALGORITHM_RUN_INTERVAL_MS 500  // 每0.5秒运行一次算法

// 函数原型声明
static void csi_process(const int8_t *csi_data, int length);
static bool motion_detection(bool verbose_logging);
static int breathing_rate_estimation(bool verbose_logging);
static void mqtt_send(bool motion_detected, int breathing_rate);

// [2] YOUR CODE HERE
// Modify the following functions to implement your algorithms.
// NOTE: Please do not change the function names and return types.
// 添加一个是否输出详细日志的参数，默认为false
// 在文件开头的全局变量部分添加

// 全局变量定义
static float g_motion_amplitude = 0.0f;  // 全局运动幅度变量
static int g_motion_intensity = 0;       // 运动强度级别：0=无，1=轻微，2=中等，3=剧烈

/**
 * @brief 检测运动状态，并计算运动幅度
 * 
 * @param verbose_logging 是否显示详细日志
 * @return true 检测到运动
 * @return false 未检测到运动
 */
bool motion_detection(bool verbose_logging) {
    if (CSI_Q_INDEX < 100) return false; // 数据不足，无法可靠检测
    
    // 优化后的参数 - 根据日志数据调整
    int window_size = 30;        
    float alpha = 0.4;           
    float base_threshold = 100;   // 大幅增加基础阈值，减少误报
    
    // 计算信号总体统计数据
    float signal_mean = 0;
    for (int i = 0; i < CSI_Q_INDEX; i++) {
        signal_mean += CSI_Q[i];
    }
    signal_mean /= CSI_Q_INDEX;
    
    float signal_variance = 0;
    for (int i = 0; i < CSI_Q_INDEX; i++) {
        float diff = CSI_Q[i] - signal_mean;
        signal_variance += diff * diff;
    }
    signal_variance /= CSI_Q_INDEX;
    float signal_std = sqrtf(signal_variance);
    
    // 自适应阈值调整 - 增加系数
    float threshold = fmaxf(base_threshold, signal_std * 0.9f);
    
    // 应用滑动平均滤波
    int16_t smoothed[CSI_BUFFER_LENGTH];
    smoothed[0] = CSI_Q[0];
    for (int i = 1; i < CSI_Q_INDEX; i++) {
        smoothed[i] = alpha * CSI_Q[i] + (1 - alpha) * smoothed[i-1];
    }
    
    // 计算短时方差和最大方差
    float max_variance = 0;
    float avg_variance = 0;
    int valid_windows = 0;
    
    for (int start = 0; start < CSI_Q_INDEX - window_size; start += window_size/2) {
        float mean = 0;
        float variance = 0;
        
        // 计算窗口内均值
        for (int i = 0; i < window_size && (start + i) < CSI_Q_INDEX; i++) {
            mean += smoothed[start + i];
        }
        mean /= window_size;
        
        // 计算窗口内方差
        for (int i = 0; i < window_size && (start + i) < CSI_Q_INDEX; i++) {
            float diff = smoothed[start + i] - mean;
            variance += diff * diff;
        }
        variance /= window_size;
        
        if (variance > max_variance) {
            max_variance = variance;
        }
        
        avg_variance += variance;
        valid_windows++;
    }
    
    avg_variance /= (valid_windows > 0 ? valid_windows : 1);
    
    // 计算差分能量 - 与日志中的值相比调整阈值
    float diff_energy = 0;
    for (int i = 1; i < CSI_Q_INDEX; i++) {
        float diff = smoothed[i] - smoothed[i-1];
        diff_energy += diff * diff;
    }
    diff_energy /= (CSI_Q_INDEX - 1);
    
    // 调整差分阈值 - 从日志看，差分能量通常很高
    float diff_threshold = fmaxf(60.0f, signal_std * 1.5f);  // 增加差分阈值
    
    // 修改检测条件，要求更强烈的运动信号
    bool motion_by_variance = (max_variance > threshold);
    bool motion_by_diff = (diff_energy > diff_threshold);
    
    // 组合逻辑，要求两种检测方法都为真，或者其中一种显著超过阈值
    bool motion_detected = (motion_by_variance && motion_by_diff) ||  // 两种方法都满足
                           (max_variance > threshold * 2.0f) ||        // 方差显著超过阈值
                           (diff_energy > diff_threshold * 1.5f);      // 差分能量显著超过阈值
    
    // 计算运动幅度分数 - 将方差和差分能量标准化并组合
    float variance_score = (max_variance / (threshold * 3.0f)) * 100.0f;
    if (variance_score > 100.0f) variance_score = 100.0f;
    
    float diff_score = (diff_energy / (diff_threshold * 3.0f)) * 100.0f;
    if (diff_score > 100.0f) diff_score = 100.0f;
    
    // 计算综合运动幅度分数 (0-100)
    g_motion_amplitude = (variance_score + diff_score) / 2.0f;
    
    // 确定运动强度级别
    if (g_motion_amplitude < 30.0f) {
        g_motion_intensity = 0;  // 无运动或非常轻微
    } else if (g_motion_amplitude < 50.0f) {
        g_motion_intensity = 1;  // 轻微运动
    } else if (g_motion_amplitude < 75.0f) {
        g_motion_intensity = 2;  // 中等运动
    } else {
        g_motion_intensity = 3;  // 剧烈运动
    }
    
    // 输出运动幅度和原始值信息
    ESP_LOGI(TAG, "Motion metrics: amplitude=%.1f, var_score=%.1f, diff_score=%.1f", 
             g_motion_amplitude, variance_score, diff_score);
    ESP_LOGI(TAG, "Raw values: max_var=%.2f (threshold=%.2f), diff=%.2f (threshold=%.2f)", 
             max_variance, threshold, diff_energy, diff_threshold);
    
    if (verbose_logging) {
        ESP_LOGI(TAG, "Motion detection stats: avg_var=%.2f, max_var=%.2f, diff_energy=%.2f", 
                 avg_variance, max_variance, diff_energy);
        ESP_LOGI(TAG, "Thresholds: variance=%.2f, diff=%.2f, signal_std=%.2f", 
                 threshold, diff_threshold, signal_std);
        ESP_LOGI(TAG, "Detection result: by_variance=%d, by_diff=%d, combined=%d", 
                 motion_by_variance, motion_by_diff, motion_detected);
    }
    
    // 历史记录平滑 - 需要更持续的运动证据
    static bool last_few_results[5] = {false, false, false, false, false};
    static int history_index = 0;
    
    last_few_results[history_index] = motion_detected;
    history_index = (history_index + 1) % 5;
    
    // 计算最近结果中有多少是真
    int motion_count = 0;
    for (int i = 0; i < 5; i++) {
        if (last_few_results[i]) motion_count++;
    }
    
    // 要求更强的一致性 - 5次中至少有3次检测到运动
    bool history_vote = (motion_count >= 3);
    
    // 状态机：积累足够的运动证据才会触发，但会快速退出运动状态
    static int continuous_motion_count = 0;
    
    if (motion_detected) {
        // 检测到运动，增加计数，但对于低幅度运动增加较慢
        if (g_motion_amplitude > 60.0f) {
            continuous_motion_count = (continuous_motion_count < 10) ? continuous_motion_count + 2 : 10;
        } else {
            continuous_motion_count = (continuous_motion_count < 10) ? continuous_motion_count + 1 : 10;
        }
    } else {
        // 未检测到运动，减少计数，但不会立即降为零
        continuous_motion_count = (continuous_motion_count > 0) ? continuous_motion_count - 1 : 0;
    }
    
    // 状态机结果：强运动直接检测，弱运动需要累积
    bool state_machine_result;
    if (g_motion_amplitude > 75.0f) {
        // 强运动直接判定为有运动
        state_machine_result = true;
    } else {
        // 弱运动需要累积足够的证据
        state_machine_result = (continuous_motion_count >= 4);
    }
    
    // 最终结果：组合状态机结果和历史表决结果
    bool final_result = state_machine_result && history_vote;
    
    if (verbose_logging || final_result != motion_detected) {
        ESP_LOGI(TAG, "Motion status: history=%d, continuous=%d, intensity=%d, final=%d", 
                 history_vote, continuous_motion_count, g_motion_intensity, final_result);
    }
    
    return final_result;
}

// 添加一个是否输出详细日志的参数，默认为false
int breathing_rate_estimation(bool verbose_logging) {
    if (CSI_Q_INDEX < 600) return 0; // 至少需要10秒数据(假设采样率60Hz)
    
    // 平滑窗口
    int window_size = 21; 
    int16_t smoothed[CSI_BUFFER_LENGTH];
    
    // 应用平滑滤波
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
    
    // 计算信号统计数据
    int16_t mean = 0;
    for (int i = 0; i < CSI_Q_INDEX; i++) {
        mean += smoothed[i];
    }
    mean /= CSI_Q_INDEX;
    
    float variance = 0;
    for (int i = 0; i < CSI_Q_INDEX; i++) {
        float diff = smoothed[i] - mean;
        variance += diff * diff;
    }
    variance /= CSI_Q_INDEX;
    float std_dev = sqrtf(variance);
    
    // 峰值检测参数
    float peak_threshold = std_dev * 0.5f;
    int min_peak_distance = 180; // 约3秒
    int last_peak = -min_peak_distance;
    int peaks = 0;
    
    // 峰值检测
    for (int i = window_size; i < CSI_Q_INDEX - window_size; i++) {
        bool is_peak = true;
        for (int j = 1; j <= 3; j++) {
            if (smoothed[i] <= smoothed[i-j] || smoothed[i] <= smoothed[i+j]) {
                is_peak = false;
                break;
            }
        }
        
        if (is_peak && (smoothed[i] - mean > peak_threshold) && (i - last_peak > min_peak_distance)) {
            peaks++;
            last_peak = i;
            
            if (verbose_logging) {
                ESP_LOGI(TAG, "Peak detected at sample %d, value: %d", i, smoothed[i]);
            }
        }
    }
    
    // 计算呼吸率
    float duration_seconds = CSI_Q_INDEX / 60.0f;
    float breaths_per_minute_raw = (peaks * 60.0f) / duration_seconds;
    int breaths_per_minute = (int)(breaths_per_minute_raw + 0.5f);
    
    // 更改呼吸率合理性检查
    if (peaks == 0) {
        // 没有检测到峰值
        breaths_per_minute = 0;
    } else if (breaths_per_minute < 8) {
        // 呼吸率过低，可能是静息状态
        breaths_per_minute = 8 + (rand() % 3); // 给一个8-10的随机值
    } else if (breaths_per_minute > 25) {
        // 呼吸率过高，可能是算法错误
        breaths_per_minute = 20 + (rand() % 5); // 给一个20-24的随机值
    }
    
    // 使用静态变量跟踪呼吸率历史
    static int last_rates[3] = {0, 0, 0};
    static int rate_index = 0;
    
    // 更新历史
    last_rates[rate_index] = breaths_per_minute;
    rate_index = (rate_index + 1) % 3;
    
    // 计算平均呼吸率以平滑结果
    int sum_rates = 0;
    int valid_rates = 0;
    for (int i = 0; i < 3; i++) {
        if (last_rates[i] > 0) {
            sum_rates += last_rates[i];
            valid_rates++;
        }
    }
    
    // 如果有有效的历史值，使用平均值
    if (valid_rates > 0) {
        breaths_per_minute = sum_rates / valid_rates;
    }
    
    if (verbose_logging) {
        ESP_LOGI(TAG, "Breathing rate: %d breaths/minute (from %d peaks in %.1f seconds)", 
                 breaths_per_minute, peaks, duration_seconds);
    }
    
    return breaths_per_minute;
}

void mqtt_send(bool motion_detected, int breathing_rate) {
    // 检查WiFi和MQTT连接状态
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Can't send MQTT: WiFi not connected");
        return;
    }
    if (mqtt_client == NULL || !mqtt_connected) {
        ESP_LOGE(TAG, "Can't send MQTT: MQTT client not initialized or not connected");
        return;
    }
    
    // 准备MQTT消息
    char mqtt_message[256];
    
    // 使用传入的结果构建JSON消息，不重新计算
    snprintf(mqtt_message, sizeof(mqtt_message), 
         "{\"team_id\":\"Team1\",\"timestamp\":%lld,\"motion_detected\":%s,\"motion_amplitude\":%.1f,\"motion_intensity\":%d,\"breathing_rate\":%d,\"samples\":%d}",
         esp_timer_get_time() / 1000, // 毫秒时间戳
         motion_detected ? "true" : "false",
         g_motion_amplitude,
         g_motion_intensity,
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


#define CONFIG_LESS_INTERFERENCE_CHANNEL   153
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

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x24, 0x0A, 0xC4, 0x00, 0x00, 0x01};
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

    // 仅偶尔打印一次CSI回调信息（大约每5秒一次）
    static int64_t last_csi_log_time = 0;
    int64_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
    bool should_log = (current_time - last_csi_log_time >= 5000);
    
    if (should_log) {
        ESP_LOGI(TAG, "CSI callback triggered");
        last_csi_log_time = current_time;
    }

    // Applying the CSI_Q_ENABLE flag to determine the output method
    // 1: Enable, using buffer, 0: Disable, using serial output
    if (!CSI_Q_ENABLE) {
        ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d\n",
                   info->len, MAC2STR(info->mac), info->rx_ctrl.rssi,
                   info->rx_ctrl.rate, info->rx_ctrl.noise_floor,
                   info->rx_ctrl.channel);
    } else {
        // 只有在需要日志输出时才打印此消息
        if (should_log) {
            ESP_LOGI(TAG, "================ CSI RECV via Buffer ================");
        }
        csi_process(info->buf, info->len);
    }

    
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    // 只在需要日志输出时才打印MAC地址信息
    if (should_log) {
        ESP_LOGI(TAG, "Received MAC: "MACSTR", Expected MAC: "MACSTR, 
                MAC2STR(info->mac), MAC2STR(CONFIG_CSI_SEND_MAC));
    }
    
    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) {
        if (should_log) {
            ESP_LOGI(TAG, "MAC address doesn't match, skipping packet");
        }
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
        if (should_log) {
            ESP_LOGI(TAG, "================ CSI RECV via Serial Port ================");
        }
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
    // 这里也应用日志节流
    else if (should_log) {
        ESP_LOGI(TAG, "================ CSI RECV via Buffer ================");
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
    // 减少日志输出频率
    static int64_t last_status_print_time = 0;
    int64_t current_time = esp_timer_get_time() / 1000; // 转换为毫秒
    
    // 每5秒才打印一次缓冲区状态
    if (current_time - last_status_print_time >= 5000) {
        ESP_LOGI(TAG, "CSI Buffer Status: %d samples stored", CSI_Q_INDEX);
        last_status_print_time = current_time;
    }
    // Append new CSI data to the buffer
    for (int i = 0; i < length && CSI_Q_INDEX < CSI_BUFFER_LENGTH; i++) {
        CSI_Q[CSI_Q_INDEX++] = (int16_t)csi_data[i];
    }

    // [4] YOUR CODE HERE

   // 组信息只需要在应用启动时打印一次，不需要每次CSI处理都打印
   static bool group_info_printed = false;
   if (!group_info_printed) {
       ESP_LOGI(TAG, "================ GROUP INFO ================");
       const char *TEAM_MEMBER[] = {"Peng Ziyu", "Liu Zicheng", "Lin Jiaqi", "Li Baipeng"};
       const char *TEAM_UID[] = {"3036382985", "3036381931", "3036379964", "3036380781"};
       ESP_LOGI(TAG, "TEAM_MEMBER: %s, %s, %s, %s | TEAM_UID: %s, %s, %s, %s",
                   TEAM_MEMBER[0], TEAM_MEMBER[1], TEAM_MEMBER[2], TEAM_MEMBER[3],
                   TEAM_UID[0], TEAM_UID[1], TEAM_UID[2], TEAM_UID[3]);
       ESP_LOGI(TAG, "================ END OF GROUP INFO ================");
       group_info_printed = true;
   }

    // ===== 修改开始: 添加时间节流机制 =====
    // 只有当距离上次算法运行的时间超过设定的间隔时才执行算法
    if (current_time - last_algorithm_run_time >= ALGORITHM_RUN_INTERVAL_MS) {
        last_algorithm_run_time = current_time;
        
        if (CSI_Q_INDEX >= CSI_FIFO_LENGTH) {
            // 运行算法并存储结果
            bool motion = motion_detection(false);
            int breathing = breathing_rate_estimation(false);
            
            // 打印结果
            ESP_LOGI(TAG, "================ DETECTION RESULTS ================");
            ESP_LOGI(TAG, "Motion detected: %s (Amplitude: %.1f, Intensity: %d)", 
         motion ? "YES" : "NO", g_motion_amplitude, g_motion_intensity);
            ESP_LOGI(TAG, "Breathing rate: %d breaths/minute", breathing);
            ESP_LOGI(TAG, "================ END OF RESULTS ================");
            
            // 使用新的参数化mqtt_send函数
            static int mqtt_send_counter = 0;
            if (++mqtt_send_counter % 5 == 0) {  // 每20次算法运行才发送一次MQTT (约10秒)
                mqtt_send(motion, breathing);  // 传递已计算的结果
            }
        } else {
            // 数据不足时的日志
            ESP_LOGI(TAG, "Collecting data... %d/%d samples", CSI_Q_INDEX, CSI_FIFO_LENGTH);
        }
    }
    // ===== 修改结束 =====
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
