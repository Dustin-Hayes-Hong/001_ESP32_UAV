#include "WIFI.h" //WIFI头文件

#define ESP_WIFI_SSID "DHH"
#define ESP_WIFI_PASS "88888888"
#define ESP_MAXIMUM_RETRY 2

/* FreeRTOS事件组，用于指示我们何时连接*/
static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect(); // Wi-Fi STA 启动后尝试连接
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect(); // 如果连接断开，且重试次数未达到上限，则重新连接
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT); // 达到最大重试次数，设置连接失败标志
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip)); // 获取到IP地址
        s_retry_num = 0;                                            // 重置重试次数
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // 设置连接成功标志
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate(); // 创建事件组

    ESP_ERROR_CHECK(esp_netif_init()); // 初始化网络接口

    ESP_ERROR_CHECK(esp_event_loop_create_default()); // 创建默认事件循环
    esp_netif_create_default_wifi_sta();              // 创建默认的Wi-Fi STA网络接口

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // 使用默认Wi-Fi初始化配置
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                // 初始化Wi-Fi

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, // 注册Wi-Fi事件处理函数
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, // 注册IP事件处理函数
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,                        // 设置SSID
            .password = ESP_WIFI_PASS,                    // 设置密码
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK, // 设置认证模式阈值
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));               // 设置Wi-Fi模式为STA
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config)); // 设置Wi-Fi配置
    ESP_ERROR_CHECK(esp_wifi_start());                               // 启动Wi-Fi

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* 等待直到连接建立（WIFI_CONNECTED_BIT）或连接因达到最大重试次数而失败（WIFI_FAIL_BIT）。
     * 这些位由event_handler()设置（见上文）*/
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits()返回调用返回之前的位，因此我们可以测试实际发生了哪个事件。*/
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    
}

void WIFI_Init(void)
{
    //初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
}