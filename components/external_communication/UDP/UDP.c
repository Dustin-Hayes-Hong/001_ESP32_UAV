#include "UDP.h"

struct sockaddr_in6 dest_addr_anotc; //地面站套接字地址
struct sockaddr_storage source_addr_anotc; //地面站发送方套接字地址
char addr_str_anotc[128]; //地面站发送方IP地址字符串
char rx_buffer_anotc[128]; //地面站接收缓冲区

int sock_anotc; //地面站套接字
int anotc_PORT = 3333; //地面站端口

struct sockaddr_in6 dest_addr_rc; //遥控器套接字地址
struct sockaddr_storage source_addr_rc; //遥控器发送方套接字地址
char addr_str_rc[128]; //遥控器发送方IP地址字符串
char rx_buffer_rc[128]; //遥控器接收缓冲区

int sock_rc; //遥控器套接字
int rc_PORT = 5555; // 遥控器端口

int addr_family = AF_INET;
int ip_protocol = 0;

bool anotc_state = false; //地面站连接状态
bool rc_state = false; //遥控器连接状态

/**
 * @brief 初始化地面站UDP套接字
 * @note  创建并绑定地面站的UDP套接字，用于通信
 */
void socket_anotc_init(void) {
    // 配置IPv4地址结构
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr_anotc;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY); // 绑定任意本地IP地址
        dest_addr_ip4->sin_family = AF_INET;               // 使用IPv4协议
        dest_addr_ip4->sin_port = htons(anotc_PORT);       // 设置端口号
        ip_protocol = IPPROTO_IP;                          // IP协议
    }

    // 创建UDP套接字
    sock_anotc = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock_anotc < 0) {
        printf("地面站UDP套接字创建失败\n");
        return;
    }
    printf("地面站UDP套接字创建成功\n");

    // 绑定套接字到指定地址和端口
    int err = bind(sock_anotc, (struct sockaddr *)&dest_addr_anotc, sizeof(dest_addr_anotc));
    if (err < 0) {
        printf("地面站UDP套接字绑定失败\n");
        return;
    }
    printf("地面站UDP套接字绑定成功\n");
    printf("地面站UDP套接字端口号：%d\n", anotc_PORT);
}

/**
 * @brief 初始化遥控器UDP套接字
 * @note  创建并绑定遥控器的UDP套接字，用于通信
 */
void socket_rc_init(void) {
    // 配置IPv4地址结构
    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr_rc;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY); // 绑定任意本地IP地址，htonl()函数用于将主机字节序转换为网络字节序（大端序）
        dest_addr_ip4->sin_family = AF_INET;               // 使用IPv4协议
        dest_addr_ip4->sin_port = htons(rc_PORT);          // 设置端口号，htons()函数用于将主机字节序转换为网络字节序
        ip_protocol = IPPROTO_IP;                          // 指定底层协议为 IP 协议
    }

    // 创建UDP套接字
    sock_rc = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock_rc < 0) {
        printf("遥控器UDP套接字创建失败\n");
        return;
    }
    printf("遥控器UDP套接字创建成功\n");

    // 绑定套接字到指定地址和端口
    int err = bind(sock_rc, (struct sockaddr *)&dest_addr_rc, sizeof(dest_addr_rc));
    if (err < 0) {
        printf("遥控器UDP套接字绑定失败\n");
        return;
    }
    printf("遥控器UDP套接字绑定成功\n");
    printf("遥控器UDP套接字端口号：%d\n", rc_PORT);
}

/**
 * @brief 地面站UDP数据接收任务
 * @param pvParameters 任务参数（未使用）
 * @note  持续监听地面站UDP套接字，接收并处理数据，错误时重启套接字
 */
static void udp_anotc_read_task(void *pvParameters) {
    (void)pvParameters; // 避免未使用参数警告

    while (1) {
        // 接收数据循环
        while (1) {
            socklen_t socklen = sizeof(source_addr_anotc);
            int len = recvfrom(sock_anotc, rx_buffer_anotc, sizeof(rx_buffer_anotc) - 1, 0,
                               (struct sockaddr *)&source_addr_anotc, &socklen);
            if (len < 0) {
                printf("地面站数据接收失败\n");
                break; // 接收失败，跳出内层循环以重启套接字
            }

            // 成功接收数据，转换为发送方IP字符串并解码
            inet_ntoa_r(((struct sockaddr_in *)&source_addr_anotc)->sin_addr, 
                        addr_str_anotc, sizeof(addr_str_anotc) - 1);
            //anotc_data_decode(rx_buffer_anotc);
            // printf("成功接收地面站数据\n"); // 调试用，可根据需要启用
        }

        // 关闭并重启套接字
        if (sock_anotc != -1) {
            printf("关闭地面站套接字并重启\n");
            shutdown(sock_anotc, 0); // 优雅关闭套接字
            close(sock_anotc);       // 释放套接字资源
            sock_anotc = -1;         // 重置为无效值，防止重复关闭
        }

    }

    vTaskDelete(NULL); // 删除任务（理论上不会到达此处）
}

/**
 * @brief 遥控器UDP数据接收任务
 * @param pvParameters 任务参数（未使用）
 * @note  持续监听遥控器UDP套接字，接收并处理数据，错误时重启套接字
 */
static void udp_rc_read_task(void *pvParameters) {
    (void)pvParameters; // 避免未使用参数警告

    while (1) {
        // 接收数据循环
        while (1) {
            socklen_t socklen = sizeof(source_addr_rc);
            int len = recvfrom(sock_rc, rx_buffer_rc, sizeof(rx_buffer_rc) - 1, 0,
                               (struct sockaddr *)&source_addr_rc, &socklen);
            if (len < 0) {
                printf("遥控器数据接收失败\n");
                break; // 接收失败，跳出内层循环以重启套接字
            }

            // 成功接收数据，转换为发送方IP字符串并解码
            inet_ntoa_r(((struct sockaddr_in *)&source_addr_rc)->sin_addr, 
                        addr_str_rc, sizeof(addr_str_rc) - 1);
            //rc_data_decode(rx_buffer_rc);
            // printf("成功接收遥控器数据\n"); // 调试用，可根据需要启用
        }

        // 关闭并重启套接字
        if (sock_rc != -1) {
            printf("关闭遥控器套接字并重启\n");
            shutdown(sock_rc, 0); // 优雅关闭套接字
            close(sock_rc);       // 释放套接字资源
            sock_rc = -1;         // 重置为无效值，防止重复关闭
        }

    }

    vTaskDelete(NULL); // 删除任务（理论上不会到达此处）
}

/**
 * @brief 向地面站发送UDP数据
 * @param data 指向要发送的数据缓冲区
 * @param len2 数据长度（字节）
 * @note  发送数据并更新地面站连接状态
 */
void UDP_write_anotc(const uint8_t *data, uint8_t len2) {
    if (data == NULL || len2 == 0) {
        anotc_state = false; // 输入无效，标记未连接
        return;
    }

    int err = sendto(sock_anotc, data, len2, 0, (struct sockaddr *)&source_addr_anotc, sizeof(source_addr_anotc));
    anotc_state = (err >= 0); // 发送成功则标记为已连接，否则未连接
}

/**
 * @brief 向遥控器发送UDP数据
 * @param data 指向要发送的数据缓冲区
 * @param len2 数据长度（字节）
 * @note  发送数据并更新遥控器连接状态
 */
void UDP_write_rc(const uint8_t *data, uint8_t len2) {
    if (data == NULL || len2 == 0) {
        rc_state = false; // 输入无效，标记未连接
        return;
    }

    int err = sendto(sock_rc, data, len2, 0, (struct sockaddr *)&source_addr_rc, sizeof(source_addr_rc));
    rc_state = (err >= 0); // 发送成功则标记为已连接，否则未连接
}

/**
 * @brief 初始化UDP通信模块
 * @note  初始化地面站和遥控器的套接字，并创建接收任务
 */
void UDP_init(void) {
    // 初始化套接字
    socket_anotc_init(); // 地面站UDP套接字
    socket_rc_init();    // 遥控器UDP套接字

    // 创建接收任务
    xTaskCreate(udp_anotc_read_task, "udp_anotc_read_task", 8192, (void *)AF_INET, 5, NULL);
    xTaskCreate(udp_rc_read_task, "udp_rc_read_task", 8192, (void *)AF_INET, 8, NULL);
}