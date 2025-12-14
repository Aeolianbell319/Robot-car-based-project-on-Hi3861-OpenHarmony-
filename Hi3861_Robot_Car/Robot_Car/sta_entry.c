/*
 * WiFi STA模式连接和网络通信程序
 * 功能：
 * 1. 配置WiFi STA模式连接到指定热点
 * 2. 获取IP地址并建立网络连接
 * 3. 启动MQTT通信服务
 * 4. 启动UDP控制服务用于小车远程控制
 */

#include <stdio.h>
#include <unistd.h>

// 鸿蒙系统相关头文件
#include "ohos_init.h"
#include "cmsis_os2.h"

// WiFi和网络相关头文件
#include "hi_wifi_api.h"
#include "lwip/ip_addr.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"

// 小车控制相关头文件
#include "robot_control.h"
#include "udp_control.h"

// WiFi连接配置
#define WIFI_SSID		"Zzz"           // WiFi热点名称
#define WIFI_PASSWD		"lk111111"      // WiFi热点密码

// WiFi资源配置
#define APP_INIT_VAP_NUM    2           // 虚拟AP数量
#define APP_INIT_USR_NUM    2           // 用户数量

// 全局变量
volatile char start_wifi_connected_flg = 0;     // WiFi连接状态标志，0-未连接，1-已连接
static struct netif *g_lwip_netif = NULL;      // LWIP网络接口指针

/**
 * @brief 清除网络接口的IP、网关和子网掩码
 * @param pst_lwip_netif LWIP网络接口指针
 * @note 将网络接口的IP地址重置为0.0.0.0，用于断开连接时清理网络配置
 */
void hi_sta_reset_addr(struct netif *pst_lwip_netif)
{
    ip4_addr_t st_gw;       // 网关地址
    ip4_addr_t st_ipaddr;   // IP地址
    ip4_addr_t st_netmask;  // 子网掩码
    
    printf("%s %d \r\n", __FILE__, __LINE__);
    if (pst_lwip_netif == NULL) {
        printf("hisi_reset_addr::Null param of netdev\r\n");
        return;
    }

    // 设置所有地址为0.0.0.0
    IP4_ADDR(&st_gw, 0, 0, 0, 0);
    IP4_ADDR(&st_ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&st_netmask, 0, 0, 0, 0);

    // 应用地址配置到网络接口
    netifapi_netif_set_addr(pst_lwip_netif, &st_ipaddr, &st_netmask, &st_gw);
}

/**
 * @brief WiFi事件回调函数
 * @param hisi_event WiFi事件结构体指针
 * @note 处理WiFi连接过程中的各种事件：
 *       - 扫描完成事件
 *       - 连接成功事件：启动DHCP获取IP
 *       - 断开连接事件：停止DHCP并清理网络配置
 *       - WPS超时事件
 */
void wifi_wpa_event_cb(const hi_wifi_event *hisi_event)
{
    if (hisi_event == NULL)
        return;

    switch (hisi_event->event) {
        case HI_WIFI_EVT_SCAN_DONE:
            printf("WiFi: Scan results available\n");
            break;
        case HI_WIFI_EVT_CONNECTED:
            printf("WiFi: Connected\n");
            netifapi_dhcp_start(g_lwip_netif);  // 启动DHCP客户端获取IP地址
            start_wifi_connected_flg = 1;       // 设置连接成功标志
            break;
        case HI_WIFI_EVT_DISCONNECTED:
            printf("WiFi: Disconnected\n");
            netifapi_dhcp_stop(g_lwip_netif);   // 停止DHCP客户端
            hi_sta_reset_addr(g_lwip_netif);    // 清除网络配置
            break;
        case HI_WIFI_EVT_WPS_TIMEOUT:
            printf("WiFi: wps is timeout\n");
            break;
        default:
            break;
    }
}

/**
 * @brief 启动WiFi STA连接
 * @return 0-成功，-1-失败
 * @note 配置WiFi连接参数并发起连接请求
 *       使用WPA2-PSK加密方式连接到指定的WiFi热点
 */
int hi_wifi_start_connect(void)
{
    int ret;
    errno_t rc;
    hi_wifi_assoc_request assoc_req = {0};

    // 复制SSID到连接请求结构体
    rc = memcpy_s(assoc_req.ssid, HI_WIFI_MAX_SSID_LEN + 1, WIFI_SSID, strlen(WIFI_SSID));
    if (rc != EOK) {
        printf("%s %d \r\n", __FILE__, __LINE__);
        return -1;
    }

    // 设置WiFi加密方式为WPA2-PSK
    assoc_req.auth = HI_WIFI_SECURITY_WPA2PSK;

    // 复制WiFi密码
    memcpy(assoc_req.key, WIFI_PASSWD, strlen(WIFI_PASSWD));

    // 发起WiFi连接请求
    ret = hi_wifi_sta_connect(&assoc_req);
    if (ret != HISI_OK) {
        printf("%s %d \r\n", __FILE__, __LINE__);
        return -1;
    }
    printf("%s %d \r\n", __FILE__, __LINE__);
    return 0;
}

/**
 * @brief 启动WiFi STA模式
 * @return 0-成功，-1-失败
 * @note 完整的WiFi STA启动流程：
 *       1. 初始化WiFi模块
 *       2. 启动STA模式
 *       3. 注册事件回调函数
 *       4. 获取网络接口
 *       5. 发起WiFi连接
 */
int hi_wifi_start_sta(void)
{
    int ret;
    char ifname[WIFI_IFNAME_MAX_SIZE + 1] = {0};
    int len = sizeof(ifname);
    const unsigned char wifi_vap_res_num = APP_INIT_VAP_NUM;
    const unsigned char wifi_user_res_num = APP_INIT_USR_NUM;

    printf("%s %d \r\n", __FILE__, __LINE__);

    // 初始化WiFi模块，配置VAP和用户资源数量
    ret = hi_wifi_init(wifi_vap_res_num, wifi_user_res_num);
    if (ret != HISI_OK) {
        printf("%s %d \r\n", __FILE__, __LINE__);
        // return -1; // 允许继续执行，某些情况下可能已经初始化
    }

    printf("%s %d \r\n", __FILE__, __LINE__);
    // 启动STA模式并获取网络接口名称
    ret = hi_wifi_sta_start(ifname, &len);
    if (ret != HISI_OK) {
        printf("%s %d \r\n", __FILE__, __LINE__);
        return -1;
    }

    // 注册WiFi事件回调函数，用于接收连接状态等事件
    ret = hi_wifi_register_event_callback(wifi_wpa_event_cb);
    if (ret != HISI_OK) {
        printf("register wifi event callback failed\n");
    }

    // 获取LWIP网络接口，用于后续的IP操作
    g_lwip_netif = netifapi_netif_find(ifname);
    if (g_lwip_netif == NULL) {
        printf("%s: get netif failed\n", __FUNCTION__);
        return -1;
    }

    // 发起WiFi连接
    ret = hi_wifi_start_connect();
    if (ret != 0) {
        printf("%s %d \r\n", __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

/**
 * @brief 停止WiFi STA模式
 * @note 清理WiFi资源，停止STA模式并去初始化WiFi模块
 */
void hi_wifi_stop_sta(void)
{
    int ret;

    // 停止STA模式
    ret = hi_wifi_sta_stop();
    if (ret != HISI_OK) {
        printf("failed to stop sta\n");
    }

    // 去初始化WiFi模块
    ret = hi_wifi_deinit();
    if (ret != HISI_OK) {
        printf("failed to deinit wifi\n");
    }

    // 清空网络接口指针
    g_lwip_netif = NULL;
}

/**
 * @brief MQTT测试任务函数
 * @param arg 任务参数（未使用）
 * @note 预留的MQTT功能入口，目前为空实现
 */
void mqtt_test_task(void *arg)
{
    arg = arg;
    
    // 预留：开始进入MQTT测试
    // mqtt_test();
}

/**
 * @brief MQTT服务入口函数
 * @note 创建MQTT测试任务，配置任务属性并启动
 */
void MqttEntry(void)
{
    osThreadAttr_t attr1;
    
    // 配置MQTT任务属性
    attr1.name = "mqtt_task";       // 任务名称
    attr1.attr_bits = 0U;          // 任务属性位
    attr1.cb_mem = NULL;           // 控制块内存
    attr1.cb_size = 0U;            // 控制块大小
    attr1.stack_mem = NULL;        // 栈内存
    attr1.stack_size = 4096;       // 栈大小4KB
    attr1.priority = 26;           // 任务优先级

    // 创建MQTT测试任务
    if (osThreadNew((osThreadFunc_t)mqtt_test_task, NULL, &attr1) == NULL) {
        printf("[MqttEntry] Falied to create mqtt_test_task!\n");
    }
}


/**
 * @brief MQTT测试主线程函数
 * @param argv 线程参数（未使用）
 * @note 完整的网络服务启动流程：
 *       1. 启动WiFi STA连接
 *       2. 等待WiFi连接成功
 *       3. 启动MQTT服务
 *       4. 启动UDP控制服务
 */
void mqtt_test_thread(void * argv)
{
    argv = argv;

    // 启动WiFi STA模式连接
    hi_wifi_start_sta();

    // 等待WiFi连接成功，轮询检查连接状态
    while(start_wifi_connected_flg == 0)
    {
        usleep(300000);  // 延时300ms后再次检查
    }

    sleep(3);            // WiFi连接成功后等待3秒，确保网络稳定
    MqttEntry();         // 启动MQTT服务
    start_udp_thread();  // 启动UDP控制线程，用于小车远程控制
}



/**
 * @brief STA示例程序入口函数
 * @note 系统启动的主入口，创建WiFi配置主线程
 */
void StaExampleEntry(void)
{
    osThreadAttr_t attr;

    // 配置WiFi配置线程属性
    attr.name = "wifi_config_thread";   // 线程名称
    attr.attr_bits = 0U;               // 线程属性位
    attr.cb_mem = NULL;                // 控制块内存
    attr.cb_size = 0U;                 // 控制块大小
    attr.stack_mem = NULL;             // 栈内存
    attr.stack_size = 4096;            // 栈大小4KB
    attr.priority = 36;                // 线程优先级

    // 创建WiFi配置主线程
    if (osThreadNew((osThreadFunc_t)mqtt_test_thread, NULL, &attr) == NULL) {
        printf("[LedExample] Falied to create LedTask!\n");
    }
}

// 系统运行时自动调用STA示例入口函数
SYS_RUN(StaExampleEntry);