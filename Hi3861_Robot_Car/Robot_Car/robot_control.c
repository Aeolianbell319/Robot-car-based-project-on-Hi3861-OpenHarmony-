/*
 * 鸿蒙WiFi小车控制程序
 * 功能：通过按键控制小车模式切换，支持停止、寻迹、避障、远程控制等模式
 * 包含超声波避障、红外寻迹、UDP远程控制等功能
 */

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

// 鸿蒙系统相关头文件
#include "ohos_init.h"
#include "cmsis_os2.h"

// GPIO和硬件控制相关头文件
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "hi_adc.h"
#include "iot_errno.h"

// 小车控制相关头文件
#include "robot_control.h"
#include "robot_l9110s.h"
#include "udp_control.h"

// GPIO和硬件配置宏定义
#define GPIO5 5                         // 按键GPIO引脚号
#define FUNC_GPIO 0                     // GPIO功能选择
#define ADC_TEST_LENGTH (20)            // ADC采样次数
#define VLT_MIN (100)                   // 最小电压值
#define OLED_FALG_ON ((unsigned char)0x01)   // OLED开启标志
#define OLED_FALG_OFF ((unsigned char)0x00)  // OLED关闭标志

// 全局变量定义
unsigned short  g_adc_buf[ADC_TEST_LENGTH] = { 0 };        // ADC缓存数组
unsigned short  g_gpio5_adc_buf[ADC_TEST_LENGTH] = { 0 };  // GPIO5 ADC缓存数组
unsigned int  g_gpio5_tick = 0;                            // GPIO5按键时间戳，用于消抖
unsigned char   g_car_control_mode = 0;                    // 小车控制模式
unsigned char   g_car_speed_control = 0;                   // 小车速度控制
unsigned int  g_car_control_demo_task_id = 0;              // 小车控制任务ID
unsigned char   g_car_status = CAR_STOP_STATUS;            // 小车当前状态
int udp_thread_created = 0;                                // UDP线程创建标志

// 外部函数声明
extern float GetDistance(void);         // 获取超声波测距距离
extern void trace_module(void);         // 红外寻迹模块
extern void car_backward(void);         // 小车后退
extern void car_forward(void);          // 小车前进  
extern void car_left(void);             // 小车左转
extern void car_right(void);            // 小车右转
extern void car_stop(void);             // 小车停止
extern void engine_turn_left(void);     // 舵机左转
extern void engine_turn_right(void);    // 舵机右转
extern void regress_middle(void);       // 舵机归中
extern unsigned int MOVING_STATUS;      // 小车运动状态

/**
 * @brief 按键开关初始化
 * @note 配置GPIO5为输入模式，用于按键检测
 */
void switch_init(void)
{
    IoTGpioInit(5);                      // 初始化GPIO5
    hi_io_set_func(5, 0);               // 设置GPIO5功能为普通GPIO
    IoTGpioSetDir(5, IOT_GPIO_DIR_IN);   // 设置GPIO5为输入模式
    hi_io_set_pull(5, 1);               // 设置GPIO5上拉
}

/**
 * @brief 按键中断响应函数 - 小车模式切换
 * @note 通过按键切换小车的四种工作模式：停止->寻迹->避障->远控->停止
 *       使用时间戳进行按键消抖处理
 */
void gpio5_isr_func_mode(void)
{
    printf("gpio5_isr_func_mode start\n");
    unsigned int tick_interval = 0;
    unsigned int current_gpio5_tick = 0; 

    // 获取当前时间戳
    current_gpio5_tick = hi_get_tick();
    tick_interval = current_gpio5_tick - g_gpio5_tick;
    
    // 按键消抖处理，避免重复触发
    if (tick_interval < KEY_INTERRUPT_PROTECT_TIME) {  
        return NULL;
    }
    g_gpio5_tick = current_gpio5_tick;

    // 小车模式状态机切换
    if (g_car_status == CAR_STOP_STATUS) {                
        g_car_status = CAR_TRACE_STATUS;                 // 切换到寻迹模式       
        printf("trace\n");
    } else if (g_car_status == CAR_TRACE_STATUS) {       
        g_car_status = CAR_OBSTACLE_AVOIDANCE_STATUS;   // 切换到避障模式
        printf("ultrasonic\n");
    } else if (g_car_status == CAR_OBSTACLE_AVOIDANCE_STATUS) {                           
        g_car_status = CAR_CONTROL_STATUS;                 // 切换到远程控制模式
        printf("control\n");
    } else if (g_car_status == CAR_CONTROL_STATUS) {    
        g_car_status = CAR_STOP_STATUS;                    // 切换到停止模式
        printf("stop\n");
    }
}

/**
 * @brief 获取GPIO5电压值并处理按键事件
 * @param param 参数（未使用）
 * @return 无返回值
 * @note 通过ADC读取GPIO5电压值，根据不同电压范围执行不同操作：
 *       - 0.01V~0.3V：执行模式切换
 *       - 0.6V~1.5V：调整小车前进速度
 */
unsigned char get_gpio5_voltage(void *param)
{
    int i;
    unsigned short data;
    unsigned int ret;
    unsigned short vlt;
    float voltage;
    float vlt_max = 0;
    float vlt_min = VLT_MIN;

    hi_unref_param(param);
    // 清空ADC缓存数组
    memset_s(g_gpio5_adc_buf, sizeof(g_gpio5_adc_buf), 0x0, sizeof(g_gpio5_adc_buf));
    
    // 连续采样ADC_TEST_LENGTH次
    for (i = 0; i < ADC_TEST_LENGTH; i++) {
        ret = hi_adc_read(HI_ADC_CHANNEL_2, &data, HI_ADC_EQU_MODEL_4, HI_ADC_CUR_BAIS_DEFAULT, 0xF0); 
        // ADC_Channal_2 自动识别模式，4次平均算法模式
        if (ret != IOT_SUCCESS) {
            printf("ADC Read Fail\n");
            return  NULL;
        }    
        g_gpio5_adc_buf[i] = data;
    }

    // 计算电压值并找出最大最小值
    for (i = 0; i < ADC_TEST_LENGTH; i++) {  
        vlt = g_gpio5_adc_buf[i]; 
        voltage = (float)vlt * 1.8 * 4 / 4096.0;  
        // vlt * 1.8 * 4 / 4096.0为将码字转换为电压值
        vlt_max = (voltage > vlt_max) ? voltage : vlt_max;
        vlt_min = (voltage < vlt_min) ? voltage : vlt_min;
    }
    
    // 根据电压范围执行不同操作
    if (vlt_max > 0.01 && vlt_max < 0.3) {
        // 电压范围0.01V~0.3V：执行模式切换
        gpio5_isr_func_mode();
    } else if(vlt_max > 0.6 && vlt_max < 1.5){
        // 电压范围0.6V~1.5V：调整小车前进速度
        if (SPEED_FORWARD <= 7000) {
            SPEED_FORWARD += 1000;  // 增加速度
        } else {
            SPEED_FORWARD = 4000;   // 重置为初始速度
        }
    }
}



/**
 * @brief 按键中断监控初始化
 * @note 注册GPIO5的中断处理函数，设置为下降沿触发
 */
void interrupt_monitor(void)
{
    unsigned int  ret = 0;
    g_gpio5_tick = hi_get_tick();  // 初始化时间戳
    // 注册GPIO5中断服务函数，下降沿触发
    ret = IoTGpioRegisterIsrFunc(GPIO5, IOT_INT_TYPE_EDGE, IOT_GPIO_EDGE_FALL_LEVEL_LOW, get_gpio5_voltage, NULL);
    if (ret == IOT_SUCCESS) {
        printf(" register gpio5\r\n");
    }
}

/**
 * @brief 舵机转向检测功能 - 确定最佳转向方向
 * @return 返回转向方向：CAR_TURN_LEFT 或 CAR_TURN_RIGHT
 * @note 通过控制舵机左右转动，测量两侧障碍物距离，选择距离更远的方向
 */
static unsigned int engine_go_where(void)
{
    float left_distance = 0;
    float right_distance = 0;
    
    // 舵机往左转动测量左边障碍物的距离
    engine_turn_left();
    hi_sleep(100);              // 等待舵机转动到位
    left_distance = GetDistance();
    hi_sleep(100);

    // 舵机归中
    regress_middle();
    hi_sleep(100);

    // 舵机往右转动测量右边障碍物的距离
    engine_turn_right();
    hi_sleep(100);              // 等待舵机转动到位
    right_distance = GetDistance();
    hi_sleep(100);

    // 舵机归中
    regress_middle();
    
    // 选择距离更远的方向作为转向方向
    if (left_distance > right_distance) {
        return CAR_TURN_LEFT;   // 左侧距离更远，选择左转
    } else {
        return CAR_TURN_RIGHT;  // 右侧距离更远，选择右转
    }
}

/**
 * @brief 小车避障行为控制函数
 * @param distance 前方障碍物距离（厘米）
 * @note 避障逻辑：
 *       1. 距离>=20cm：继续前进
 *       2. 距离<20cm：停止->后退0.5s->测距判断->选择转向->转向0.75s->停止
 */
static void car_where_to_go(float distance)
{
    if (distance < DISTANCE_BETWEEN_CAR_AND_OBSTACLE) {
        // 距离小于安全距离，执行避障操作
        car_stop();         // 立即停止
        MOVING_STATUS = 0;
        hi_sleep(500);
        
        car_backward();     // 后退避让
        MOVING_STATUS = 5;
        hi_sleep(500);
        
        car_stop();         // 停止后退
        MOVING_STATUS = 0;
        
        // 通过舵机测距选择最佳转向方向
        unsigned int ret = engine_go_where();
        printf("ret is %d\r\n", ret);
        
        if (ret == CAR_TURN_LEFT) {
            car_left();     // 左转避障
            MOVING_STATUS = 2;
            hi_sleep(750);  // 转向持续时间
        } else if (ret == CAR_TURN_RIGHT) {
            car_right();    // 右转避障
            MOVING_STATUS = 1;
            hi_sleep(750);  // 转向持续时间
        }
        car_stop();         // 转向完成后停止
        MOVING_STATUS = 0;
    } else {
        // 距离足够，继续前进
        car_forward();
        MOVING_STATUS = 3;
    } 
}

/**
 * @brief 小车避障模式控制函数
 * @note 在避障模式下持续运行，通过超声波传感器获取距离并执行避障逻辑
 *       当模式切换时自动退出并将舵机归中
 */
void car_mode_control_func(void)
{
    pwm_init();                 // 初始化PWM，用于舵机控制
    float m_distance = 0.0;
    regress_middle();          // 舵机归中
    
    while (1) {
        // 检查是否还在避障模式
        if (g_car_status != CAR_OBSTACLE_AVOIDANCE_STATUS) {
            printf("car_mode_control_func 1 module changed\n");
            regress_middle();   // 退出前舵机归中
            break;
        }

        // 获取前方物体的距离
        m_distance = GetDistance();
        // printf("m_distance = %f\n",m_distance);
        
        // 根据距离执行避障逻辑
        car_where_to_go(m_distance);
        hi_sleep(20);          // 短暂延时，避免CPU占用过高
    }
}

/**
 * @brief 小车主控制任务函数
 * @param param 任务参数（未使用）
 * @return 无返回值
 * @note 主任务循环，根据g_car_status状态执行不同的控制逻辑：
 *       - CAR_STOP_STATUS: 停止状态
 *       - CAR_OBSTACLE_AVOIDANCE_STATUS: 超声波避障模式
 *       - CAR_TRACE_STATUS: 红外寻迹模式
 *       - CAR_CONTROL_STATUS: UDP远程控制模式
 */
void *RobotCarTestTask(void* param)
{
    printf("switch\r\n");
    switch_init();              // 初始化按键开关
    interrupt_monitor();        // 初始化按键中断监控

    // 在启动时创建UDP线程用于远程控制
    if (!udp_thread_created) {
        start_udp_thread();
        udp_thread_created = 1;
        printf("UDP thread started at startup\r\n");
    }

    while (1) {
        // 根据当前状态执行相应的控制逻辑
        switch (g_car_status) {
            case CAR_STOP_STATUS:
                car_stop();     // 停止状态：小车停止
                break;
            case CAR_OBSTACLE_AVOIDANCE_STATUS:
                car_mode_control_func();    // 避障模式：执行超声波避障
                break;
            case CAR_TRACE_STATUS:
                trace_module();             // 寻迹模式：执行红外寻迹
                break;
            case CAR_CONTROL_STATUS:
                // 远控模式：不在这里执行car_stop()，避免与UDP控制指令冲突
                // UDP控制指令会直接控制小车，这里只需要等待
                break;
            default:
                break;
        }
        IoTWatchDogDisable();   // 关闭看门狗
        osDelay(20);            // 任务延时20ms
    }
}

/**
 * @brief 小车演示程序入口函数
 * @note 创建小车控制主任务，配置任务属性并启动
 */
void RobotCarDemo(void)
{
    osThreadAttr_t attr;

    // 配置任务属性
    attr.name = "RobotCarTestTask";     // 任务名称
    attr.attr_bits = 0U;               // 任务属性位
    attr.cb_mem = NULL;                // 控制块内存
    attr.cb_size = 0U;                 // 控制块大小
    attr.stack_mem = NULL;             // 栈内存
    attr.stack_size = 10240;           // 栈大小10KB
    attr.priority = 25;                // 任务优先级

    // 创建小车控制主任务
    if (osThreadNew(RobotCarTestTask, NULL, &attr) == NULL) {
        printf("[Ssd1306TestDemo] Falied to create RobotCarTestTask!\n");
    }
}

// 系统启动时自动初始化小车演示程序
APP_FEATURE_INIT(RobotCarDemo);