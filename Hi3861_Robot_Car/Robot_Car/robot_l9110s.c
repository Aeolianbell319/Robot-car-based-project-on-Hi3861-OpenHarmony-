/*
 * L9110S双H桥电机驱动控制程序
 * 功能：控制两个直流电机实现小车的前进、后退、左转、右转、停止等动作
 * 硬件：L9110S电机驱动芯片 + 两个直流减速电机
 * 控制方式：PWM调速控制
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
#include "iot_pwm.h"
#include "hi_pwm.h"

// GPIO引脚定义 - 用于电机控制
#define GPIO0 0                         // GPIO0引脚 - 左轮前进PWM
#define GPIO1 1                         // GPIO1引脚 - 左轮后退PWM  
#define GPIO9 9                         // GPIO9引脚 - 右轮前进PWM
#define GPIO10 10                       // GPIO10引脚 - 右轮后退PWM
#define GPIOFUNC 0                      // GPIO功能选择

// PWM参数定义
#define PWM_DUTY_MAX 8000               // 最大占空比 (100%对应8000)
#define PWM_FREQ 8000                   // PWM频率 8KHz

// GPIO-PWM功能映射定义
#define IO_NAME_GPIO_0 0                // GPIO0引脚编号
#define IO_NAME_GPIO_1 1                // GPIO1引脚编号
#define IO_NAME_GPIO_9 9                // GPIO9引脚编号
#define IO_NAME_GPIO_10 10              // GPIO10引脚编号

// PWM功能复用定义
#define IO_FUNC_GPIO_0_PWM3_OUT HI_IO_FUNC_GPIO_0_PWM3_OUT    // GPIO0复用为PWM3输出
#define IO_FUNC_GPIO_1_PWM4_OUT HI_IO_FUNC_GPIO_1_PWM4_OUT    // GPIO1复用为PWM4输出
#define IO_FUNC_GPIO_9_PWM0_OUT HI_IO_FUNC_GPIO_9_PWM0_OUT    // GPIO9复用为PWM0输出
#define IO_FUNC_GPIO_10_PWM1_OUT HI_IO_FUNC_GPIO_10_PWM1_OUT  // GPIO10复用为PWM1输出

// 小车运动速度控制参数
unsigned short SPEED_TURN = 6000;      // 转弯速度 (占空比值)
unsigned short SPEED_FORWARD = 6000;   // 前进速度 (占空比值)
unsigned short SPEED_BACKWARD = 5000;   // 后退速度 (占空比值)

/**
 * @brief PWM初始化函数
 * @note 初始化用于电机控制的PWM通道和GPIO引脚
 *       配置GPIO复用功能，将GPIO引脚映射到对应的PWM输出
 *       
 * GPIO-PWM映射关系：
 * - GPIO0  -> PWM3 (左轮前进)
 * - GPIO1  -> PWM4 (左轮后退)  
 * - GPIO9  -> PWM0 (右轮前进)
 * - GPIO10 -> PWM1 (右轮后退)
 */
void pwm_init(){
    // 初始化PWM相关GPIO引脚
    IoTGpioInit(IO_NAME_GPIO_0);
    IoTGpioInit(IO_NAME_GPIO_1);
    IoTGpioInit(IO_NAME_GPIO_9);
    IoTGpioInit(IO_NAME_GPIO_10);

    // 设置GPIO复用功能为PWM输出
    hi_io_set_func(IO_NAME_GPIO_0, IO_FUNC_GPIO_0_PWM3_OUT);   // 左轮前进PWM
    hi_io_set_func(IO_NAME_GPIO_9, IO_FUNC_GPIO_9_PWM0_OUT);   // 右轮前进PWM
    hi_io_set_func(IO_NAME_GPIO_1, IO_FUNC_GPIO_1_PWM4_OUT);   // 左轮后退PWM
    hi_io_set_func(IO_NAME_GPIO_10, IO_FUNC_GPIO_10_PWM1_OUT); // 右轮后退PWM

    // 初始化PWM端口
    hi_pwm_init(HI_PWM_PORT_PWM3);  // 左轮前进PWM端口
    hi_pwm_init(HI_PWM_PORT_PWM4);  // 左轮后退PWM端口
    hi_pwm_init(HI_PWM_PORT_PWM0);  // 右轮前进PWM端口
    hi_pwm_init(HI_PWM_PORT_PWM1);  // 右轮后退PWM端口
}

/**
 * @brief 停止所有PWM输出
 * @note 停止所有电机相关的PWM信号输出，使电机停止转动
 */
void pwm_stop(){
    hi_pwm_stop(HI_PWM_PORT_PWM3);  // 停止左轮前进PWM
    hi_pwm_stop(HI_PWM_PORT_PWM4);  // 停止左轮后退PWM
    hi_pwm_stop(HI_PWM_PORT_PWM0);  // 停止右轮前进PWM
    hi_pwm_stop(HI_PWM_PORT_PWM1);  // 停止右轮后退PWM
}

/**
 * @brief GPIO控制函数
 * @param gpio GPIO引脚编号
 * @param value GPIO输出电平值 (IOT_GPIO_VALUE0 或 IOT_GPIO_VALUE1)
 * @note 将指定GPIO设置为输出模式并设置输出电平
 */
void gpio_control (unsigned int  gpio, IotGpioValue value) {
    hi_io_set_func(gpio, GPIOFUNC);             // 设置GPIO功能为普通GPIO
    IoTGpioSetDir(gpio, IOT_GPIO_DIR_OUT);      // 设置GPIO为输出方向
    IoTGpioSetOutputVal(gpio, value);           // 设置GPIO输出电平值
}

/**
 * @brief 小车前进函数
 * @note 控制两个电机同时正转，实现小车前进运动
 *       左轮使用PWM4通道，右轮使用PWM1通道
 */
void car_forward(void) {
    pwm_stop();  // 先停止所有PWM输出
    // 启动左右轮前进PWM，使用SPEED_FORWARD速度和最大占空比
    hi_pwm_start(HI_PWM_PORT_PWM4, SPEED_FORWARD, PWM_DUTY_MAX);  // 左轮前进
    hi_pwm_start(HI_PWM_PORT_PWM1, SPEED_FORWARD, PWM_DUTY_MAX);  // 右轮前进
}

/**
 * @brief 小车后退函数
 * @note 控制两个电机同时反转，实现小车后退运动
 *       左轮使用PWM3通道，右轮使用PWM0通道
 */
void car_backward(void) {
    pwm_stop();  // 先停止所有PWM输出
    // 启动左右轮后退PWM，使用SPEED_BACKWARD速度和最大占空比
    hi_pwm_start(HI_PWM_PORT_PWM3, SPEED_BACKWARD, PWM_DUTY_MAX);  // 左轮后退
    hi_pwm_start(HI_PWM_PORT_PWM0, SPEED_BACKWARD, PWM_DUTY_MAX);  // 右轮后退
}

/**
 * @brief 小车右转函数
 * @note 控制左轮前进、右轮前进，但右轮速度较慢，实现向右转弯
 *       通过差速转向实现：右轮高速前进，左轮低速前进
 */
void car_right(void) {
    pwm_stop();  // 先停止所有PWM输出
    // 右轮全速前进，左轮慢速前进，实现右转
    hi_pwm_start(HI_PWM_PORT_PWM0, SPEED_FORWARD, PWM_DUTY_MAX);   // 右轮前进(全速)
    hi_pwm_start(HI_PWM_PORT_PWM4, SPEED_TURN, PWM_DUTY_MAX);     // 左轮前进(慢速)
}

/**
 * @brief 小车左转函数
 * @note 控制右轮前进、左轮前进，但左轮速度较慢，实现向左转弯
 *       通过差速转向实现：左轮高速前进，右轮低速前进
 */
void car_left(void) {
    pwm_stop();  // 先停止所有PWM输出
    // 左轮全速前进，右轮慢速前进，实现左转
    hi_pwm_start(HI_PWM_PORT_PWM3, SPEED_FORWARD, PWM_DUTY_MAX);   // 左轮前进(全速) 
    hi_pwm_start(HI_PWM_PORT_PWM1, SPEED_TURN, PWM_DUTY_MAX);     // 右轮前进(慢速)
}

/**
 * @brief 小车停止函数
 * @note 停止所有PWM输出，使所有电机停止转动
 */
void car_stop(void) {
    pwm_stop();  // 停止所有PWM输出，电机停止
}
