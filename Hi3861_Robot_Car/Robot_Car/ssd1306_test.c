#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <string.h>  // 添加string.h头文件

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "hi_io.h"
#include "hi_time.h"
#include "ssd1306.h"
#include "iot_i2c.h"
#include "iot_watchdog.h"
#include "robot_control.h"
#include "iot_errno.h"
#include <unistd.h>

#define OLED_I2C_BAUDRATE 400*1000
#define GPIO13 13
#define GPIO14 14
#define FUNC_SDA 6
#define FUNC_SCL 6
extern unsigned char g_car_status;
extern unsigned short SPEED_FORWARD;
extern unsigned int MOVING_STATUS;

// 定义状态模式的名称，用于标题栏显示
char mode_name[4][15] = {
    "STOP MODE",
    "TRACE MODE",
    "CONTROL MODE",
    "AVOID MODE"
};

// 移动状态描述
char moving_status[6][20] = {
    "Stopping...",
    "Turing right...",
    "Turing left...",
    "Moving forward...",
    "Obstacles ahead...",
    "Moving backward..."
};

// 绘制美化的边框和标题栏
void DrawUIFrame(const char* title) {
    // 绘制外边框
    ssd1306_DrawRectangle(0, 0, 127, 63, White);
    
    // 绘制标题栏
    ssd1306_DrawRectangle(2, 2, 125, 12, White);
    
    // 显示标题文字居中
    int title_len = strlen(title);
    int title_pos = (128 - title_len * 7) / 2;  // 估算居中位置，Font_7x10中7是字符宽度
    if (title_pos < 5) title_pos = 5;
    
    ssd1306_SetCursor(title_pos, 3);
    ssd1306_DrawString(title, Font_7x10, White);
    
    // 绘制分隔线
    for (int i = 2; i < 126; i += 4) {
        ssd1306_DrawPixel(i, 15, White);
    }
}

// 显示电池图标
void DrawBatteryIcon(uint8_t level) {
    // level范围为0-4，表示电池电量等级
    uint8_t bat_x = 105;
    uint8_t bat_y = 3;
    
    // 绘制电池外框
    ssd1306_DrawRectangle(bat_x, bat_y, bat_x + 18, bat_y + 8, White);
    ssd1306_DrawRectangle(bat_x + 18, bat_y + 2, bat_x + 20, bat_y + 6, White);
    
    // 根据电量级别填充电池
    for (int i = 0; i < level; i++) {
        int x_pos = bat_x + 2 + i * 4;
        for (int j = 0; j < 3; j++) {
            ssd1306_DrawLine(x_pos, bat_y + 2, x_pos, bat_y + 6, White);
            x_pos++;
        }
    }
}

void Ssd1306TestTask(void* arg)
{
    (void) arg;
    hi_io_set_func(GPIO13, FUNC_SDA);
    hi_io_set_func(GPIO14, FUNC_SCL);
    IoTI2cInit(0, OLED_I2C_BAUDRATE);

    IoTWatchDogDisable();

    usleep(20*1000);
    ssd1306_Init();
    ssd1306_Fill(Black);
    
    // 启动动画
    ssd1306_SetCursor(0, 0);
    ssd1306_DrawString("Hello OpenHarmony!", Font_7x10, White);
    
    // 绘制启动画面边框
    ssd1306_DrawRectangle(0, 0, 127, 63, White);
    ssd1306_DrawRectangle(5, 15, 122, 48, White);
    
    ssd1306_SetCursor(15, 25);
    ssd1306_DrawString("ROBOT CAR SYSTEM", Font_7x10, White);
    
    ssd1306_SetCursor(25, 38);
    ssd1306_DrawString("STARTING...", Font_7x10, White);

    uint32_t start = HAL_GetTick();
    ssd1306_UpdateScreen();
    uint32_t end = HAL_GetTick();
    printf("ssd1306_UpdateScreen time cost: %d ms.\r\n", end - start);
    osDelay(1000); // 显示启动画面时间延长

    // 循环显示电池充电动画
    for (int i = 0; i <= 4; i++) {
        ssd1306_Fill(Black);
        ssd1306_DrawRectangle(0, 0, 127, 63, White);
        ssd1306_SetCursor(10, 25);
        ssd1306_DrawString("INITIALIZING...", Font_7x10, White);
        DrawBatteryIcon(i);
        ssd1306_UpdateScreen();
        osDelay(200);
    }
    
    osDelay(300);

    while (1)
    {
        ssd1306_Fill(Black);
        char title[20] = {0};
        char status_info[30] = {0};
        char speed_str[20] = {0};

        // 根据当前模式设置标题
        if(g_car_status == CAR_OBSTACLE_AVOIDANCE_STATUS) {
            strcpy(title, mode_name[3]); // "AVOID MODE"
        } else if (g_car_status == CAR_STOP_STATUS) {
            strcpy(title, mode_name[0]); // "STOP MODE"
        } else if (g_car_status == CAR_TRACE_STATUS) {
            strcpy(title, mode_name[1]); // "TRACE MODE"
        } else if (g_car_status == CAR_CONTROL_STATUS) {
            strcpy(title, mode_name[2]); // "CONTROL MODE"
        }
        
        // 绘制UI框架
        DrawUIFrame(title);
        
        // 显示电池图标（模拟75%电量）
        DrawBatteryIcon(3);
        
        // 显示速度信息
        sprintf(speed_str, "Speed: %u", SPEED_FORWARD);
        ssd1306_SetCursor(10, 20);
        ssd1306_DrawString(speed_str, Font_7x10, White);
        
        // 绘制速度条指示器
        int speed_bar_width = (SPEED_FORWARD * 80) / 1000; // 假设最大速度为1000
        if (speed_bar_width > 80) speed_bar_width = 80;
        
        ssd1306_DrawRectangle(10, 32, 90, 38, White);
        for (int i = 0; i < speed_bar_width; i++) {
            ssd1306_DrawLine(10 + i, 33, 10 + i, 37, White);
        }
        
        // 显示移动状态
        ssd1306_SetCursor(10, 45);
        if (MOVING_STATUS >= 0 && MOVING_STATUS <= 5) {
            ssd1306_DrawString(moving_status[MOVING_STATUS], Font_7x10, White);
            
            // 状态图标
            if (MOVING_STATUS == 0) { // 停止
                ssd1306_DrawRectangle(100, 45, 110, 55, White);
            } else if (MOVING_STATUS == 1) { // 右转
                ssd1306_DrawCircle(105, 50, 5, White);
                ssd1306_DrawLine(110, 50, 105, 45, White);
            } else if (MOVING_STATUS == 2) { // 左转
                ssd1306_DrawCircle(105, 50, 5, White);
                ssd1306_DrawLine(100, 50, 105, 45, White);
            } else if (MOVING_STATUS == 3) { // 前进
                ssd1306_DrawLine(105, 45, 105, 55, White);
                ssd1306_DrawLine(105, 45, 100, 50, White);
                ssd1306_DrawLine(105, 45, 110, 50, White);
            } else if (MOVING_STATUS == 4) { // 障碍物
                ssd1306_DrawLine(100, 45, 110, 55, White);
                ssd1306_DrawLine(100, 55, 110, 45, White);
            } else if (MOVING_STATUS == 5) { // 后退
                ssd1306_DrawLine(105, 45, 105, 55, White);
                ssd1306_DrawLine(105, 55, 100, 50, White);
                ssd1306_DrawLine(105, 55, 110, 50, White);
            }
        }
        
        ssd1306_UpdateScreen();
        osDelay(10);
    }
    
}

void Ssd1306TestDemo(void)
{
    osThreadAttr_t attr;

    attr.name = "Ssd1306TestTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 4096;
    attr.priority = 25;

    if (osThreadNew(Ssd1306TestTask, NULL, &attr) == NULL) {
        printf("[Ssd1306TestDemo] Failed to create Ssd1306TestTask!\n");
    }
}
APP_FEATURE_INIT(Ssd1306TestDemo);
