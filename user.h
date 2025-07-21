/******************************************************************************
;  *       @型号                   : MC32F7361
;  *       @创建日期               : 2021.12.21
;  *       @公司/作者              : SINOMCU-FAE
;  *       @晟矽微技术支持         : 2048615934
;  *       @晟矽微官网             : http://www.sinomcu.com/
;  *       @版权                   : 2021 SINOMCU公司版权所有.
;  *---------------------- 建议 ---------------------------------
;  *                   变量定义时使用全局变量
******************************************************************************/
#ifndef USER
#define USER
#include "mc32-common.h"
#include "MC32F7361.h"

/*****************************************************************
;       Function : Define variables
;*****************************************************************/

#define u8 unsigned char
#define u16 unsigned int
#define u32 unsigned long int
#define uint8_t unsigned char
#define uint16_t unsigned int
#define uint32_t unsigned long int

#define DEF_SET_BIT0 0x01
#define DEF_SET_BIT1 0x02
#define DEF_SET_BIT2 0x04
#define DEF_SET_BIT3 0x08
#define DEF_SET_BIT4 0x10
#define DEF_SET_BIT5 0x20
#define DEF_SET_BIT6 0x40
#define DEF_SET_BIT7 0x80

#define DEF_CLR_BIT0 0xFE
#define DEF_CLR_BIT1 0xFD
#define DEF_CLR_BIT2 0xFB
#define DEF_CLR_BIT3 0xF7
#define DEF_CLR_BIT4 0xEF
#define DEF_CLR_BIT5 0xDF
#define DEF_CLR_BIT6 0xBF
#define DEF_CLR_BIT7 0x7F

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))

#define USE_MY_DEBUG 0
#define USE_MY_DEBUG_PIN 0 // 是否使用仿真板的引脚来代替实际使用的引脚，只在测试时使用

#if USE_MY_DEBUG_PIN

#define LED_1_PIN
#define LED_2_PIN

#else

#define LED_1_PIN P17D
#define LED_2_PIN P00D

#endif // #if USE_MY_DEBUG_PIN

#define LED_3_PIN P01D
#define LED_4_PIN P03D
#define LED_5_PIN P04D

// LED点亮或关闭时，对应的引脚电平
#define LED_ON_LEVEL 1
#define LED_OFF_LEVEL 0

//===============Field Protection Variables===============
u8 abuf;
u8 statusbuf;

//===============Global Function===============

//============Define  Flag=================
typedef union
{
    unsigned char byte;
    struct
    {
        u8 bit0 : 1;
        u8 bit1 : 1;
        u8 bit2 : 1;
        u8 bit3 : 1;
        u8 bit4 : 1;
        u8 bit5 : 1;
        u8 bit6 : 1;
        u8 bit7 : 1;
    } bits;
} bit_flag;
volatile bit_flag flag1;
volatile bit_flag flag2;
#define flag_is_recved_data flag1.bits.bit0         // 是否收到了红外数据，并且没有做处理
#define last_level_in_ir_pin flag1.bits.bit1        // 在红外接收对应的中断函数中，表示上次引脚对应的电平
#define flag_is_recv_ir_repeat_code flag1.bits.bit2 // 是否收到了红外的重复码

#define flag_is_charging_adjust_time_come flag1.bits.bit3       // 调节充电的时间到来
#define flag_is_in_setting_mode flag1.bits.bit4                 // 是否处于设置模式
#define flag_is_in_struction_mode flag1.btis.bit5               // 是否处于指示模式
#define flag_led_struction_mode_exit_times_come flag1.bits.bit6 // 退出指示灯指示模式的时间到来
#define flag_is_led_off_enable flag_bits.bit7                   // 标志位，是否要回到 led_off 模式

#define flag_is_led_1_enable flag2.bits.bit0 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_2_enable flag2.bits.bit1 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_3_enable flag2.bits.bit2 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_4_enable flag2.bits.bit3 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_5_enable flag2.bits.bit4 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮

#define last_level_in_ir_pin flag2.bits.bit5 // 红外接收脚，上一次检测到的电平，在定时器中断内使用

#define LED_1_ON()                    \
    {                                 \
        do                            \
        {                             \
            flag_is_led_1_enable = 1; \
        } while (0);                  \
    }
#define LED_1_OFF()                   \
    {                                 \
        do                            \
        {                             \
            flag_is_led_1_enable = 0; \
        } while (0);                  \
    }
#define LED_2_ON()                    \
    {                                 \
        do                            \
        {                             \
            flag_is_led_2_enable = 1; \
        } while (0);                  \
    }
#define LED_2_OFF()                   \
    {                                 \
        do                            \
        {                             \
            flag_is_led_2_enable = 0; \
        } while (0);                  \
    }
#define LED_3_ON()                    \
    {                                 \
        do                            \
        {                             \
            flag_is_led_3_enable = 1; \
        } while (0);                  \
    }
#define LED_3_OFF()                   \
    {                                 \
        do                            \
        {                             \
            flag_is_led_3_enable = 0; \
        } while (0);                  \
    }
#define LED_4_ON()                    \
    {                                 \
        do                            \
        {                             \
            flag_is_led_4_enable = 1; \
        } while (0);                  \
    }
#define LED_4_OFF()                   \
    {                                 \
        do                            \
        {                             \
            flag_is_led_4_enable = 0; \
        } while (0);                  \
    }
#define LED_5_ON()                    \
    {                                 \
        do                            \
        {                             \
            flag_is_led_5_enable = 1; \
        } while (0);                  \
    }
#define LED_5_OFF()                   \
    {                                 \
        do                            \
        {                             \
            flag_is_led_5_enable = 0; \
        } while (0);                  \
    }

// 定义当前充电阶段
enum
{
    CUR_CHARGE_PHASE_NONE,
    CUR_CHARGE_PHASE_TRICKLE_CHARGE,                            // 电池电量低，涓流充电
    CUR_CHARGE_PHASE_NORMAL_CHARGE,                             // 电池电量正常，恒功率充电
    CUR_CHARGE_PHASE_TRICKLE_CHARGE_WHEN_APPROACH_FULLY_CHARGE, // 接近满电，进行涓流充电
    CUR_CHARGE_PHASE_FULLY_CHARGE,                              // 满电，此时需要等充电端断电
};

// 定义当前控制充电的PWM状态
enum
{
    CUR_CHARGING_PWM_STATUS_NONE = 0x00,
    CUR_CHARGING_PWM_STATUS_LOW_FEQ,  // pwm 低频率
    CUR_CHARGING_PWM_STATUS_HIGH_FRQ, // pwm 高频率
};

enum
{
    ADC_PIN_NONE = 0x00,
    ADC_PIN_DETECT_CHARGE,  // 检测充电分压后的电压（1/11分压）
    ADC_PIN_DETECT_BATTERY, // 检测电池分压后的电压（1/2分压）
    ADC_PIN_DETECT_CURRENT, // 检测充电电流的引脚
};

enum
{
    ADC_REF_2_0_VOL = 0x00, // adc使用2.0V参考电压
    ADC_REF_3_0_VOL,        // adc使用3.0V参考电压
};

extern void delay_ms(u16 xms);
// 毫秒级延时 (误差：在1%以内，1ms、10ms、100ms延时的误差均小于1%)
// 前提条件：FCPU = FHOSC / 8
void delay_ms(u16 xms)
{
    while (xms)
    {
        // u16 i = 290; // 这里不能换成u8类型，会严重影响延时
        u16 i = 289; // 这里不能换成u8类型，会严重影响延时
        while (i--)
        {
            Nop();
        }
        xms--; // 把 --操作放在while()判断条件外面，更节省空间

        // __asm;
        // clrwdt; // 喂狗
        // __endasm;
    }
}

// #if USE_MY_DEBUG
#define DEBUG_PIN P22D
#if 0  // 以下程序约占用81字节空间
// 通过一个引脚输出数据(发送一次约400ms)
// #define DEBUG_PIN P22D
void send_data_msb(u32 send_data)
{
    // 先发送格式头
    // __set_input_pull_up(); // 高电平
    DEBUG_PIN = 1;
    delay_ms(15);
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(7); //

    for (u8 i = 0; i < 32; i++)
    {
        if ((send_data >> (32 - 1 - i)) & 0x01)
        {
            // 如果要发送逻辑1
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(10); //
        }
        else
        {
            // 如果要发送逻辑0
            // __set_input_pull_up();  	   // 高电平
            DEBUG_PIN = 1;
            delay_ms(5); //
            // __set_output_open_drain(); // 低电平
            DEBUG_PIN = 0;
            delay_ms(5); //
        }
    }

    // 最后，设置为低电平
    // __set_output_open_drain(); // 低电平
    DEBUG_PIN = 0;
    delay_ms(1);
    DEBUG_PIN = 1;
    delay_ms(1);
    DEBUG_PIN = 0;
}
#endif // #if USE_MY_DEBUG

#endif

/**************************** end of file *********************************************/
