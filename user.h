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
#define USE_MY_DEBUG_PIN 1 // 是否使用仿真板的引脚来代替实际使用的引脚，只在测试时使用

#if USE_MY_DEBUG_PIN

#define LED_1_PIN P13D
#define LED_2_PIN P14D

#else

#define LED_1_PIN P16D
#define LED_2_PIN P17D

#endif // #if USE_MY_DEBUG_PIN

#define LED_3_PIN P00D
#define LED_4_PIN P03D
#define LED_5_PIN P04D

// LED点亮或关闭时，对应的引脚电平
#define LED_ON_LEVEL 1
#define LED_OFF_LEVEL 0

// 定义红外按键键值
enum
{
    IR_KEY_NONE = 0,

    /* 大摇控器的按键 */
    IR_KEY_ON = 0x62,
    IR_KEY_OFF = 0x22,

    IR_KEY_RED = 0xA2,                     // 大摇控器的红色按键
    IR_KEY_FULL_BRIGHTNESS = 0xC2,         // 全亮
    IR_KEY_HALF_BRIGHTNESS = 0xB0,         // 半亮
    IR_KEY_BRIGHTNESS_ADD_OR_NUM_4 = 0xE0, // 亮度加，也是小遥控器的数字4
    IR_KEY_BRIGHTNESS_SUB_OR_NUM_2 = 0x90, // 亮度减，也是小遥控器的数字2
    // IR_KEY_M = 0x10,               // M

    IR_KEY_AUTO_OR_NUM_5 = 0x98, // 自动模式 ，也是小遥控器的数字5
    IR_KEY_3H_OR_NUM_3 = 0xA8,   // 3H，也是小遥控器的数字3
    IR_KEY_5H_OR_M1 = 0x68,      // 5H，也是小遥控器的M1
    IR_KEY_8H_OR_M3 = 0x18,      // 8H，也是小遥控器的M3

    /* 小遥控器的按键，有些键值是跟大摇控器一样的，都归类为大摇控器的按键 */
    IR_KEY_SET = 0xE2,   // SET 模式设置
    IR_KEY_NUM_1 = 0x02, // 数字1
    // IR_KEY_NUM_2 = 0x90, // 数字2
    // IR_KEY_NUM_3 = 0xA8, // 数字3
    // IR_KEY_NUM_4 = 0xE0, // 数字4
    // IR_KEY_NUM_5 = 0x98, // 数字5
    // IR_KEY_M1 = 0x68,
    IR_KEY_M2 = 0x30,
    // IR_KEY_M3 = 0x18,
};

enum
{
    ADC_PIN_NONE = 0x00,
    ADC_PIN_DETECT_CHARGE,  // 检测充电分压后的电压（1/10分压） P12 AN7
    ADC_PIN_DETECT_BATTERY, // 检测电池分压后的电压（1/2分压）  P05 AN4
};

// 定义当前充电的状态
enum
{
    CUR_CHARGE_STATUS_NONE = 0x00, // 未在充电
    CUR_CHARGE_STATUS_IN_CHARGING, // 正在充电
    // CUR_CHARGE_STATUS_PRE_CHARGING, // 准备进入充电
    // CUR_CHARGE_STATUS_TRICKLE_CHARGE_WHEN_BAT_IS_LOW,       // 电池电量低，进行涓流充电
    // CUR_CHARGE_STATUS_CHARGE_NORMALLY,                      // 电池正常充电
    // CUR_CHARGE_STATUS_TRICKLE_CHARGE_WHEN_BAT_IS_NEAR_FULL, // 电池快满电，进行涓流充电
};

// 定义当前控制充电的PWM状态
enum
{
    CUR_CHARGING_PWM_STATUS_NONE = 0x00,
    CUR_CHARGING_PWM_STATUS_LOW_FEQ,  // pwm 低频率
    CUR_CHARGING_PWM_STATUS_HIGH_FRQ, // pwm 高频率
};

/*
    在充电一端检测到可以给电池充电的电压
    样机是检测到充电输入端大于4.9V，使能给电池的充电

    adc使用 内部 2.0V作为参考电压
    单片机检测脚检测到 大于 0.49V （4.9V 经过 1/10分压后）,
    使能给电池的充电

    对应的ad值 1003.52
*/
#define ADC_VAL_ENABLE_IN_CHARGE_END ((u16)1004)
/*
    在充电一端检测到 断开给电池充电的电压
    样机是检测到充电输入端小于4V，断开给电池的充电

    adc使用 内部 2.0V作为参考电压
    单片机检测脚检测到 小于 0.40V （4.0V 经过 1/10分压后）,
    断开给电池的充电

    对应的ad值 819.2
*/
#define ADC_VAL_DISABLE_IN_CHARGE_END ((u16)819)
/*
    电池低电量时，对应的ad值
    样机是低于 2.85V ，进行涓流充电

    adc使用 内部 2.0V作为参考电压
    单片机检测脚检测到 小于 1.425V （2.85V 经过 1/2分压后），
    进行涓流充电

    对应ad值 2918.4
*/
#define ADC_VAL_BAT_IS_LOW ((u16)2918)
/*
    电池快满电时，对应的ad值
    样机是大于 3.55V ，进行涓流充电（只测试了一次，认为快满电3.6V前应该进行涓流充电）

    adc使用 内部 2.0V作为参考电压
    单片机检测脚检测到 大于 1.775 V （3.55 V 经过 1/2分压后），
    进行涓流充电

    对应ad值 3635.2
*/
#define ADC_VAL_BAT_IS_NEAR_FULL ((u16)3635)
/*
    充电期间，电池充满电时对应的ad值
    目前只知道样机充满电后的电池电压约为3.6V

    adc使用 内部 2.0V作为参考电压
    单片机检测脚检测到 大于 1.8 V （3.6 V 经过 1/2分压后），
    断开给电池的充电，不使能对应的PWM

    对应的ad值为 3686.4
*/
#define ADC_VAL_BAT_IS_FULL ((u16)3686)

// #define IR_RECV_PIN P15D // 红外信号接收引脚
#define IR_RECV_PIN P13D // 红外信号接收引脚

volatile u8 ir_data; // 存放红外接收到的数据
// volatile u8 ir_type; // 区分不同地址的遥控器
volatile u16 adc_val; // 存放得到的ad值

// volatile u8 cur_charge_status = CUR_CHARGE_STATUS_NONE;
volatile u8 cur_charge_status;
//  static u8 cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_NONE;
// u8 cur_charging_pwm_status;

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

#define flag_is_led_1_enable flag2.bits.bit0 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_2_enable flag2.bits.bit1 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_3_enable flag2.bits.bit2 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_4_enable flag2.bits.bit3 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮
#define flag_is_led_5_enable flag2.bits.bit4 // led  是否使能，0--不使能，led 熄灭，1--使能，led 点亮

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

// enum
// {

// };

extern void delay_ms(u16 xms);

// 毫秒级延时 (误差：在1%以内，1ms、10ms、100ms延时的误差均小于1%)
// 前提条件：FCPU = FHOSC / 8
void delay_ms(u16 xms)
{
    while (xms)
    {
        u16 i = 252; // 这里不能换成u8类型，会严重影响延时
        while (i--)
        {
            Nop();
        }
        xms--; // 把 --操作放在while()判断条件外面，更节省空间

        __asm;
        clrwdt; // 喂狗
        __endasm;
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
