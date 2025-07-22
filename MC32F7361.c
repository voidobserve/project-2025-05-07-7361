/******************************************************************************
;  *       @型号                 : MC32F7361
;  *       @创建日期             : 2021.12.21
;  *       @公司/作者            : SINOMCU-FAE
;  *       @晟矽微技术支持       : 2048615934
;  *       @晟矽微官网           : http://www.sinomcu.com/
;  *       @版权                 : 2021 SINOMCU公司版权所有.
;  *----------------------摘要描述---------------------------------

******************************************************************************/

#include "user.h"

// 控制充电的pwm，初始化/配置
#define PWM_CTL_FOR_CHARGING_CONFIG()                            \
    do                                                           \
    {                                                            \
        PWMCR3 |= (0x01 << 2); /* timer0 pwm输出端口，选择P02 */ \
    } while (0);

// 控制充电的pwm，设置为高频率，104KHz，0%占空比输出
#define PWM_CTL_FOR_CHARGING_SET_HIGH_FEQ() timer0_pwm_set_high_feq()
// 控制充电的pwm，设置为低频率，20KHz，固定占空比输出
#define PWM_CTL_FOR_CHARGING_SET_LOW_FEQ() timer0_pwm_set_low_feq()
// 使能 控制充电的pwm
#define PWM_CTL_FOR_CHARGING_ENABLE()                                  \
    do                                                                 \
    {                                                                  \
        T0CR |= ((0x01 << 7) | (0x01 << 6)); /* 打开定时器，使能PWM */ \
    } while (0);

// 不使能 控制充电的pwm
#define PWM_CTL_FOR_CHARGING_DISABLE() timer0_pwm_disable()
// 设置 控制充电的pwm 的占空比
// #define PWM_CTL_FOR_CHARGING_SET_DUTY(duty) timer0_pwm_set_duty(duty)

// 控制充电的pwm 对应的寄存器
#define PWM_CTL_FOR_CHARGING_DUTY_REG T0DATA

// 控制充电的pwm 的频率 对应的寄存器
#define PWM_CTL_FOR_CHARGING_FEQ_REG T0LOAD
// 控制充电的pwm高频率时，对应的寄存器的最大值
#define PWM_CTL_FOR_CHARGING_HIGH_FEQ_VAL ((u8)(165 - 1))

// 控制灯光的pwm
#define PWM_CTL_FOR_LIGHTS() timer2_pwm_config()
#define LIGHT_TIMER_FEQ_VAL ((u8)(255 - 1))
#define LIGHT_SET_PWM_DUTY(pwm_duty_val) \
    do                                   \
    {                                    \
        T2DATA = pwm_duty_val;           \
    } while (0);

#define LIGHT_ON()                                                     \
    do                                                                 \
    {                                                                  \
        T2CR |= ((0x01 << 7) | (0x01 << 6)); /* 打开定时器，使能PWM */ \
    } while (0);

#define LIGHT_OFF()                                                             \
    do                                                                          \
    {                                                                           \
        T2CR &= ~((0x01 << 7) | (0x01 << 6)); /* 不使能定时器，不使能PWM输出 */ \
    } while (0);

// 红外信号接收引脚
#define IR_RECV_PIN P16D

#define MIN_PWM_DUTY_IN_LOW_POWER (8)      // 快满电而降低充电功率时，最低的占空比，单位：1%
#define MIN_PWM_DUTY_IN_TRICKLE_CHARGE (5) // 涓流充电时，最低的占空比，单位：1%

// =================================================================
// 红外接收相关变量                                                //
// =================================================================
volatile u8 ir_data = 0;
// volatile bit flag_is_recv_ir_repeat_code = 0; （在 volatile bit_flag flagx 中定义）
// volatile bit flag_is_recved_data = 0; （在 volatile bit_flag flagx 中定义）

// =================================================================
// 充电控制相关变量                                                 //
// =================================================================
volatile u16 bat_adc_val;      // 电池电压检测脚采集到的ad值
volatile u16 charging_adc_val; // 充电电压检测脚采集的ad值
volatile u16 current_adc_val;  // 充电电流检测脚采集的ad值
// volatile bit flag_is_charging_adjust_time_come = 0; // 调节充电的时间到来 （在 volatile bit_flag flagx 中定义）
volatile u8 cur_charging_pwm_status; // 控制充电的PWM状态（上电清除RAM之后，默认为 CUR_CHARGING_PWM_STATUS_NONE ）
volatile u8 cur_charge_phase;        // 记录当前充电阶段（上电清除RAM之后，默认为 CUR_CHARGE_PHASE_NONE ）

// =================================================================
// 指示灯控制相关变量                                               //
// =================================================================
volatile u8 cur_initial_discharge_gear; // 初始放电挡位（需要记忆）
volatile u8 cur_discharge_rate;         // 初始放电速率（需要记忆）
volatile u8 cur_led_mode;               // 当前的LED模式
volatile u8 cur_led_gear;               // 当前led挡位
volatile u8 last_led_gear;              // 上次led挡位（只能在刚上电时清零赋初始值）
volatile u8 cur_led_gear_in_charging;   // 充电指示，对应的挡位

// volatile bit flag_is_in_setting_mode = 0;              // 是否处于设置模式 （在 volatile bit_flag flagx 中定义）
volatile u8 flag_led_setting_mode_exit_times_come = 0; // 标志位，led退出设置模式的时间到来
volatile u16 led_setting_mode_exit_times_cnt = 0;      // 特殊的LED模式，退出时间计数

// volatile bit flag_is_in_struction_mode = 0;               // 是否处于指示模式 （在 volatile bit_flag flagx 中定义）
// volatile bit flag_led_struction_mode_exit_times_come = 0; // 退出指示灯指示模式的时间到来 （在 volatile bit_flag flagx 中定义）
volatile u16 led_struction_mode_exit_times_cnt = 0; // 退出指示灯指示模式时间计数

// volatile bit flag_led_gear_update_times_come = 0; // 指示灯状态更新的时间到来 （在 volatile bit_flag flagx 中定义）

// 标志位，是否要回到 led_off 模式
// volatile bit flag_is_led_off_enable = 0; （在 volatile bit_flag flagx 中定义）

#if 1
// =================================================================
// 主灯光控制相关变量                                               //
// =================================================================
volatile u32 light_adjust_time_cnt = 0;    // 调节灯光的时间计数，暂定为每1s加一
volatile u8 light_ctl_phase_in_rate_1 = 1; // 在放电速率M1时，使用到的变量，在计算公式里面用作系数，每次唤醒时需要初始化为1

// TODO：3260使用16位寄存器，7361使用8位寄存器，要进行适配修改
volatile u16 cur_light_pwm_duty_val = 0;                     // 当前灯光对应的占空比值
volatile u8 flag_is_light_adjust_time_come = 0;              // 调节灯光的时间到来，目前为1s
volatile u8 flag_is_light_pwm_duty_val_adjust_time_come = 0; // 灯光占空比值调节时间到来

volatile u8 flag_is_ctl_light_blink = 0; // 是否控制主灯光闪烁
volatile u8 light_ctl_blink_times = 0;   // 要控制主灯光闪烁的次数
/*
    是否要在设置模式期间关闭主灯光

    如果已经关灯，在设置模式期间，主灯闪烁完成后，直接关灯
*/
// volatile bit flag_allow_light_in_setting_mode = 0; （在 volatile bit_flag flagx 中定义）

// 是否要缓慢调节主灯光的占空比
// volatile bit flag_is_adjust_light_slowly = 0; （在 volatile bit_flag flagx 中定义）
volatile u16 expect_light_pwm_duty_val = 0; // 期望缓慢调节到的、主灯光对应的占空比值

// 是否开启了定时关机功能：
// volatile bit flag_is_auto_shutdown_enable = 0; （在 volatile bit_flag flagx 中定义）
volatile u32 light_auto_shutdown_time_cnt = 0; // 定时关机功能的定时器计数，单位：ms
// volatile bit flag_is_auto_shutdown_times_come = 0; // 定时关机的时间到来 （在 volatile bit_flag flagx 中定义）
#endif

// 短按减小灯光亮度，对应各个挡位亮度的占空比值
const u16 light_pwm_sub_table[9] = {
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 8367 / 10000), // 83.67 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 7371 / 10000), // 73.71 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 6375 / 10000), // 63.75 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 5379 / 10000), // 53.79 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 4383 / 10000), // 43.83 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 3387 / 10000), // 33.87 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 2391 / 10000), // 23.91 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 1395 / 10000), // 13.95 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 478 / 10000),  // 4.78 %
};

// 短按增加灯光亮度，对应各个挡位亮度的占空比值
const u16 light_pwm_add_table[9] = {
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 478 / 10000),  // 4.78 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 1474 / 10000), // 14.74 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 2470 / 10000), // 24.70 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 3466 / 10000), // 34.66 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 4462 / 10000), // 44.62 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 5458 / 10000), // 54.58 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 6554 / 10000), // 65.54 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 7450 / 10000), // 74.50 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 8367 / 10000), // 83.67 %
};

const u16 light_pwm_duty_init_val_table[5] = {
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 8367 / 10000), // 83.67 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 7411 / 10000), // 74.11 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 6455 / 10000), // 64.55 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 5698 / 10000), // 56.98 %
    (u16)((u32)LIGHT_TIMER_FEQ_VAL * 4980 / 10000), // 49.80 %
};

/************************************************
;  *    @函数名          : CLR_RAM
;  *    @说明            : 清RAM
;  *    @输入参数        :
;  *    @返回参数        :
;  ***********************************************/
void CLR_RAM(void)
{
    for (FSR0 = 0; FSR0 < 0xff; FSR0++)
    {
        INDF0 = 0x00;
    }

    FSR0 = 0xFF;
    INDF0 = 0x00;
}

/************************************************
;  *    @函数名            : IO_Init
;  *    @说明              : IO初始化
;  *    @输入参数          :
;  *    @返回参数          :
;  ***********************************************/
void IO_Init(void)
{
    IOP0 = 0x00;   // io口数据位
    OEP0 = 0x3F;   // io口方向 1:out  0:in
    PUP0 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP0 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P0ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP1 = 0x00;   // io口数据位
    OEP1 = 0xFF;   // io口方向 1:out  0:in
    PUP1 = 0x00;   // io口上拉电阻   1:enable  0:disable
    PDP1 = 0x00;   // io口下拉电阻   1:enable  0:disable
    P1ADCR = 0x00; // io类型选择  1:模拟输入  0:通用io

    IOP2 = 0x00; // io口数据位
    OEP2 = 0x0F; // io口方向 1:out  0:in
    PUP2 = 0x00; // io口上拉电阻   1:enable  0:disable
    PDP2 = 0x00; // io口下拉电阻   1:enable  0:disablea

    PMOD = 0x00;  // P00、P01、P13 io端口值从寄存器读，推挽输出
    DRVCR = 0x80; // 普通驱动
}

/*
    pwm0 控制充电的PWM
    将 pwm0 配置为 低频，对应电池电压较低时，涓流充电的 PWM
*/
void timer0_pwm_set_low_feq(void)
{
    // 20KHz，18.15%
    T0CR = (0x01 << 7) | (0x01 << 6); // 使能定时器，使能PWM，时钟源使用CPU，不分频
    T0LOAD = 215 - 1;
    T0DATA = 39;
}

/*
    pwm0 控制充电的PWM
    将 PWM0 配置为 高频，对应正常充电的PWM
*/
void timer0_pwm_set_high_feq(void)
{
    PWMCR1 |= (0x01 << 4); // FTMR 选择 FHOSC

    // 105KHz，0% (测试时用 非0% 的占空比)
    // T0CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 3) | (0x01 << 0); // 使能定时器，使能PWM, 时钟源选择FTMR，时钟源2分频
    // T0LOAD = 164 - 1;

    // 104KHz，0 %
    T0CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 3) | (0x01 << 0); // 使能定时器，使能PWM, 时钟源选择FTMR，时钟源2分频
    T0LOAD = PWM_CTL_FOR_CHARGING_HIGH_FEQ_VAL;
    T0DATA = 0;
}

// void timer0_pwm_enable(void)
// {
//     T0CR |= ((0x01 << 7) | (0x01 << 6)); // 打开定时器，使能PWM
// }

// 关闭 PWM0 输出，引脚输出低电平
void timer0_pwm_disable(void)
{
    T0CR &= ~((0x01 << 7) | (0x01 << 6)); // 关闭定时器，关闭PWM
    P02D = 0;                             // 端口输出低电平
}

// timer1 100us定时器中断配置
void timer1_config(void)
{
    T1CR |= (0x01 << 7) | (0x01 << 1); // 使能定时器，时钟源选择CPU, 4分频
    T1LOAD = 108 - 1;                  // 100us (从sdk复制过来的会有些误差，这里做了补偿)
    T1IE = 1;
}

/*
    timer2 控制灯光的定时器
    控制灯光的PWM，只使用 NFPWM2
*/
void timer2_pwm_config(void)
{
    // T2CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 2); // 使能定时器，使能PWM, 时钟源选择 CPU，16 分频 -- 测试时使用
    T2CR = (0x01 << 2); // 不使能定时器，不使能PWM, 时钟源选择 CPU，16 分频
    // T2LOAD = 215 - 1;                 //

    // T2LOAD = 255 - 1; // 1.056KHz
    T2LOAD = LIGHT_TIMER_FEQ_VAL; // 1.056KHz

    // T2DATA = 100; /* 使用NFPWM，占空比 == 比较值/周期值 * 100% */
    // T2DATA = 0;

    // PWMCR3 = DEF_SET_BIT6 | DEF_SET_BIT5 | DEF_SET_BIT4; // 使能FPWM,NPWM,正向输出
    PWMCR3 |= (0x01 << 4); // 使能NFPWM2输出波形
}

void timer2_pwm_set_feq(void)
{
    PWMCR1 |= (0x01 << 4); // FTMR 选择 FHOSC

    T2CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 3); // 使能定时器，使能PWM, 时钟源选择FTMR，时钟源不分频
    T2LOAD = 255 - 1;                               //
    T2DATA = 255 - 50;                              /* 由于只使用 NPWM 互补PWM，这里占空比计算要用 周期计数值 减去 比较值，才是互补PWM的占空比 */
}

// void timer2_pwm_enable(void)
// {
//     T2CR &= ~((0x01 << 7) | (0x01 << 6)); // 不使能定时器，不使能PWM输出
// }

// void timer2_pwm_disable(void)
// {
//     T2CR |= (0x01 << 7) | (0x01 << 6); // 不使能定时器，不使能PWM输出
// }

void adc_config(void)
{
    // 检测充电电压得引脚， 外部10K上拉，1K下拉，约 1/11分压
    P14OE = 0; // 输入模式
    P14DC = 1; // 模拟模式

    // 检测电池电压的引脚，电池电压 1/2分压
    P05OE = 0; // 输入模式
    P05DC = 1; // 模拟模式

    // 检测电流的引脚
    P11OE = 0; // 输入模式
    P11DC = 1; // 模拟模式

    ADCR0 = 0x0B; // 12位精度，使能adc
    // ADCR1 = 0x80; // adc转换时钟选择 FHIRC/32，使用內部2.0V参考电压
    ADCR2 = 0x0F; // 采样时间，只能固定是15 个 ADCLK
}

/**
 * @brief 切换adc的参考电压（不单独使用）
 *
 * @param adc_ref_voltage
 *              ADC_REF_2_0_VOL = 0x00, // adc使用2.0V参考电压
                ADC_REF_3_0_VOL,        // adc使用3.0V参考电压
 */
void adc_sel_ref_voltage(u8 adc_ref_voltage)
{
    // ADEN = 0; // 不使能ADC

    if (ADC_REF_2_0_VOL == adc_ref_voltage)
    {
        ADCR1 = (0x01 << 7) | /* ADC转换时钟选择 FHIRC / 256 */
                (0x01 << 6) |
                (0x01 << 5); /* 寄存器【2:0】，参考电压配置寄存器，清零，对应内部 2.0V 参考电压 */
    }
    else if (ADC_REF_3_0_VOL == adc_ref_voltage)
    {
        ADCR1 = (0x01 << 7) | /* ADC转换时钟选择 FHIRC / 256 */
                (0x01 << 6) |
                (0x01 << 5) |
                (0x01 << 0); /* 寄存器【2:0】，参考电压配置寄存器 == 0b 001，对应内部 3.0V 参考电压 */
    }

    // ADEN = 1;    // 使能ADC
    // delay_ms(1); // 等待adc稳定
}

/**
 * @brief 切换检测ad的引脚（函数内部只切换引脚，不切换参考电压）
 *
 * @param adc_pin
 * @return * void
 */
void adc_sel_pin(u8 adc_pin)
{
    if (ADC_PIN_DETECT_CHARGE == adc_pin)
    {
        // P14  AN8    检测充电电压的引脚
        ADCR0 = (0x01 << 7) | /* AN 8 */
                (0x01 << 3) | /* ADC 转换结果为 12 位数据 */
                (0x01 << 1) | /* 不开始ad转换 */
                (0x01 << 0);  /* 使能ADC */
    }
    else if (ADC_PIN_DETECT_BATTERY == adc_pin)
    {
        // P05  AN4    检测电池的电压
        ADCR0 = (0x01 << 6) | /* AN 4 */
                (0x01 << 3) | /* ADC 转换结果为 12 位数据 */
                (0x01 << 1) | /* 不开始ad转换 */
                (0x01 << 0);  /* 使能ADC */
    }
    else if (ADC_PIN_DETECT_CURRENT == adc_pin)
    {
        // P11  AN6     检测电流
        ADCR0 = (0x01 << 6) | (0x01 << 5) | /* AN 6 */
                (0x01 << 3) |               /* ADC 转换结果为 12 位数据 */
                (0x01 << 1) |               /* 不开始ad转换 */
                (0x01 << 0);                /* 使能ADC */
    }

    delay_ms(1); // 等待adc稳定
}

// adc采集+滤波
u16 adc_getval(void)
{
    u16 adc_val_tmp = 0;     // 存放单次采集到的ad值
    u32 adc_val_sum = 0;     // 存放所有采集到的ad值的累加
    u16 get_adcmax = 0;      // 存放采集到的最大的ad值
    u16 get_adcmin = 0xFFFF; // 存放采集到的最小的ad值(初始值为最大值)
    u8 i = 0;
    for (i = 0; i < 20; i++)
    {
        ADEOC = 0; // 写0开始转换
        while (!ADEOC)
            ; // 等待转换完成
        adc_val_tmp = ADRH << 4 | (ADRL & 0x0F);

        if (i < 2)
            continue; // 丢弃前两次采样的
        if (adc_val_tmp > get_adcmax)
            get_adcmax = adc_val_tmp; // 更新当前采集到的最大值
        if (adc_val_tmp < get_adcmin)
            get_adcmin = adc_val_tmp; // 更新当前采集到的最小值
        adc_val_sum += adc_val_tmp;
    }

    adc_val_sum -= get_adcmax;        // 去掉一个最大
    adc_val_sum -= get_adcmin;        // 去掉一个最小
    adc_val_tmp = (adc_val_sum >> 4); // 除以16，取平均值

    return adc_val_tmp;
}

/**
 * @brief 更新电池对应的ad值，内部使用2.0V参考电压
 *
 */
void adc_update_bat_adc_val(void)
{
    adc_sel_ref_voltage(ADC_REF_2_0_VOL);
    adc_sel_pin(ADC_PIN_DETECT_BATTERY);
    bat_adc_val = adc_getval();
}

/**
 * @brief 更新充电对应的ad值
 *
 * @param adc_ref_voltage
 *          ADC_REF_2_0_VOL 使用2.0V作为参考电压
 *          ADC_REF_3_0_VOL 使用3.0V作为参考电压
 */
void adc_update_charge_adc_val(u8 adc_ref_voltage)
{
    adc_sel_ref_voltage(adc_ref_voltage);
    adc_sel_pin(ADC_PIN_DETECT_CHARGE);
    charging_adc_val = adc_getval();
}

/**
 * @brief 更新电流对应的ad值，内部使用3.0V参考电压
 *
 */
void adc_update_current_adc_val(void)
{
    adc_sel_ref_voltage(ADC_REF_3_0_VOL);
    adc_sel_pin(ADC_PIN_DETECT_CURRENT);
    current_adc_val = adc_getval();
}

void sys_init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();
    PWM_CTL_FOR_CHARGING_CONFIG(); // 控制充电的pwm，初始化/配置
    timer1_config();               // timer1 100us中断

    PWM_CTL_FOR_LIGHTS(); // 驱动 主灯光 的pwm

    delay_ms(1); // 等待系统稳定
    GIE = 1;
}

#if 1 // 充电控制

// 充电控制
void charge_handle(void)
{
    if (flag_is_charging_adjust_time_come) // 一定要加入缓慢调节，不能迅速调节，否则会等不到电压稳定
    {
        static u8 pwm_duty = 0; // 单位：1%

        // 检测到电池快满电，是否进入涓流充电的计数
        static u8 trickle_charge_cnt = 0;

        u16 current = 0;        // 充电电流，单位：mA
        u16 voltage_of_bat = 0; // 电池电压，单位：mV
        u32 power = 0;          // 功率，单位：mW 毫瓦
        // u16 pwm_reg = 0;        // 存放要写入到寄存器中的占空比值
        // u8 pwm_reg = 0;         // 存放要写入到寄存器中的占空比值
        u16 expected_power = 0; // 期望功率 ，单位：mW 毫瓦

        flag_is_charging_adjust_time_come = 0; // 清除标志位

        // 如果当前没有在充电
        if (CUR_CHARGE_PHASE_NONE == cur_charge_phase)
        {
            adc_update_charge_adc_val(ADC_REF_2_0_VOL);

            // 如果充电输入电压大于4.9V，使能充电
            if (charging_adc_val >= (u16)((u32)4900 * 4096 / 11 / 2 / 1000))
            {
                cur_charge_phase = CUR_CHARGE_PHASE_TRICKLE_CHARGE; // 刚进入充电，默认是电池电量低对应的涓流充电
            }

            // 如果充电电压未满足使能充电的条件，会进入下面的语句块
            if (CUR_CHARGE_PHASE_NONE == cur_charge_phase)
            {
                return;
            }
        }
        else
        {
            // 如果当前在充电
            adc_update_charge_adc_val(ADC_REF_3_0_VOL);

            // 如果充电电压过大，PWM百分比设置为0，等到电压变小才打开
            // if (charging_adc_val >= (u16)((u32)30000 * 4096 / 11 / 3 / 1000)) // 充电电压超过30V
            if (charging_adc_val >= (u16)((u32)28000 * 4096 / 11 / 3 / 1000)) // 充电电压超过 xx V
            {
                // pwm_reg = 0;
                // TMR1_PWMH = (pwm_reg >> 8) & 0xFF;
                // TMR1_PWML = pwm_reg & 0xFF;
                PWM_CTL_FOR_CHARGING_DUTY_REG = 0;
                return;
            }

            // 如果充电输入电压小于4V，断开充电
            if (charging_adc_val <= (u16)((u32)4000 * 4096 / 11 / 3 / 1000))
            {
                // pwm_reg = 0;
                // TMR1_PWMH = (pwm_reg >> 8) & 0xFF;
                // TMR1_PWML = pwm_reg & 0xFF;
                // timer1_pwm_disable();
                PWM_CTL_FOR_CHARGING_DUTY_REG = 0;
                PWM_CTL_FOR_CHARGING_DISABLE();

                cur_charge_phase = CUR_CHARGE_PHASE_NONE;
                cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_NONE;
                return;
            }

            // 如果已经充满电，直接退出
            if (CUR_CHARGE_PHASE_FULLY_CHARGE == cur_charge_phase)
            {
                return;
            }
        }

        // 进入到这里，说明正在充电，且充电电压在 4V~30V之间，不包括4V和30V

        // 检测电池电压，使用内部2.0V参考电压
        adc_update_bat_adc_val();

        // 刚进入充电，会进入下面这个语句块：
        if (CUR_CHARGE_PHASE_TRICKLE_CHARGE == cur_charge_phase)
        {
            // 如果电池电压小于2.7V，进行涓流充电
            if (bat_adc_val <= (u16)((u32)2700 * 4096 / 2 / 2 / 1000))
            {
                if (CUR_CHARGING_PWM_STATUS_LOW_FEQ != cur_charging_pwm_status)
                {
                    PWM_CTL_FOR_CHARGING_SET_LOW_FEQ(); // 20KHz，固定占空比输出
                    cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_LOW_FEQ;
                }

                return;
            }

            // 如果电池电压不小于2.7V，进行正常充电
            cur_charge_phase = CUR_CHARGE_PHASE_NORMAL_CHARGE;
        }

        // if ((bat_adc_val >= (u16)((u32)3400 * 4096 / 2 / 2 / 1000)) &&
        //     (CUR_CHARGE_PHASE_TRICKLE_CHARGE_WHEN_APPROACH_FULLY_CHARGE != cur_charge_phase))
        // if ((bat_adc_val >= (u16)((u32)(3500 + 100) * 4096 / 2 / 2 / 1000)) && /* 3500 + 100 毫伏，实际测试在 电池电压3.56V，单片机检测脚1.81V都没有进入*/
        //     (CUR_CHARGE_PHASE_TRICKLE_CHARGE_WHEN_APPROACH_FULLY_CHARGE != cur_charge_phase))
        if ((bat_adc_val >= (u16)((u32)(3500 + 50 + 20) * 4096 / 2 / 2 / 1000)) && /* xxx 毫伏，实际测试在 3.53V，单片机检测脚电压1.793V*/
            (CUR_CHARGE_PHASE_TRICKLE_CHARGE_WHEN_APPROACH_FULLY_CHARGE != cur_charge_phase))
        {
            trickle_charge_cnt++;

            if (pwm_duty > MIN_PWM_DUTY_IN_TRICKLE_CHARGE)
            {
                // pwm_duty  -= MIN_PWM_DUTY_IN_TRICKLE_CHARGE; // -=5，调节幅度有点大
                pwm_duty -= 2;
            }

            // 除了检电池电压，还要检占空比，加上这个条件后，约3.55V之后进入涓流充电，进入涓流充电后，测得电池电压3.55V
            if (pwm_duty < 10)
            {
                trickle_charge_cnt++;
            }

            if (trickle_charge_cnt >= 100)
            {
                trickle_charge_cnt = 0;

                // 准备进入涓流充电，设置PWM，样机最小PWM为4.8%
                // pwm_duty = MIN_PWM_DUTY_IN_TRICKLE_CHARGE;
                // pwm_reg = (u32)TIMER1_HIGH_FEQ_PEROID_VAL * pwm_duty / 100; // 最终的占空比值
                // TMR1_PWMH = (pwm_reg >> 8) & 0xFF;
                // TMR1_PWML = pwm_reg & 0xFF;

                // 准备进入涓流充电，设置PWM，样机最小PWM为4.8%
                PWM_CTL_FOR_CHARGING_DUTY_REG = (u8)((u16)MIN_PWM_DUTY_IN_TRICKLE_CHARGE * (u16)PWM_CTL_FOR_CHARGING_HIGH_FEQ_VAL / 100);
                cur_charge_phase = CUR_CHARGE_PHASE_TRICKLE_CHARGE_WHEN_APPROACH_FULLY_CHARGE;
            }
        }
        else
        {
            trickle_charge_cnt = 0;
        }

        // 如果充电阶段已经到了电池接近满电的阶段
        if (CUR_CHARGE_PHASE_TRICKLE_CHARGE_WHEN_APPROACH_FULLY_CHARGE == cur_charge_phase)
        {
            if (bat_adc_val >= (u16)((u32)(3600 + 50) * 4096 / 2 / 2 / 1000))
            {

                // pwm_reg = 0;
                // TMR1_PWMH = (pwm_reg >> 8) & 0xFF;
                // TMR1_PWML = pwm_reg & 0xFF;
                // timer1_pwm_disable();                             // 已经充满电，断开控制充电的PWM

                PWM_CTL_FOR_CHARGING_DUTY_REG = 0;
                PWM_CTL_FOR_CHARGING_DISABLE(); // 已经充满电，断开控制充电的PWM

                cur_charge_phase = CUR_CHARGE_PHASE_FULLY_CHARGE; // 表示已经充满电，接下来需要等充电电压低于4.0V
                cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_NONE;
                return;
            }

            return;
        }

        // ===================================================================
        // 以下都是正常充电对应的控制程序，cur_charge_phase == CUR_CHARGE_PHASE_NORMAL_CHARGE
        // ===================================================================

        // 不是正常充电，提前返回
        if (CUR_CHARGE_PHASE_NORMAL_CHARGE != cur_charge_phase)
        {
            return;
        }

        // 如果PWM未切换到高频
        if (CUR_CHARGING_PWM_STATUS_HIGH_FRQ != cur_charging_pwm_status)
        {
            // pwm_reg = MIN_PWM_DUTY_IN_LOW_POWER;
            // TMR1_PWMH = (pwm_reg >> 8) & 0xFF;
            // TMR1_PWML = pwm_reg & 0xFF;
            // timer1_set_pwm_high_feq();

            // 频率切换为高频，从0%占空比开始
            PWM_CTL_FOR_CHARGING_SET_HIGH_FEQ();
            cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_HIGH_FRQ;
        }

        adc_update_bat_adc_val();
        // 如果电池电压大于 xx V，开始降低功率
        // if (bat_adc_val >= (u16)((u32)3450 * 4096 / 2 / 2 / 1000)) // 3.45V，实际测试是在 3.38V 附近开始降低功率
        // if (bat_adc_val >= (u16)((u32)(3500 + 50) * 4096 / 2 / 2 / 1000)) // 3.55V，实际测试在 3.48V 附近开始降低功率
        // if (bat_adc_val >= (u16)((u32)(3400) * 4096 / 2 / 2 / 1000)) // 在3.36V左右 降低功率（不确定3.36V以下是否就开始降功率，因为电池是从3.3V左右开始充电）
        if (bat_adc_val >= (u16)((u32)(3500 + 50) * 4096 / 2 / 2 / 1000)) //
        {
            // expected_power = 12000;
            // expected_power = 10000; // 功率太高，会直接跳到充满电的阶段（跳到涓流充电，再进入充满电阶段）
            expected_power = (u16)5000; // 充电速度会很慢，1h才提升0.01V~0.02V
        }
        else if (bat_adc_val >= (u16)((u32)(3400) * 4096 / 2 / 2 / 1000)) //
        {
            expected_power = (u16)26500 / 2;
        }
        else // 如果电池电压小于 xx V，按正常的功率进行充电
        {
            expected_power = (u16)26500;
        }

        adc_update_current_adc_val();

        /*
            检测电流引脚检测到的电压 == ad值 / 4096 * 参考电压
            current_adc_val * 3 / 4096

            检测电流的引脚检测到的充电电流 == 检测电流引脚检测到的电压 / 110(运放放大倍数) / 0.005R，
            逐步换成单片机可以计算的形式：
            current_adc_val * 3 / 4096 / 110 / 0.005
            current_adc_val * 3 / 4096 / 110 * 1000 / 5
            current_adc_val * 3 * 1000 / 5 / 4096 / 110
            得到的是以A为单位的电流，需要再转换成以mA为单位：
            current_adc_val * 3 * 1000 * 1000 / 5 / 4096 / 110，计算会溢出，需要再化简
            (u32)current_adc_val * 3 * 1000 * (1000 / 5) / 4096 / 110
            current =  (u32)current_adc_val * 3 * 1000 * (1000 / 5) / 4096 / 110;
        */
        current = (u16)((u32)current_adc_val * 3 * 1000 * (1000 / 5) / 4096 / 76); // 计算电流，单位：mA

        /*
            计算电池电压
            电池电压（mV） == 采集到的ad值 / 4096 * 参考电压 * 分压系数 * 1000（mV）
        */
        voltage_of_bat = (u16)((u32)bat_adc_val * 2 * 1000 * 2 / 4096); // 计算电池电压，单位：mV

        // 如果检测到电流的ad值已经爆表
        if (current_adc_val >= 4095)
        // if (current >= 5400) // 如果电流值已经爆表，超过单片机能检测的值（理论值）：5454.54
        {
            // printf("current overflow\n");
            if (pwm_duty > 0)
            {
                pwm_duty--;
            }
        }
        else //
        {
            // power = voltage_of_charging * current / 1000; // 1000mV * 1000mA == 1000000 (1 Watt)
            // 1000mV * 1000mA == 1000000 (1 Watt)
            if (expected_power != 26500)
            {
                // 0.728V电压，对应的电流是 1915 mA
                if (current < 1916)
                {
                    current = 1;
                }
            }

            power = (u32)voltage_of_bat * current / 1000; // 计算功率，单位：毫瓦

            if (power < expected_power) // 如果当前的功率 小于 限制的功率
            {
                if (pwm_duty < 100) // 100%占空比
                {
                    pwm_duty++;
                }
            }
            else if (power > expected_power) // 如果当前的功率 大于 限制的功率
            {
                if (pwm_duty > MIN_PWM_DUTY_IN_LOW_POWER) // 防止向下溢出
                {
                    pwm_duty--;
                }
            }
            else // power == 目标值，不做调节
            {
            }
        }

        // printf("pwm_duty : %bu %%\n", pwm_duty);
        // pwm_reg = (u32)TIMER1_HIGH_FEQ_PEROID_VAL * pwm_duty / 100; // 最终的占空比值
        //     // printf("pwm_duty :%u\n", pwm_duty);
        // TMR1_PWMH = (pwm_reg >> 8) & 0xFF;
        // TMR1_PWML = pwm_reg & 0xFF;

        PWM_CTL_FOR_CHARGING_DUTY_REG = (u8)((u16)PWM_CTL_FOR_CHARGING_HIGH_FEQ_VAL * pwm_duty / 100);
    }
}
#endif // 充电控制

// 程序占用348
#if 0 // led指示灯控制

// 只能刚上电时调用：
void led_init(void)
{
    led_all_off();

    last_led_gear = 0; // 赋值为初始值
    cur_led_gear_in_charging = 0;
    led_setting_mode_exit_times_cnt = 0; // 清除设置模式下的退出时间计时
    flag_is_in_struction_mode = 0;
}

void led_all_off(void)
{
    LED_1_OFF();
    LED_2_OFF();
    LED_3_OFF();
    LED_4_OFF();
    LED_5_OFF();
}

// 清除led有关的状态
void led_status_clear(void)
{
    led_all_off();

    // 清空设置模式相关的变量
    flag_is_in_setting_mode = 0;
    flag_led_setting_mode_exit_times_come = 0;
    led_setting_mode_exit_times_cnt = 0;

    // 清空指示模式相关的变量
    flag_is_in_struction_mode = 0;
    flag_led_struction_mode_exit_times_come = 0;
    led_struction_mode_exit_times_cnt = 0;

    // flag_is_led_off_enable = 0; // 不能加入这句，如果重复按下SET或红色按键，会导致无法关指示灯
}

/**
 * @brief 更改led_mode
 *
 * @param led_mode
 *              相关定义在 枚举类型 CUR_LED_MODE_XXX 中

 */
void led_mode_alter(u8 led_mode)
{
    led_all_off();

    cur_led_mode = led_mode;
}

/**
 * @brief 在设置模式，设置初始放电挡位或放电速率
 *          函数内部会判断 当前 是否处于设置模式
 *
 * @param set_led_mode
 *          CUR_LED_MODE_INITIAL_DISCHARGE_GEAR_IN_SETTING_MODE 要设置的是初始放电挡位
 *          CUR_LED_MODE_DISCHARGE_RATE_IN_SETTING_MODE     要设置的是放电速率
 * @param val 初始放电挡位 或 放电速率对应的值（需要注意不能超过范围）
 */
void set_led_mode_status(u8 set_led_mode, u8 val)
{
    // 如果在设置模式，才会进入
    if (flag_is_in_setting_mode)
    {
        if (CUR_LED_MODE_INITIAL_DISCHARGE_GEAR_IN_SETTING_MODE == set_led_mode)
        {
            cur_initial_discharge_gear = val;

            // 设置了初始挡位之后，更新当前主灯光对应的占空比值
            cur_light_pwm_duty_val = light_pwm_duty_init_val_table[cur_initial_discharge_gear - 1];
        }
        else // CUR_LED_MODE_DISCHARGE_RATE_IN_SETTING_MODE == set_led_mode
        {
            cur_discharge_rate = val;
        }

        cur_led_mode = set_led_mode;
        led_all_off();
        flag_led_setting_mode_exit_times_come = 0;
        led_setting_mode_exit_times_cnt = 0; // 清空退出设置模式的时间计数

        light_blink(val);
    }
}

//
void led_handle(void)
{
    // if (CUR_LED_MODE_OFF == cur_led_mode)
    // {
    //     // led_status_refresh();
    //     led_all_off();

    //     // last_led_mode = cur_led_mode;
    //     return;
    // }

    // 如果当前处于电池电量指示模式
    // 设备处于放电时，电量指示灯只显示电池电量降低的部分
    if (CUR_LED_MODE_BAT_INDICATOR == cur_led_mode ||
        CUR_LED_MODE_BAT_INDICATIOR_IN_INSTRUCTION_MODE == cur_led_mode)
    {
        /* 点亮指示灯1（放电模式，指示灯1始终点亮） */
        LED_1_ON();

        if (bat_adc_val > BAT_ADC_VAL_4) // 电池电量大于4格
        {
            cur_led_gear = 5; // 亮5格
        }
        else if (bat_adc_val > BAT_ADC_VAL_3) // 电池电量大于3格
        {
            cur_led_gear = 4; // 亮4格
        }
        else if (bat_adc_val > BAT_ADC_VAL_2) // 电池电量大于2格
        {
            cur_led_gear = 3; // 亮3格
        }
        else if (bat_adc_val > BAT_ADC_VAL_1) // 电池电量大于1格
        {
            cur_led_gear = 2; // 亮2格
        }
        else
        {
            cur_led_gear = 1; // 亮1格
        }

        if (0 == last_led_gear)
        {
            // 如果未初始化
            last_led_gear = cur_led_gear;
        }
        else
        {
            // 如果 last_led_gear 不为0，则说明已经初始化过了

            if (cur_led_gear > last_led_gear ||         //
                (0 == flag_led_gear_update_times_come)) /* 如果更新时间还未到来 */
            {
                // 如果当前要显示的指示灯 大于 上次显示的指示灯（样机在电池电压上升的情况下，不会更新显示）
                cur_led_gear = last_led_gear;
            }

            if (flag_led_gear_update_times_come)
            {
                flag_led_gear_update_times_come = 0;
            }
        }

        if (cur_led_gear >= 5)
        {
            LED_5_ON();
        }
        else
        {
            LED_5_OFF();
        }

        if (cur_led_gear >= 4)
        {
            LED_4_ON();
        }
        else
        {
            LED_4_OFF();
        }

        if (cur_led_gear >= 3)
        {
            LED_3_ON();
        }
        else
        {
            LED_3_OFF();
        }

        if (cur_led_gear >= 2)
        {
            LED_2_ON();
        }
        else
        {
            LED_2_OFF();
        }

    } // if (CUR_LED_MODE_BAT_INDICATOR == cur_led_mode)
    else if (CUR_LED_MODE_DISCHARGE_RATE_IN_SETTING_MODE == cur_led_mode ||
             CUR_LED_MODE_DISCHARGE_RATE_IN_INSTRUCTION_MODE == cur_led_mode)
    {
        // 放电速率指示模式，M1、M2、M3，需要对应的指示灯常亮
        switch (cur_discharge_rate)
        {
        case 1:
            LED_1_ON();
            break;
        case 2:
            LED_2_ON();
            break;
        case 3:
            LED_3_ON();
            break;
        }
    }
    else if (CUR_LED_MODE_CHARGING == cur_led_mode) // 充电指示模式
    {
        // 在充电指示模式中，如果电池电量降低，不更新显示
        // TODO：样机是在电池电压超过3.55V并且进入涓流充电，绿色指示灯才一直点亮(第5格指示灯)

        /* 点亮指示灯1（指示灯1始终点亮） */
        LED_1_ON();

        // if (bat_adc_val > BAT_ADC_VAL_5) // 电池电量大于5格
        // {
        //     cur_led_gear = 6; // 亮5格，并且所有指示灯常亮
        // }
        // else if (bat_adc_val > BAT_ADC_VAL_4) // 电池电量大于4格
        if (bat_adc_val > BAT_ADC_VAL_4) // 电池电量大于4格
        {
            cur_led_gear = 5; // 亮5格，让第5格闪烁
        }
        else if (bat_adc_val > BAT_ADC_VAL_3) // 电池电量大于3格
        {
            cur_led_gear = 4; // 亮4格，让第4格闪烁
        }
        else if (bat_adc_val > BAT_ADC_VAL_2) // 电池电量大于2格
        {
            cur_led_gear = 3; // 亮3格，让第3格闪烁
        }
        else if (bat_adc_val > BAT_ADC_VAL_1) // 电池电量大于1格
        {
            cur_led_gear = 2; // 亮2格，让第2格闪烁
        }
        else
        {
            cur_led_gear = 1; // 亮1格，让第2格闪烁
        }

        if (CUR_CHARGE_PHASE_TRICKLE_CHARGE_WHEN_APPROACH_FULLY_CHARGE == cur_charge_phase || /* 快满电，涓流充电 */
            CUR_CHARGE_PHASE_FULLY_CHARGE == cur_charge_phase)                                /* 已经充满电 */
        {
            // 样机是进入涓流充电才把所有指示灯变为常亮
            cur_led_gear = 6;
        }

        if (0 == last_led_gear)
        {
            last_led_gear = cur_led_gear;
        }
        else
        {
            if (cur_led_gear < last_led_gear ||         /* 如果电池电量比原来的还要低 */
                (0 == flag_led_gear_update_times_come)) /* 如果更新时间还未到来 */
            {
                // 在充电指示模式中，如果电池电量降低，不更新显示
                cur_led_gear = last_led_gear;
            }

            if (flag_led_gear_update_times_come)
            {
                flag_led_gear_update_times_come = 0;
            }
        }

        cur_led_gear_in_charging = cur_led_gear; // 更新数值，给定时器中断使用
        delay_ms(1);

        if (cur_led_gear >= 3)
        {
            LED_2_ON();
        }

        if (cur_led_gear >= 4)
        {
            LED_3_ON();
        }

        if (cur_led_gear >= 5)
        {
            LED_4_ON();
        }

        if (cur_led_gear >= 6)
        {
            LED_5_ON();
        }
    }

    // led设置模式的时间到来
    if (flag_led_setting_mode_exit_times_come)
    {
        flag_led_setting_mode_exit_times_come = 0;

        flag_is_in_setting_mode = 0; // 退出设置模式
        flag_allow_light_in_setting_mode = 0;

        // 如果要回到 led_off
        if (flag_is_led_off_enable)
        {
            flag_is_led_off_enable = 0;
            cur_led_mode = CUR_LED_MODE_OFF;
        }
        else
        {
            cur_led_mode = CUR_LED_MODE_BAT_INDICATOR; // 恢复到电池电量指示模式
        }
    }

    // led指示灯退出指示模式的时间到来
    if (flag_led_struction_mode_exit_times_come)
    {
        flag_led_struction_mode_exit_times_come = 0;

        flag_is_in_struction_mode = 0;

        // 如果要回到 led_off
        if (flag_is_led_off_enable)
        {
            flag_is_led_off_enable = 0;
            cur_led_mode = CUR_LED_MODE_OFF;
        }
        else
        {
            cur_led_mode = CUR_LED_MODE_BAT_INDICATOR;
        }
    }

    if (CUR_LED_MODE_OFF == cur_led_mode)
    {
        led_all_off();
    }

    // last_led_mode = cur_led_mode;
}

#endif // led指示灯控制

void light_blink(u8 blink_cnt)
{
    light_ctl_blink_times = blink_cnt;
    flag_is_ctl_light_blink = 1; // 使能主灯光闪烁
}

void light_init(void)
{
    /* 根据初始的放电挡位来设定灯光对应的pwm占空比 */
    // 查表，获得挡位对应的占空比值
    cur_light_pwm_duty_val = light_pwm_duty_init_val_table[cur_initial_discharge_gear - 1];

    LIGHT_SET_PWM_DUTY(cur_light_pwm_duty_val); // 立刻更新PWM占空比
    LIGHT_ON();                                 // 使能PWM输出
    light_blink(3);                             // 开机前，主灯需要闪烁
    light_adjust_time_cnt = 0;                  // 灯光调整时间清零
}

// 程序占用212
#if 0
/**
 * @brief 灯光控制（放电控制）
 *          进入前要先确认 expect_light_pwm_duty_val 的值是否初始化过一次，
 *          进入前要先确认 cur_light_pwm_duty_val 的值是否初始化过一次，
 *          light_adjust_time_cnt调节灯光的时间计时是否正确，如果切换了模式或放电速度，要先清零
 */
void light_handle(void)
{
#if 1

    // 如果正在充电，直接返回
    if (cur_charge_phase != CUR_CHARGE_PHASE_NONE ||
        cur_led_mode == CUR_LED_MODE_OFF) /* 如果指示灯已经关闭 */
    {
        return;
    }

    // 如果未在充电

    if (1 == cur_discharge_rate) // 放电速率1档，M1
    {
        /*
            速度为M1，
            1200s后变化一次占空比，(1200 * 1)
            3600s后再变化一次，    (1200 * 3)
            7200s后再变化一次，    (1200 * 6)
            ...
            假设之后是：
            (1200 * 9)
            (1200 * 12)
            (1200 * 15)
            ...
            每次变化约10%占空比
        */

        if (light_adjust_time_cnt >= (1200 * light_ctl_phase_in_rate_1)) // 如果到了调节时间
        {
            light_adjust_time_cnt = 0;

            if (1 == light_ctl_phase_in_rate_1)
            {
                light_ctl_phase_in_rate_1 = 3;
            }
            else
            {
                light_ctl_phase_in_rate_1 += 3;
            }

            // 定时器对应的重装载值最大值 对应 100%占空比
            if (cur_light_pwm_duty_val >= ((u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000) + ((u32)LIGHT_TIMER_FEQ_VAL * 10 / 100))
            {
                // 如果仍大于 4.8% + 10%， 减少10%占空比
                cur_light_pwm_duty_val -= (u32)LIGHT_TIMER_FEQ_VAL * 10 / 100;
            }
            else
            {
                // 4.8%占空比
                cur_light_pwm_duty_val = (u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000;
            }
        }
    }
    else // 2 == cur_discharge_rate || 3 == cur_discharge_rate
    {
        /*
            一开始每40s降低一次占空比
            从47%开始，每240s降低一次占空比
            从42%开始，每420s降低一次占空比

            暂定每次降低 0.6%
        */

        // 当前的占空比在47%以上时，不包括47%，每40s降低一次占空比
        if (cur_light_pwm_duty_val > (u32)LIGHT_TIMER_FEQ_VAL * 47 / 100)
        {
            if (light_adjust_time_cnt >= 40)
            {
                light_adjust_time_cnt = 0;

                if (cur_light_pwm_duty_val >= ((u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000) + ((u32)LIGHT_TIMER_FEQ_VAL * 6 / 1000))
                {
                    // 如果仍大于 4.8% + xx %， 减少 xx %占空比
                    cur_light_pwm_duty_val -= (u32)LIGHT_TIMER_FEQ_VAL * 6 / 1000;
                }
                else
                {
                    // 4.8%占空比
                    cur_light_pwm_duty_val = (u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000;
                }
            }
        }
        // 当前的占空比在42%以上时，不包括42%，每240秒降低一次占空比
        else if (cur_light_pwm_duty_val > (u32)LIGHT_TIMER_FEQ_VAL * 42 / 100)
        {
            if (light_adjust_time_cnt >= 240)
            {
                light_adjust_time_cnt = 0;

                if (cur_light_pwm_duty_val >= ((u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000) + ((u32)LIGHT_TIMER_FEQ_VAL * 6 / 1000))
                {
                    // 如果仍大于 4.8% + xx %， 减少 xx %占空比
                    cur_light_pwm_duty_val -= (u32)LIGHT_TIMER_FEQ_VAL * 6 / 1000;
                }
                else
                {
                    // 4.8%占空比
                    cur_light_pwm_duty_val = (u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000;
                }
            }
        }
        else // 当前的占空比在42%及以下，每420秒降低一次占空比
        {
            if (light_adjust_time_cnt >= 420)
            {
                light_adjust_time_cnt = 0;

                if (cur_light_pwm_duty_val >= ((u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000) + ((u32)LIGHT_TIMER_FEQ_VAL * 6 / 1000))
                {
                    // 如果仍大于 4.8% + xx %， 减少 xx %占空比
                    cur_light_pwm_duty_val -= (u32)LIGHT_TIMER_FEQ_VAL * 6 / 1000;
                }
                else
                {
                    // 4.8%占空比
                    cur_light_pwm_duty_val = (u32)LIGHT_TIMER_FEQ_VAL * 48 / 1000;
                }
            }
        }
    } // 放电速率M2，放电速率M3

    LIGHT_SET_PWM_DUTY(cur_light_pwm_duty_val);

#endif
}
#endif

void main(void)
{
    sys_init();

    // timer0_pwm_set_low_feq();
    // timer0_pwm_set_high_feq();
    while (1)
    {

#if 0

        charge_handle();
        ir_handle(); // 函数内部会判断是否在充电，如果在充电则退出

        /*
            【非充电模式】 -> 【充电模式】

            如果当前正在充电，但是指示灯没有切换到充电指示模式，则切换：
        */
        if (CUR_CHARGE_PHASE_NONE != cur_charge_phase)
        {
            // if (cur_led_mode != CUR_LED_MODE_CHARGING && /* 指示灯不处于充电模式 */
            //     cur_led_mode != CUR_LED_MODE_OFF)
            if (cur_led_mode != CUR_LED_MODE_CHARGING) /* 指示灯不处于充电模式 */
            {
                // 清空定时关机相关的变量
                flag_is_auto_shutdown_enable = 0;
                led_status_clear();
                led_mode_alter(CUR_LED_MODE_CHARGING);
            }

            // 需要关闭主灯光
            LIGHT_OFF();
        } // if (CUR_CHARGE_PHASE_NONE != cur_charge_phase)
        else // CUR_CHARGE_PHASE_NONE == cur_charge_phase
        {
            /*
                【充电模式】 -> 【放电、点亮主灯光、指示灯对应电池电量指示】
                如果当前没有在充电，并且指示灯处于充电指示模式，
                切换回电池电量指示模式

                测试时发现从充电到断开充电，led指示灯还在闪烁，需要加上这补丁
            */
            if (cur_led_mode == CUR_LED_MODE_CHARGING)
            {
                led_status_clear();
                led_mode_alter(CUR_LED_MODE_BAT_INDICATOR);
                // 需要打开主灯光

                // 查表，获得挡位对应的占空比值
                cur_light_pwm_duty_val = light_pwm_duty_init_val_table[cur_initial_discharge_gear - 1];

                LIGHT_SET_PWM_DUTY(cur_light_pwm_duty_val); // 立刻更新PWM占空比
                LIGHT_ON();                                 // 使能 PWM 输出
            }
        }

        // 如果定时关机的时间到来
        if (flag_is_auto_shutdown_times_come)
        {
            flag_is_auto_shutdown_times_come = 0; // 清空定时关机标志
            flag_is_auto_shutdown_enable = 0;     // 不允许自动关机
            led_status_clear();
            cur_led_mode = CUR_LED_MODE_OFF;
            cur_light_pwm_duty_val = 0;
            LIGHT_OFF();

            // printf("power off\n");
        }

        adc_update_bat_adc_val();
        led_handle();
        light_handle();
#endif


    }
}

void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;
    swapar _PFLAG;
    movra _statusbuf;
    __endasm;

    if (T1IF & T1IE) // 定时器1 中断，100us
    {
        // P21D = ~P21D; // 测试是不是100us中断

#if 1     // 红外解码【需要放到100us的定时器中断来处理】
        { // 红外解码
            // static volatile u8 ir_fliter;
            static volatile u16 ir_level_cnt; // 红外信号的下降沿时间间隔计数
            // static volatile u32 __ir_data;    //
            static volatile u8 __ir_data; //

            // static volatile bit last_level_in_ir_pin = 0;
            // static volatile u16 ir_long_press_cnt; // 檢測紅外遙控長按的計數值

            // 对每个下降沿进行计数
            if (ir_level_cnt <= 1300)
                ir_level_cnt++;

            // 滤波操作
            // ir_fliter <<= 1;
            // if (IR_RECV_PIN)
            // {
            //     ir_fliter |= 1;
            // }
            // ir_fliter &= 0x07;

            // if (ir_fliter == 0x07)
            //     filter_level = 1;
            // else if (ir_fliter == 0x00)
            //     filter_level = 0;

            // if (filter_level)
            if (IR_RECV_PIN)
            {
                last_level_in_ir_pin = 1; // 表示接收到的是高电平

                // 如果之前也是高电平
                if (ir_level_cnt > 1200) // 超时时间(120000us,120ms)
                {
                    // if (__ir_data != 0) // 超时，且接收到数据(红外接收处理函数中会把ir_data清零)
                    if (__ir_data != 0) // 超时，且接收到数据(现在是在中断服务函数中把__ir_data自动清零)
                    {
                        // // 带校验的版本：
                        // if ((u8)(__ir_data >> 8) == (u8)(~__ir_data)) // 如果键值的原码和反码相等
                        // {
                        // flag_is_recved_data = 1;
                        // }

                        // 不带校验的版本
                        if (0 == flag_is_recved_data)
                        {
                            // if ((__ir_data & 0xFF0000) == 0xFF0000)
                            {
                                ir_data = ~__ir_data;
                                __ir_data = 0;
                                flag_is_recved_data = 1;
                            }
                        }
                    }

                    flag_is_recv_ir_repeat_code = 0; // 认为遥控器按键已经按下，然后松开
                }
            }
            else
            {
                if (last_level_in_ir_pin)
                {
                    // 如果之前检测到的是高电平，现在检测到了低电平
                    if (ir_level_cnt <= 8) // 小于800us，说明接收到无效的数据，重新接收
                    {
                        // FLAG_IS_RECVED_ERR_IR_DATA = 1;
                        flag_is_recv_ir_repeat_code = 0;
                    }
                    else if (ir_level_cnt <= 18) // 小于1800us,说明接收到了逻辑0
                    {
                        __ir_data <<= 1;

                        // P15D = 0; // 测试红外解码
                        // P15D = ~P15D; // 测试红外解码
                    }
                    else if (ir_level_cnt <= 26) // 小于2600us,说明接收到了逻辑1
                    {
                        __ir_data <<= 1;
                        __ir_data |= 0x01;

                        // P15D = 1; // 测试红外解码
                    }
                    else if (ir_level_cnt <= 150) // 小于15000us,说明接收到了格式头
                    {
                        // FLAG_IS_RECVED_ERR_IR_DATA = 1;
                        // ir_long_press_cnt = 0; // 加上这一条会检测不到长按
                    }
                    else if (ir_level_cnt <= 420) // 小于42ms,说明接收完一次完整的红外信号
                    {
#if 0 // 带校验的版本，命令源码与命令反码进行校验
    
                if ((u8)(__ir_data >> 8) == (u8)(~__ir_data)) // 如果键值的原码和反码相等
                {
                    flag_is_recved_data = 1;
                    flag_is_recv_ir_repeat_code = 1; //
                    ir_long_press_cnt = 0;
                }

#else // 不带校验的版本

                        if (0 == flag_is_recved_data) // 如果之前未接收到数据/接收到的数据已经处理完毕
                        {
                            // if ((__ir_data & 0xFF0000) == 0xFF0000)
                            {
                                ir_data = ~__ir_data;
                                __ir_data = 0;
                                flag_is_recved_data = 1;
                                // flag_is_recv_ir_repeat_code = 1; //
                            }
                        }

#endif // 不带校验的版本
                    }
                    else if (ir_level_cnt <= 1200) // 小于120000,120ms,说明接收到了重复码
                    {
                        // if (ir_long_press_cnt < 65535)
                        //     ir_long_press_cnt++;

                        flag_is_recv_ir_repeat_code = 1;
                    }
                    // else // 超过120000,说明接收到无效的数据
                    // {
                    // }

                    ir_level_cnt = 0;
                }

                last_level_in_ir_pin = 0; // 表示接收到的是低电平
            }
        } // 红外解码
#endif // 红外解码【需要放到100us的定时器中断来处理】

#if 1 // 控制LED指示灯 (软件PWM驱动)

        {                  // 控制LED指示灯--需要放在100us的中断
            static u8 cnt; // 用软件实现PWM驱动LED的相关变量
            cnt++;
            if (cnt <= 20) // 20 * 100us
            {
                if (flag_is_led_1_enable)
                {
                    LED_1_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_2_enable)
                {
                    LED_2_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_3_enable)
                {
                    LED_3_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_4_enable)
                {
                    LED_4_PIN = LED_ON_LEVEL;
                }

                if (flag_is_led_5_enable)
                {
                    LED_5_PIN = LED_ON_LEVEL;
                }
            }
            else
            {
                LED_1_PIN = LED_OFF_LEVEL;
                LED_2_PIN = LED_OFF_LEVEL;
                LED_3_PIN = LED_OFF_LEVEL;
                LED_4_PIN = LED_OFF_LEVEL;
                LED_5_PIN = LED_OFF_LEVEL;
            }

            if (cnt >= 100) // 100 * 100us
            {
                cnt = 0;
            }

        } // 控制LED指示灯--需要放在100us的中断

#endif // 控制LED指示灯(软件PWM驱动)

        T1IF = 0; // 清除定时器中断标志
    } //  if (T1IF & T1IE) // 定时器1 中断，100us

    //=======T3========================
    // if (T3IF & T3IE)
    // {
    //     // P14D = !P14D;
    //     T3IF = 0;
    // }

    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
