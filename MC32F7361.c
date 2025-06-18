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

// 控制充电的PWM
void timer0_pwm_config(void)
{
    // 20KHz，18.15%
    T0CR = (0x01 << 7) | (0x01 << 6); // 使能定时器，使能PWM，时钟源使用CPU，不分频
    T0LOAD = 215 - 1;
    // T0DATA = 39;
    // T0DATA = 0; // 刚上点，T0DATA默认就是0

    /* 为了不对其他PWM造成影响，这里用或操作 */
    PWMCR3 |= (0x01 << 2); // P02端口输出
}

// 将 pwm0 配置为 低频，对应涓流充电的 PWM
void timer0_pwm_set_low_feq(void)
{
    // 20KHz，18.15%
    T0CR = (0x01 << 7) | (0x01 << 6); // 使能定时器，使能PWM，时钟源使用CPU，不分频
    T0LOAD = 215 - 1;
    T0DATA = 39;
}

// 将 PWM0 配置为 高频，对应正常充电的PWM
void timer0_pwm_set_high_feq(void)
{
    PWMCR1 |= (0x01 << 4); // FTMR 选择 FHOSC

    // 105 KHz，0% (测试时用 非0% 的占空比)
    T0CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 3) | (0x01 << 0); // 使能定时器，使能PWM, 时钟源选择FTMR，时钟源2分频
    T0LOAD = 164 - 1;
    T0DATA = 0;
    // T0DATA = 50;
}

// 关闭 PWM0 输出，引脚输出低电平
void timer0_pwm_disable(void)
{
    T0CR &= ~((0x01 << 7) | (0x01 << 6)); // 关闭定时器，关闭PWM
    P02D = 0;
}

void timer1_config(void)
{
    T1CR |= (0x01 << 7) | (0x01 << 1); // 使能定时器，时钟源选择CPU, 4分频
    T1LOAD = 109 - 1;                  // 100us (从sdk复制过来的会有些误差，这里做了补偿)
}

// 控制灯光的PWM，只使用 NPWM2
void timer2_pwm_config(void)
{
    // T2CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 1) | (0x01 << 0); // 使能定时器，使能PWM, CPU, 8分频
    T2CR = (0x01 << 7) | (0x01 << 6); // 使能定时器，使能PWM, CPU,
    T2LOAD = 215 - 1;                 //
    T2DATA = 215 - 38;                /* 由于只使用 NPWM 互补PWM，这里占空比计算要用 周期计数值 减去 比较值，才是互补PWM的占空比 */
    // PWMCR3 = DEF_SET_BIT6 | DEF_SET_BIT5 | DEF_SET_BIT4; // 使能FPWM,NPWM,正向输出
    PWMCR3 |= (0x01 << 5); // 使能NPWM2输出波形
    // T2EN = 1;
}

void timer2_pwm_set_feq(void)
{
    PWMCR1 |= (0x01 << 4); // FTMR 选择 FHOSC

    T2CR = (0x01 << 7) | (0x01 << 6) | (0x01 << 3); // 使能定时器，使能PWM, 时钟源选择FTMR，时钟源不分频
    T2LOAD = 255 - 1;                               //
    T2DATA = 255 - 50;                              /* 由于只使用 NPWM 互补PWM，这里占空比计算要用 周期计数值 减去 比较值，才是互补PWM的占空比 */
}

void adc_config(void)
{
    // 检测充电座放电电流的引脚：
    // P15OE = 0; // 输入模式
    // P15DC = 1; // 模拟模式

    // 检测充电电压的引脚，充电电压 1/10分压
    P12OE = 0; // 输入模式
    P12DC = 1; // 模拟模式

    // 检测电池电压的引脚，电池电压 1/2分压
    P05OE = 0; // 输入模式
    P05DC = 1; // 模拟模式

    ADCR0 = 0x0B; // 12位精度，使能adc
    ADCR1 = 0x80; // adc转换时钟选择 FHIRC/32，使用內部2.0V参考电压
    ADCR2 = 0x0F; // 采样时间，只能固定是15 个 ADCLK
}

void adc_sel_pin(u8 adc_pin)
{

    // P12 AN7
    if (ADC_PIN_DETECT_CHARGE == adc_pin)
    {
        ADCHS3 = 0;
        ADCHS2 = 1;
        ADCHS1 = 1;
        ADCHS0 = 1;
    }
    else if (ADC_PIN_DETECT_BATTERY == adc_pin)
    {
        // P05 AN4
        ADCHS3 = 0;
        ADCHS2 = 1;
        ADCHS1 = 0;
        ADCHS0 = 0;
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

void sys_init(void)
{
    GIE = 0;
    CLR_RAM();
    IO_Init();
    timer0_pwm_config();
    timer1_config();
    // timer2_pwm_config(); // 控制 主灯 灯光 的pwm

    adc_config();

    INTE |= (0x01 << 1); // 使能 timer1中断

    delay_ms(1); // 等待系统稳定
    GIE = 1;
}

// 红外按键处理
void ir_handle(void)
{
    static u8 last_ir_data = 0;

    // 如果正在充电，不响应按键，直接退出
    if (CUR_CHARGE_STATUS_IN_CHARGING == cur_charge_status)
    {
        ir_data = 0;
        flag_is_recved_data = 0;
        flag_is_recv_ir_repeat_code = 0;
        return;
    }

    if (flag_is_recved_data || flag_is_recv_ir_repeat_code)
    {

        /*
            如果有按键长按
            长按的按键是亮度加或亮度减，才执行，否则直接退出
        */
        if (flag_is_recv_ir_repeat_code)
        {
            // printf("recv repeat code 0x %bx\n", last_ir_data);
            // last_ir_data = ir_data;
            flag_is_recv_ir_repeat_code = 0;
            return;
        }

        // printf("recv ir data 0x %bx\n", ir_data);
        switch (ir_data)
        {
        case IR_KEY_ON:
            /* code */
            break;

        case IR_KEY_OFF:
            /* code */
            break;

        case IR_KEY_RED:
            /* code */
            break;

        case IR_KEY_FULL_BRIGHTNESS: // 全亮
            /* code */
            break;
        case IR_KEY_HALF_BRIGHTNESS: // 半亮
            /* code */
            break;
        case IR_KEY_BRIGHTNESS_ADD_OR_NUM_4: /* 亮度加，也是小遥控器的数字4 */
            /* code */
            break;

        case IR_KEY_BRIGHTNESS_SUB_OR_NUM_2: /* 亮度减，也是小遥控器的数字2 */
            /* code */
            break;

        case IR_KEY_AUTO_OR_NUM_5: /* 自动模式 ，也是小遥控器的数字5 */
            /* code */
            break;

        case IR_KEY_3H_OR_NUM_3: /* 3H，也是小遥控器的数字3 */
            /* code */
            break;
        case IR_KEY_5H_OR_M1: /* 5H，也是小遥控器的M1 */
            /* code */
            break;
        case IR_KEY_8H_OR_M3: /* 8H，也是小遥控器的M3 */
            /* code */
            break;

        case IR_KEY_SET: // SET 模式设置
            /* code */
            break;
        case IR_KEY_NUM_1: // 数字1
            /* code */
            break;

        case IR_KEY_M2: //
            /* code */
            break;

        default:
            break;
        }

        last_ir_data = ir_data;
        ir_data = 0;
        flag_is_recved_data = 0;
    }
}

// 电池放电控制，实际是控制 主灯光 对应的pwm占空比
void bat_discharge_handle(void)
{
}

// void fun(void)
// {
//     u16 charging_adc_val;
//     u16 bat_adc_val;
//     u32 tmp;

//     // T1DATA = 124286 - (u32)bat_adc_val * 7857 * T1DATA / (164 - 1 ) / 1000;

//     // 如果充电输入电压小于14V:
//     // 这里得到的是 1000倍的占空比值
//     tmp = (u32)124286 - (u32)charging_adc_val * 7857 * 2 * 10 / 4096;
//     T1DATA = T1LOAD * (tmp / 1000) / 100; // 最终的占空比值
// }

// 充电控制
void charge_handle(void)
{
    // static volatile u8 cur_charge_status = CUR_CHARGE_STATUS_NONE;
    u16 charging_adc_val;
    u16 bat_adc_val;

    adc_sel_pin(ADC_PIN_DETECT_CHARGE);
    charging_adc_val = adc_getval(); // 采集充电输入对应的ad值
    adc_sel_pin(ADC_PIN_DETECT_BATTERY);
    bat_adc_val = adc_getval(); // 采集电池电压对应的ad值

    // 如果当前未在充电
    if (CUR_CHARGE_STATUS_NONE == cur_charge_status)
    {
        if (charging_adc_val >= ADC_VAL_ENABLE_IN_CHARGE_END)
        {
            // 如果在未充电时，检测到充电输入电压而使能充电

            // 可以先开着 控制充电的PWM，等 单片机在充电时检测到满电再断开PWM
            // // 如果电池电压 大于 快满电 的电压，不使能充电
            // if (bat_adc_val >= ADC_VAL_BAT_IS_NEAR_FULL)
            // {
            //     return;
            // }

            cur_charge_status = CUR_CHARGE_STATUS_IN_CHARGING;
        }
    }
    else // 如果当前正在充电
    {
        static u8 cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_NONE;

        // TODO:
        if (charging_adc_val <= ADC_VAL_DISABLE_IN_CHARGE_END ||
            charging_adc_val >= 2663 || /* 充电输入电压大于 13V ，断开充电 ，公式还有缺陷*/
            bat_adc_val >= ADC_VAL_BAT_IS_FULL)
        {
            // 充电输入的电压小于一定值，断开充电
            // 电池 已经满电， 断开充电
            timer0_pwm_disable();
            cur_charge_status = CUR_CHARGE_STATUS_NONE;
            cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_NONE;
            return;
        }

        // 电池电压小于一定值或是大于一定值，进行涓流充电
        if (bat_adc_val <= ADC_VAL_BAT_IS_LOW ||
            bat_adc_val >= ADC_VAL_BAT_IS_NEAR_FULL)
        {
            if (CUR_CHARGING_PWM_STATUS_LOW_FEQ != cur_charging_pwm_status)
            {
                timer0_pwm_set_low_feq(); // 函数内部设定了固定频率和固定占空比
                cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_LOW_FEQ;
            }
        }
        else
        {
            u32 tmp;

            // 如果电池电压不在涓流充电的区间，则正常充电
            // 注意这里不能随意改变 PWM占空比，最好设置PWM占空比为0
            if (CUR_CHARGING_PWM_STATUS_HIGH_FRQ != cur_charging_pwm_status)
            {
                timer0_pwm_set_high_feq();
                cur_charging_pwm_status = CUR_CHARGING_PWM_STATUS_HIGH_FRQ;
            }

            // 根据充电电压调整pwm占空比

            /*
                测得样机充电控制的功能：
                5V输入，pwm 105Khz，90%，（可能要使用 105Khz，85%）；12V输入，pwm 105Khz，30%
                用（5V，85%）和（12V，30%）这两个点来计算，
                已知两点坐标(5,85),(12,30)，求对应的方程，
                对应直线的方程： y  =  −7.857x  + 124.286
                y 对应占空比，单位：%；x 对应充电输入电压，单位：V
                在公式上去掉小数点：
                1000y = -7857x + 124286

                单片机adc使用内部2.0V参考电压，
                将公式转换成 占空比 和 ad值 的关系:
                外部充电输入的电压 1/10 分压后，到单片机检测脚
                外部输入电压 / 10 / 单片机adc参考电压 * 4096 == 单片机采集到的ad值
                外部输入电压 == 单片机采集到的ad值 / 4096 * 单片机adc参考电压 * 10
                换算成单片机可以计算的形式：
                外部输入电压 == 单片机采集到的ad值 * 单片机adc参考电压 * 10 / 4096
                x = adc_val * 2 * 10 / 4096

                得到的公式：
                1000y = 124286 - 7857 (adc_val * 2 * 10 / 4096)
                1000y = 124286 - adc_val * 7857 * 2 * 10 / 4096
                这个公式运算不会超出 2^32 范围
            */
            tmp = (u32)124286 - (u32)charging_adc_val * 7857 * 2 * 10 / 4096;
            T1DATA = (u32)T1LOAD * (u32)tmp / 1000 / 100; // 最终的占空比值
        }
    }
}

void main(void)
{
    sys_init();

    while (1)
    {
        charge_handle();
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

        // P22D = ~P22D;

#if 1     // 红外解码【需要放到100us的定时器中断来处理】
        { // 红外解码
            // static volatile u8 ir_fliter;
            static volatile u16 ir_level_cnt; // 红外信号的下降沿时间间隔计数
            static volatile u32 __ir_data;    //
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

#if 1                       // 控制LED指示灯
        {                   // 控制LED指示灯
            static u16 cnt; // 用软件实现PWM驱动LED的相关变量
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

        } // 控制LED指示灯
#endif // 控制LED指示灯

        T1IF = 0;
    } //  if (T1IF & T1IE) // 定时器1 中断，100us

    //=======T3========================
    if (T3IF & T3IE)
    {
        // P14D = !P14D;
        T3IF = 0;
    }
    
    __asm;
    swapar _statusbuf;
    movra _PFLAG;
    swapr _abuf;
    swapar _abuf;
    __endasm;
}

/**************************** end of file *********************************************/
