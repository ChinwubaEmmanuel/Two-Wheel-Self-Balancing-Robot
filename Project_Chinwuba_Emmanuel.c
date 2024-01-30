#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "clock.h"
#include "i2c0.h"
#include "i2c1.h"
#include "nvic.h"
#include "gpio.h"
#include "uart0.h"

// PortE masks
#define RIGHT_WHEEL_FORWARD 32
// PortF masks
#define LEFT_WHEEL_BACKWARD 2
#define LEFT_WHEEL_FORWARD 4
#define RIGHT_WHEEL_BACKWARD 8
// Pins
#define IR PORTD,0
#define SLP PORTE,3
#define OPBR PORTC,6 // WT1CCPO
#define OPBL PORTC,7 // WT1CCP1

#define MAX_EDGES 34
#define ADDR 0x68 // 0x69

float print_angle = 0;
float static_angle = 0;
float error = 0;
uint32_t RPM_L = 0;
uint32_t RPM_R = 0;
float left_pwm = 830; //837;
float right_pwm = 872; //892;
float u_low_L = 0;
float u_high_L = 0;
int32_t tick_err = 0;
float tick_err_L = 0;
float u_low_R = 0;
float u_high_R = 0;
float tick_err_R = 0;
float modifier = 0;
float opbLTAVG = 0;
float opbRTAVG = 0;
float edgeTimings[MAX_EDGES];
float opbLtime[16];
float opbRtime[16];
//float k = 0.000099; //0.0001;
float k = 0.00001;
float ki = 0.0001;
uint32_t denom_L = 0;
uint32_t denom_R = 0;
uint32_t bitStream[32];
uint32_t edgeCount = 0;
uint32_t data_bin = 0b00000000;
uint32_t addr = 0b00000000;
uint32_t indexL = 0;
uint32_t indexR = 0;
int16_t accelX = 0;
int16_t accelY = 0;
int16_t accelZ = 0;
int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
float accel_x = 0;
float accel_y = 0;
float accel_z = 0;
float gyro_x = 0;
float gyro_y = 0;
float gyro_z = 0;
char flag = 'N';
char button_pressed = 'N';
char balance = 'B';

uint8_t mpu_data[14];
uint8_t cal_data[6];

float avg_bias_gx = 0;
float avg_bias_gy = 0;
float avg_bias_gz = 0;

float sum_gyro_x = 0;
float sum_gyro_y = 0;
float sum_gyro_z = 0;

float theta = 0;
float mpu_error = 0;
float plant_l = 720;
float plant_r = 800;
float integral = 0;

int32_t count = 0;

int32_t l = 0;
int32_t r = 0;

bool mpu = 0;

#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

void getsUart0(USER_DATA *data)
{
    int count = 0;
    while(true)
    {
        char c = getcUart0();
        if(((c == 8) || (c == 127)) && (count > 0))
        {
            count--;
        }
        else if(c == 13)
        {
            data->buffer[count] = '\0';
            return;
        }
        else
        {
            if(count == MAX_CHARS)
            {
                data->buffer[MAX_CHARS] = '\0';
                return;
            }
            data->buffer[count] = c;
            count++;
        }
    }
}

void parseFields(USER_DATA *data)
{
    char alpha = 'a';
    char num = 'n';
    char delim = 'd';
    char temp[MAX_CHARS+1];
    data->fieldCount = 0;
    temp[0] = 'd';

    //while true, increment count, store in field type at count
    int it = 0;
    while (data->buffer[it] != '\0')
    {
        if(((data->buffer[it] >= 65) && (data->buffer[it] <= 90)) || ((data->buffer[it] >= 97) && (data->buffer[it] <= 122)))
        {
            temp[it+1] = alpha;
        }
        else if(((data->buffer[it] >= 48) && (data->buffer[it] <= 57)) || (data->buffer[it] == 45) || (data->buffer[it] == 46))
        {
            temp[it+1] = num;
        }
        else
        {
                temp[it+1] = delim;
        }
        it++;
    }
    temp[it+1] = '\0';

    int i = 0;
    while (temp[i] != '\0')
    {
        if(temp[i] == delim)
        {
            if(temp[i+1] != delim)
            {
                data->fieldType[data->fieldCount] = temp[i+1];
                data->fieldPosition[data->fieldCount] = i;
                data->fieldCount++;
            }
        }
        i++;
    }

    int count = 0;
    while(temp[count] != '\0')
    {
        temp[count] =  temp[count+1];
        count++;
    }

    int x = 0;
    while (data->buffer[x] != '\0')
    {
        if(temp[x] == delim)
        {
            data->buffer[x] = '\0';
        }
        x++;
    }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber <= data->fieldCount)
    {
         return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return NULL;
    }
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if((fieldNumber < data->fieldCount) && (data->fieldType[fieldNumber] == 'n'))
    {
        return atoi(&(data->buffer[data->fieldPosition[fieldNumber]]));
    }
    else
    {
        return 0;
    }
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    int i = 0;
    bool valid = 0;
    //i = data->fieldPosition[0];

    while (data->buffer[i] != '\0')
    {
        if ((data->buffer[i] == strCommand[i]) && ((data->fieldCount-1) >= minArguments))
        {
            valid = 1;
            i++;
        }
        else
        {
            valid = 0;
            return valid;
        }
    }
    return valid;
}

int strcmp(const char first[], const char second[])
{
    int i = 0;
    int valid = 0;
    while ((first[i] != '\0') && (second[i] != '\0'))
    {
        if (first[i] == second[i])
        {
            valid = 0;
            i++;
        }
        else if (first[i] < second[i])
        {
            valid = -1;
            return valid;
        }
        else if (first[i] > second[i])
        {
            valid = 1;
            return valid;
        }
     }
     return valid;
}

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2;

    enablePort(PORTA);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);
    //configure pin
    selectPinPushPullOutput(SLP);
    setPinValue(SLP, 1);
    selectPinPushPullOutput(PORTE, 5);
    selectPinPushPullOutput(PORTF, 1);
    selectPinPushPullOutput(PORTF, 2);
    selectPinPushPullOutput(PORTF, 3);
    selectPinDigitalInput(IR);
    selectPinDigitalInput(OPBL);
    selectPinDigitalInput(OPBR);
    setPinAuxFunction(OPBL, 7);
    setPinAuxFunction(OPBR, 7);
    setPinAuxFunction(IR, 7);
}

void initMPU()
{
    writeI2c1Register(ADDR, 0x6B, 0x00);
    //writeI2c1Register(ADDR, 0x6B, 0x03);
    waitMicrosecond(1000);
    writeI2c1Register(ADDR, 0x19, 0x07);
    waitMicrosecond(1000);
    writeI2c1Register(ADDR, 0x1A, 0x00);
    waitMicrosecond(1000);
    writeI2c1Register(ADDR, 0x1B, 0x00);
    waitMicrosecond(1000);
    writeI2c1Register(ADDR, 0x1C, 0x00);
    waitMicrosecond(1000);
}

void enableCounterMode()
{
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 40E3;  // set load value to 40e6 for 1 Hz interrupt rate
    //TIMER1_TAILR_R = 20E3;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)

    // Configure Wide Timer 1 as counter of external events on CCP0 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;           // count positive edges
    WTIMER1_IMR_R |= 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    //NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);

    // Configure Wide Timer 1 as counter of external events on CCP1 pin
    WTIMER1_CTL_R &= ~TIMER_CTL_TBEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TBMR_R = TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCDIR; // configure for edge count mode, count up
    WTIMER1_CTL_R |= TIMER_CTL_TBEVENT_NEG;           // count positive edges
    WTIMER1_IMR_R |= 0;                               // turn-off interrupts
    WTIMER1_TBV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TBEN;                 // turn-on counter
    //NVIC_EN3_R = 1 << (INT_WTIMER1B-16-96);
}

void enableTimerModeIR()
{

// Configure Timer 3 as the time base
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER3_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER3_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER3_TAILR_R = 40E3;                       // set load value to 40e6 for 1 Hz interrupt rate
    //TIMER3_TAILR_R = 50000;
    TIMER3_IMR_R = TIMER_IMR_TATOIM;;                 // turn-on interrupts
    TIMER3_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    enableNvicInterrupt(INT_TIMER3A);             // turn-on interrupt 37 (TIMER1A)
    /******/
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER2_CTL_R |= TIMER_CTL_TAEVENT_NEG;           // measure time from positive edge to positive edge
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

    NVIC_EN3_R = 1 << (INT_WTIMER2A-16-96);         // turn-on interrupt 112 (WTIMER1A)
}

void opbISR_L()
{
    //RPM_L = (WTIMER1_TBR_R * (0.000000025) * 1000) - RPM_L;                  // read counter input
    RPM_L = (WTIMER1_TBV_R * (0.000000025) * 1000);
    l = l + 1;

    opbLTAVG -= opbLtime[indexL];
    opbLTAVG += RPM_L;
    denom_L = (sizeof(opbLtime)/sizeof(opbLtime[0]));
    opbLTAVG = opbLTAVG / denom_L;
    opbLtime[indexL] = RPM_L;
    indexL = (indexL + 1) & 15;

//    tick_err_L = opbLTAVG - opbRTAVG;
//    u_low_L = left_pwm;
//    u_high_L = left_pwm;
//    u_low_L = u_low_L - abs(k*tick_err_L);
//    u_high_L = u_high_L + abs(k*tick_err_L);
//    if(tick_err_L > 0) // right is faster
//    {
//        if(u_high_L < 1000)
//        {
//          left_pwm = u_high_L;
//          // set pwm in isr?
//        }
//        else
//        {
//            u_high_L = 1000;
//        }
//    }
//    else if(tick_err_L < 0) // left is faster
//    {
//        if(u_low_L > 850)
//        {
//          left_pwm = u_low_L;
//        }
//        else
//        {
//            u_low_L = 850;
//        }
//    }
    WTIMER1_TBV_R = 0;                           // reset counter for next period
    WTIMER1_ICR_R = TIMER_ICR_CBECINT;           // clear interrupt flag
}

void opbISR_R()
{
    //RPM_R = (WTIMER1_TAR_R * (0.000000025) * 1000) - RPM_R;                  // read counter input
    RPM_R = (WTIMER1_TAR_R * (0.000000025) * 1000);
    r = r + 1;

    opbRTAVG -= opbRtime[indexR];
    opbRTAVG += RPM_R;
    denom_R = (sizeof(opbRtime)/sizeof(opbRtime[0]));
    opbRTAVG = opbRTAVG / denom_R;
    opbRtime[indexR] = RPM_R;
    indexR = (indexR + 1) & 15;

//    tick_err_R = opbLTAVG - opbRTAVG;
//    u_low_R = right_pwm;
//    u_high_R = right_pwm;
//    u_low_R = u_low_R - abs(k*tick_err_R);
//    u_high_R = u_high_R + abs(k*tick_err_R);
//    if(tick_err_R > 0) // right is faster
//    {
//        if(u_high_R < 1000)
//        {
//          right_pwm = u_high_R;
//        }
//        else
//        {
//            u_high_R = 1000;
//        }
//    }
//    if(tick_err_R < 0) // left is faster
//    {
//        if(u_low_R > 850)
//        {
//          right_pwm = u_low_R;
//        }
//        else
//        {
//            u_low_R = 850;
//        }
//    }

    WTIMER1_TAR_R = 0;                           // reset counter for next period
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void irWTISR()
{
    // if (button_pressed != Y)
    if (edgeCount < MAX_EDGES)
    {
        edgeTimings[edgeCount] = (WTIMER2_TAR_R * (0.000000025 * 1000)) / 2;
        edgeCount++;
    }
    if (edgeCount == 34)
    {
        button_pressed = 'Y';
    }
    WTIMER2_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void modISR()
{
    if((flag == 'F') || (flag == 'B') || (flag == 'C') || (flag == 'W'))
    {
        RPM_L += (WTIMER1_TBV_R);

    //    denom_L = (sizeof(opbLtime)/sizeof(opbLtime[0]));
    //    opbLTAVG -= opbLtime[indexL];
    //    opbLTAVG += RPM_L/denom_L;
    //    opbLtime[indexL] = RPM_L/denom_L;
    //    indexL = (indexL + 1) & 15;

        RPM_R += (WTIMER1_TAV_R);

    //    denom_R = (sizeof(opbRtime)/sizeof(opbRtime[0]));
    //    opbRTAVG -= opbRtime[indexR];
    //    opbRTAVG += RPM_R/denom_R;
    //    opbRTAVG = opbRTAVG;
    //    opbRtime[indexR] = RPM_R/denom_R;
    //    indexR = (indexR + 1) & 15;

        tick_err = RPM_L - RPM_R;
        //tick_err = l-r;
        //modifier = (k*abs(tick_err)) + (ki*abs(error));
    //    if(tick_err < 50 && tick_err > -50)
    //    {
    //        k = 0.00001;
    //    }
    //    if(tick_err < 10 && tick_err > -10)
    //    {
    //        k = 0.0000001;
    //    }
    //    if(tick_err < 4 && tick_err > -4)
    //    {
    //        k = 0.00000001;
    //    }

        modifier = (k*abs(tick_err));

        u_low_R = right_pwm;
        u_high_R = right_pwm;

        u_low_L = left_pwm;
        u_high_L = left_pwm;

        u_low_L = u_low_L - modifier;
        u_high_L = u_high_L + modifier;

        u_low_R = u_low_R - modifier;
        u_high_R = u_high_R + modifier;

        if(tick_err > 0) // left is faster
        {
            if(u_high_R < 1000 && u_high_R > 810)
            {
              right_pwm = u_high_R;
            }
            else
            {
                u_high_R = 1000;
            }
            if(u_low_L > 710 && u_low_L < 1000)
            {
                left_pwm = u_low_L;
            }
            else
            {
                u_low_L = 710;
            }
        }
        else if(tick_err < 0) // right is faster
        {
            if(u_low_R > 811 && u_low_R < 1000)
            {
              right_pwm = u_low_R;
            }
            else
            {
                u_low_R = 811;
            }
            if(u_high_L < 1000 && u_high_L > 710)
            {
                left_pwm = u_high_L;
            }
            else
            {
                u_high_L = 1000;
            }
        }
    //    if(l >= 200 || r >= 200)
    //    {
    //        l = 0;
    //        r = 0;
    //    }
    }
    WTIMER1_TAV_R = 0;                           // reset counter for next period
    WTIMER1_TBV_R = 0;                           // reset counter for next period
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void getIR()
{
    int i = 0;
    int time = 0;
    if (edgeCount == MAX_EDGES)
    {
        addr = 0b00000000;
        data_bin = 0b00000000;
        for(i = 1; i < MAX_EDGES-1; i++)
        {
            if( (((edgeTimings[i+1] - edgeTimings[i]) * 2) >= 0.5) && (((edgeTimings[i+1] - edgeTimings[i]) * 2) <= 1.7) )
            {
                bitStream[time] = 0;
            }
            else if( (((edgeTimings[i+1] - edgeTimings[i]) * 2) >= 1.8) && (((edgeTimings[i+1] - edgeTimings[i]) * 2) <= 3.0) )
            {
                bitStream[time] = 1;
            }
            time++;
        }
        time = 0;
        flag = 'T';
        int x;
        int z = 7;
        for (x = 16; x <= 23; x++)
        {
            data_bin |= bitStream[x] << z;
            z--;
        }
//        int y;
//        int a = 7;
//        for (y = 0; y <= 7; y++)
//        {
//            addr |= bitStream[y] << a;
//            a--;
//        }
    }
    if(edgeCount == MAX_EDGES)
    {
        edgeCount = 0;
    }
}

void parseIR()
{
    //if (data_bin == 152 || data_bin == 148)
    if (data_bin == 152)
    {
        flag = 'F'; // forward
    }
    else if (data_bin == 204)
    {
        flag = 'B'; // backwards
    }
    else if (data_bin == 180)
    {
        flag = 'C'; // CW
    }
    else if (data_bin == 120)
    {
        flag = 'W'; // CCW
    }
    else if (data_bin == 50)
    {
        balance = 'B'; // Balance on
    }
    else if (data_bin == 70)
    {
        balance = 'N'; // Balance off
    }
    else
    {
        flag = 'N'; // NULL
    }
}

void initMotor()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTF_DEN_R |= LEFT_WHEEL_FORWARD | RIGHT_WHEEL_BACKWARD | LEFT_WHEEL_BACKWARD;
    GPIO_PORTF_AFSEL_R |= LEFT_WHEEL_FORWARD | RIGHT_WHEEL_BACKWARD | LEFT_WHEEL_BACKWARD;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;
    // Configure three LEDs
    GPIO_PORTE_DEN_R |= RIGHT_WHEEL_FORWARD;
    GPIO_PORTE_AFSEL_R |= RIGHT_WHEEL_FORWARD;
    GPIO_PORTE_PCTL_R &= ~(GPIO_PCTL_PE5_M);
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE5_M0PWM5;

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM0_2_CTL_R = 0;
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ONE | PWM_0_GENB_ACTLOAD_ZERO;

    PWM1_2_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)
    PWM0_2_LOAD_R = 1024;

    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off
    PWM0_2_CMPB_R = 0;

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
    PWM0_ENABLE_R = PWM_ENABLE_PWM5EN;
                                                     // enable outputs
}

void left_b(float pwm)
{
    PWM1_2_CMPB_R = pwm;
}
void left_f(float pwm)
{
    PWM1_3_CMPA_R = pwm;
}
void right_b(float pwm)
{
    PWM1_3_CMPB_R = pwm;
}
void right_f(float pwm)
{
    PWM0_2_CMPB_R = pwm;
}

void mpu_isr()
{
    readI2c1Registers(ADDR, 0x3B, mpu_data, 14);

    accelX = mpu_data[0] << 8 | mpu_data[1];
    accel_x = (float)(accelX) / 16384;
    accelY = mpu_data[2] << 8 | mpu_data[3];
    accel_y = (float)(accelY) / 16384;
    accelZ = mpu_data[4] << 8 | mpu_data[5];
    accel_z = (float)(accelZ) / 16384;

    gyroX = mpu_data[8] << 8 | mpu_data[9];
    gyro_x = (float)(gyroX - avg_bias_gx) / (131*1000);
    sum_gyro_x += gyro_x;
    gyroY = mpu_data[10] << 8 | mpu_data[11];
    gyro_y = (float)(gyroY - avg_bias_gy) / (131*1000);
    sum_gyro_y += gyro_y;
    gyroZ = mpu_data[12] << 8 | mpu_data[13];
    gyro_z = (float)(gyroZ - avg_bias_gz) / (131*1000);
    sum_gyro_z += gyro_z;

    print_angle = atan2f(accel_z, accel_x);
    print_angle = print_angle * 180.0/M_PI;
    static_angle = atan2f(accel_x, accel_z);
    static_angle = static_angle * 180.0/M_PI;

    theta = (0.02*sum_gyro_y) + (0.98 * static_angle);
    sum_gyro_y = theta;

    mpu_error = 0.7 - theta;
    integral += mpu_error;
    plant_l = plant_l + (integral * ki);
    plant_r = plant_r + (integral * ki);

    if(plant_l > 999)
    {
        plant_l = 999;
    }
    if(plant_r > 999)
    {
        plant_r = 999;
    }
    if(plant_l < 720)
    {
        plant_l = 720;
    }
    if(plant_r < 810)
    {
        plant_r = 810;
    }
    if(count == 0 && flag == 'N' && balance == 'B')
    {
        if(theta > 25 && theta < 70)
        {
            //forward
            left_b(780);
            right_b(840);
        }
        else if(theta < -15 && theta > -70)
        {
            //back
            left_f(780);
            right_f(840);
        }
        else if(theta < -71 && theta > -120)
        {
            //back
            left_f(760);
            right_f(870);
        }
        else if(theta > 71 && theta < 120)
        {
            //forward
            left_b(770);
            right_b(880);
        }
        else
        {
            right_f(0);
            left_f(0);
            right_b(0);
            left_b(0);
        }
    }
    count = (count + 1) & 15;

    TIMER3_ICR_R = TIMER_ICR_TATOCINT;
}

void calibrate_mpu()
{
    float sum_gx = 0;
    float sum_gy = 0;
    float sum_gz = 0;
    int32_t cal = 0;
    for(cal = 0; cal < 2000; cal++)
    {
        readI2c1Registers(ADDR, 0x43, cal_data, 6);
        gyroX = cal_data[0] << 8 | cal_data[1];
        //gyro_x = (float)(gyroX) / (131*1000);
        gyroY = cal_data[2] << 8 | cal_data[3];
        //gyro_y = (float)(gyroY) / (131*1000);
        gyroZ = cal_data[4] << 8 | cal_data[5];
        //gyro_z = (float)(gyroZ) / (131*1000);

        sum_gx +=  gyroX;
        sum_gy +=  gyroY;
        sum_gz +=  gyroZ;
    }
    avg_bias_gx = sum_gx / 2000.0;
    avg_bias_gy = sum_gy / 2000.0;
    avg_bias_gz = sum_gz / 2000.0;
}
int main(void)
{
    initHw();
    initUart0();
    initI2c0();
    initI2c1();
    initMotor();
    USER_DATA data;
    int ind = 0;
    char flag2 = 'N';
    for (ind = 0; ind < 16; ind++)
    {
        opbLtime[ind] = 0;
        opbRtime[ind] = 0;
    }
    initMPU();
    calibrate_mpu();
    waitMicrosecond(1000000);
    enableCounterMode();
    enableTimerModeIR();
    bool valid = false;
    bool command = false;
    char str[40];
    char mode = 'c'; // c: closed loop | o: open loop

    if (mode == 'o')
    {
        while(true)
        {
            mpu = pollI2c1Address(ADDR);
//            left_f(left_pwm);
//            right_f(right_pwm);
            left_b(left_pwm);
            right_b(right_pwm);
//            snprintf(str, sizeof(str), "Collector R RPM: %7.3lf \n", opbRTAVG);
//            putsUart0(str);
//            snprintf(str, sizeof(str), "Collector L RPM: %7.3lf \n", opbLTAVG);
//            putsUart0(str);

            snprintf(str, sizeof(str), "Collector R raw: %"PRIu32" \n", RPM_R);
            putsUart0(str);
            snprintf(str, sizeof(str), "Collector L raw: %"PRIu32" \n", RPM_L);
            putsUart0(str);

            snprintf(str, sizeof(str), "left_pwm: %7.3lf \n", left_pwm);
            putsUart0(str);
            snprintf(str, sizeof(str), "right_pwm: %7.3lf \n", right_pwm);
            putsUart0(str);

            snprintf(str, sizeof(str), "tick_err:  %"PRId32" \n", tick_err);
            putsUart0(str);
            waitMicrosecond(1000000);
        }
    }
    else if(mode == 'c')
    {
        while(true)
        {
            if(kbhitUart0())
            {
                getsUart0(&data);
                parseFields(&data);

                if(isCommand(&data, "reset", 0))
                {
                    valid = true;
                    NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
                }
                else if(isCommand(&data, "angle", 0))
                {
                    valid = true;
                    print_angle = sum_gyro_z;
                    snprintf(str, sizeof(str), "angle: %f \n", print_angle);
                    putsUart0(str);
                }
                else if(isCommand(&data, "clear", 0))
                {
                    valid = true;
                    print_angle = 0;
                    sum_gyro_z = 0;

                }
                else if(isCommand(&data, "tilt", 0))
                {
                    valid = true;
                    snprintf(str, sizeof(str), "Tilt angle: %f \n", static_angle);
                    putsUart0(str);
                }
                else if(isCommand(&data, "forward", 0))
                {
                    valid = true;
                    flag2 = 'F';
                    command = true;
                }
                else if(isCommand(&data, "reverse", 0))
                {
                    valid = true;
                    flag2 = 'B';
                    command = true;
                }
                else if(isCommand(&data, "rotate cw", 0))
                {
                    valid = true;
                    flag2 = 'C';
                    command = true;
                }
                else if(isCommand(&data, "countercw", 0))
                {
                    valid = true;
                    flag2 = 'W';
                    command = true;
                }
                else if(isCommand(&data, "balance on", 0))
                {
                    valid = true;
                    balance = 'B';
                    command = true;
                }
                else if(isCommand(&data, "balance off", 0))
                {
                    valid = true;
                    balance = 'N';
                    command = true;
                }
                if (!valid)
                {
                    putsUart0("Invalid command\n");
                }
                valid = false;
            }

            if (button_pressed == 'Y' || command == true)
            {
                getIR();
                if(flag == 'T' || command == true)
                {
//                    snprintf(str, sizeof(str), "data: %"PRIu32" \n", data_bin);
//                    putsUart0(str);
                    parseIR();
                    if (flag == 'F' || flag2 == 'F')
                    {
                        left_f(830);
                        right_f(879);
//                        left_f(left_pwm);
//                        right_f(right_pwm);
                        waitMicrosecond(500000);
                        left_f(0);
                        right_f(0);
                    }
                    else if (flag == 'B' || flag2 == 'B')
                    {
                        left_b(left_pwm);
                        right_b(right_pwm);
                        waitMicrosecond(500000);
                        left_b(0);
                        right_b(0);
                    }
                    else if (flag == 'C' || flag2 == 'C')
                    {
                        right_f(830);
                        left_b(872);
                        waitMicrosecond(140000);
                        right_f(0);
                        left_b(0);
                    }
                    else if (flag == 'W' || flag2 == 'W')
                    {
                        right_b(830);
                        left_f(872);
                        waitMicrosecond(120000);
                        right_b(0);
                        left_f(0);
                    }
                    data_bin = 0b00000000;
                    addr = 0b00000000;
                    flag = 'N';
                    flag2 = 'N';
                    button_pressed = 'N';
                    command = false;
                }
            }
            waitMicrosecond(500000);
        }
    }
}
