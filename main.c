//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
//#include <string.h>
#include "eeprom.h"
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include <inttypes.h>
#include "wait.h"


//bitband trigpins

#define TRIG1  (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define TRIG2  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define TRIG3  (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))

// PortC and PortD masks
#define ECHO_1 64   //2^4 2^N; PC6; GPI
#define TRIG_1 128   //PC7; GPO
#define ECHO_2 1   //PD0
#define TRIG_2 2  //PD1
#define ECHO_3 4    //PD2
#define TRIG_3 8    //PD3
#define MTR 2   //PF1

#define MAX_CHARS 80
#define MAX_FIELDS 5
typedef struct _USER_DATA
{
    char buffer[MAX_CHARS+1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

bool timeMode = false;
uint32_t frequency = 0;
uint32_t channel = 0;
int distance[3];
uint32_t phase = 0;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
void enableTimerMode()
{
// Configure Timer 1 as the time base
   TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
   TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
   TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
   TIMER1_TAILR_R = 10000000;                       // set load value to 40e6 for 1 Hz interrupt rate
   TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
   TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
   NVIC_EN0_R = 1 << (INT_TIMER1A-16-0);             // turn-on interrupt 37 (TIMER1A)
// change load 1000000 and nvic  -0
    /******/
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_BOTH;           // measure time from positive edge to positive edge
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
//    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);         // turn-on interrupt 112 (WTIMER1A)

    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER2_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_BOTH;           // measure time from positive edge to positive edge
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER2_TAV_R = 0;                               // zero counter for first period
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
//    NVIC_EN3_R = 1 << (INT_WTIMER2A-16-96);         // turn-on interrupt 112 (WTIMER1A)

    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER3_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR;
                                                     // configure for edge time mode, count up
    WTIMER3_CTL_R = TIMER_CTL_TAEVENT_BOTH;           // measure time from positive edge to positive edge
    WTIMER3_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER3_TAV_R = 0;                               // zero counter for first period
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
//    NVIC_EN3_R = 1 << (INT_WTIMER3A-16-96);         // turn-on interrupt 112 (WTIMER1A)

    NVIC_EN3_R = 1 << (INT_WTIMER1A-16-96);         // turn-on interrupt 112 (WTIMER1A)
    NVIC_EN3_R = 1 << (INT_WTIMER2A-16-96);         // turn-on interrupt 112 (WTIMER1A)
    NVIC_EN3_R = 1 << (INT_WTIMER3A-16-96);         // turn-on interrupt 112 (WTIMER1A)

}

void timerISR()
{
    if(phase != 2)
    {
        distance[channel] = -1;
    }
    channel++;
    phase = 0;
    if(channel > 2)
    {
        channel = 0;
    }
    if(channel == 0)
    {
        TRIG1 = 1;
        waitMicrosecond(10);
        TRIG1 = 0;
        //channel++;
    }
    else if(channel == 1)
    {
        TRIG2 = 1;
        waitMicrosecond(10);
        TRIG2 = 0;
        //channel++;
    }
    else if(channel == 2)
    {
        TRIG3 = 1;
        waitMicrosecond(10);
        TRIG3 = 0;
        //channel = 0;
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void wideTimer1Isr()
{
    if(phase == 0)
    {
        WTIMER1_TAV_R = 0;                           // zero counter for next edge
        phase = 1;
    }
    else if(phase == 1)
    {
        distance[channel] = WTIMER1_TAV_R;                        // read counter input
        distance[channel] = (distance[channel] * (345 * 0.000000025 * 1000)) / 2 ;
        phase = 2;
    }
    WTIMER1_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}
void wideTimer2Isr()
{
    if(phase == 0)
    {
        WTIMER2_TAV_R = 0;                           // zero counter for next edge
        phase = 1;
    }
    else if(phase == 1)
    {
        distance[channel] = WTIMER2_TAV_R;                        // read counter input
        distance[channel] = (distance[channel] * (345 * 0.000000025 * 1000)) / 2 ;
        phase = 2;
    }
    WTIMER2_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void wideTimer3Isr()
{
    if(phase == 0)
    {
        WTIMER3_TAV_R = 0;                           // zero counter for next edge
        phase = 1;
    }
    else if(phase == 1)
    {
        distance[channel] = WTIMER3_TAV_R;                        // read counter input
        distance[channel] = (distance[channel] * (345 * 0.000000025 * 1000)) / 2 ;
        phase = 2;
    }
    WTIMER3_ICR_R = TIMER_ICR_CAECINT;           // clear interrupt flag
}

void enable1()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    //NVIC_EN0_R = 1 << (INT_TIMER1A-16-0);
}
void disableTimerMode()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    //NVIC_DIS3_R = 1 << (INT_TIMER1A-16-0);
}

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
// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1 | SYSCTL_RCGCWTIMER_R2 | SYSCTL_RCGCWTIMER_R3;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);                                                        // enable LEDs and pushbuttons

    // Configure SIGNAL_IN for frequency and time measurements
    //GPI
    GPIO_PORTC_AFSEL_R |= ECHO_1;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= ECHO_1;                // enable bit 6 for digital input

    GPIO_PORTD_AFSEL_R |= ECHO_2;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD0_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD0_WT2CCP0;
    GPIO_PORTD_DEN_R |= ECHO_2;                // enable bit 6 for digital input

    GPIO_PORTD_AFSEL_R |= ECHO_3;              // select alternative functions for SIGNAL_IN pin
    GPIO_PORTD_PCTL_R &= ~GPIO_PCTL_PD2_M;           // map alt fns to SIGNAL_IN
    GPIO_PORTD_PCTL_R |= GPIO_PCTL_PD2_WT3CCP0;
    GPIO_PORTD_DEN_R |= ECHO_3;                // enable bit 6 for digital input

    //GPO
    GPIO_PORTC_DIR_R |= TRIG_1;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTD_DIR_R |= TRIG_2 | TRIG_3;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTC_DR2R_R |= TRIG_1; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DR2R_R |= TRIG_2 | TRIG_3; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= TRIG_1;
    GPIO_PORTD_DEN_R |= TRIG_2 | TRIG_3;
}
void initmtr()
{
   SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
   SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
   _delay_cycles(3);

   // Configure three LEDs
   GPIO_PORTF_DEN_R |= MTR;
   GPIO_PORTF_AFSEL_R |= MTR;
   GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
   GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

   SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
   SYSCTL_SRPWM_R = 0;                              // leave reset state
   PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
   PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
   PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                    // output 5 on PWM1, gen 2b, cmpb
   PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                    // output 6 on PWM1, gen 3a, cmpa
   PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                    // output 7 on PWM1, gen 3b, cmpb

   PWM1_2_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
   PWM1_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)

   PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
   PWM1_3_CMPB_R = 0;                               // green off
   PWM1_3_CMPA_R = 0;                               // blue off

   PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
   PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
   PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                    // enable outputs
}

void setMTR(uint16_t x)
{
    PWM1_2_CMPB_R = x;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    waitMicrosecond(500000);
    initHw();
    initEeprom();
    initmtr();
    enableTimerMode();

    //waitMicrosecond(2000000);
    //setUart0BaudRate(115200, 40e6);
    USER_DATA data;
    initUart0();

    putsUart0("\nWelcome!\n");
    putsUart0("Enter Values now: \n");
    bool valid = false;
    bool rmw = false;
    bool print_dist = false;
    uint16_t events_to_run = 19;
    uint16_t sensor = 0;
    uint16_t min_dist = 1;
    uint16_t max_dist = 2;
    uint16_t haptic = 3;
    uint16_t beat_count = 4;
    uint16_t beat_on_time = 5;
    uint16_t beat_off_time = 6;
    uint16_t pwm = 7;
    uint16_t first_ev = 0;
    uint16_t second_ev = 1;
    //uint16_t empty = 2;
    char str1[40];

    //0,1,2; 4 different events for each sensor and 1 extra for any sensor (wall, person, trip hazard, compound events for stairs)
    // print distance and confirm limit 0- 400(min) 1- 300(min) 2- 560(large)
    //sensor 0 - event(0, 3, 6, 9, 12, 15) 3000; 2000 ; 1000; 500
    //sensor 1 - event(1, 4, 7, 10, 13)
    //sensor 2 - event(2, 5, 8, 11, 14)
    // write to eeprom
    //EVENT 0
/*
    writeEeprom(0, 0);  writeEeprom(1, 500);  writeEeprom(2, 600);  writeEeprom(3, 1);
    writeEeprom(4, 1);  writeEeprom(5, 100);  writeEeprom(6, 100);  writeEeprom(7, 80);
    //EVENT 1
    writeEeprom(8, 1);  writeEeprom(9, 400);  writeEeprom(10, 890);  writeEeprom(11, 1);
    writeEeprom(12, 1);  writeEeprom(13, 100);  writeEeprom(14, 100);  writeEeprom(15, 80);
    //EVENT 2
    writeEeprom(16, 2);  writeEeprom(17, 350);  writeEeprom(18, 560);  writeEeprom(19, 1);
    writeEeprom(20, 2);  writeEeprom(21, 100);  writeEeprom(22, 100);  writeEeprom(23, 85);
    //EVENT 3
    writeEeprom(24, 0);  writeEeprom(25, 500);  writeEeprom(26, 600);  writeEeprom(27, 1);
    writeEeprom(28, 1);  writeEeprom(29, 100);  writeEeprom(30, 100);  writeEeprom(31, 85);
    //EVENT 4
    writeEeprom(32, 1);  writeEeprom(33, 350);  writeEeprom(34, 400);  writeEeprom(35, 1);
    writeEeprom(36, 1);  writeEeprom(37, 100);  writeEeprom(38, 100);  writeEeprom(39, 85);
    //EVENT 5
    writeEeprom(40, 2);  writeEeprom(41, 300);  writeEeprom(42, 349);  writeEeprom(43, 1);
    writeEeprom(44, 2);  writeEeprom(45, 100);  writeEeprom(46, 100);  writeEeprom(47, 85);
    //EVENT 6
    writeEeprom(48, 0);  writeEeprom(49, 500);  writeEeprom(50, 600);  writeEeprom(51, 1);
    writeEeprom(52, 1);  writeEeprom(53, 100);  writeEeprom(54, 100);  writeEeprom(55, 85);
    //EVENT 7
    writeEeprom(56, 1);  writeEeprom(57, 400);  writeEeprom(58, 500);  writeEeprom(59, 1);
    writeEeprom(60, 1);  writeEeprom(61, 100);  writeEeprom(62, 100);  writeEeprom(63, 80);
    //EVENT 8
    writeEeprom(64, 2);  writeEeprom(65, 250);  writeEeprom(66, 299);  writeEeprom(67, 1);
    writeEeprom(68, 2);  writeEeprom(69, 100);  writeEeprom(70, 100);  writeEeprom(71, 80);
    //EVENT 9
    writeEeprom(72, 0);  writeEeprom(73, 500);  writeEeprom(74, 600);  writeEeprom(75, 1);
    writeEeprom(76, 1);  writeEeprom(77, 100);  writeEeprom(78, 100);  writeEeprom(79, 80);
    //EVENT 10
    writeEeprom(80, 1);  writeEeprom(81, 400);  writeEeprom(82, 500);  writeEeprom(83, 1);
    writeEeprom(84, 1);  writeEeprom(85, 100);  writeEeprom(86, 100);  writeEeprom(87, 80);
    //EVENT 11
    writeEeprom(88, 2);  writeEeprom(89, 200);  writeEeprom(90, 300);  writeEeprom(91, 1);
    writeEeprom(92, 1);  writeEeprom(93, 100);  writeEeprom(94, 100);  writeEeprom(95, 80);
    //EVENT 12
    writeEeprom(96, 0);  writeEeprom(97, 400);  writeEeprom(98, 600);  writeEeprom(99, 1);
    writeEeprom(100, 1);  writeEeprom(101, 100);  writeEeprom(102, 100);  writeEeprom(103, 80);
    //EVENT 13
    writeEeprom(104, 1);  writeEeprom(105, 1);  writeEeprom(106, 300);  writeEeprom(107, 1);
    writeEeprom(108, 1);  writeEeprom(109, 100);  writeEeprom(110, 100);  writeEeprom(111, 80);
    //EVENT 14
    writeEeprom(112, 2);  writeEeprom(113, 1);  writeEeprom(114, 200);  writeEeprom(115, 1);
    writeEeprom(116, 1);  writeEeprom(117, 100);  writeEeprom(118, 100);  writeEeprom(119, 80);
    //EVENT 15
    writeEeprom(120, 0);  writeEeprom(121, 1);  writeEeprom(122, 400);  writeEeprom(123, 1);
    writeEeprom(124, 1);  writeEeprom(125, 100);  writeEeprom(126, 100);  writeEeprom(127, 80);

    //EVENT 16 stairs up
    writeEeprom(128, 9);  writeEeprom(129, 10);  writeEeprom(130, 0);  writeEeprom(131, 1);
    writeEeprom(132, 3);  writeEeprom(133, 100);  writeEeprom(134, 100);  writeEeprom(135, 85);
    //EVENT 17 stairs down
    writeEeprom(136, 1);  writeEeprom(137, 2);  writeEeprom(138, 0);  writeEeprom(139, 1);
    writeEeprom(140, 3);  writeEeprom(141, 100);  writeEeprom(142, 100);  writeEeprom(143, 85);
    //EVENT 18 low stairs
    writeEeprom(144, 4);  writeEeprom(145, 5);  writeEeprom(146, 0);  writeEeprom(147, 1);
    writeEeprom(148, 3);  writeEeprom(149, 100);  writeEeprom(150, 100);  writeEeprom(151, 85);
    //EVENT 19 weird obstacle
    writeEeprom(152, 1);  writeEeprom(153, 2);  writeEeprom(154, 0);  writeEeprom(155, 1);
    writeEeprom(156, 3);  writeEeprom(157, 100);  writeEeprom(158, 100);  writeEeprom(159, 85);

*/

    while (true)
    {
        waitMicrosecond(10000);
        if(kbhitUart0())
        {
            getsUart0(&data);
            parseFields(&data);
            /*uint8_t i;
            for(i = 0; i < data.fieldCount; i++)
            {
                putcUart0(data.fieldType[i]);
                putcUart0('\t');
                putsUart0(&data.buffer[data.fieldPosition[i]]);
                putcUart0('\n');
            }*/
            if(isCommand(&data, "reboot", 0))
            {
                valid = true;
                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            }
            if(isCommand(&data, "event", 4))
            {
                valid = true;
                uint16_t event_n = getFieldInteger(&data, 1);
                uint32_t sensor_n = getFieldInteger(&data, 2);
                uint32_t min_d = getFieldInteger(&data, 3);
                uint32_t max_d = getFieldInteger(&data, 4);
                writeEeprom((8*event_n)+sensor, sensor_n);
                writeEeprom((8*event_n)+min_dist, min_d);
                writeEeprom((8*event_n)+max_dist, max_d);
            }
            if(isCommand(&data, "and", 3))
            {
                valid = true;
                uint16_t event_n = getFieldInteger(&data, 1);
                uint16_t event1_n = getFieldInteger(&data, 2);
                uint16_t event2_n = getFieldInteger(&data, 3);
                writeEeprom((8*event_n)+first_ev, event1_n);
                writeEeprom((8*event_n)+second_ev, event2_n);
            }
            if (isCommand(&data, "erase", 1))
            {
                valid = true;
                uint16_t event_n = getFieldInteger(&data, 1);
                writeEeprom((8*event_n)+sensor, 0);
                writeEeprom((8*event_n)+min_dist, 0);
                writeEeprom((8*event_n)+max_dist, 0);
                writeEeprom((8*event_n)+haptic, 0);
                writeEeprom((8*event_n)+beat_count, 0);
                writeEeprom((8*event_n)+beat_on_time, 0);
                writeEeprom((8*event_n)+beat_off_time, 0);
                writeEeprom((8*event_n)+pwm, 0);
            }
            if (isCommand(&data, "show events", 0))
            {
                valid = true;
                uint16_t j;
                for(j = 0; j < 20; j++)
                {
                    if(strcmp(&data.buffer[data.fieldPosition[1]], "events") == 0)
                    {
                        if((j == 19) || (j == 18) || (j == 17) || (j == 16))
                        {
                            snprintf(str1, sizeof(str1), "For Event %"PRIu32": \n", j);
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "First Compound Event: %1"PRIu32" \n", readEeprom((j*8)+0));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Sensor: %1"PRIu32" \n", readEeprom((readEeprom((j*8)+0)*8)+0));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Minimum distance: %1"PRIu32" (mm)\n", readEeprom((readEeprom((j*8)+0)*8)+1));
                            putsUart0(str1);
                            //(readEeprom((j*8)+0) = 1
                            snprintf(str1, sizeof(str1), "Maximum distance: %1"PRIu32" (mm)\n", readEeprom((readEeprom((j*8)+0)*8)+2));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Second Compound Event: %1"PRIu32" \n", readEeprom((j*8)+1));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Sensor: %1"PRIu32" \n", readEeprom((readEeprom((j*8)+1)*8)+0));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Minimum distance: %1"PRIu32" (mm)\n", readEeprom((readEeprom((j*8)+1)*8)+1));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Maximum distance: %1"PRIu32" (mm)\n", readEeprom((readEeprom((j*8)+1)*8)+2));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Haptic: %1"PRIu32" \n", readEeprom((j*8)+3));
                            putsUart0(str1);
                            if(readEeprom((j*8)+3) == 0)
                            {
                                putsUart0("Haptic: OFF\n");
                            }
                            else if(readEeprom((j*8)+3) == 1)
                            {
                                putsUart0("Haptic: ON\n");
                            }
                            snprintf(str1, sizeof(str1), "Beat Count: %1"PRIu32" \n", readEeprom((j*8)+4));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Beat on time: %1"PRIu32" \n", readEeprom((j*8)+5));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Beat off time: %1"PRIu32" \n", readEeprom((j*8)+6));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "PWM: %1"PRIu32" \n", readEeprom((j*8)+7));
                            putsUart0(str1);
                        }
                        else
                        {
                            snprintf(str1, sizeof(str1), "For Event %"PRIu32": \n", j);
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Sensor: %1"PRIu32" \n", readEeprom((j*8)+0));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Minimum distance: %1"PRIu32" (mm)\n", readEeprom((j*8)+1));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Maximum distance: %1"PRIu32" (mm)\n", readEeprom((j*8)+2));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Haptic: %1"PRIu32" \n", readEeprom((j*8)+3));
                            putsUart0(str1);
                            if(readEeprom((j*8)+3) == 0)
                            {
                                putsUart0("Haptic: OFF\n");
                            }
                            else if(readEeprom((j*8)+3) == 1)
                            {
                                putsUart0("Haptic: ON\n");
                            }
                            snprintf(str1, sizeof(str1), "Beat Count: %1"PRIu32" \n", readEeprom((j*8)+4));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Beat on time: %1"PRIu32" \n", readEeprom((j*8)+5));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "Beat off time: %1"PRIu32" \n", readEeprom((j*8)+6));
                            putsUart0(str1);
                            snprintf(str1, sizeof(str1), "PWM: %1"PRIu32" \n", readEeprom((j*8)+7));
                            putsUart0(str1);
                        }
                    }
                }
            }
            if (isCommand(&data, "haptic", 2))
            {
                valid = true;
                uint16_t event_n = getFieldInteger(&data, 1);
                char* str = getFieldString(&data, 2);
                int value = strcmp(&data.buffer[data.fieldPosition[2]], "on");
                int value1 = strcmp(&data.buffer[data.fieldPosition[2]], "off");
                if(value == 0)
                {
                    writeEeprom((8*event_n)+haptic, 1);
                }
                else if(value1 == 0)
                {
                    writeEeprom((8*event_n)+haptic, 0);
                }
                else
                {
                    putsUart0("wrong input\n");
                }
            }
            if (isCommand(&data, "pattern", 5))
            {
                valid = true;
                uint16_t event_n = getFieldInteger(&data, 1);
                uint32_t pwm_perc = getFieldInteger(&data, 2);
                uint32_t beats = getFieldInteger(&data, 3);
                uint32_t on_time = getFieldInteger(&data, 4);
                uint32_t off_time = getFieldInteger(&data, 5);
                writeEeprom((8*event_n)+pwm, pwm_perc);
                writeEeprom((8*event_n)+beat_count, beats);
                writeEeprom((8*event_n)+beat_on_time, on_time);
                writeEeprom((8*event_n)+beat_off_time, off_time);
            }
            if (isCommand(&data, "show patterns", 0))
            {
                if(strcmp(&data.buffer[data.fieldPosition[1]], "patterns") == 0)
                {
                    valid = true;
                    uint16_t j;
                    for(j = 0; j < 20; j++)
                    {
                        snprintf(str1, sizeof(str1), "For Event %"PRIu32": \n", j);
                        putsUart0(str1);
                        snprintf(str1, sizeof(str1), "Haptic: %1"PRIu32" \n", readEeprom((j*8)+3));
                        putsUart0(str1);
                        if(readEeprom((j*8)+3) == 0)
                        {
                            putsUart0("Haptic: OFF\n");
                        }
                        else if(readEeprom((j*8)+3) == 1)
                        {
                            putsUart0("Haptic: ON\n");
                        }
                        snprintf(str1, sizeof(str1), "Beat Count: %1"PRIu32" \n", readEeprom((j*8)+4));
                        putsUart0(str1);
                        snprintf(str1, sizeof(str1), "Beat on time: %1"PRIu32" \n", readEeprom((j*8)+5));
                        putsUart0(str1);
                        snprintf(str1, sizeof(str1), "Beat off time: %1"PRIu32" \n", readEeprom((j*8)+6));
                        putsUart0(str1);
                        snprintf(str1, sizeof(str1), "PWM: %1"PRIu32" \n", readEeprom((j*8)+7));
                        putsUart0(str1);
                    }
                }
            }
            if (isCommand(&data, "print distance", 0))
            {
                valid = true;
                print_dist = true;
            }

            if (isCommand(&data, "stop distance", 0))
            {
                valid = true;
                print_dist = false;
            }

            if (!valid)
            {
                putsUart0("Invalid command\n");
            }
            valid = false;
        }

        if(print_dist)
        {
            snprintf(str1, sizeof(str1), "distance1: %7"PRIu32" (mm)\n", distance[0]);
            putsUart0(str1);
            snprintf(str1, sizeof(str1), "distance2: %7"PRIu32" (mm)\n", distance[1]);
            putsUart0(str1);
            snprintf(str1, sizeof(str1), "distance3: %7"PRIu32" (mm)\n", distance[2]);
            putsUart0(str1);
        }


        uint16_t i;
        for(i = events_to_run; i; i--) // try i != -1; and (i =n-1; i+1>0; i--)
        {
            if((i == 19) || (i == 18) || (i == 17) || (i == 16))
            {
            // what if both events are erased
            // still play event if both haptics are off?
            // different beats?
            // set flag to break
            // check both sensors; dont always print distance so distance command
                uint16_t tempev1 = readEeprom((8*i)+first_ev);
                uint16_t tempev2 = readEeprom((8*i)+second_ev);
                if((readEeprom((tempev1*8)+sensor) != readEeprom((tempev2*8)+sensor)))
                {
                    if((readEeprom((tempev1*8)+max_dist) != 0) && (readEeprom((tempev2*8)+max_dist) != 0))
                     {
                        if(readEeprom((8*i)+pwm) != 0)
                        {
                          if(((distance[readEeprom((tempev1*8)+sensor)] >= readEeprom((tempev1*8)+min_dist)) && (distance[readEeprom((tempev1*8)+sensor)] <= readEeprom((tempev1*8)+max_dist)))
                          &&((distance[readEeprom((tempev2*8)+sensor)] >= readEeprom((tempev2*8)+min_dist)) && (distance[readEeprom((tempev2*8)+sensor)] <= readEeprom((tempev2*8)+max_dist))))
                          {
                              if((readEeprom((tempev1*8)+haptic) == 1) && (readEeprom((tempev2*8)+haptic) == 1))
                              {
                                  if(readEeprom((8*i)+haptic) == 1)
                                  {
                                      //putsUart0("works compound\n");

                                      snprintf(str1, sizeof(str1), "This is Event: %1"PRIu32": \n", i);
                                      putsUart0(str1);
                                      //waitMicrosecond(1000000);
                                      uint16_t j;
                                      //disableTimerMode();
                                      for(j = 0; j < readEeprom((i*8)+beat_count); j++)
                                      {

                                          setMTR((readEeprom((i*8)+pwm)*1024)/100);
                                          waitMicrosecond(readEeprom((i*8)+beat_on_time) * 1000);
                                          setMTR(0);
                                          waitMicrosecond(readEeprom((i*8)+beat_off_time) * 1000);
                                          rmw = true;

                                      }
                                     // enable1();
                                      waitMicrosecond(1000000);
                                  }
                              }
                          }
                        }
                     }
                }
            }
            else
            {
                if(readEeprom((8*i)+max_dist) != 0)
                {
                    if((distance[readEeprom((8*i)+sensor)] >= readEeprom((8*i)+min_dist)) && (distance[readEeprom((8*i)+sensor)] <= readEeprom((8*i)+max_dist)))
                    {
                        if(readEeprom((i*8)+haptic) == 1)
                        {
                            //putsUart0("works\n");
                            snprintf(str1, sizeof(str1), "This is Event: %1"PRIu32": \n", i);
                            putsUart0(str1);
                            //waitMicrosecond(1000000);
                            uint16_t j;
                            //disableTimerMode();
                            for(j = 0; j < readEeprom((i*8)+beat_count); j++)
                            {

                                setMTR((readEeprom((i*8)+pwm)*1024)/100);
                                waitMicrosecond(readEeprom((i*8)+beat_on_time)*1000);
                                setMTR(0);
                                waitMicrosecond(readEeprom((i*8)+beat_off_time)*1000);
                                rmw = true;

                            }
                            //enable1();
                           waitMicrosecond(1000000);
                        }
                    }
                }
            }
            if(rmw)
            {
                rmw = false;
                break;
            }
        }
        waitMicrosecond(100000);
    }
/*
 for loop goes through eeprom table comparing distances of each sensor (for loop decrements from 19 to 0),
 check if event is erased and check if haptic is on or off after event is found not before, if it is just break
 check sensor number and use that number for distance[sensor number] compare to max and min distance
 nested for loop in if statement to play beats (i < beat_count); wait microseconds for time on and off
 compound event uses two events, so separate if statements for 19 - 16
 if any event true play event and break or goto
 do compound events have different beats and haptic? if haptic off do we still play event?
 change show events and pattern for compound
 erase compound events
 shorter times
 garbage values when motot runs
 object detection
 */
    return 0;
}

