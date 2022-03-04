/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************
#include <stdio.h>
#include <math.h>

#define _XTAL_FREQ          200000000               //200MHz
#define _XTAL_FREQ_DIV1MHz  (_XTAL_FREQ / 1000000)

char str[100];
uint8_t serial_buf[256];
int serial_buf_counter = 0;
unsigned int start_time,end_time;
bool is_trig_fall;
float trig_level;
int oscillo_mode;

volatile uint32_t *CH1_ADDR = (&ADCDATA0) + ADCHS_CH1;
volatile uint32_t *CH2_ADDR = (&ADCDATA0) + ADCHS_CH2;
volatile uint32_t *CH3_ADDR = (&ADCDATA0) + ADCHS_CH3;
volatile uint32_t *CH4_ADDR = (&ADCDATA0) + ADCHS_CH4;

void delay_ms(unsigned int ms){
    unsigned int i,time;
    time=250*_XTAL_FREQ_DIV1MHz*ms;
    for(i=0;i<time;i++){
        Nop();
    }
}

void cobs_decode(uint8_t return_data[], int *return_size, uint8_t packet[], int size){
    *return_size = 0;
    if(size > 256){
        //packet length must be less than 257
        return ;
    }

    int len = size;

    int zero_index = (int)(packet[0] & 0xff);

    int i;
    for(i=1;i<len-1;i++){
        if(i == zero_index){
            return_data[i-1] = 0;
            zero_index = i + (int)(packet[i] & 0xff);
        }else{
            return_data[i-1] = packet[i];
        }
    }
    if((packet[len-1] == 0) && (zero_index == len-1)){
        *return_size = len - 2;
        return;
    }else{
        //zero_index error
        return;
    }
}

int main ( void )
{   
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    //harmony3 user    
    UART5_Initialize();
    TMR4_Start();
    LED_R_Clear();
    char str[]="32MZ setup end\r\n";
    UART1_Write((uint8_t*)str, sizeof(str));
    
    bool is_send_voltage=false;
    unsigned int voltage_buf[4][1024];
    uint8_t send_buf[4][1024];
    int voltage_buf_counter=0;
//    
//    while(1){
//        char recv;
//        if(UART5_Read(&recv,1)){
//            UART1_Write(&recv, 1);
//        }
//    }
    
    while ( true )
    {
        uint8_t recv;
        while(is_send_voltage){
            //start_time=TMR4; 
            //while(!ADCHS_ChannelResultIsReady(ADCHS_CH0)){
            while(!(ADCDSTAT1 & 0x02));
            //while(!((ADCDSTAT1 >> ADCHS_CH2) & 0x01));
            //while(!((ADCDSTAT1 >> ADCHS_CH3) & 0x01));
            //while(!((ADCDSTAT1 >> ADCHS_CH4) & 0x01));
            //end_time=TMR4; 
            
            int tmp;
            float vol;
            
            switch(oscillo_mode){
                case 0://trigger準備
                    tmp = *CH1_ADDR & 0x0fff;
                    vol = tmp * 3.3 / 4096.0;
                    if(is_trig_fall){
                        if(vol > trig_level){
                            oscillo_mode = 1;
                        }
                    }else{
                        if(vol < trig_level){
                            oscillo_mode = 1;
                        }
                    }
                    break;
                case 1://trigger本番
                    tmp = *CH1_ADDR & 0x0fff;
                    vol = tmp * 3.3 / 4096.0;
                    if(is_trig_fall){
                        if(vol < trig_level){
                            voltage_buf[0][voltage_buf_counter] = tmp;
                            voltage_buf[1][voltage_buf_counter] = (*((&ADCDATA0) + ADCHS_CH2));
                            voltage_buf[2][voltage_buf_counter] = (*((&ADCDATA0) + ADCHS_CH3));
                            voltage_buf[3][voltage_buf_counter] = (*((&ADCDATA0) + ADCHS_CH4));
                            voltage_buf_counter++;
                            oscillo_mode = 2;
                        }
                    }else{
                        if(vol > trig_level){
                            voltage_buf[0][voltage_buf_counter] = tmp;
                            voltage_buf[1][voltage_buf_counter] = *CH2_ADDR;
                            voltage_buf[2][voltage_buf_counter] = *CH3_ADDR;
                            voltage_buf[3][voltage_buf_counter] = *CH4_ADDR;
                            voltage_buf_counter++;
                            oscillo_mode = 2;
                        }
                    }
                    break;
                case 2://データ記録
                    while(voltage_buf_counter < 1000){
                        while(!(ADCDSTAT1 & 0x02));
                        voltage_buf[0][voltage_buf_counter] = *CH1_ADDR;
                        voltage_buf[1][voltage_buf_counter] = *CH2_ADDR;
                        voltage_buf[2][voltage_buf_counter] = *CH3_ADDR;
                        voltage_buf[3][voltage_buf_counter] = *CH4_ADDR;
                        voltage_buf_counter++;
                    }
                    if(voltage_buf_counter>=1000){
                        TMR5_Stop();
                        is_send_voltage = false;
                        int i;
                        for(i=0;i<1000;i++){
                            send_buf[0][i] = (voltage_buf[0][i] >> 4) & 0xff;
                            send_buf[1][i] = (voltage_buf[1][i] >> 4) & 0xff;
                            send_buf[2][i] = (voltage_buf[2][i] >> 4) & 0xff;
                            send_buf[3][i] = (voltage_buf[3][i] >> 4) & 0xff;
                        }
                        for(i=0;i<10;i++){
                            UART1_Write(&send_buf[0][i*100],100);
                            delay_ms(30);
                        }
                        for(i=0;i<10;i++){
                            UART1_Write(&send_buf[1][i*100],100);
                            delay_ms(30);
                        }
                        for(i=0;i<10;i++){
                            UART1_Write(&send_buf[2][i*100],100);
                            delay_ms(30);
                        }
                        for(i=0;i<10;i++){
                            UART1_Write(&send_buf[3][i*100],100);
                            delay_ms(30);
                        }
                        LED_R_Clear();
                    }
                    break; 
            }  
        }
        if(UART1_ReadCountGet() != 0){
            if(UART1_Read(&recv,1)){
                serial_buf[serial_buf_counter] = recv;
                serial_buf_counter++;
                if(recv == 0){
                    uint8_t data[256];
                    int data_size;
                    cobs_decode(data, &data_size, serial_buf, serial_buf_counter);
                    serial_buf_counter = 0;
                    if(data_size == 3){
                        if(data[0] & 0x80){
                            voltage_buf_counter = 0;
                            is_send_voltage = true;
                            oscillo_mode = 0;
                            if(data[0] & 0x40){
                                is_trig_fall = true;
                            }else{
                                is_trig_fall = false;
                            }
                            trig_level = (float)(data[0] & 0x3f) / 10.0;
                            //TMR5は1count=10nsであることに注意してperiodを計算する
                            uint32_t sampling_interval = (((data[1] << 8) & 0xff00) + (data[2] & 0xff)) * 10;//1count = 10ns
                            if(sampling_interval < 50){
                                sampling_interval = 50;
                            }
                            TMR5_PeriodSet(sampling_interval - 1);
                            TMR5_Start();
                            LED_R_Set();
                        }
                    }
                }else if(serial_buf_counter > 255){
                    //error
                    serial_buf_counter = 0;
                }
            } 
        }
                
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

