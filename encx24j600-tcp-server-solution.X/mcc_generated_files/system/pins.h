/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.0.0
*/

/*
© [2023] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RA3 aliases
#define ETH_CS_TRIS                 TRISAbits.TRISA3
#define ETH_CS_LAT                  LATAbits.LATA3
#define ETH_CS_PORT                 PORTAbits.RA3
#define ETH_CS_WPU                  WPUAbits.WPUA3
#define ETH_CS_OD                   ODCONAbits.ODCA3
#define ETH_CS_ANS                  ANSELAbits.ANSA3
#define ETH_CS_SetHigh()            do { LATAbits.LATA3 = 1; } while(0)
#define ETH_CS_SetLow()             do { LATAbits.LATA3 = 0; } while(0)
#define ETH_CS_Toggle()             do { LATAbits.LATA3 = ~LATAbits.LATA3; } while(0)
#define ETH_CS_GetValue()           PORTAbits.RA3
#define ETH_CS_SetDigitalInput()    do { TRISAbits.TRISA3 = 1; } while(0)
#define ETH_CS_SetDigitalOutput()   do { TRISAbits.TRISA3 = 0; } while(0)
#define ETH_CS_SetPullup()          do { WPUAbits.WPUA3 = 1; } while(0)
#define ETH_CS_ResetPullup()        do { WPUAbits.WPUA3 = 0; } while(0)
#define ETH_CS_SetPushPull()        do { ODCONAbits.ODCA3 = 0; } while(0)
#define ETH_CS_SetOpenDrain()       do { ODCONAbits.ODCA3 = 1; } while(0)
#define ETH_CS_SetAnalogMode()      do { ANSELAbits.ANSA3 = 1; } while(0)
#define ETH_CS_SetDigitalMode()     do { ANSELAbits.ANSA3 = 0; } while(0)
// get/set IO_RA4 aliases
#define LED3_TRIS                 TRISAbits.TRISA4
#define LED3_LAT                  LATAbits.LATA4
#define LED3_PORT                 PORTAbits.RA4
#define LED3_WPU                  WPUAbits.WPUA4
#define LED3_OD                   ODCONAbits.ODCA4
#define LED3_ANS                  ANSELAbits.ANSA4
#define LED3_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define LED3_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define LED3_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define LED3_GetValue()           PORTAbits.RA4
#define LED3_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define LED3_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define LED3_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define LED3_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define LED3_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define LED3_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define LED3_SetAnalogMode()      do { ANSELAbits.ANSA4 = 1; } while(0)
#define LED3_SetDigitalMode()     do { ANSELAbits.ANSA4 = 0; } while(0)
// get/set IO_RA5 aliases
#define LED2_TRIS                 TRISAbits.TRISA5
#define LED2_LAT                  LATAbits.LATA5
#define LED2_PORT                 PORTAbits.RA5
#define LED2_WPU                  WPUAbits.WPUA5
#define LED2_OD                   ODCONAbits.ODCA5
#define LED2_ANS                  ANSELAbits.ANSA5
#define LED2_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define LED2_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define LED2_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define LED2_GetValue()           PORTAbits.RA5
#define LED2_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define LED2_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define LED2_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define LED2_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define LED2_SetPushPull()        do { ODCONAbits.ODCA5 = 0; } while(0)
#define LED2_SetOpenDrain()       do { ODCONAbits.ODCA5 = 1; } while(0)
#define LED2_SetAnalogMode()      do { ANSELAbits.ANSA5 = 1; } while(0)
#define LED2_SetDigitalMode()     do { ANSELAbits.ANSA5 = 0; } while(0)
// get/set IO_RA6 aliases
#define LED1_TRIS                 TRISAbits.TRISA6
#define LED1_LAT                  LATAbits.LATA6
#define LED1_PORT                 PORTAbits.RA6
#define LED1_WPU                  WPUAbits.WPUA6
#define LED1_OD                   ODCONAbits.ODCA6
#define LED1_ANS                  ANSELAbits.ANSA6
#define LED1_SetHigh()            do { LATAbits.LATA6 = 1; } while(0)
#define LED1_SetLow()             do { LATAbits.LATA6 = 0; } while(0)
#define LED1_Toggle()             do { LATAbits.LATA6 = ~LATAbits.LATA6; } while(0)
#define LED1_GetValue()           PORTAbits.RA6
#define LED1_SetDigitalInput()    do { TRISAbits.TRISA6 = 1; } while(0)
#define LED1_SetDigitalOutput()   do { TRISAbits.TRISA6 = 0; } while(0)
#define LED1_SetPullup()          do { WPUAbits.WPUA6 = 1; } while(0)
#define LED1_ResetPullup()        do { WPUAbits.WPUA6 = 0; } while(0)
#define LED1_SetPushPull()        do { ODCONAbits.ODCA6 = 0; } while(0)
#define LED1_SetOpenDrain()       do { ODCONAbits.ODCA6 = 1; } while(0)
#define LED1_SetAnalogMode()      do { ANSELAbits.ANSA6 = 1; } while(0)
#define LED1_SetDigitalMode()     do { ANSELAbits.ANSA6 = 0; } while(0)
// get/set IO_RA7 aliases
#define LED0_TRIS                 TRISAbits.TRISA7
#define LED0_LAT                  LATAbits.LATA7
#define LED0_PORT                 PORTAbits.RA7
#define LED0_WPU                  WPUAbits.WPUA7
#define LED0_OD                   ODCONAbits.ODCA7
#define LED0_ANS                  ANSELAbits.ANSA7
#define LED0_SetHigh()            do { LATAbits.LATA7 = 1; } while(0)
#define LED0_SetLow()             do { LATAbits.LATA7 = 0; } while(0)
#define LED0_Toggle()             do { LATAbits.LATA7 = ~LATAbits.LATA7; } while(0)
#define LED0_GetValue()           PORTAbits.RA7
#define LED0_SetDigitalInput()    do { TRISAbits.TRISA7 = 1; } while(0)
#define LED0_SetDigitalOutput()   do { TRISAbits.TRISA7 = 0; } while(0)
#define LED0_SetPullup()          do { WPUAbits.WPUA7 = 1; } while(0)
#define LED0_ResetPullup()        do { WPUAbits.WPUA7 = 0; } while(0)
#define LED0_SetPushPull()        do { ODCONAbits.ODCA7 = 0; } while(0)
#define LED0_SetOpenDrain()       do { ODCONAbits.ODCA7 = 1; } while(0)
#define LED0_SetAnalogMode()      do { ANSELAbits.ANSA7 = 1; } while(0)
#define LED0_SetDigitalMode()     do { ANSELAbits.ANSA7 = 0; } while(0)
// get/set IO_RB1 aliases
#define IO_RB1_TRIS                 TRISBbits.TRISB1
#define IO_RB1_LAT                  LATBbits.LATB1
#define IO_RB1_PORT                 PORTBbits.RB1
#define IO_RB1_WPU                  WPUBbits.WPUB1
#define IO_RB1_OD                   ODCONBbits.ODCB1
#define IO_RB1_ANS                  ANSELBbits.ANSB1
#define IO_RB1_SetHigh()            do { LATBbits.LATB1 = 1; } while(0)
#define IO_RB1_SetLow()             do { LATBbits.LATB1 = 0; } while(0)
#define IO_RB1_Toggle()             do { LATBbits.LATB1 = ~LATBbits.LATB1; } while(0)
#define IO_RB1_GetValue()           PORTBbits.RB1
#define IO_RB1_SetDigitalInput()    do { TRISBbits.TRISB1 = 1; } while(0)
#define IO_RB1_SetDigitalOutput()   do { TRISBbits.TRISB1 = 0; } while(0)
#define IO_RB1_SetPullup()          do { WPUBbits.WPUB1 = 1; } while(0)
#define IO_RB1_ResetPullup()        do { WPUBbits.WPUB1 = 0; } while(0)
#define IO_RB1_SetPushPull()        do { ODCONBbits.ODCB1 = 0; } while(0)
#define IO_RB1_SetOpenDrain()       do { ODCONBbits.ODCB1 = 1; } while(0)
#define IO_RB1_SetAnalogMode()      do { ANSELBbits.ANSB1 = 1; } while(0)
#define IO_RB1_SetDigitalMode()     do { ANSELBbits.ANSB1 = 0; } while(0)
// get/set IO_RB2 aliases
#define IO_RB2_TRIS                 TRISBbits.TRISB2
#define IO_RB2_LAT                  LATBbits.LATB2
#define IO_RB2_PORT                 PORTBbits.RB2
#define IO_RB2_WPU                  WPUBbits.WPUB2
#define IO_RB2_OD                   ODCONBbits.ODCB2
#define IO_RB2_ANS                  ANSELBbits.ANSB2
#define IO_RB2_SetHigh()            do { LATBbits.LATB2 = 1; } while(0)
#define IO_RB2_SetLow()             do { LATBbits.LATB2 = 0; } while(0)
#define IO_RB2_Toggle()             do { LATBbits.LATB2 = ~LATBbits.LATB2; } while(0)
#define IO_RB2_GetValue()           PORTBbits.RB2
#define IO_RB2_SetDigitalInput()    do { TRISBbits.TRISB2 = 1; } while(0)
#define IO_RB2_SetDigitalOutput()   do { TRISBbits.TRISB2 = 0; } while(0)
#define IO_RB2_SetPullup()          do { WPUBbits.WPUB2 = 1; } while(0)
#define IO_RB2_ResetPullup()        do { WPUBbits.WPUB2 = 0; } while(0)
#define IO_RB2_SetPushPull()        do { ODCONBbits.ODCB2 = 0; } while(0)
#define IO_RB2_SetOpenDrain()       do { ODCONBbits.ODCB2 = 1; } while(0)
#define IO_RB2_SetAnalogMode()      do { ANSELBbits.ANSB2 = 1; } while(0)
#define IO_RB2_SetDigitalMode()     do { ANSELBbits.ANSB2 = 0; } while(0)
// get/set IO_RB3 aliases
#define IO_RB3_TRIS                 TRISBbits.TRISB3
#define IO_RB3_LAT                  LATBbits.LATB3
#define IO_RB3_PORT                 PORTBbits.RB3
#define IO_RB3_WPU                  WPUBbits.WPUB3
#define IO_RB3_OD                   ODCONBbits.ODCB3
#define IO_RB3_ANS                  ANSELBbits.ANSB3
#define IO_RB3_SetHigh()            do { LATBbits.LATB3 = 1; } while(0)
#define IO_RB3_SetLow()             do { LATBbits.LATB3 = 0; } while(0)
#define IO_RB3_Toggle()             do { LATBbits.LATB3 = ~LATBbits.LATB3; } while(0)
#define IO_RB3_GetValue()           PORTBbits.RB3
#define IO_RB3_SetDigitalInput()    do { TRISBbits.TRISB3 = 1; } while(0)
#define IO_RB3_SetDigitalOutput()   do { TRISBbits.TRISB3 = 0; } while(0)
#define IO_RB3_SetPullup()          do { WPUBbits.WPUB3 = 1; } while(0)
#define IO_RB3_ResetPullup()        do { WPUBbits.WPUB3 = 0; } while(0)
#define IO_RB3_SetPushPull()        do { ODCONBbits.ODCB3 = 0; } while(0)
#define IO_RB3_SetOpenDrain()       do { ODCONBbits.ODCB3 = 1; } while(0)
#define IO_RB3_SetAnalogMode()      do { ANSELBbits.ANSB3 = 1; } while(0)
#define IO_RB3_SetDigitalMode()     do { ANSELBbits.ANSB3 = 0; } while(0)
// get/set IO_RB4 aliases
#define SW0_TRIS                 TRISBbits.TRISB4
#define SW0_LAT                  LATBbits.LATB4
#define SW0_PORT                 PORTBbits.RB4
#define SW0_WPU                  WPUBbits.WPUB4
#define SW0_OD                   ODCONBbits.ODCB4
#define SW0_ANS                  ANSELBbits.ANSB4
#define SW0_SetHigh()            do { LATBbits.LATB4 = 1; } while(0)
#define SW0_SetLow()             do { LATBbits.LATB4 = 0; } while(0)
#define SW0_Toggle()             do { LATBbits.LATB4 = ~LATBbits.LATB4; } while(0)
#define SW0_GetValue()           PORTBbits.RB4
#define SW0_SetDigitalInput()    do { TRISBbits.TRISB4 = 1; } while(0)
#define SW0_SetDigitalOutput()   do { TRISBbits.TRISB4 = 0; } while(0)
#define SW0_SetPullup()          do { WPUBbits.WPUB4 = 1; } while(0)
#define SW0_ResetPullup()        do { WPUBbits.WPUB4 = 0; } while(0)
#define SW0_SetPushPull()        do { ODCONBbits.ODCB4 = 0; } while(0)
#define SW0_SetOpenDrain()       do { ODCONBbits.ODCB4 = 1; } while(0)
#define SW0_SetAnalogMode()      do { ANSELBbits.ANSB4 = 1; } while(0)
#define SW0_SetDigitalMode()     do { ANSELBbits.ANSB4 = 0; } while(0)
/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);


#endif // PINS_H
/**
 End of File
*/