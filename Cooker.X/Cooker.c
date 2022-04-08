#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#include <xc.h>
#include <stdio.h>
#include "my_adc.h"
#include "lcd_x8.h"
#include <xc.h>

#define STARTVALUE  3036
int hours = 0;
int min = 0;
int sec = 0;
float SP;
float Temp;
int H = 1;
int heater = 0;
int cooker = 0;
int mode = 0;

void buzzerON(void) {
    //BUZZER
    for (int i = 0; i < 4; i++) {
        PORTCbits.RC1 = 1;
        delay_ms(1000);
        PORTCbits.RC1 = 0;
        delay_ms(1000);
    }
}

void reloadTimer0(void) {
    TMR3H = (unsigned char) ((STARTVALUE >> 8) & 0x00FF);
    TMR3L = (unsigned char) (STARTVALUE & 0x00FF);
}

void Timer0_isr(void) {

    INTCONbits.TMR0IF = 0;
    if (sec > 0)
        sec--;
    else if (min > 0) {
        sec = 59;
        min--;
    } else if (hours > 0) {
        hours--;
        min = 59;
        sec = 59;
    } else {
        hours = 0;
        min = 0;
        sec = 0;
        cooker = 0;
        heater = 0;
        PORTCbits.RC5 = 0;
        T0CONbits.TMR0ON = 0; // WE NEED TO TURN OFF THE TIMER
        buzzerON();
    }
    reloadTimer0();
}

void EXT_Int0_isr(void) {
    INTCONbits.INT0IF = 0;
    if (mode == 4) {
        mode = 0;
    } else {
        mode++;
    }
}

void EXT_Int1_isr(void) {
    INTCON3bits.INT1IF = 0;
    if((hours+min+sec)>0){
        T0CONbits.TMR0ON = 1;
        cooker = 1;
    }
}

void EXT_Int2_isr(void) {
    INTCON3bits.INT2IF = 0;
    T0CONbits.TMR0ON = 0;
    cooker = 0;
    heater = 0;
    PORTCbits.RC5 = 0;
}

void __interrupt(high_priority)highIsr(void) {
    
    if (INTCONbits.TMR0IF) Timer0_isr();
    else if (INTCONbits.INT0IF)EXT_Int0_isr();
    else if (INTCON3bits.INT1IF)EXT_Int1_isr();
    else if (INTCON3bits.INT2IF)EXT_Int2_isr();
    
}

void delay_ms(unsigned int n) {
    int i;
    for (i = 0; i < n; i++) {
        __delaywdt_ms(1);
    }
}

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; // Because We need AN2/AN0, Then we need 3 analog inputs
    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x00; // outputs
    PORTC = 0;
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

void display(void) {
    char HT[6];
    char CK[6];
    char LCD[64];
    char LCDP[64];
    
    switch (mode) {
        case 0:
            sprintf(LCDP, "Sec          ");
            break;

        case 1:
            sprintf(LCDP, "10Sec          ");
            break;
        case 2:
            sprintf(LCDP, "Min          ");
            break;
        case 3:
            sprintf(LCDP, "10Min      ");
            break;
        case 4:
            sprintf(LCDP, "HR        ");
            break;
    }

    if (heater) {
        sprintf(HT, "HT: ON");
    } else {
        sprintf(HT, "HT:OFF");
    }

    if (cooker) {
        sprintf(CK, "CK: ON");
    } else {
        sprintf(CK, "CK:OFF");
    }
    //////////////////////////////////////////////////////
    lcd_gotoxy(1, 1);
    sprintf(LCD, "Time: %.2d:%.2d:%.2d", hours, min, sec);
    lcd_puts(LCD);

    /////////////////////////////////////////////////////
    lcd_gotoxy(1, 2);
    sprintf(LCD, "CT:%5.1fC  ", Temp);
    lcd_puts(LCD);

    lcd_gotoxy(11, 2);
    lcd_puts(CK);
    /////////////////////////////////////////////////////
    lcd_gotoxy(1, 3);
    sprintf(LCD, "SP:%5.1fC  ", SP);
    lcd_puts(LCD);

    lcd_gotoxy(11, 3);
    lcd_puts(HT);

    /////////////////////////////////////   
    lcd_gotoxy(1, 4);
    sprintf(LCD, "MD:");
    lcd_puts(LCD);

    lcd_gotoxy(4, 4);
    lcd_puts(LCDP);
    /////////////////////////////////////////
}

void main(void) {
    setupPorts();
    init_adc_no_lib();

    //Timer-0 Initialization | Without starting
    T0CON = 0;
    T0CONbits.TMR0ON = 0; // We don't want the timer to run yet
    T0CONbits.T08BIT = 0; // We want Timer-0 = 16 bit
    T0CONbits.T0CS = 0;
    T0CONbits.T0SE = 0;
    T0CONbits.PSA = 0; //IF PSA IS ONE, THEN pre-scaler = 1, Regardless of T0PS2:T0PS0
    T0CONbits.T0PS2 = 0;
    T0CONbits.T0PS1 = 1; // 011 for pre-scaler = 16
    T0CONbits.T0PS0 = 1;


    //Interrupts Initialization
    RCONbits.IPEN = 0; // disable Priority

    INTCON = 0; // disable interrupts first, then enable the ones u want
    INTCONbits.INT0IE = 1; //INT0 Enable, for Mode Increment
    INTCONbits.TMR0IE = 1;
    INTCONbits.GIEH = 1; // enable global interrupt bits
    INTCONbits.GIEL = 1;

    INTCON2 = 0;
    INTCON2bits.INTEDG0 = 1;
    INTCON2bits.INTEDG1 = 1; //FOR ALL INT, WE CHOOSE RISING EDGE
    INTCON2bits.INTEDG2 = 1;

    INTCON3 = 0;
    INTCON3bits.INT1IE = 1; //INT1 Enable, To START (RESUME) The Cooker
    INTCON3bits.INT2IE = 1; //INT2 Enable, to STOP (PAUSE) the cooker


    PIE1 = 0;
    PIR1 = 0;
    IPR1 = 0;
    PIE2 = 0;
    PIE2 = 0;
    PIR2 = 0;
    IPR2 = 0;
    PORTB = 0;




    lcd_init();
    lcd_send_byte(0, 1);
    while (1) {

        CLRWDT();
        SP = (read_adc_voltage(0))*40.0; //Read the value from Channel 0 => AN0 (The Value Will be from 0-5 volt) | Pt.1
        // We want to scale the 0-5 Volt we have to 0-200C, So we multiply by 40

        Temp = (read_adc_voltage(2))*200.0; //Read the value from Channel 2 => AN2 (The Value Will be from 0-5 volt) |


        if (!PORTBbits.RB3) {
            if (mode == 0 && hours < 5) {
                if (sec <= 58)
                    sec++;
                else {
                    sec = 0;
                    if (min <= 58)
                        min++;
                    else {
                        min = 0;
                        hours++;
                    }
                }
            }
            if (mode == 1 && hours < 5) {
                if (sec <= 49)
                    sec = sec + 10;
                else {
                    sec = sec + 10 - 60;
                    if (min <= 58) {
                        min++;
                    }
                    else {
                        min = 0;
                        hours++;
                        if (hours == 5) {
                            min = 0;
                            sec = 0;
                        }
                    }
                }
            }

            if (mode == 2 && hours < 5) {
                if (min <= 58) {
                    min++;
                } else {
                    min = 0;
                    hours++;
                }
            }

            if (mode == 3 && hours < 5) {
                if (min <= 49)
                    min = min + 10;
                else {
                    min = min + 10 - 60;
                    if (hours < 5) {
                        hours++;
                    }
                    if (hours == 5) {
                        min = 0;
                        sec = 0;
                    }
                }
            }

            else if (mode == 4 && hours < 5) {
                if (hours < 5) {
                    hours++;
                }
                if (hours == 5) {
                    min = 0;
                    sec = 0;
                }
            }
            delay_ms(250);
        }
        else if (!PORTBbits.RB4) {
            if (mode == 0) {
                if (sec > 0)
                    sec--;
                else if (min > 0) {
                    sec = 59;
                    min--;
                } else if (hours > 0) {
                    hours--;
                    min = 59;
                    sec = 59;
                } else {
                    hours = 0;
                    min = 0;
                    sec = 0;
                }
            }

            if (mode == 1) {
                if (sec >= 10)
                    sec = sec - 10;
                else if (min > 0) {
                    sec = 60 + sec - 10;
                    min--;
                } else if (hours > 0) {
                    hours--;
                    min = 59;
                    sec = 60 + sec - 10;
                } else {
                    hours = 0;
                    min = 0;
                    sec = 0;
                }
            }

            if (mode == 2) {
                if (min > 0) {
                    min--;
                } else if (hours > 0) {
                    hours--;
                    min = 59;
                } else {
                    hours = 0;
                    min = 0;
                    sec = 0;
                }
            }

            if (mode == 3) {
                if (min >= 10)
                    min = min - 10;
                else if (hours > 0) {
                    min = 60 + min - 10;
                    hours--;
                } else {
                    hours = 0;
                    min = 0;
                    sec = 0;
                }
            }

            if (mode == 4) {
                if (hours > 0)
                    hours--;
                else {
                    hours = 0;
                    min = 0;
                    sec = 0;
                }
            }

            delay_ms(250);
        }
        else if (!PORTBbits.RB5) {
            hours = 0;
            min = 0;
            sec = 0;
            cooker = 0;
            heater = 0;
            PORTCbits.RC5 = 0;
            T0CONbits.TMR0ON = 0;
        }

        if (cooker) {
            if (Temp < SP - H) {
                PORTCbits.RC5 = 1;
                heater = 1;
            } else if (Temp > SP + H) {
                PORTCbits.RC5 = 0;
                heater = 0;
            }
        }
        display();
    }
    return;
}