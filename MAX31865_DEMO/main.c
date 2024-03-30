#include <msp430.h> 
#include <stdint.h>
#include <math.h>
#include "printf.h"
//#include <stdlib.h>
//#include <stdio.h>

// LED P3.0
#define COMMS_LED_OUT   P3OUT
#define COMMS_LED_DIR   P3DIR
#define COMMS_LED_PIN   BIT0

// SLAVE CS P5.0
#define SLAVE_CS_OUT    P5OUT
#define SLAVE_CS_DIR    P5DIR
#define SLAVE_CS_PIN    BIT0

#define RTD_A                     3.9083e-3f       /**< rtd a */
#define RTD_B                     -5.775e-7f       /**< rtd b */


// Define flags used by the interrupt routines
#define TX  BIT0

void initSPI();
void initGPIO();
void initClockTo16MHz();
void SendUCB0Data(uint8_t val);
void initUART();
void sendByte(unsigned char byte);
void initTimer();
static float a_max31865_temperature_conversion(float rt, float rtd_nominal, float ref_resistor);



uint8_t RxData = 0; // Global var for receive buffer


// Flag register
volatile unsigned char FLAGS = 0;

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	initClockTo16MHz();
	initGPIO();
	initSPI();
	initUART();
//	initTimer();

	uint16_t resValue = 0x0000;
	uint16_t adcValue = 0x0000;
	uint32_t temperture = 0;

	while(1) {
        switch(FLAGS) {
            case 0:                                     // No flags set
                __bis_SR_register(LPM3_bits + GIE);     // Enter LPM3
                break;
            case TX:                        // Values need to be transmitted
                // Display all the different types of values
//                printf("String         %s\r\n", s);
//                printf("Char           %c\r\n", c);
//                printf("Integer        %i\r\n", i);
//                printf("Unsigned       %u\r\n", u);
//                printf("Long           %l\r\n", l);
//                printf("uNsigned loNg  %n\r\n", n);
//                printf("Addresing MAX3815 Config Register........0x%x\r\n", 0x80);
                SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);
                SendUCB0Data(0x80);
                SendUCB0Data(0xF3);
                __delay_cycles(450);
                SLAVE_CS_OUT |= SLAVE_CS_PIN;
//                printf("Wrote MAX3815 Config Register............0x%x\r\n", 0xF3);
//                printf("Reading MAX3815 Config Register..........0x%x\r\n", 0x00);
                SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);
                SendUCB0Data(0x00);
                SendUCB0Data(0x00);
                __delay_cycles(450);
                SLAVE_CS_OUT |= SLAVE_CS_PIN;
//                printf("Read back in ........................... 0x%x\r\n",RxData);
                __delay_cycles(6000000);
                SLAVE_CS_OUT |= SLAVE_CS_PIN;

//                while(1) {
//                    printf("\r\n\r\n");
//                    printf("Reading MAX3815 MSB Register.............0x%x\r\n", 0x01);
                    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);
                    SendUCB0Data(0x01);
                    SendUCB0Data(0x00);
                    __delay_cycles(450);
//                    printf("Read back in ........................... 0x%x\r\n",RxData);
                    resValue = RxData;
                    SLAVE_CS_OUT |= SLAVE_CS_PIN;
//                    printf("Reading MAX3815 LSB Register.............0x%x\r\n", 0x02);
                    SLAVE_CS_OUT &= ~(SLAVE_CS_PIN);
                    SendUCB0Data(0x02);
                    SendUCB0Data(0x00);
                    __delay_cycles(450);
//                    printf("Read back in ........................... 0x%x\r\n",RxData);
                    resValue = (resValue << 8) | RxData;
//                    resValue = resValue >> 1; // Get rid of status bit
                    adcValue = resValue >> 1;
                    SLAVE_CS_OUT |= SLAVE_CS_PIN;
                    temperture = (adcValue * 430) >> 15;
//                    printf("RTD Curr Val ........................... 0x%x\r\n",resValue);
//                    printf("ADC Val ................................ %i\r\n",adcValue);
                    printf("%i\r\n",adcValue);
//                    printf("temp Curr Val ........................... %i\r\n",temperture);

                    __delay_cycles(6000000);

//                }

                FLAGS &= ~TX;               // Clear transmit flag
                __bis_SR_register(LPM3_bits + GIE);     // Enter LPM3

                break;
        }
	}
	
	return 0;
}



void initSPI()
{
    //Clock Polarity: The inactive state is high
    //MSB First, 8-bit, Master, 3-pin mode, Synchronous
    UCB0CTLW0 = UCSWRST;                       // **Put state machine in reset**
    UCB0CTLW0 |= UCCKPL | UCMSB | UCSYNC
                | UCMST | UCSSEL__SMCLK;      // 3-pin, 8-bit SPI Slave
    UCB0BRW = 0x20;
    //UCB0MCTLW = 0;
    UCB0CTLW0 &= ~UCSWRST;                     // **Initialize USCI state machine**
    UCB0IE |= UCRXIE;                          // Enable USCI0 RX interrupt
}

void initUART()
{
//    // Configure UART
//    UCA0CTLW0 |= UCSWRST;                     // Put eUSCI in reset
//    UCA0CTLW0 |= UCSSEL__SMCLK;
//    // Baud Rate calculation
//    UCA0BR0 = 8;                              // 1000000/115200 = 8.68
//    UCA0MCTLW = 0xD600;                       // 1000000/115200 - INT(1000000/115200)=0.68
//                                              // UCBRSx value = 0xD6 (See UG)
//    UCA0BR1 = 0;
//    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
//    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
    __bis_SR_register(SCG0);                 // disable FLL
     CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
     CSCTL0 = 0;                              // clear DCO and MOD registers
     CSCTL1 &= ~(DCORSEL_7);                  // Clear DCO frequency select bits first
     CSCTL1 |= DCORSEL_3;                     // Set DCO = 8MHz
     CSCTL2 = FLLD_0 + 243;                   // DCODIV = 8MHz
     __delay_cycles(3);
     __bic_SR_register(SCG0);                 // enable FLL
     while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked

     CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                              // default DCODIV as MCLK and SMCLK source


     // Configure UART
     UCA0CTLW0 |= UCSWRST;
     UCA0CTLW0 |= UCSSEL__SMCLK;

     // Baud Rate calculation
     // 8000000/(16*9600) = 52.083
     // Fractional portion = 0.083
     // User's Guide Table 14-4: UCBRSx = 0x49
     // UCBRFx = int ( (52.083-52)*16) = 1
     UCA0BR0 = 52;                             // 8000000/16/9600
     UCA0BR1 = 0x00;
     UCA0MCTLW = 0x4900 | UCOS16 | UCBRF_1;

     UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
     UCA0IE |= UCRXIE;

}

void initGPIO()
{
    P1DIR = 0xFF; P2DIR = 0xFF; P3DIR = 0xFF; P4DIR = 0xFF;
    P5DIR = 0xFF; P6DIR = 0xFF; P7DIR = 0xFF; P8DIR = 0xFF;
    P1REN = 0xFF; P2REN = 0xFF; P3REN = 0xFF; P4REN = 0xFF;
    P5REN = 0xFF; P6REN = 0xFF; P7REN = 0xFF; P8REN = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00; P3OUT = 0x00; P4OUT = 0x00;
    P5OUT = 0x00; P6OUT = 0x00; P7OUT = 0x00; P8OUT = 0x00;

    //LEDs
    COMMS_LED_DIR |= COMMS_LED_PIN;
    COMMS_LED_OUT &= ~COMMS_LED_PIN;

    // Configure SPI
    // P5.1 SCK
    // P5.2 MOSI
    // P5.3 MISO
    P5SEL0 |= BIT1 | BIT2 | BIT3;

    // Configure UART pins
    P1SEL0 |= BIT0 | BIT1; // set 2-UART pin as second function

    P1SEL0 |= BIT5; // P1.5 selected as TA0CLK

    SLAVE_CS_DIR |= SLAVE_CS_PIN;
    SLAVE_CS_OUT |= SLAVE_CS_PIN;


    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initClockTo16MHz()
{
    // Configure one FRAM waitstate as required by the device datasheet for MCLK
    // operation beyond 8MHz _before_ configuring the clock system.
    FRCTL0 = FRCTLPW | NWAITS_1;

    __bis_SR_register(SCG0);    // disable FLL
    CSCTL3 |= SELREF__REFOCLK;  // Set REFO as FLL reference source
    CSCTL0 = 0;                 // clear DCO and MOD registers
    CSCTL1 &= ~(DCORSEL_7);     // Clear DCO frequency select bits first
    CSCTL1 |= DCORSEL_5;        // Set DCO = 16MHz
    CSCTL2 = FLLD_0 + 487;      // set to fDCOCLKDIV = (FLLN + 1)*(fFLLREFCLK/n)
                                //                   = (487 + 1)*(32.768 kHz/1)
                                //                   = 16 MHz

    __delay_cycles(3);
    __bic_SR_register(SCG0);                        // enable FLL
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1));      // FLL locked
}

/**
 * Initializes the timer to call the interrupt every 1 second
 */
void initTimer()
{
    // Timer1_A3 setup
    TA1CCTL0 = CCIE;                          // TACCR0 interrupt enabled
    TA1CCR0 = 4096;
    TA1CTL = TASSEL_1 | MC_2;                 // ACLK, continuous mode
}


static float a_max31865_temperature_conversion(float rt, float rtd_nominal, float ref_resistor)
{
    float z1, z2, z3, z4, temp;
    float rpoly = rt;

    rt /= 32768;                             /* div 32768 */
    rt *= ref_resistor;                      /* ref_resistor */
    z1 = -RTD_A;                             /* -RTD A */
    z2 = RTD_A * RTD_A - (4 * RTD_B);        /* get z2 */
    z3 = (4 * RTD_B) / rtd_nominal;          /* get z3 */
    z4 = 2 * RTD_B;                          /* get z4 */
    temp = z2 + (z3 * rt);                   /* calculate temp */
    temp = (sqrtf(temp) + z1) / z4;          /* sqrt */
    if (temp >= 0)                           /* check temp */
    {
        return temp;                         /* return temp */
    }
    rt /= rtd_nominal;                       /* nominal */
    rt *= 100;                               /* normalize to 100 ohm */
    temp = -242.02f;                         /* -242.02 */
    temp += 2.2228f * rpoly;                 /* add offset */
    rpoly *= rt;                             /* square */
    temp += 2.5859e-3f * rpoly;              /* add offset */
    rpoly *= rt;                             /* ^3 */
    temp -= 4.8260e-6f * rpoly;              /* add offset */
    rpoly *= rt;                             /* ^4 */
    temp -= 2.8183e-8f * rpoly;              /* add offset */
    rpoly *= rt;                             /* ^5 */
    temp += 1.5243e-10f * rpoly;             /* add offset */

    return temp;                             /* return error */
}


void SendUCB0Data(uint8_t val)
{
    while (!(UCB0IFG & UCTXIFG));              // USCI_B0 TX buffer ready?
    UCB0TXBUF = val;
}

/**
 * puts() is used by printf() to display or send a string.. This function
 *     determines where printf prints to. For this case it sends a string
 *     out over UART, another option could be to display the string on an
 *     LCD display.
 **/
void puts(char *s) {
    char c;

    // Loops through each character in string 's'
    while (c = *s++) {
        sendByte(c);
    }
}
///**
// * puts() is used by printf() to display or send a character. This function
// *     determines where printf prints to. For this case it sends a character
// *     out over UART.
// **/
void putc(unsigned b) {
    sendByte(b);
}

//int putchar(int c)
//{
//   sendByte(c);
//   return 0;
//}

/**
 * Sends a single byte out through UART
 **/
void sendByte(unsigned char byte )
{
    while (!(UCA0IFG&UCTXIFG));          // USCI_A0 TX buffer ready?
    UCA0TXBUF = byte;                   // TX -> RXed character
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) USCI_B0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_NONE: break;
        // Receive Logic
        case USCI_SPI_UCRXIFG:
            RxData = UCB0RXBUF;
            UCB0IFG &= ~UCRXIFG;
//            __delay_cycles(1000);
            break;

        // Transmit Logic
        case USCI_SPI_UCTXIFG:
            break;
        default: break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    char r = UCA0RXBUF;                 // Get the received character
    if (r == 't' || r == 'T') // Transmit Data
    {
        FLAGS |= TX;                    // Set flag to transmit data
    } else { // Do nothing
        FLAGS &= ~(TX);
    }

    __bic_SR_register_on_exit(LPM3_bits);   // Wake-up CPU

}

// Timer A1 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    COMMS_LED_OUT ^= COMMS_LED_PIN;
//    printf("Hello\r\n");
//    FLAGS |= TX;                    // Set flag to transmit data
    __bic_SR_register_on_exit(LPM3_bits);   // Wake-up CPU
//    TA1CCR0 += 50000;                         // Add Offset to TA1CCR0
}




