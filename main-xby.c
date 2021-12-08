//CCS version 10.2.0.00009
#include <msp430.h>

void start_clock_sys(void);
void start_I2C(void);


/*------- I2C ------*/
//SDA pin 14 (P1.2) - UCB0SDA
//SCL pin 15 (P1.3) - UCB0SCL

//SDA pin 38 (P3.2) - UCB1SDA
//SCL pin 39 (P3.6) - UCB1SCL

// Slave name: PCF8574
// Slave addres: 0 1 0 0 A2 A1 A0 R/~W= 0x27 (7 bits | without resistor Ax = 1)
// A2 A1 A0 are 1
// If the R/W bit is high, the data from this device are the values
// read from the P port. If the R/W bit is low, the data are from the master, to be output to the P port.
#define SLAVE_ADDR  0x27 //0010 0111
//#define SLAVE_ADDR  0x4E //0100 1110 não funcionou

int Data_Cnt = 0;
//char init_LCD_4bits[] = {0x20, 0x24, 0x20};
char init_LCD_4bits[] = {0x28, 0x2C, 0x28};
/*---- Connections ----*/
//  P0 = RS (LSB)
//  P1 = RW
//  P2 = E
//  P3 = --
//  P4 = D4
//  P5 = D5
//  P6 = D6
//  P7 = D7 (MSB)
// Send an enable signal - Falling edge  -> 0010xxxx
//  RW (LCD) = GND (MSP) // Read = 1 and Write = 0
// RS (LCD) = P4.1 (MSP) // Text = 1 and Command = 0

int main(void)
{
     WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer

    //Set LED1 for test routines
    // Set P1.0 as output direction [J7 needs to be connected]
    // LED White: Port P1 / Pin 0
    P1SEL0 &= ~BIT0;
    P1SEL1 &= ~BIT0;
    P1DIR |= BIT0; // Set LED1 as Output
    P1OUT &= ~BIT0; // turn off LED1

    PM5CTL0 &= ~LOCKLPM5;       //to apply the GPIO configurations disable the LOCKLPM5 bit in PM5CTL0

    start_I2C();

    // send_LCD_nibble(0x02); //Tell the LCD to work with 4 bits
    // send_LCD_byte(0,0x28); // 0x0011.1000 FUNCTION SET - 4 bits, 2 lines, 5x7 dots

    // send_LCD_byte(0,0x0F); // 0x0000.1111 DISPLAY CONTROL - Display on, no cursor
    // send_LCD_byte(0,0x01); // CLEAN LCD
    // send_LCD_byte(0,0x06); // ENTRY MODE SET - Shift cursor to the right each written character

    int i;

    while(1){
        UCB0CTLW0 |= UCTXSTT; //manually start message (START)
        for (i=0;i<100;i++); //delay

    }
    P1OUT ^= BIT0;

    return 0;
}

//-------------ISR-----------------
#pragma vector = EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void){
    if (Data_Cnt == (sizeof(init_LCD_4bits) -1)){
    //UCB0TXBUF = 0x20; //value that we want to send
    UCB0TXBUF = init_LCD_4bits[Data_Cnt];
    Data_Cnt = 0;
    } else {
        UCB0TXBUF = init_LCD_4bits[Data_Cnt];
        Data_Cnt++;
    }
}


void start_I2C(void){

    // always put the eUSCI module into its reset state
    UCB0CTLW0 = UCSWRST;                      //Put B0 in SW reset

    //configure the eUSCI module as a synchronous I2C peripheral, master mode, using the SMCLK source
    UCB0CTLW0 |= UCSSEL__SMCLK; // UCSSEL_3 = SMCLK,
    //UCB0CTLW0 |= UCMODE_3;
    // The SMCLK is configured for 10MHz, so we must divide it down to meet the 100kHz I2C clock requirement
    UCB0BRW = 10;                            // fSCL = SMCLK/10 = ~100kHz // set prescalar to 10

    UCB0CTLW0 |= UCMODE_3; //put into I2C mode
    UCB0CTLW0 |= UCMST; //set as master
    UCB0CTLW0 |= UCTR; //put into Tx mode (to write)
    UCB0I2CSA = SLAVE_ADDR; //set slave addres

    UCB0CTLW1 |= UCASTP_2; //auto STOP mode
    //UCB0TBCNT = 1; // transfer buffer count = 1 (1 byte)
    UCB0TBCNT = sizeof(init_LCD_4bits); //not it has the number of bytes I allocate in this variable

    //Config PIN SEL  - for UCB0SDA e UCB0SCL PxSELx = 01
    //SDA pin 14 (P1.2) - UCB0SDA
    //SCL pin 15 (P1.3) - UCB0SCL
    P1SEL0 |= BIT2 | BIT3;
    P1SEL1 &= ~(BIT2 | BIT3);

    //To use UCB1SDA e UCB1SCL
    //SDA pin 38 (P3.2) - UCB1SDA
    //SCL pin 39 (P3.6) - UCB1SCL
    //P3SEL0 |= BIT2 | BIT6;
    //P3SEL1 &= ~(BIT2 | BIT6);

    //Obs.: To set an interrupt for I2C (in case MSP is slave), set an ordinary GPIO;

    PM5CTL0 &= ~LOCKLPM5;  //to apply the GPIO configurations disable the LOCKLPM5 bit in PM5CTL0

    UCB0CTLW0 &= ~UCSWRST;  //Take B0 out of SW RST

    //Enable B0 TX0 IRQ
    UCB0IE |= UCTXIE0;  //local enable for TX0
    __enable_interrupt(); //enable maskables
}

