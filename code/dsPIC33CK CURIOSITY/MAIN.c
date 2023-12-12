#include <p33CK1024MP710.h>

#include "CONFIG.h"

// Macros for registers
#define LED6 _LATE6
#define LED8 _LATE7
#define LED10 _LATE8
#define LED11 _LATE9
#define LED12 _LATF11
#define RED _LATB12
#define GREEN _LATB10
#define BLUE _LATD1
#define SW1 PORTDbits.RD13
#define SW2 PORTFbits.RF3 
#define SW3 PORTFbits.RF4 

// ------------------
// Macros used for masking and filtering
// ------------------

/* CAN filter and mask defines */
/* Macro used to write filter/mask ID to Register CiRXMxSID and
CiRXFxSID. For example to setup the filter to accept a value of
0x123, the macro when called as CAN_FILTERMASK2REG_SID(0x123) will
write the register space to accept message with ID 0x123
USE FOR STANDARD MESSAGES ONLY */
#define CAN_FILTERMASK2REG_SID(x) ((x & 0x07FF)<< 5)
/* the Macro will set the "MIDE" bit in CiRXMxSID */
#define CAN_SETMIDE(sid) (sid | 0x0008)
/* the macro will set the EXIDE bit in the CiRXFxSID to
accept extended messages only */
#define CAN_FILTERXTD(sid) (sid | 0x0008)
/* the macro will clear the EXIDE bit in the CiRXFxSID to
accept standard messages only */
#define CAN_FILTERSTD(sid) (sid & 0xFFF7)

#define NUM_OF_ECAN_BUFFERS 32
#define MSG_SID 0x123              // the arbitrary CAN SID of the transmitted message

/* ECAN message type identifiers */
#define CAN_MSG_DATA 0x01
#define CAN_MSG_RTR 0x02
#define CAN_FRAME_EXT 0x03
#define CAN_FRAME_STD 0x04
#define CAN_BUF_FULL 0x05
#define CAN_BUF_EMPTY 0x06

// Initializes hardware for peripherals and timers
void init_HW();

// Sets up the CAN module
void init_CAN();

// Sets up the oscillators and timers
void oscConfig(void);

// Returns 1 if switches are being pressed 
char _SW1();
char _SW2();
char _SW3();

int main(void) {
    
    // Initialize peripherals
    init_HW();
    
    // Initialize CAN
    //init_CAN();
    
    while(1)
    {
        LED6 = ~SW2;
        LED8 = ~SW1;
    }
    
    return 0;
}

void init_CAN()
{
    // ------------------
    // Standby pin set to low
    // ------------------
    
    _TRISD15 = 0; // Set as an output 
    _LATD15 = 0; // Set to low (CAN module not in standby)
    
    // ------------------
    // Configure TX and RX to input and outputs
    // ------------------
    
    // Configure CANTX for output
    _TRISC15 = 0;
    
    // Configure CANRX for input
    _TRISC14 = 1;
    
    // ------------------
    // Remap CAN module pins
    // ------------------
    
    RPINR26bits.CAN1RXR = 62; // Remap CANRX to RP62
    
    RPOR15bits.RP63R = 21; // Remap CANTX to RP63
    
    // ------------------
    // Set the CAN module into configuration mode
    // ------------------
    
    // Request operation mode 
    C1CONHbits.REQOP = 4; // Sets to configuration 
    
    // Check the operation mode status
    while (C1CONHbits.OPMOD != 4); // The program should stop here until the mode changes to configuration mode
    
    // WIN registers can be added later
    
    // ------------------
    // Configure the CAN module & timing WIP
    // ------------------
    
    //C1CONLbits.CON = 1; // Enable the CAN module 
    //DMACONbits.DMAEN = 1; // Enable the DMA module
    
    /*    
    C1CFG1 = 0x47;          // BRP = 8 SJW = 2 Tq
    C1CFG2 = 0x2D2;
    C1FCTRL = 0xC01F;       // No FIFO, 32 Buffers [Still needs to be found]
    */
    
    C1DBTCFGHbits.BRP = 7; // TQ = 8/FSYS
    C1DBTCFGLbits.SJW = 1; // Length is 2 x TQ
    
    DMACH0bits.SIZE = 0; // Data size is word (16-bit)
    DMACH0bits.DAMODE = 3; // DMADSTn is used in Peripheral Indirect Addressing and remains unchanged
    
    // DMA0REQ.IRQSEL missing (it sets DMA0)
    
    // Set DMA channel 0 to 8 DMA transfers
    DMACNT0 = 7; // DMA Transaction Counter bits set to 8 (I think; try alternating between 7 and 8)
    
    // DMA0PAD = (volatile unsigned int)&C1TXD; sets DMA channel 0 peripheral address to TX data request CAN1
    DMADST0 = 53; // Set DMA Data Destination Address Pointer bits to CAN1 data request
    
    // Unable to find registers related to DMA channel 0 start address low or high (DMA0STAL & DMA0STAH)
    
    // Unable to find registers related to CANx TX/RX buffer mn control (CxTRmnCON ; C1TR01CON)
    
    DMACH0bits.CHEN = 1; // Enable DMA channel 0
    
    DMACH2bits.DAMODE = 3; // DMADSTn is used in Peripheral Indirect Addressing and remains unchanged
    
    DMADST2 = 
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    // ------------------
    // Set the CAN module into normal mode
    // ------------------
    
    C1CONHbits.REQOP = 0;
    while (C1CONHbits.OPMOD != 0);
    
    // ------------------
    // CAN RX interrupt enable - 'double arm' since 2-level nested interrupt
    // ------------------
    
    /*
    C1INTEbits.RBIE = 1; // RX Buffer Interrupt Enable bit
    IEC2bits.C1IE = 1; // ????
    */
    
    IEC1bits.C1RXIE = 1; // Interrupt enabled
    IEC2bits.C2IE = 1; // Combined error interrupt enable bit
}

void oscConfig(void)
{

    // Configure Oscillator to operate the device at 80MHz/40MIPs
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    // To be safe, always load divisors before feedback 
    //example 9-3
    
    CLKDIVbits.PLLPRE = 0; // N1=2
    PLLDIVbits.POST1DIV = 0; // N2=5
    PLLFBDbits.PLLFBDIV = 38; //PLLFBD = 38; // M=(40-2), Fcyc = 40MHz for ECAN baud timer


   // Disable Watch Dog Timer
   // we already disable it
   // RCONbits.SWDTEN = 0;

}

/*
void Transmit_Data(void)
{
    //
    // The LEDs reflect the switch status
    //
    LED1 = ~SW1;
    LED2 = ~SW2;
    LED3 = ~SW3;

    //
    // read the pot value and save it
    //
    ADCConvert(19);
    PotValue = AverageValue;
    Delayus(100);
    Pot_Volts = (float)(PotValue * (float)5.0 / (float)4096.0);
    //
    // convert to ASCII
    //
    pBuf = Buf_result;
    ftoa(Pot_Volts, pBuf);

    tagU2MODEBITS.UTXEN = 1; //tagU2MODEBITS.URXEN = 1;
    putsU2("***TRANSMITTING ON-BOARD SENSOR VALUES***");
    while (tagU2STABITS.TRMT == 0);//while (tagU2MODEBITS.TRMT == 0);
    U2TXREG = 0x0a; //check this
     while (tagU2STABITS.TRMT == 0);//while (tagU2MODEBITS.TRMT == 0);
    U2TXREG = 0x0d; //check this

    putsU2("Local Pot Voltage: Reading = ");

    for (i = 0; i <= (NUM_DIGITS - 1); i++)
    {
        while (tagU2STABITS.TRMT == 0);
        U2TXREG = Buf_result[i]; //check this
    }
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0a;
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0d;

    //
    // read temperature sensor and save it
    //
    ADCConvert(18);
    TempValue = AverageValue;
    Delayus(100);

    //
    // test print the temperature reading out the UART
    //
    Pot_Volts = (float)((TempValue - 368) / 15.974);
    //
    // convert to ASCII
    //
    pBuf = Buf_result;
    ftoa(Pot_Volts, pBuf);

    tagU2MODEBITS.UTXEN = 1;

    putsU2("Local Temperature: Reading = ");

    for (i = 0; i <= (NUM_DIGITS - 1); i++)
    {
        while (tagU2STABITS.TRMT == 0);
        U2TXREG = Buf_result[i];
    }
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0a;
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0d;
    //
    // test print the 3 switch statuses
    //

    putsU2("Local Switch status");
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0a;
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0d;
    // ON = pressed  OFF = up
    if (SW1)
    {
        putsU2("SW1: OFF ");
    }
    else
    {
        putsU2("SW1: ON");
    }
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0a;
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0d;

    // ON = pressed  OFF = up
    if (SW2)
    {
        putsU2("SW2: OFF ");
    }
    else
    {
        putsU2("SW2: ON");
    }
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0a;
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0d;

    // ON = pressed  OFF = up
    if (SW3)
    {
        putsU2("SW3: OFF ");
    }
    else
    {
        putsU2("SW3: ON");
    }
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0a;
    while (tagU2STABITS.TRMT == 0);
    U2TXREG = 0x0d;
    // format and send out the CAN port
    //
    // In order for the demo to run, the CAN controller needs an ACK signal
    // If you desire to run the demo for SENT/LIN only, then comment out the
    // following line of code and recompile
    
    CAN_Transmit();     // Transmit CAN. COMMENT OUT FOR LIN/SENT ONLY!!
    Delayus(5000);

}
*/

void init_HW()
{
    // Set all LEDS to output
    _TRISD1 = 0; //Blue
    _TRISB10 = 0; // Green
    _TRISB12 = 0; // Red
    
    _TRISE8 = 0; // LED10
    _TRISE7 = 0; // LED8
    _TRISE6 = 0; // LED6
    _TRISE9 = 0; // LED11
    _TRISF11 = 0; // LED12
    
    // Set all switches to inputs 
    _TRISD13 = 1; // SW1 
    _TRISF3 = 1; // SW2
    _TRISF4 = 1; // SW3
    
    // Set potentiometer to input
    _TRISA0 = 1;
    
    // SW3 needs to be manually set to digital
    ANSELDbits.ANSELD13 = 0;
    
    // Set potentiometer to analog
    _ANSELA0 = 0;
    
    //
    // Timer 1 to generate an interrupt every 250ms
    //
    T1CONbits.TON = 0;          // Disable Timer1
    T1CONbits.TCS = 0;          // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        // Disable Gated Timer mode
    T1CONbits.TCKPS = 0x3;      // Select 1:256 Prescaler
    PR1 = 39062;                // Load the period value (250ms/(256*25ns))
    IPC0bits.T1IP = 0x03;       // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;          // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;          // Enable Timer1 interrupt
    
    //
    // Timer 1 to generate an interrupt every 250ms
    //
    T1CONbits.TON = 0;          // Disable Timer1
    T1CONbits.TCS = 0;          // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        // Disable Gated Timer mode
    T1CONbits.TCKPS = 0x3;      // Select 1:256 Prescaler
    PR1 = 39062;                // Load the period value (250ms/(256*25ns))
    IPC0bits.T1IP = 0x03;       // Set Timer 1 Interrupt Priority Level
    IFS0bits.T1IF = 0;          // Clear Timer 1 Interrupt Flag
    IEC0bits.T1IE = 1;          // Enable Timer1 interrupt
}

// The following switch functions will return True when either of
// the switches are being pressed
char _SW1()
{
    if(SW1 == 0)
        return 1;
}
char _SW2()
{
    if(SW2 == 0)
        return 1;
}
char _SW3()
{
    if(SW3 == 0)
        return 1;
}
