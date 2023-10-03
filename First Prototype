// Function Prototypes
void oscConfig(void);
void clearIntrflags(void);
void init_hw(void);
void InitMonitor(void);
void ADCInit(void);
void InitCAN(void);
void rxECAN(mID *message);
void Receive_Data(void);
void Transmit_Data(void);
void Delayus(int delay);
void CAN_Transmit(void);
// send a character to the serial port
void putU2(int);
void putsU2(char*);


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

int main(void)
{

    // Configure Oscillator Clock Source
    oscConfig();

    // Clear Interrupt Flags
    clearIntrflags();

    // Initialize hardware on the board
    init_hw();

    // Initialize the monitor UART2 module
    InitMonitor();

    // Initialize the ADC converter
    ADCInit();

    // Initialize the CAN module
        InitCAN();

    // main loop: every 4 Timer1 ticks (1sec), scan the sensors and transmit the data
    // or wait for a Receive interrupt from 1 of the 3 interfaces
    //
    s_tick = 0;
    while (1)
    {
        if (mode == RECEIVE)
        {
     /* check to see when a message is received and move the message
		into RAM and parse the message */
		if(canRxMessage.buffer_status == CAN_BUF_FULL)
		{
			rxECAN(&canRxMessage);

			/* reset the flag when done */
			canRxMessage.buffer_status = CAN_BUF_EMPTY;
        }
       
            Receive_Data();
            s_tick = 0;
        }
        else if (mode == TRANSMIT)
        {
 //
 // wait for the 250ms timer. User can accumulate s_ticks to get longer delays
 //
            while (s_tick <= 3);        // wait for 1 second
            s_tick = 0;                 // clear flag
            Transmit_Data();

        }
    }
}

void oscConfig(void)
{

    //  Configure Oscillator to operate the device at 80MHz/40MIPs
    // 	Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // 	Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    // To be safe, always load divisors before feedback
    
   
    CLKDIVbits.PLLPOST = 0;     // N1=2
    CLKDIVbits.PLLPRE = 0;      // N2=2
    PLLFBD = 38;                // M=(40-2), Fcyc = 40MHz for ECAN baud timer


    // Disable Watch Dog Timer

    RCONbits.SWDTEN = 0;

}

void clearIntrflags(void)
{
    /* Clear Interrupt Flags */

    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
    IPC16bits.U1EIP = 6;        //service the LIN framing error before the RX
    IPC2bits.U1RXIP = 4;
}

void init_hw(void)
{
    int j;

    // set up the LED and switch ports

    TRISLED1 = 0;
    TRISLED2 = 0;
    TRISLED3 = 0;
    ANSEL_POT = 1;
    ANSEL_TEMP = 1;
    TRIS_POT = 1;
    TRIS_TEMP = 1;
    ANSELC = ANSELC & 0xFC3F;   // (re)set the 3 switch bits + CAN due to error in v1.20 header
    s_tick = 0;
    f_tick = 0;                 // the timer ticks
    lin_index = 0;
    lin_start = 0;
    for (j = 0; j < 8; j++)
    {
        LIN_RXBUF[j] = 0;
    }


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
    // Timer 2 to generate an interrupt every 10ms
    //
    T2CONbits.TON = 0;          // Disable Timer2
    T2CONbits.TCS = 0;          // Select internal instruction cycle clock
    T2CONbits.TGATE = 0;        // Disable Gated Timer mode
    T2CONbits.TCKPS = 0x3;      // Select 1:256 Prescaler
    TMR2 = 0x00;                // Clear timer register
    PR2 = 1562;                 // Load the period value (10ms/(256*25ns))
    IPC1bits.T2IP = 0x02;       // Set Timer 2 Interrupt Priority Level
    IFS0bits.T2IF = 0;          // Clear Timer 2 Interrupt Flag
    IEC0bits.T2IE = 1;          // Enable Timer2 interrupt

    T2CONbits.TON = 1;          // Start Timer2
    T1CONbits.TON = 1;          // Start Timer1
}

void InitMonitor(void)
{
    // digital outputTD
    TRIS_MON = 0;

    //
    // map MONITOR_TX pin to port RB4, which is remappable RP36
    //
    RPOR1bits.RP36R = 0x03; // map UART2 TXD to pin RB4
    //
    // set up the UART for default baud, 1 start, 1 stop, no parity
    //
    U2MODEbits.STSEL = 0;       // 1-Stop bit
    U2MODEbits.PDSEL = 0;       // No Parity, 8-Data bits
    U2MODEbits.ABAUD = 0;       // Auto-Baud disabled
    U2MODEbits.BRGH = 0;        // Standard-Speed mode
    U2BRG = BAUD38400;          // Baud Rate setting for 38400 (default)
    U2STAbits.UTXISEL0 = 0;     // Interrupt after TX buffer done
    U2STAbits.UTXISEL1 = 1;
    IEC1bits.U2TXIE = 1;        // Enable UART TX interrupt
    U2MODEbits.UARTEN = 1;      // Enable UART (this bit must be set *BEFORE* UTXEN)

}

void ADCInit(void) // will set 12bit, 4.96us/sample or 202KS/sec
{
    AD1CON1 = 0;            // POR: 10-bit @4ch mode, ADC disabled, manual sample
    AD1CON2 = 0;            // POR: AVdd/Avss for Vref, do not scan, IRQ every sample
    AD1CON3 = 0;            // POR: Use system clock, TAD = 1Tcyc, SAMC = 0TAD
    AD1CON4 = 0;            // POR: no DMA

    AD1CHS123 = 0;          // not used in 12bit mode, as only 1 S/H available
    AD1CON1bits.FORM = 0;   // integer data format (unsigned)
    AD1CON1bits.ASAM = 1;   // continuous automatic sampling enabled

    AD1CON3bits.ADCS = 8;   // 9 Tcy = 1TAD (so TAD = 9*25ns = 225ns = 4.44MHz)
    AD1CON3bits.SAMC = 8;   // set auto sample time as 8TAD = 1.8us


    AD1CON1bits.AD12B = 1;  // 12-bit conversion, 14TAD convert time

    AD1CON1bits.ADON = 1;   // enable converter

    //
    // Turn on port RG8, which supplies +5V to pot
    //
    _TRISG8 = 0;
    _LATG8 = 1;

    //
    // there is a delay time required from ADC ebable until application can begin converting
    // 150us is sufficient. Also allows pot voltage to stabilize, charges up anti-aliasing filter
    Delayus(150);

    AD1CON1bits.SAMP = 1; // begin continuous sampling/conversion

}

/*
void Delayus(int delay)
{
    int i;
    for (i = 0; i < delay; i++)
    {
        __asm__ volatile ("repeat #39");
        __asm__ volatile ("nop");
    }
}
*/

void oscConfig(void)
{

    //  Configure Oscillator to operate the device at 80MHz/40MIPs
    // 	Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // 	Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    // To be safe, always load divisors before feedback
    
   
    CLKDIVbits.PLLPOST = 0;     // N1=2
    CLKDIVbits.PLLPRE = 0;      // N2=2
    PLLFBD = 38;                // M=(40-2), Fcyc = 40MHz for ECAN baud timer


    // Disable Watch Dog Timer

    RCONbits.SWDTEN = 0;

}

void InitCAN(void)
{
    //
    // drive the CAN STANDBY driver pin low
    //
    _TRISG9 = 0;
    _LATG9 = 0;
    _TRISF1 = 0;
    _TRISF0 = 1;

    //
    // remap the CAN module to the proper pins on the board
    //
    RPINR26 = 0x60;         // connect CAN RX to RPI96
    RPOR9 = 0x000E;         // connect CAN TX to RP97

    C1CTRL1bits.REQOP = 4;

    while (C1CTRL1bits.OPMODE != 4);
    C1CTRL1bits.WIN = 0;

    /* Set up the CAN module for 250kbps speed with 10 Tq per bit. */

    C1CFG1 = 0x47;          // BRP = 8 SJW = 2 Tq
    C1CFG2 = 0x2D2;
    C1FCTRL = 0xC01F;       // No FIFO, 32 Buffers

    //
    // set up the CAN DMA0 for the Transmit Buffer
    //
    DMA0CONbits.SIZE = 0x0;
    DMA0CONbits.DIR = 0x1;
    DMA0CONbits.AMODE = 0x2;
    DMA0CONbits.MODE = 0x0;
    DMA0REQ = 70;
    DMA0CNT = 7;
    DMA0PAD = (volatile unsigned int)&C1TXD;
    DMA0STAL = (unsigned int)&ecan1MsgBuf;
    DMA0STAH = (unsigned int)&ecan1MsgBuf;

    C1TR01CONbits.TXEN0 = 0x1;          // Buffer 0 is the Transmit Buffer
    C1TR01CONbits.TX0PRI = 0x3;         // transmit buffer priority

    DMA0CONbits.CHEN = 0x1;

    /* initialise the DMA channel 2 for ECAN Rx */
;
    /* setup channel 2 for peripheral indirect addressing mode
    normal operation, word operation and select as Rx to peripheral */
    DMA2CON = 0x0020;
    /* setup the address of the peripheral ECAN1 (C1RXD) */
	DMA2PAD = (volatile unsigned int)&C1RXD;
 	/* Set the data block transfer size of 8 */
 	DMA2CNT = 7;
 	/* automatic DMA Rx initiation by DMA request */
	DMA2REQ = 0x0022;
	/* start adddress offset value */
	DMA2STAL=(unsigned int)(&ecan1MsgBuf);
    DMA2STAH=(unsigned int)(&ecan1MsgBuf);
	/* enable the channel */
	DMA2CONbits.CHEN=1;

	/* 4 CAN Messages to be buffered in DMA RAM */
	C1FCTRLbits.DMABS=0b000;

    /* Filter configuration */
	/* enable window to access the filter configuration registers */
	C1CTRL1bits.WIN = 0b1;
	/* select acceptance mask 0 filter 0 buffer 1 */
	C1FMSKSEL1bits.F0MSK = 0;

    /* setup the mask to check every bit of the standard message, the macro when called as */
    /* CAN_FILTERMASK2REG_SID(0x7FF) will write the register C1RXM0SID to include every bit in */
    /* filter comparison */
    C1RXM0SID=CAN_FILTERMASK2REG_SID(0x7FF);

	/* configure accpetence filter 0
	setup the filter to accept a standard id of 0x123,
	the macro when called as CAN_FILTERMASK2REG_SID(0x123) will
	write the register C1RXF0SID to accept only standard id of 0x123
	*/
	C1RXF0SID = CAN_FILTERMASK2REG_SID(MSG_SID);
	/* set filter to check for standard ID and accept standard id only */
	C1RXM0SID = CAN_SETMIDE(C1RXM0SID);
	C1RXF0SID = CAN_FILTERSTD(C1RXF0SID);
	/* acceptance filter to use buffer 1 for incoming messages */
	C1BUFPNT1bits.F0BP = 0b0001;
	/* enable filter 0 */
	C1FEN1bits.FLTEN0 = 1;
    /* clear window bit to access ECAN control registers */
	C1CTRL1bits.WIN = 0;

    /* ECAN1, Buffer 1 is a Receive Buffer */
	C1TR01CONbits.TXEN1 = 0;

    /* clear the buffer and overflow flags */
	C1RXFUL1=C1RXFUL2=C1RXOVF1=C1RXOVF2=0x0000;

    // Place the ECAN module in Normal mode.
    C1CTRL1bits.REQOP = 0;
    while (C1CTRL1bits.OPMODE != 0);

    //
    // CAN RX interrupt enable - 'double arm' since 2-level nested interrupt
    //
    C1INTEbits.RBIE = 1;
    IEC2bits.C1IE = 1;
}

/******************************************************************************
*
*    Function:			rxECAN
*    Description:       moves the message from the DMA memory to RAM
*
*    Arguments:			*message: a pointer to the message structure in RAM
*						that will store the message.
******************************************************************************/
void rxECAN(mID *message)
{
	unsigned int ide=0;
	unsigned int rtr=0;
	unsigned long id=0;

	/*
	Standard Message Format:
	Word0 : 0bUUUx xxxx xxxx xxxx
			     |____________|||
 					SID10:0   SRR IDE(bit 0)
	Word1 : 0bUUUU xxxx xxxx xxxx
			   	   |____________|
						EID17:6
	Word2 : 0bxxxx xxx0 UUU0 xxxx
			  |_____||	     |__|
			  EID5:0 RTR   	  DLC
	word3-word6: data bytes
	word7: filter hit code bits

	Remote Transmission Request Bit for standard frames
	SRR->	"0"	 Normal Message
			"1"  Message will request remote transmission
	Substitute Remote Request Bit for extended frames
	SRR->	should always be set to "1" as per CAN specification

	Extended  Identifier Bit
	IDE-> 	"0"  Message will transmit standard identifier
	   		"1"  Message will transmit extended identifier

	Remote Transmission Request Bit for extended frames
	RTR-> 	"0"  Message transmitted is a normal message
			"1"  Message transmitted is a remote message
	Don't care for standard frames
	*/

	/* read word 0 to see the message type */
	ide=ecan1MsgBuf[message->buffer][0] & 0x0001;

	/* check to see what type of message it is */
	/* message is standard identifier */
	if(ide==0)
	{
		message->id=(ecan1MsgBuf[message->buffer][0] & 0x1FFC) >> 2;
		message->frame_type=CAN_FRAME_STD;
		rtr=ecan1MsgBuf[message->buffer][0] & 0x0002;
	}
	/* mesage is extended identifier */
	else
	{
		id=ecan1MsgBuf[message->buffer][0] & 0x1FFC;
		message->id=id << 16;
		id=ecan1MsgBuf[message->buffer][1] & 0x0FFF;
		message->id=message->id+(id << 6);
		id=(ecan1MsgBuf[message->buffer][2] & 0xFC00) >> 10;
		message->id=message->id+id;
		message->frame_type=CAN_FRAME_EXT;
		rtr=ecan1MsgBuf[message->buffer][2] & 0x0200;
	}
	/* check to see what type of message it is */
	/* RTR message */
	if(rtr==1)
	{
		message->message_type=CAN_MSG_RTR;
	}
	/* normal message */
	else
	{
		message->message_type=CAN_MSG_DATA;
		message->data[0]=(unsigned char)ecan1MsgBuf[message->buffer][3];
		message->data[1]=(unsigned char)((ecan1MsgBuf[message->buffer][3] & 0xFF00) >> 8);
		message->data[2]=(unsigned char)ecan1MsgBuf[message->buffer][4];
		message->data[3]=(unsigned char)((ecan1MsgBuf[message->buffer][4] & 0xFF00) >> 8);
		message->data[4]=(unsigned char)ecan1MsgBuf[message->buffer][5];
		message->data[5]=(unsigned char)((ecan1MsgBuf[message->buffer][5] & 0xFF00) >> 8);
		message->data[6]=(unsigned char)ecan1MsgBuf[message->buffer][6];
		message->data[7]=(unsigned char)((ecan1MsgBuf[message->buffer][6] & 0xFF00) >> 8);
		message->data_length=(unsigned char)(ecan1MsgBuf[message->buffer][2] & 0x000F);
	}
	clearRxFlags(message->buffer);
}

/*
void clearRxFlags(unsigned char buffer_number)
{
	if((C1RXFUL1bits.RXFUL1) && (buffer_number==1))
		// clear flag 
		C1RXFUL1bits.RXFUL1=0;
	// check to see if buffer 2 is full 
	else if((C1RXFUL1bits.RXFUL2) && (buffer_number==2))
		// clear flag 
		C1RXFUL1bits.RXFUL2=0;
	// check to see if buffer 3 is full
	else if((C1RXFUL1bits.RXFUL3) && (buffer_number==3))
		// clear flag 
		C1RXFUL1bits.RXFUL3=0;
	else;

}
*/

void Receive_Data(void)
{
    //
    // have we received any messages from somewhere?
    //
    if (can_rx == 1)
    {
        Can_RX_to_UART();
        can_rx = 0;
    }
}

/*
void Can_RX_to_UART(void)
{
    // CAN message out the monitor UART
    //
    U2STAbits.UTXEN = 1;
    putsU2("*** REMOTE CAN MESSAGE ID = 0x");
   //
    // display remote ID byte
    //
    hex_dig = (char)((canRxMessage.id & 0xff00) >> 8);   // upper byte
    ascii_hi = hex_dig & 0xF0;                  // Obtain the upper 4 bits (MSBs) of hex number
    ascii_hi = (ascii_hi >> 4) + 0x30;          // ASCII conversion
    ascii_lo = (hex_dig & 0x0F) + 0x30;         // Obtain the lower 4 bits (LSBs) of hex number

    putU2(ascii_hi);
    putU2(ascii_lo);                    // send out the upper ID byte as ASCII

    hex_dig = (char)(canRxMessage.id & 0x00ff); // lower byte
    ascii_hi = hex_dig & 0xF0;                  // Obtain the upper 4 bits (MSBs) of hex number
    ascii_hi = (ascii_hi >> 4) + 0x30;          // ASCII conversion
    ascii_lo = (hex_dig & 0x0F) + 0x30;         // Obtain the lower 4 bits (LSBs) of hex number

    putU2(ascii_hi);
    putU2(ascii_lo);                    // send out the lower ID byte as ASCII

    putsU2(" RECEIVED ***");
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;
    putsU2("Remote Pot Voltage: ");

    PotValue = ((canRxMessage.data[4]) | (canRxMessage.data[5] << 8));
    Pot_Volts = (float)(PotValue * (float)5.0 / (float)4096.0);
    //
    // convert to ASCII
    //
    pBuf = Buf_result;
    ftoa(Pot_Volts, pBuf);
    for (i = 0; i <= (NUM_DIGITS - 1); i++)
    {
        while (U2STAbits.TRMT == 0);
        U2TXREG = Buf_result[i];
    }
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;

    putsU2("Remote Temperature: ");

    //
    //  print the temperature reading out the UART
    //
    datal = ((canRxMessage.data[2]) | (canRxMessage.data[3] << 8));
    Pot_Volts = (float)((datal - 368) / 15.974);
    //
    // convert to ASCII
    //
    pBuf = Buf_result;
    ftoa(Pot_Volts, pBuf);
    for (i = 0; i <= (NUM_DIGITS - 1); i++)
    {
        while (U2STAbits.TRMT == 0);
        U2TXREG = Buf_result[i];
    }
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;

    putsU2("Remote Switch Status");

    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;
    // ON = pressed  OFF = up
    if ((canRxMessage.data[0] & 0x4) == 0)
    {
        putsU2("SW3: OFF ");
    }
    else
    {
        putsU2("SW3: ON");
    }
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;

    // ON = pressed  OFF = up
    if ((canRxMessage.data[0] & 0x2) == 0)
    {
        putsU2("SW2: OFF ");
    }
    else
    {
        putsU2("SW2: ON");
    }
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;

    // ON = pressed  OFF = up
    if ((canRxMessage.data[0] & 0x1) == 0)
    {
        putsU2("SW1: OFF ");
    }
    else
    {
        putsU2("SW1: ON");
    }

    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;
    //
    // toggle LED 1 to show CAN message reveived
    //
    LED1 = 1;
    //
    // wait 100ms, turn off LED1
    //
    delay_10ms(10);
    LED1 = 0;
}
*/

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

    U2STAbits.UTXEN = 1;
    putsU2("***TRANSMITTING ON-BOARD SENSOR VALUES***");
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;

    putsU2("Local Pot Voltage: Reading = ");

    for (i = 0; i <= (NUM_DIGITS - 1); i++)
    {
        while (U2STAbits.TRMT == 0);
        U2TXREG = Buf_result[i];
    }
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
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

    U2STAbits.UTXEN = 1;

    putsU2("Local Temperature: Reading = ");

    for (i = 0; i <= (NUM_DIGITS - 1); i++)
    {
        while (U2STAbits.TRMT == 0);
        U2TXREG = Buf_result[i];
    }
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;
    //
    // test print the 3 switch statuses
    //

    putsU2("Local Switch status");
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
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
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
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
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
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
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0a;
    while (U2STAbits.TRMT == 0);
    U2TXREG = 0x0d;
    //
    // Format data packets and send out the SENT port
    //
    SENT1DATH = ((~SW3 & 1) << 14) | ((~SW2 & 1) << 13) | ((~SW1 & 1) << 12) | PotValue;
    SENT1DATL = (TempValue << 4);
    SENT1STATbits.TXEN = 1; // note!!! Datasheet has different name than .H file
    Delayus(8000);
    SENT1STATbits.TXEN = 0; // note!!! Datasheet has different name than .H file
    //
    // format and send out the CAN port
    //
    // In order for the demo to run, the CAN controller needs an ACK signal
    // If you desire to run the demo for SENT/LIN only, then comment out the
    // following line of code and recompile
    
    CAN_Transmit();     // Transmit CAN. COMMENT OUT FOR LIN/SENT ONLY!!
    Delayus(5000);

}

/*
void ADCConvert(int channel)
//
// read the channel value
// averages 4 readings to reduce 'jitter'
{
    AverageValue = 0;
    for (i = 0; i < 4; i++)
    {
        AD1CHS0bits.CH0SA = channel;
        _AD1IF = 0;

        AD1CON1bits.SAMP = 0;
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        while (!_AD1IF);
        AverageValue = AverageValue + ADC1BUF0;
    }

    AverageValue = AverageValue >> 2;

}
*/

/*
void ftoa(float f, char *buf)
{
    int pos, ix, dp, num;
    pos = 0;
    ix = 0;
    dp = 0;
    num = 0;

    if (f < 0)
    {
        buf[pos++] = '-';
        f = -f;
    }
    dp = 0;
    while (f >= 10.0)
    {
        f = f / 10.0;
        dp++;
    }
    for (ix = 1; ix < (NUM_DIGITS + 1); ix++)
    {
        num = (int)f;
        f = f - num;
        buf[pos++] = '0' + num;
        if (dp == 0) buf[pos++] = '.';
        f = f * 10.0;
        dp--;
    }
}
*/

/*
void putsU2(char *s)
{
    while (*s)
    { // loop until *s =\0, end of string
        putU2(*s++);
    } // send the character and point to the next one
}
*/

/*
void putU2(int c)
{
    while (U2STAbits.UTXBF); // wait while Tx buffer full
    U2TXREG = c;
}
*/

/*
void CAN_Transmit(void)
{
    ecan1MsgBuf[0][0] = MSG_SID << 2;

    ecan1MsgBuf[0][1] = 0x0000;
    // CiTRBnDLC = 0b0000 0000 xxx0 1111
    // EID<17:6> = 0b000000
    // RTR = 0b0
    // RB1 = 0b0
    // RB0 = 0b0
    // DLC = 6 
    ecan1MsgBuf[0][2] = 0x0006;

    // Write message 6 data bytes as follows:
    //
    // POTH POTL TEMPH TEMPL 0000 S3S2S1
    //
    ecan1MsgBuf[0][3] = (~SW1 & 1) | ((~SW2 & 1) << 1) | ((~SW3 & 1) << 2); // switch data, leading zeros
    ecan1MsgBuf[0][4] = TempValue;
    ecan1MsgBuf[0][5] = PotValue;

    Nop();
    Nop();
    Nop();
    // Request message buffer 0 transmission 
    C1TR01CONbits.TXREQ0 = 0x1;
    // The following shows an example of how the TXREQ bit can be polled to check if transmission is complete. 
    Nop();
    Nop();
    Nop();
    while (C1TR01CONbits.TXREQ0 == 1);
    // Message was placed successfully on the bus, return
}
*/
