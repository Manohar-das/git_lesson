// Changes in Git hub V1




#pragma config DEBUG =      OFF
#pragma config JTAGEN =     OFF
#pragma config ICESEL =     ICS_PGx2
#pragma config TRCEN =      OFF
#pragma config BOOTISA =    MIPS32
#pragma config FSLEEP =     OFF
#pragma config DBGPER =     PG_ALL
#pragma config SMCLR =      MCLR_NORM
#pragma config SOSCGAIN =   GAIN_2X
#pragma config SOSCBOOST =  ON
#pragma config POSCGAIN =   GAIN_LEVEL_3
#pragma config POSCBOOST =  ON
#pragma config EJTAGBEN =   NORMAL

/*** DEVCFG1 ***/
#pragma config FNOSC =      SPLL
#pragma config DMTINTV =    WIN_127_128
#pragma config FSOSCEN =    ON
#pragma config IESO =       ON
#pragma config POSCMOD =    HS
#pragma config OSCIOFNC =   OFF
#pragma config FCKSM =      CSECME
#pragma config WDTPS =      PS1048576
#pragma config WDTSPGM =    STOP
#pragma config FWDTEN =     OFF
#pragma config WINDIS =     NORMAL
#pragma config FWDTWINSZ =  WINSZ_25
#pragma config DMTCNT =     DMT31
#pragma config FDMTEN =     OFF

/*** DEVCFG2 ***/
#pragma config FPLLIDIV =   DIV_1
#pragma config FPLLRNG =    RANGE_8_16_MHZ
#pragma config FPLLICLK =   PLL_POSC
#pragma config FPLLMULT =   MUL_40
#pragma config FPLLODIV =   DIV_4
#pragma config VBATBOREN =  ON
#pragma config DSBOREN =    ON
#pragma config DSWDTPS =    DSPS32
#pragma config DSWDTOSC =   LPRC
#pragma config DSWDTEN =    OFF
#pragma config FDSEN =      ON
#pragma config BORSEL =     LOW
#pragma config UPLLEN =     ON

/*** DEVCFG3 ***/
#pragma config USERID =     0xffff
#pragma config FUSBIDIO2 =   ON
#pragma config FVBUSIO2 =  ON
#pragma config PGL1WAY =    ON
#pragma config PMDL1WAY =   ON
#pragma config IOL1WAY =    ON
#pragma config FUSBIDIO1 =   ON
#pragma config FVBUSIO1 =  ON
#pragma config PWMLOCK =  OFF

/*** BF1SEQ ***/
#pragma config TSEQ =       0x0
#pragma config CSEQ =       0xffff    





// SEQ

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
//#include <plib.h>
#include <cp0defs.h>
#include <sys/attribs.h>

#define LEDS_ON_OFF 0x55
#define LEDS_OFF_ON 0xAA
//#define GetPeripheral2Clock() (PB2CLK)

void SPI_PortInit(void);
void SPI1_Init(void);
void UART2_Init(void);
void Timer1_init(void);
void Timer2_init(void);
void Start_Timer1(void);
void Start_Timer2(void);
void Flash_CSLow(void);
void Flash_CSHigh(void);
void ReadDeviceId(unsigned int );
void spisendbytes(int , unsigned int , int* );
void UART_Send_String(char *);
void UART_Receive(int , char* );

char recdata[20];// = {0};

void delay (void)
{
    int n = 5000000;
    while(n>0) {n--;}
    
}
//  USE IPL4SRS/SOFT/AUTO in the ISR - correspondingly set the priority levels in PERIPHERALS
void __ISR(_TIMER_1_VECTOR, IPL4AUTO)Timer1Handler(void)
//void __ISR_AT_VECTOR(_TIMER_1_VECTOR, IPL3AUTO)Timer1Handler(void)
{
    LATGINV = 0X0002;
    IFS0CLR = 0x00000010;
    
}

void __ISR(_TIMER_2_VECTOR, IPL3AUTO)Timer2Handler(void)
//void __ISR_AT_VECTOR(_TIMER_1_VECTOR, IPL3AUTO)Timer1Handler(void)
{
    LATDINV = 0X1000;
    IFS0CLR = 0x00000200;
    
}

void __ISR(_SPI1_RX_VECTOR, IPL6AUTO)SPI1_RX_Handler(void)
//void __ISR_AT_VECTOR(_TIMER_1_VECTOR, IPL3AUTO)Timer1Handler(void)
{
    IFS1bits.SPI1RXIF = 0;
}

void __ISR(_SPI1_TX_VECTOR, IPL7AUTO)SPI1_TX_Handler(void)
//void __ISR_AT_VECTOR(_TIMER_1_VECTOR, IPL3AUTO)Timer1Handler(void)
{
    Nop();
    LATDSET = 0x1000;
    IFS1bits.SPI1TXIF = 0;
}


int main () {
    int i;
    unsigned int pbFreq;
    unsigned int rdata;
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    //char uartdata[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    //UART2_Init();
    
    SPI1_Init();
    //Timer1_init();
    
//    TRISGCLR = 0x1000; //RG15 as output
//    LATGSET = 0x1000; // Set RG15 = 1;
//    LATGCLR = 0x1000; // Set RG15 = 0;
//    LATGSET = 0x1000; // Set RG15 = 1;
//    LATGCLR = 0x1000; // Set RG15 = 0;
//    LATGSET = 0x1000; // Set RG15 = 1;
//    LATGCLR = 0x1000; // Set RG15 = 0;
    
    //Timer1_init();
    //PB2DIV |= 0x00008000; //Sys clk /4 = 30MHZ
    
    //Timer2_init();
    __builtin_enable_interrupts();
    PRISS = 0x76543210;
    //INTCONSET = 0x00001000; //_INTCON_MVEC_MASK;
    INTCONbits.MVEC = 1;
    
    //UART_Send_String(uartdata);
    //UART_Receive(10, recdata);
    while(1){
            do {
                    SPI1BUF = 'A';
                    for(i=0; i<0xffff; i++);
                }while(IFS1bits.SPI1TXIF);
    }

//    SPI1BUF = 'A';
//    SPI1BUF = 'A';
//    SPI1BUF = 'A';
//    SPI1BUF = 'A';
    //Start_Timer1();
//    Start_Timer2();
    
//    ReadDeviceId(rdata);
//    ReadDeviceId(rdata);
    while (1) {
    ;
    }
    return -1;

}

void UART2_Init() {
    
    //Port Init for UART
    //Remap Peripherals
    
    U2RXR = 0x0011; // RPB11 - U2RX as input
    RPG1R = 0x0010; // RPG1 - U2TX as output
    
    
    TRISGCLR = 0x0002; //Set TX as output
    TRISBSET = 0x0800; // Set RX as input
    
    LATGSET = 0x0002; // Set Output as 1
    
    U2MODEbits.BRGH = 0;
    U2BRG = 0xC2;
    U2MODEbits.STSEL = 0;   

    U2MODEbits.ON = 0;
    U2MODEbits.SIDL = 0;
    U2MODEbits.IREN = 0;
    U2MODEbits.RTSMD = 0;
    U2MODEbits.UEN = 0b00;
    U2MODEbits.WAKE = 0;
    U2MODEbits.LPBACK = 0;
    U2MODEbits.RXINV = 0;
    U2MODEbits.PDSEL = 0b00;
    
    U2STAbits.UTXEN = 1;
    U2STAbits.URXEN = 1;
    
    U2MODEbits.ON = 1;
    
      
}

void UART_Receive(int len, char* rdata) {
    int i;
    U2STAbits.URXEN = 1;
    
    for(i = 0; i < len; i++){
        while(!U1STAbits.URXDA)
            rdata[i] = U2RXREG;
    }
    
}

void UART_Send_String(char *data) {
    int i = 0;
    U2STAbits.UTXEN = 1;
    
    while(*data) {
        while(U2STAbits.UTXBF);
        U2TXREG = *data;
        data++;
    }
    
}
void ReadDeviceId(unsigned int rbuf) {
    
    unsigned int wr[4] = {0};
    wr[0] = 0xAB;
    wr[1] = 0x55;
    wr[2] = 0xAA;
    wr[3] = 0x55;
    spisendbytes(4, rbuf, wr);
    
}

void spisendbytes(int num, unsigned int rbuffer, int* data) {
    
    int i;
    Flash_CSLow();
    for (i = 0; i < num; i++){
        SPI1BUF = data[i];
        //while (!SPI1STATbits.SPITBE);
    }
    //while (!SPI1STATbits.SPIRBF);
    Flash_CSHigh();
    //rbuffer = SPI1BUF;
}

void Start_Timer1(void){
    T1CONSET = 0x8000; // Start the timer 
}

void Timer2_init(void) {
    TRISDCLR = 0x1000; //RG15 as output
    LATDCLR = 0X1000; // Set RG15 = 1;
    //TRISDbits.TRISD15 = 0;
    
    //T2CON = 0x0; // Stop timer and clear control register,
    //T2CON = 0x0030;
// set prescaler at 1:1, internal clock source
    T2CON = 0x0; // Clear timer register
    T2CONbits.TCKPS = 0b111; //PB2CLK/256
    PR2 = 0xFFFF; // Load period register
    //T2CONbits.TCKPS = 0b111;
    //T2CONSET = 0x8000; // Start timer
    
    IFS0CLR =  0x00000200; // Clear the timer interrupt status flag
    IPC2 = 0x00000E00;  //Set priority level = 3, 2
      
    IEC0SET = 0x00000200; // Enable timer interrupts
    
    
}

void Start_Timer2(void){
    T2CONSET = 0x8000; // Start the timer 
}


void Timer1_init(void) {
    
    TRISGCLR = 0x0002; //RG1 as output
    LATGSET = 0X0002; // Set RG1 = 1;
    //Clear the ON control bit (TxCON<15> = 0) to disable the timer.
    //Clear the TCS control bit (TxCON<1> = 0) to select the internal PBCLK source.
    T1CON = 0x0; // Stop the timer and clear the control register,
    // prescaler at 1:1,internal clock source
    //PB2DIV = 0x0000807F;
    T1CONbits.TCS = 0;
    T1CON |= 0x70;
    //TMR1 = 0x0070; // Clear the timer register
    
    PR1 = 0xFFFF; // Load the period register
    
    //IPC1SET = 0x00000018; //Set priority level = 2
    IFS0CLR =  0x00000010; // Clear the timer interrupt status flag
    IPC1bits.T1IP = 4;  //Set priority level = 4
    IPC1SET = 0x00000002;//Set priority level = 2   
    IEC0SET = 0x00000010; // Enable timer interrupts
   
    
}

void SPI_PortInit(void) {
    
    TRISBSET = 0X0008; //SET RB3 as input SDI
    TRISBCLR = 0x0010; //SET RB4 as output SDO
    TRISCCLR = 0x0060; // SET RC6 as output SS
    TRISBCLR = 0x0080; //SET SPI_CK as output
    
    LATBCLR = 0x0080; //SPI_CK= 0
    LATBCLR = 0x0010; //SDO = 0;
          
    
}
void SPI1_Init(void) {
    
    int rdata;
    
    SDI1R = 0x01;   // MISO - SDI1 is assigned to RPB3 Input
    
    RPB4R = 0x03;   //MOSI I- SDO1 RPB4 Output
    RPB6R = 0x03;   //SS OR CS Output RPB6
    SPI_PortInit();
    
    SPI1BRG = 0x00000000;   //FPBCLK2 / 2
    //SPI1CONSET = 0x00008120;   //ON, CKE,SSEN and Master Mode Enabled   
    IEC1bits.SPI1EIE = 0;        
    IEC1bits.SPI1RXIE = 0;  
    IEC1bits.SPI1TXIE = 0;  
    SPI1CON = 0;
    rdata = SPI1BUF;
    IFS1CLR = 0x00000038;
    
    IPC9SET = 0x00001E00; //TX DONE 7Prio - 2-sub-prio
    IPC9SET = 0x0000001A; //RX DONE 6Prio - 2-sub-prio
    IPC8SET = 0x15000000; //FAULT 5 - 1
    
    SPI1BRG = 0;    
    SPI1STATCLR = 0x0040;
    
    SPI1CON = 0x8220;
    
    IEC1bits.SPI1RXIE = 1;  
    IEC1bits.SPI1TXIE = 1;
    IEC1bits.SPI1EIE = 1; 
 
}

void Flash_CSLow(void) {
    LATBCLR = 0x0080;
}

void Flash_CSHigh(void){
     LATBSET = 0x0080;
}

