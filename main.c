//1.6.2015 rf initQEI in main vergessen
//2.6.2016
#include "p33FJ128MC802.h"
#include "HardwareProfile.h"  //for Microstick
#include "typedefs.h"
#include <libpic30.h>       //delay
//#include <qei.h>

/* Setting up configuration bits */
#if defined(__PIC24HJ64GP502__) || defined(__dsPIC33FJ64MC802__) || defined(__PIC24HJ128GP502__) || defined(__dsPIC33FJ128MC802__)
  _FOSCSEL(FNOSC_FRC);                              // Use internal oscillator  (FOSC ~ 8 Mhz)
  _FOSC((FCKSM_CSECMD & OSCIOFNC_ON) & POSCMD_NONE);  // RA3 pin is output
  _FWDT(FWDTEN_OFF);                                // Watchdog timer is disabled
  _FICD(JTAGEN_OFF);                                // JTAG Port is disabled
#else
  #error "The config bits aren't configured! Please, configure it!"
#endif

//#include "uart1.h"
#define Stepper_MaxDrehzahl = 180        //maximale Drehzahl U/Min
#define Stepper_Polzahl = 400
#define Stepper_MicroStep = 16 
#define Stepper_MaxPulseFrequenz = Stepper_MaxDrehzahl/60*Stepper_Polzahl*Stepper_MicroStep  //19200Hz
//#define BAUD_RATE_UART1	19200L 
#define BAUD_RATE_UART1	38400L 
#define TimeOut         312         //timeout in 1/312 ms   (256/80 000kHz)
#define Modul   1000  
  
void initApp();
void initQEI();
void initSM();
void initRS485();
void readData();
void sendData();

void blink_led(int ms);  
void LED(int on)            {PORTAbits.RA0 = on;}
void QEIstep();              //for Test only
void GMSstep(int);



volatile int RXvalid=0;
struct
 {
// das wird vom dsPC empfangen 
  int  Mode;  //0=Einstellen; 1= Schleifen
  long A;
  long S;
  int  T;    //Zähnezahl bzw. aktueller Zahn
 } RXdata;


 struct
 {
// das wird von dsPIC gesendet
  long X;     //X in µm
  long Z;     //Z in µm 
  int  Err; 
  int  e0;  
  int  e1;    
  int  e2;
  int  e3;
  int  e4;
 } TXdata;


int main( void )
{
 initApp();
 initQEI();
 initSM();
 initRS485();
 //int* Z0;
 //Z0=(int*)&TXdata.Z;
 
 
/*        TRISAbits.TRISA4 = 0;     // verbunden mit RB8
        TRISBbits.TRISB4 = 0;     // verbunden mit RB9 
        PORTAbits.RA4 = 1; 
        PORTBbits.RB4 = 1;
*/ 
 
 while(1) 
 {
  readData();
  sendData();
//  blink_led(10);
 
//  GMSstep(-1);
// *Z0=(int)POS2CNT;
 } 
 return 0; 
}  


void initApp( void )
{
  // Description: Configuring internal oscillator with PLL (7.37 MHz -> 80 MHz )
    PLLFBD = 41;            // M = 43
    CLKDIVbits.PLLPOST=0;   // N1 = 2
    CLKDIVbits.PLLPRE=0;    // N2 = 2
  __builtin_write_OSCCONH(0x01);
  __builtin_write_OSCCONL(0x01);
    while (OSCCONbits.COSC != 0b001);
    while (OSCCONbits.LOCK != 1);
    AD1PCFGL          =	0xFFFF;   // all digital IO 
    TRISA             = 0xFF; // Set all RA* as inputs
    TRISAbits.TRISA0  = 0;    // LED output
    TRISB             = 0xFF; // Set all RB* as inputs
}


//################# Schrittmotorpulse: berechnen, ausgeben  
void initTimer2();
void stepSM(int dir);   //0 +1
void enableSM(int en);
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void) ;


void initSM()
 {
    ODCBbits.ODCB5    = 1;    // open drain   
    ODCBbits.ODCB10   = 1;    // open drain   
    ODCBbits.ODCB11   = 1;    // open drain   
    PORTBbits.RB5     = 0;    // das bleibt auf 0, geschaltet wird mit TRIS  
    PORTBbits.RB10    = 0;    //
    PORTBbits.RB11    = 0;    //  
    TRISBbits.TRISB5  = 1;    // step enable  
    TRISBbits.TRISB10 = 1;    // step  
    TRISBbits.TRISB11 = 1;    // dir
    LATBbits.LATB5    = 0;    // do NOT set the LAT to 1; this will set the pin to 3.3v and the pull pu will try to pull it to 5v.  
    LATBbits.LATB10   = 0;  
    LATBbits.LATB11   = 0;  
    initTimer2();
    enableSM(1);
}

void initTimer2()           //Motor Step Timer; das ist die maximale SM-Pulsfolge   
{
	T2CONbits.TON   = 0;    // Disable Timer
	T2CONbits.TCS   = 0;    // TimeClockSource = internal instruction cycle clock (40MHz))
	T2CONbits.TGATE = 0;	// Disable Gated Timer mode
	T2CONbits.TCKPS = 0b00;	// preScaler= 1:1 ;  40MHz 
	PR2             = 2000; // Load the period value: 20kHz
//	PR2             = 500;  // Load the period value: 80kHz
	TMR2            = 0x00; // Clear timer register
  	IFS0bits.T2IF   = 0;    // Clear Timer2 Interrupt Flag  
    IEC0bits.T2IE   = 1;    // Enable Timer2 interrupt 
    T2CONbits.TON   = 1;    // Start Timer
}

void enableSM(int en)
 {
    TRISBbits.TRISB5 = !en;
 }   
 

void stepSM(int dir)   //0 +1
 {
    static int olddir=0;    //um IOport-read zu vermeiden
    LED(1);  
    PORTBbits.RB10    = 0;    
    PORTBbits.RB11    = 0;      

    TRISBbits.TRISB11 = dir;  // dir    
    if (olddir!=dir)
      __delay_us(4); 
    olddir=dir;
    TRISBbits.TRISB10 = 0; // step 
    __delay_us(4);
    LED(0);
    TRISBbits.TRISB10 = 1; // step  
 }

    
    
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)     //for motorsteps
 {   
   int* x0 =((int*)&TXdata.X);   //update X  
   int* z0 =((int*)&TXdata.Z);   //update Z
   static long delta = 0;        //Nachlaufvariable, multipiziert mit S 
   static long oldX  = 0;

   if (RXvalid)                  //neuen QEI-Werte in TXdata speichern
    {
     *x0=(int)POS1CNT;              
     *z0=(int)POS2CNT;
    } 
   
   delta+=((TXdata.X - oldX) * RXdata.S);
   oldX = TXdata.X;              //altes X merken 
   
   if (delta >= Modul)
    {  
     delta -= Modul;        
     stepSM(1);
    } 
   else if (delta < 0)        
    {
     delta += Modul;        
     stepSM(0);
    }
  _T2IF              = 0;      // Clear Timer2Interrupt Flag 
}   







//########  RS485 ########################################
void initTimer4();
char readByte();     //wartet bis ein Byte eingetroffen ist

void initRS485(void)        
{ 
    initTimer4();
    PR4=TimeOut;                

    TRISBbits.TRISB13 = 0;  // output: -read/write
    TRISBbits.TRISB14 = 0;  // output: TX
    TRISBbits.TRISB15 = 1;  // input: RX
    
    PORTBbits.RB13 = 0;     // RS485 auf recieve stellen
    U1MODE  = 0;             // Clear UART1 mode register
	U1STA   = 0;             // Clear UART1 status register
   _RP14R   = 0b00011;      // U1TX output -> RP14 pin
   _U1RXR   = 15;           // U1RX input  <- RP15 pin 
    
   U1MODEbits.STSEL = 0;    // 1-stop bit
   U1MODEbits.PDSEL = 0;    // No Parity, 8-data bits
   U1MODEbits.ABAUD = 0;    // Auto-Baud disabled
   U1MODEbits.BRGH = 0;     // Standard-Speed mode 
   U1BRG = (FCY/(16*BAUD_RATE_UART1))-1; // Calculate value of baud rate register
    
    
   _U1RXIE = 0;             // Disable Rx interruptrs 
   _U1TXIE = 0;             // Disable Tx interruptrs    
   
    U1MODEbits.UARTEN = 1;  // Enable UART1 module
    U1STAbits.UTXEN   = 1;  // Enable UART1 transmit 
}   

void initTimer4()
{ // Konfiguration von TIMER 2; für RX 
    T4CONbits.TON   = 0;      // Disable Timer
    T4CONbits.TCS   = 0;      // Select internal instruction cycle clock
    T4CONbits.TGATE = 0;      // Disable Gated Timer mode
    T4CONbits.TCKPS = 0b11;   // Select 1:256 Prescaler // Set timer 2 prescaler (0=1:1, 1=1:8, 2=1:64, 3=1:256)  //proove
   	IEC1bits.T4IE   = 0;      // disable Timer1 interrupt
    T4CONbits.TON   = 0;      // Stopp Timer
} 

char readByte()               //wartet bis ein Byte eingetroffen ist
{
    PORTBbits.RB13 = 0;     
    TMR4=0;                   // nullen
    IFS1bits.T4IF = 0;        // Interrupt Flag
    T4CONbits.TON = 1;        // Start Timer
    while (!U1STAbits.URXDA); // warten bis ein Byte eintrifft     
    return (BYTE)U1RXREG;     // Byte eintragen
 }


void readData()  
 {
  PORTBbits.RB13 = 0;    
  TXdata.Err=0;
  TXdata.e0=0;
  TXdata.e1=0;
  TXdata.e2=0;
  TXdata.e3=0;

  int   i=0;                        
  char  c[sizeof(RXdata)];          // Zwischenspeicher    
  char  b;
// U1STAbits.OERR = 0;    
b = readByte();                     //Test, ein dummybyte lesen 
  while (i<sizeof(RXdata)) 
   {
    b = readByte();                 //Byte eintragen

// 0. Fall     
    if (IFS1bits.T4IF && i==0)      //erstes Byte muß mit Überlauf eintreffen
     {
      TXdata.e0+=(1<<i);
      c[i++]=b; 
     } 

// 1. Fall    
    else if (!IFS1bits.T4IF && i>0)                      
     {   
       TXdata.e1+=(1<<i);
       c[i++]=b;
     } 
    
// 2. Fall (Fehler, unterbochener Datenfluß)    
    else if (IFS1bits.T4IF && i>0)
    {
     TXdata.e2+=(1<<i);
     i=0;   
     c[i++]=b; 
     TXdata.Err++;
    }   
        
 // 3.Fall (1. Byte kommt zu schnell; beim Start von readData sind schon Bytes im RX-Puffer)    
    else if (!IFS1bits.T4IF && i==0)
    {
     TXdata.e3+=(1<<i);
     TXdata.Err++;
     i=0;
    }   
  } 
    
 RXvalid=0;  
  for (i=0; i<sizeof(RXdata); i++) ((char*)&RXdata)[i]=c[i];     //Buffer in DataStruct schreiben;
  TXdata.Z=RXdata.A;  //for Test olny
 RXvalid=1;
}    
 


void sendData()  //25.5.15 fct
{ 
 int i;   
 PORTBbits.RB13 = 1;            // TX 
 __delay_us(50);                //50
 char *c=(char*)&TXdata;
 for (i=0; i<sizeof(TXdata); i++)   
 {
  while (U1STAbits.TRMT==0);    // Waits when the output buffer is full
  U1TXREG = c[i];
 }   
 while (U1STAbits.TRMT==0);     //warte bis TX-Buffer leer; TRMT = TransmitshiftRegister empty = 1
 PORTBbits.RB13 = 0;            // RX
}







//################ Q E I ######################################
void __attribute__ (( interrupt, no_auto_psv )) _QEI1Interrupt(void);
void __attribute__ (( interrupt, no_auto_psv )) _QEI2Interrupt(void); 

void initQEI(void)
{
 TRISBbits.TRISB6    = 1; // QAE input A  
 TRISBbits.TRISB7    = 1; // QAB input B  
 TRISBbits.TRISB8    = 1; // QAE input A  
 TRISBbits.TRISB9    = 1; // QAB input B  

 //TABLE 4-24: PERIPHERAL PIN SELECT INPUT REGISTER MAP
 RPINR14bits.QEA1R   = 6; // QEA on RP6     5V tolerant   pin15
 RPINR14bits.QEB1R   = 7; // QEB on RP7     5V tolerant   pin16
 RPINR16bits.QEA2R   = 8; // QEA on RP8     5V tolerant   pin17 
 RPINR16bits.QEB2R   = 9; // QEB on RP9     5V tolerant   pin18
 
//QEI1
 QEI1CONbits.QEIM    = 0; // Disable QEI Module
 QEI1CONbits.CNTERR  = 0; // Clear any count errors
 QEI1CONbits.QEISIDL = 0; // Continue operation during sleep
 QEI1CONbits.SWPAB   = 1; // QEA and QEB swapped
 QEI1CONbits.PCDOUT  = 0; // Normal I/O pin operation
 QEI1CONbits.POSRES  = 0; // Index pulse resets position counter
 DFLT1CONbits.CEID   = 1; // Count error interrupts disabled
 DFLT1CONbits.QEOUT  = 1; // Digital filters output enabled for QEn pins
 DFLT1CONbits.QECK   = 5; // 4=1:32  5=1:64 (7=1:256 zu langsam) clock divide for digital filter for QEn
 POS1CNT             = 0; // Reset position counter
 _QEI1IF              = 0; //Interruptflag   
_QEI1IE              =      0; //Interrupt enable
 QEI1CONbits.QEIM    = 7; // Quadrature Encoder Interface enabled (x4 mode) with position counter reset by match (MAXxCNT)

//_QEI2IP             = 0x11;     //int priority (rf3.5.15))
 QEI2CONbits.QEIM    = 0; // Disable QEI Module
 QEI2CONbits.CNTERR  = 0; // Clear any count errors
 QEI2CONbits.QEISIDL = 0; // Continue operation during sleep
 QEI2CONbits.SWPAB   = 0; // QEA and QEB not swapped
 QEI2CONbits.PCDOUT  = 0; // Normal I/O pin operation
 QEI2CONbits.POSRES  = 0; // Index pulse resets position counter
 DFLT2CONbits.CEID   = 1; // Count error interrupts disabled
 DFLT2CONbits.QEOUT  = 1; // Digital filters output enabled for QEn pins
 DFLT2CONbits.QECK   = 5; // 1:64 clock divide for digital filter for QEn
 POS2CNT             = 0; // Reset position counter
_QEI2IF              = 0;
_QEI2IE              =       0;
 QEI2CONbits.QEIM    = 7; // X4 mode; das muß 7 sein, sonst kein Interrupt

}
  

void __attribute__ (( interrupt, no_auto_psv )) _QEI1Interrupt(void) 
{                           //wird aufgerüfen bei POS1CNT Überlauf; MSint von long X  nachführen 
  int* x1 =((int*)&TXdata.X)+1;     
  if (QEI1CONbits.UPDN == 1)
      (*x1)++;
  else 
    (*x1)--;
 _QEI1IF=0;  
}


void __attribute__ (( interrupt, no_auto_psv )) _QEI2Interrupt(void) 
{
  int* z1=((int*)&TXdata.Z)+1;     
  if (QEI2CONbits.UPDN == 1)
    (*z1)++;
  else 
    (*z1)--;
 _QEI2IF=0;
}




//######## auxiliary ##############################
/* 
 auf dem microstick kann 11 - 18 und 12 - 17 gebrückt werden:
 RA4 uns RA4 simulieren den QEI-Signale für QUE2
 die Brücken müssen nachher wieder entfernt werden
 */
void GMSstep(int dir)
{
 TRISBbits.TRISB4  = 0; // out  
 TRISAbits.TRISA4  = 0; // out    
 static int s=0;
 if (dir==1)
   s--;
 else
   s++;
 s=s&0x0003;

 switch(s)
    { 
         case 0: PORTAbits.RA4 = 0; PORTBbits.RB4 = 0;  break;
         case 1: PORTAbits.RA4 = 0; PORTBbits.RB4 = 1;  break;
         case 2: PORTAbits.RA4 = 1; PORTBbits.RB4 = 1;  break;
         case 3: PORTAbits.RA4 = 1; PORTBbits.RB4 = 0;  break;
    }
 __delay_us(10);
}



void blink_led(int ms)
{
    LED(1);
    __delay_ms(ms);
    LED(0);
}



