#include <msp430g2553.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define rozmiar 4

long datI=0;
long dattmp=0;
double Uwe,Uwy,Iwe,Uwyn1;
double mocwzrasta=1;
int krok=1;
int n=0;
char znak;
char jest_znak=0;
//static double trend[rozmiar]={-0.1429, -0.0857, -0.0286, 0.0286, 0.0857, 0.1429};
static double trend[rozmiar]={-0.3000, -0.1000, 0.1000, 0.3000};

//prototypy funkcji 
void DCO_start();
void ADC_init();
int getchar(void);
int putchar(int c);
void print(const char *s);
void printx(const uint8_t c);
void uart_init();  
void printi(const uint8_t c); 
double przelicz(int dat, double off, double maksPP);
void printXXXX(double tmp, int krop);
void printXXX(double tmp, int krop);
void printx16(const uint16_t c);
long pomiarAC(int kanal);
double getUwy();
double getUwe();
double getIwe();
void MPPT();

double licz_srednia(double wczesniej,double aktual, double zapominanie);

int main(void)
{
  int n;
  double tmp;
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
   __bic_SR_register(GIE);
  P1DIR |= BIT0 + BIT6;  
  P2DIR |= BIT1+BIT2;
  P2OUT &=~BIT1+BIT2;
  
  DCO_start();
  
  //PWM
  
  P1DIR |= BIT6;                            // P1.2 output
  P1SEL |= BIT6;                            // P1.2 for TA0.1 output
  P1SEL2 = 0;								// Select default function for P1.2
  CCR0 = 512-1;                             // PWM Period
  CCTL1 = OUTMOD_7;                         // CCR1 reset/set
  CCR1 = 10;//0x1b0;                               // CCR1 PWM duty cycle
  TACTL = TASSEL_2 + MC_1 + ID_0;                  // SMCLK, up mode


  uart_init();
    
  

  Uwy=((double) pomiarAC(5))*0.604;//V
           
  while(1) {
    
    //for(n=0;n<10;n++){
        P1OUT |= BIT0; 
       
        MPPT();
        //datI=pomiarAC(3); CCR1=datI;
        //if(Uwy<230) CCR1++; else CCR1--; //pwm-set 
       
        P1OUT ^= BIT0; 
       
        //__delay_cycles(5000000);
 
        print("Ui=");
        printXXX(getUwe(),0);
        print(" Uo=");
        printXXX(getUwy(),0);
        
        print("  P=");
        printXXX(getIwe()*getUwe()/1000,0);
        print(" ");
        if(krok>0) print("++"); else print("--");
         
        tmp= (double) CCR1;
        tmp=tmp/51.1;
          
        print(" .");
        printi((int) tmp);
        //print(" ");  
        //printXXX(tmp,1);
               
        //print("%");
        //Uwy=licz_srednia(Uwyn1,Uwy, 0.001);
        //printXXXX(krok,1);
        //printx16(CCR1);
          
        print("\r\n");
  
     }
}
//*********************************************************************************************** 
void MPPT(){
    
    double a;
    int tmp0,tmp1,n;
    
    /*
    a=-0.3*getUwy();
    CCR1=CCR1+krok;
    a=a-0.1*getUwy();
    CCR1=CCR1+krok;
    a=a+0.1*getUwy();
    CCR1=CCR1+krok; 
    a=a+0.3*getUwy();
    */
    
    a=trend[0]*getUwy();
    for(n=1;n<rozmiar;n++){
        CCR1=CCR1+krok;
        a=a+trend[n]*getUwy();
    }
      
      
    if(a>0) mocwzrasta=1; else mocwzrasta=0;
    if(mocwzrasta==0) krok=-krok;
    
    tmp0=CCR0; 
    tmp1=CCR1;
    if(tmp1>(tmp0-10)) CCR1=tmp0-10;
    if(CCR1<10) CCR1=10;
    
    if((getUwe()-getUwy())<1) CCR1=10;
    
}
//*********************************************************************************************** 
void DCO_start(){

  if (CALBC1_16MHZ==0xFF)					// If calibration constant erased
  {											
    while(1);                               // do not load, trap CPU!!	
  }
  DCOCTL = 0;                               // Select lowest DCOx and MODx settings
  BCSCTL1 = CALBC1_16MHZ;                   // Set range
  DCOCTL = CALDCO_16MHZ;                    // Set DCO step + modulation*/
}
 //*********************************************************************************************** 

 //*********************************************************************************************** 
int getchar(void) {
    //while(!(IFG2 & UCA0RXIFG));
    //IFG2 &= ~UCA0RXIFG;
    //return UCA0RXBUF;
    while(jest_znak==0);
    jest_znak=0;
    return znak;
}
 //*********************************************************************************************** 
int putchar(int c) {
    while(!(IFG2 & UCA0TXIFG));                  // wait for TX buffer to be empty
    UCA0TXBUF = c;

    return c;
}
 //*********************************************************************************************** 
void print(const char *s) {
    while(*s) putchar(*s++);
}
 //*********************************************************************************************** 
void printx(const uint8_t c) {
    static char hex_table[] = "0123456789abcdef";

    putchar(hex_table[(c & 0xF0) >> 4]); putchar(hex_table[c & 0x0F]);
}
void printx16(const uint16_t c) {
    static char hex_table[] = "0123456789abcdef";

    putchar(hex_table[(c & 0xF000) >> 12]);putchar(hex_table[(c & 0x0F00) >> 8]);putchar(hex_table[(c & 0x00F0) >> 4]); putchar(hex_table[c & 0x000F]);
}
 //*********************************************************************************************** 
void printi(const uint8_t c) {
    static char hex_table[] = "0123456789abcdef";

    //putchar(hex_table[(c & 0xF0) >> 4]); 
    putchar(hex_table[c & 0x0F]);
}
double przelicz(int dat, double off, double maksPP){
      double tmp;
      tmp=(double) dat;
      //tmp=1023;
      tmp=tmp/1023;
      tmp=tmp*maksPP;
      return tmp-off;
}
void printXXXX(double tmp, int krop){
      int cyf1000,cyf100,cyf10,cyf1,cyft,cyfs;
      
      if (krop==4) tmp=tmp*10000;
      if (krop==3) tmp=tmp*1000;    
      if (krop==2) tmp=tmp*100;
      if (krop==1) tmp=tmp*10;
      
      
      if(tmp<0) {tmp=-tmp;  print("-");}else{print(" ");}
      cyf1000=(int) (tmp/1000);
      cyft=cyf1000*1000;
      cyf100=(int) (tmp-cyft)/100;
      cyfs=cyf100*100;
      cyf10=(int) (tmp-cyft-cyfs)/10;
      cyf1=(int) tmp-cyft-cyfs-cyf10*10;

      if (krop==4) print(".");
      //if (krop<4 && cyf1000!=0) 
        printi(cyf1000); 
      if (krop==3) print(".");      
      //if (krop<3 && cyf100!=0) 
        printi(cyf100);
      if (krop==2) print(".");
      //if (krop<2 && cyf10!=0) 
        printi(cyf10);
      if (krop==1) print(".");
      printi(cyf1);
}
//*********************************************************************************************** 
void printXXX(double tmp, int krop){
      int cyf100,cyf10,cyf1,cyfs;
      
      if (krop==3) tmp=tmp*1000;    
      if (krop==2) tmp=tmp*100;
      if (krop==1) tmp=tmp*10;
      
      
      if(tmp<0) {tmp=-tmp;  print("-");}else{print(" ");}

      cyf100=(int) tmp/100;
      cyfs=cyf100*100;
      cyf10=(int) (tmp-cyfs)/10;
      cyf1=(int) tmp-cyfs-cyf10*10;

      if (krop==3) print(".");      
      //if (krop<3 && cyf100!=0) 
        printi(cyf100);
      if (krop==2) print(".");
      //if (krop<2 && cyf10!=0) 
        printi(cyf10);
      if (krop==1) print(".");
      printi(cyf1);
}
//*********************************************************************************************** 
// USCI A0/B0 Transmit ISR
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
//  unsigned char TxByte=0;
//  if (P1IN & BIT3)
//    TxByte |= BIT6;
//  if (P1IN & BIT4)
//    TxByte |= BIT0;
//  UCA0TXBUF = TxByte;                       // Read, justify, and transmit
    IFG2 &= ~UCA0TXIFG;
}
//*********************************************************************************************** 
// USCI A0/B0 Receive ISR
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  //P1OUT ^= BIT6;                            // Toggle P1.0
  znak = UCA0RXBUF;                        // Display RX'ed charater
  jest_znak=1;
  IFG2 &= ~UCA0RXIFG;
}
//*********************************************************************************************** 
void uart_init(void){
  //UART*************************************************
    const unsigned long MCLK_HZ = 16000000;          // SMCLK frequency in Hz
    const unsigned BPS = 9600;                       // ASYNC serial baud rate
    const unsigned long baud_rate_20_bits = (MCLK_HZ + (BPS >> 1)) / BPS; // Bit rate divisor

    // Configure P1.1 and P1.2 as UART controlled pins
    P1DIR &= ~(BIT1 | BIT2);                        // Revert to default to GPIO input
    P1SEL |= BIT1 | BIT2;                            // P1.1=RXD, P1.2=TXD
    P1SEL2 |= BIT1 | BIT2;                           // P1.1=RXD, P1.2=TXD

    // Configure USCI UART for 2400
    UCA0CTL1 = UCSWRST;                             // Hold USCI in reset to allow configuration
    UCA0CTL0 = 0;                                   // No parity, LSB first, 8 bits, one stop bit, UART (async)
    UCA0BR1 = (baud_rate_20_bits >> 12) & 0xFF;     // High byte of whole divisor
    UCA0BR0 = (baud_rate_20_bits >> 4) & 0xFF;      // Low byte of whole divisor
    UCA0MCTL = ((baud_rate_20_bits << 4) & 0xF0) | UCOS16; // Fractional divisor, over sampling mode
    UCA0CTL1 = UCSSEL_2;                            // Use SMCLK for bit rate generator, then release reset   
    IE2 |= UCA0RXIE;// + UCA0TXIE;               // Enable USCI_A0 TX/RX interrupt

  __bis_SR_register(GIE);       // Enter LPM3 w/ interrupts enabled
}
//*********************************************************************************************** 
double getUwy(){
  return ((double) pomiarAC(5))*0.604;//V
}
double getUwe(){
  return ((double) pomiarAC(7))*0.640;//V
}
double getIwe(){
  return ((double) pomiarAC(4))*3.375;//mA
}

long pomiarAC(int kanal){
      
      int ile_iter=2048;
      
      //pomiar AC na P1.7
      //ADC10CTL0=SREF_0 + REFON + ADC10ON + ADC10SHT_0 + REF2_5V + REFOUT; 
      ADC10CTL0=SREF_1 + REFON + ADC10ON + ADC10SHT_0 + REF2_5V; //referencja do 2.5 V 
      /*
      if(kanal==7)  ADC10CTL1=INCH_7+ ADC10DIV_0; 
      if(kanal==6)  ADC10CTL1=INCH_6+ ADC10DIV_0; 
      if(kanal==5)  ADC10CTL1=INCH_5+ ADC10DIV_0; 
      if(kanal==4)  ADC10CTL1=INCH_4+ ADC10DIV_0; 
      if(kanal==3)  ADC10CTL1=INCH_3+ ADC10DIV_0; 
      if(kanal==2)  ADC10CTL1=INCH_2+ ADC10DIV_0; 
      if(kanal==1)  ADC10CTL1=INCH_1+ ADC10DIV_0; 
      if(kanal==0)  ADC10CTL1=INCH_0+ ADC10DIV_0; 
      */
      ADC10CTL1=kanal<<12+ ADC10DIV_0;
      
      dattmp=0;
       for(n=0;n<ile_iter;n++){
          ADC10CTL0 |= ENC + ADC10SC;
          while(ADC10CTL1 & BUSY);
          dattmp=dattmp+ADC10MEM;  
          //ADC10CTL0 &= ~ENC;
       }
       return (dattmp/ile_iter); 
       //koniec AC
}

//void ADC_init(){
  //init ADC
    //ADC10CTL0=SREF_1 + REFON + ADC10ON + ADC10SHT_3; //1.5V ref,Ref on,64 clocks for sample
    //ADC10CTL0=SREF_1 + REFON + ADC10ON + ADC10SHT_3 + REF2_5V ; //1.5V ref,Ref on,64 clocks for sample
    //ADC10CTL0=SREF_1 + REFON + ADC10ON + ADC10SHT_3 + REF2_5V + REFOUT; // pa P1.4 ustawione wyjscie nap. referencyjnego 2.5 V
//    ADC10CTL0=SREF_0 + REFON + ADC10ON + ADC10SHT_3 + REF2_5V + REFOUT; // pa P1.4 ustawione wyjscie nap. referencyjnego 2.5 V, nap odniesienia 3.3V
//    ADC10CTL1=INCH_7+ ADC10DIV_3; //temp sensor is at 10 and clock/4
//}
//*********************************************************************************************** 
/*
int ADC_pomiarP17(){
  //ADC pomiar
    ADC10CTL1=INCH_7+ ADC10DIV_3; //temp sensor is at 10 and clock/4
    ADC10CTL0 |= ENC + ADC10SC;
    while(ADC10CTL1 & BUSY);
    ADC10CTL0 &= ~ENC;
    return ADC10MEM;
}
//-*********************************************************************************************** 
int ADC_pomiarP15(){
  //ADC pomiar
    ADC10CTL1=INCH_5+ ADC10DIV_3; //temp sensor is at 10 and clock/4
    ADC10CTL0 |= ENC + ADC10SC;
    while(ADC10CTL1 & BUSY);
    ADC10CTL0 &= ~ENC;
    return ADC10MEM;
}
//-*********************************************************************************************** 
void ADC_pomiarUI(){
  //ADC pomiar U na P1.5
  //P1OUT ^= 0x01;
  
  //  ADC10CTL1=INCH_5+ ADC10DIV_3; //temp sensor is at 10 and clock/4
   // ADC10CTL0 |= ENC + ADC10SC;
  //  while(ADC10CTL1 & BUSY);
  //  datU= ADC10MEM;    
  //  ADC10CTL0 &= ~ENC;
  
    //P1OUT ^= 0x01;
    //P1OUT ^= BIT6;
    //ADC pomiar I na P1.7
    ADC10CTL1=INCH_7+ ADC10DIV_3; //temp sensor is at 10 and clock/4
    ADC10CTL0 |= ENC + ADC10SC;
    while(ADC10CTL1 & BUSY);
    datI= ADC10MEM;    
    ADC10CTL0 &= ~ENC;
    //P1OUT ^= BIT6;
}
*/
 //*********************************************************************************************** 
//*********************************************************************************************** 
double licz_srednia(double wczesniej,double aktual, double zapominanie){
    //zapominanie=0.001;
    return wczesniej + zapominanie*(aktual-wczesniej);
}
 //*********************************************************************************************** 
