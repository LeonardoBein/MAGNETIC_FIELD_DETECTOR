
//****************** MAGNETIC FIELD DETECTOR *************************
//    Authors:  Wesley Thiago Egea Tiem
//    Marlio Bonfim
//********************************************************************
//    DATE      |   Description
//********************************************************************
// 07/12/2017   |   Initial code created from BILI UFPR
// 12/12/2017   |   Final code MAGNETIC FIELD DETECTOR 2.1
// 18/01/2018   |   Changed linear equation of amplitude (Marlio and Augusto)
//  Entradas:
// P1.4 - Entrada p/ comparador interno
// P1.5 - Tensão p/ ADC
// P2.0 - Trigger - comparador
//
// Saídas
// P1.1 - UART RX
// P1.2 - UART TX
// P1.3 - Comparator Out
// P1.6 - I2C - CLK
// P1.7 - I2C - Data
// P2.1 - Descarga capacitor de integração
// P2.2 - LED' - bateria
//
//----------------------------------------------------------------------

#include <msp430.h>
#include <stdio.h>

void LCD_init();
void LCD_out(short linha,char *imprimir);
void LCD_clear();


float width[2] = {0.0,0.0}, ampl[2]={0.0,0.0};  // store pulse width, pulse amplitude [n, n-1]
char buffer[25], buffer2[25], buffer3[25],buffer4[25], timer_count, n_pulses=0;          // text to display buffer, timer multiplier, pulse counter

//---------------------------------------------------------------------------
//  Microcontroler initilization
//---------------------------------------------------------------------------
void config_hardware(void) {
  WDTCTL = WDTPW+WDTHOLD;                 // Stop WDT
  //configura clock 16MHz
  BCSCTL1 = CALBC1_16MHZ;
  DCOCTL = CALDCO_16MHZ;

  //set_osc_freq_MHZ(16);                   // configuration internal Clock and funcion delay_ms using aux_time.h
}

//---------------------------------------------------------------------------
//  Ports configuration
//---------------------------------------------------------------------------
void config_IO(void) {
  P1DIR = 0x00;                     // Rst P1 -- All pins input
  P2DIR = 0x00;                     // Rst P2 -- All pins input
  P3DIR = 0x0F;                     // P3 -- P3.0 - P3.3: pins input -- P3.4 - P3.7: pins output
  P2IE  |= BIT0;                    // activate interrupt on P2.0
  P2IES &= ~BIT0;                   // interrupt border (up = 0, down = 1), start with rising border
  P2IFG &= ~(BIT0);                 // clear interrupt flag
  P1IE  |= BIT4;                    // activate interrupt on P1.4
  P1IES &= ~BIT4;                   // interrupt border (up = 0, down = 1), start with rising border
  P1IFG &= ~(BIT4);                 // clear interrupt flag

}

//---------------------------------------------------------------------------
// NESTA VERSAO FOI USADO UM COMPARADOR EXTERNO
/*//  Comparator configuration
//---------------------------------------------------------------------------
void config_compare(void) {
  CACTL1 = CAON + CAREF_2  + CAIE;        // Comparator ON, 0.5*Vcc ref on + pin,
                                          //  enable interrupts, rising edge (CAIES = 0).
  CACTL2 = P2CA3 + CAF;                   // Input CA4 on - pin (P1.4), Comparator Filtered

  CACTL1 &= ~CAIFG;                       // comparator interrupt flag clear

  P1DIR  |= BIT3;                         // Configure P1.3 as CAOUT (comparator out)
  P1SEL  |= BIT3;
  P1SEL2 |= BIT3;
}
*/
//---------------------------------------------------------------------------
//  TIMER0A0 Configuration -> Pulse width measurement
//---------------------------------------------------------------------------
void config_timer(void) {
  TA0CTL   =  TASSEL_2 + ID_0 + MC_0;     // Clock source SMCLK, Divide by 1, Stop mode
  TA0CCTL0 = 0;                           // No capture, No interrupt,
  TA0CCR0  = 0xFFFF;                      // Timer limit in up mode, don't care
}

//---------------------------------------------------------------------------
//  TIMER1A0 Configuration -> Delay to wirte result -> diplay
//---------------------------------------------------------------------------
void config_timer_1(void) {
  TA1CTL   =  TASSEL_2 + ID_3 + MC_0;     // Clock source SMCLK, Divide by 8, Stop mode
  TA1CCTL0 |= CCIE;                       // No capture, interrupt enable
  TA1CCR0  = 0xFFFF;                      // Timer limit in up mode, don't care
}

//---------------------------------------------------------------------------
//  UART Configuration
//---------------------------------------------------------------------------
void config_uart(void) {
  UCA0CTL1 |= UCSWRST;                    // force software reset

  UCA0CTL1 |= UCSSEL_2;                   // Clock = SMCLK

  P1SEL    |= (BIT1 + BIT2);              // P1.1 = RXD, P1.2 = TXD
  P1SEL2   |= (BIT1 + BIT2);              // P1.1 = RXD, P1.2 = TXD

 // UCA0BR0  = 0x82;                    // 9600 bps @16 MHz
  UCA0BR0  = 0x8A;                    // 115200 bps @16 MHz
//UCA0BR1  = 0x06;                    // 9600 bps @16 MHz
  UCA0BR1  = 0x00;                    // 115200 bps @16 MHz
//  UCA0MCTL = UCBRS_6;                     // 9600 bps @16 MHz
  UCA0MCTL |= 0;                     // 115200 bps @16 MHz
  UCA0CTL1 &= ~UCSWRST;                   // **Initialize USCI state machine**

 //? IE2    |= UCA0RXIE;                   // Enable USCI_A0 RX interrupt
}

//---------------------------------------------------------------------------
//  Send a byte to UART
//---------------------------------------------------------------------------
void send_byte( unsigned char byte_env ) {
  while (!(IFG2&UCA0TXIFG));              // wait TX buffer empty
  UCA0TXBUF = byte_env;                   // write byte in TX buffer
}

//----------------------------------------------------------------------------
//      Send data (char*) to UART
//----------------------------------------------------------------------------
void send_data_serial(char* buffer_serial) {
  int i = 0;
  while (buffer_serial[i] != '\0') {
    send_byte(buffer_serial[i]);          // send one byte to serial
    i++;
  }
}

//---------------------------------------------------------------------------
//  ADC Configuration - voltage measure from coil
//---------------------------------------------------------------------------
void config_adc()
{
    ADC10CTL0 = REFON + REF2_5V + SREF_1;               //Vref = 2.5 V
    ADC10CTL0 |= ADC10SHT_2;                            // Sample and Hold Time = 16*ADC10CLKs = 250 kHz
    ADC10CTL0 |= ADC10ON;                               // ADC10  ON
    ADC10CTL1 = INCH_5 + ADC10DIV_2 + ADC10SSEL_2;      //in channel 5 + ADC10 clock divide by 4 (16MHz/4=4MHz) + ADC10 clock= MCLK
    ADC10AE0  = BIT5;                                   // analog input just on P1.5
}

//----------------------------------------------------------------------------
//      I2C configuration
//----------------------------------------------------------------------------
void config_I2C(void) {
  // configuration of I2C, according user guide manual order
  UCB0CTL1  |= UCSWRST;                         // I2C Reset

  UCB0CTL0  = UCMST + UCMODE_3 + UCSYNC;        // Master, I2C, synchronous mode
  UCB0CTL1  |= UCSSEL_2;                        // CLK = SMCLK
  UCB0BR1   = 0;
  UCB0BR0   = 160;                              // fSCL = SMCLK/160 = 100 kHz
  UCB0I2CSA = 0x3E;                             // SLAVE ADDRESS (display)

  P1DIR     |= BIT6 + BIT7;

  P1SEL     |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0
  P1SEL2    |= BIT6 + BIT7;                     // Assign I2C pins to USCI_B0

  UCB0CTL1  &= ~UCSWRST;                        // Clear SW reset, resume operation
}


//----------------------------------------------------------------------------
//            Delay miliseconds routine
//----------------------------------------------------------------------------
void delay_ms(int ms){
  while((ms) > 0){
     __delay_cycles(15988);
     ms = ms -1;
  }
}
////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------------------------------------
//            pulse amplitude measure routine
//----------------------------------------------------------------------------

float pulse_a(void) {
  float sample = 0;

  for (int k = 0; k < 100; k++) {   // oversampling 256
    ADC10CTL0 |= ENC + ADC10SC;           // start conversion
    while(ADC10CTL1 & BUSY);            // wait until conversion is running
    sample += ADC10MEM;                   // acumulate conversion sample
  }

  // **** Capacitor discharge ****
  P2DIR |= BIT1;                          // set P2.1 as output
  P2OUT &= ~(BIT1);                       // SET P2.1 = "0"

  delay_ms(2);                            // 2 ms Delay for capacitor discharge
  P2DIR &= ~BIT1;                         // put P2.1 back to input

  n_pulses++;                                // increase pulse counter

  return sample;                    // offset 1570 bits

  //return sample / 256.0;
  //return sample * 0.00390625;             // divide by 256 and increase resolution
  //return sample * 0.00000953674316406;        // divide by 256 and increase resolution
}

//----------------------------------------------------------------------------
//            pulse width measure routine
//----------------------------------------------------------------------------
unsigned int pulse_w(void) {
  unsigned int width;

  //width = TAR >> 4;
  width = TAR;                            // pulse width us, obtained from TIMER (16 MHz / 1 = 16 MHz)
  TAR   = 0;          // reset timer counter

  return width;
}

//---------------------------------------------------------------------------
//  Battery status test
//---------------------------------------------------------------------------
char verifica_bateria(void) {
  float v_bat = 0;

  ADC10CTL0 &= ~ENC;                                  // disable ADC, permits change input channel
  ADC10CTL1 = INCH_11 + ADC10DIV_2 + ADC10SSEL_2;       // enable AD channel 11 (Vcc/2 internal)

  delay_ms (10);                                        // delay to stabilize the AD

  for (int k = 0; k < 16; k++) {      // oversampling
    ADC10CTL0 |= ENC + ADC10SC;                         // start conversion
    while(ADC10CTL1 & BUSY);                          // wait until conversion is running
    v_bat += ADC10MEM;                                  // acumulate conversion sample
  }

  // convert binary acumulated value in Voltage v_bat*5.0/(16.0*1024.0)
  v_bat *= 0.00030517578125;

  ADC10CTL0 &= ~ENC;                                  // disable ADC, permits change input channel
  ADC10CTL1 = INCH_5 + ADC10DIV_2 + ADC10SSEL_2;        // reenable AD P1.5

  char *conversor = dtostrf(v_bat,1,2,buffer);    //converts float to string, we don't use sprintf now because Energia has a problem compiling this function with float values.
  strcat(buffer2,"V_Bat = ");
  strcat(buffer2,buffer);
  strcat(buffer2," V\n");

  send_data_serial(buffer2);                             // send V_bat to serial

  sprintf(buffer," BATTERY VOLTAGE");             // creates text to send
  LCD_out(1,buffer);          // send text to display
  char *conversor2 = dtostrf(v_bat,1,2,buffer);    //converts float to string, we don't use sprintf now because Energia has a problem compiling this function with float values.
  strcat(buffer3,"     ");            // makes the data pretty
  strcat(buffer3,buffer);
  strcat(buffer3," V");
  LCD_out(2,buffer3);
  delay_ms(3000);         // wait 3 seconds

  //LCD_clear();            // display clear
//



  if (v_bat < 3.7) return 1;                            // verifies if v_bat < 3.7 V
  else return 0;
}

//---------------------------------------------------------------------------
//  Main
//---------------------------------------------------------------------------
 int main() {
      config_hardware();
      config_IO();
      config_uart();
     // send_data_serial("ok")
      //config_I2C();
      config_adc();
      config_timer();
      config_timer_1();
//    config_compare();
      P3OUT = BIT2 + BIT3;                         // set P3.2 to 1
      P3OUT = 0;                         // set P3.2 to 0

      //P2DIR |= BIT3;
      //P2OUT &= ~BIT3;

      delay_ms(1);
      P3OUT = BIT2 + BIT3;                         // set P3.2 to 1
      P3OUT = 0;                         // set P3.2 to 0
      __enable_interrupt();
      P3OUT = BIT2 + BIT3;                         // set P3.2 to 1
      P3OUT = 0;                         // set P3.2 to 0
      //LCD_init();
      //LCD_clear();

      LCD_out(1," MAGNETIC FIELD");
      //send_data_serial(" MAGNETIC FIELD");
      LCD_out(2," DETECTOR V 4.0");
      //send_data_serial(" DETECTOR V 4.0");
      delay_ms(2000);

      //LCD_clear();

      LCD_out(1,"      UFPR");
      //send_data_serial("UFPR");
      LCD_out(2,"   LAMMI 2019");
      //send_data_serial("   LAMMI 2019");

      delay_ms(2000);

      //LCD_clear();

      if(verifica_bateria()){     // if v_bat is low sinalize
        P2DIR |= BIT2;                          // define P2.2 as output
        P2OUT &= ~BIT2;                         // set P2.2 to 0, turn on battery led

        LCD_out(1,"   BATTERY LOW");
        LCD_out(2,"    RECHARGE");

        delay_ms(2000);
        //LCD_clear();
      }

      LCD_out(1,"    WAITING");
      LCD_out(2,"     PULSE");

      while(1);
}

//-------------------------------------------------------------------------
//    UART RX Interrupt Routine
//-------------------------------------------------------------------------
#pragma vector=USCIAB0RX_VECTOR
__interrupt void recebe_byte(void)
{
  //  byte_rec = UCA0RXBUF;     // copy received data to byte_rec
}

//-------------------------------------------------------------------------
//    Interrupt Routine, P2.0, external comparator (Negative Pulse)
//-------------------------------------------------------------------------
#pragma vector = PORT2_VECTOR
__interrupt void interrup_P2(void) {
  P2IE &= ~(BIT0);                      // disable interrupt of external comparator - P2.0
  //send_data_serial("P2.0\n");

  if (P2IES & BIT0) {                   // if interrupt from falling edge
   //send_data_serial("P2.0\n");
    TA0CTL &= ~MC_2;                    // Stop timer0A0
 // P2OUT  &= ~BIT3;                    // pin to test comparator time

    ampl[1]  = ampl[0];                 // move old values
    width[1] = width[0];

 // ampl[0] = pulse_a() * (-0.00244140625);     // acquire amplitude
    __delay_cycles(800);  // wait for signal to estabilize
    ampl[0]  = -pulse_a() ;        // acquire amplitude and set signal
    width[0] = pulse_w();               // acquire width in cycles

    P2IES  &= ~BIT0;                    // set interrupt on rising edge on both comparators
    P1IES  &= ~BIT4;


    P2IFG  &= 0x00;                     // clear interrupt flags
    CACTL1 &= ~CAIFG;

    P2IE   |= BIT0;                     // P2.0 interrupt enable
    CACTL1 |= CAIE;                     // internal comparator interrupt enable

    TA1R   = 0;                         // reset TA1 counter register
    TA1CTL |= MC_2;                     // start timerA1, to write the result or wait a interruption.
  } else {
 //   send_data_serial("P2.0 else\n");// if interrupt on rising edge
    TA0CTL |= MC_2;                     // Start timer0A0 to measure width
 // P2OUT  |= BIT3;                     // pin to test comparator time

    TA1CTL &= ~MC_2;                    // stop timerA1, not send the result to display.


    P1IE  = 0;                          // disable interrupt on P1
    P1IES &= ~BIT4;                      // set interrupt P1.4 to rising edge
    P1IE  = BIT4;                       // enable interrupt on P1.4

    P1IFG = 0;                          // P2 interrupt flag clear

  }
}

//    Interrupt Routine, P1.4, external comparator (Positive Pulse)
//-------------------------------------------------------------------------
#pragma vector = PORT1_VECTOR
__interrupt void interrup_P1(void) {
  P1IE &= ~(BIT4);                      // disable interrupt of external comparator - P1.4
  //send_data_serial("P1.4");
  if (P1IES & BIT4) {                   // if interrupt from falling edge
    TA0CTL &= ~MC_2;                    // Stop timer0A0
 // P2OUT  &= ~BIT3;                    // pin to test comparator time

    ampl[1]  = ampl[0];                 // move old values
    width[1] = width[0];

 // ampl[0] = pulse_a() * (+0.00244140625);     // acquire amplitude
    __delay_cycles(800);  // wait for signal to estabilize
    ampl[0]  = pulse_a();         // acquire amplitude and set signal
    width[0] = pulse_w();               // acquire width in cycles

    P2IES  &= ~BIT0;                    // set interrupt on rising edge on both comparators
    P1IES  &= ~BIT4;


    P2IFG  &= 0x00;                     // clear interrupt flags
    P1IFG  &= 0x00;

    P2IE   |= BIT0;                     // P2.0 interrupt enable
    P1IE   |= BIT4;                     // P1.4 interrupt enable


    TA1R   = 0;                         // reset TA1 counter register
    TA1CTL |= MC_2;                     // start timerA1, to write the result or wait a interruption.
  } else {                              // if interrupt on rising edge
    TA0CTL |= MC_2;                     // Start timer0A0 to measure width
 // P2OUT  |= BIT3;                     // pin to test comparator time

    TA1CTL &= ~MC_2;                    // stop timerA1, not send the result to display.

    P2IE  = 0;                          // disable interrupt on P2
    P2IES |= BIT0;                      // set interrupt P2.0 to falling edge
    P2IE  = BIT0;                       // enable interrupt on P2

    P2IFG = 0;                          // P2 interrupt flag clear
  }
}

//-------------------------------------------------------------------------
//    Interrupt Routine, T0A1, result return when TA1 reaches limit
//-------------------------------------------------------------------------
#pragma vector=TIMER1_A0_VECTOR
__interrupt void interr_timer_A(void) { // timer A1, wirte result after pulses
  if(timer_count > 3){
    TA1CTL &= ~MC_2;                    // stop timerA1, to write the result.
    TA1R = 0;                       // reset TA1 counter register
    timer_count = 0;

    //LCD_clear();

    if(ampl[1]>0){
      ampl[1] += 406;
      ampl[0] -= 114;
    }else{
      ampl[1] -= 406;
      ampl[0] += 114;
    }

    for(int i = 0; i < n_pulses; i++) {
  //  sprintf(buffer,"ampl(%d) = %+0.5f V\n",i, ampl[i]* 0.00000953674316406);        // send pulse ampl to serial
      //sprintf(buffer,"ampl(%d) = %+0.3f T\n",i, ampl[i] * 0.000368 - 0.06326);        // send pulse ampl to serial (calibration equation)
      sprintf(buffer,"ampl(%d) = %.3f T\n",i, ampl[i] * 0.000368  );        // send pulse ampl to serial (calibration equation)
    send_data_serial(buffer);
      send_data_serial("ampl ");
      strcpy(buffer,"ampl (");
      dtostrf(i,1,1,buffer);
      dtostrf(ampl[i] * 0.000368,1,2, buffer2);

      sprintf(buffer,"width(%d) = %.1f us\n",i, width[i] * 0.0625);                  // send pulse width to serial  (widht / 16 -> 16Mhz / 16 = 1MHz -> width in us)
      send_data_serial(buffer);

      //sprintf(buffer,"%+0.3fV %0.1fus   ",(ampl[i]* 0.00000953674316406), width[i] * 0.0625);
      //sprintf(buffer,"%+0.2fT   %0.1fus   ",(ampl[i]* 0.000368 - 0.06326), width[i] * 0.0625);        // scale adjust and conversion

//      char *conversor2 = dtostrf(v_bat,1,2,buffer);    //converts float to string, we don't use sprintf now because Energia has a problem compiling this function with float values.
//      strcat(buffer3,"     ");            // makes the data pretty
//      strcat(buffer3,buffer);
//      strcat(buffer3," V");
//      LCD_out(2,buffer3);

//    sprintf(buffer,"%+0.2fT   %0.1fus   ",(ampl[i]* 0.000368 ), width[i] * 0.0625);        // scale adjust and conversion
//      LCD_out(n_pulses - i,buffer);

    ampl[i] = ampl[i] * 0.000368;
    width[i] = width[i] * 0.0625;
    char *conversor3 = dtostrf(ampl[i],1,2,buffer);
    char *conversor4 = dtostrf(width[i],1,2,buffer2);
    strcat(buffer4,buffer);
    strcat(buffer4,"T");
    strcat(buffer4,"   ");
    strcat(buffer4,buffer2);
    strcat(buffer4,"us");

    LCD_out(n_pulses - i,buffer4);

    }
    n_pulses = 0;                        // reset pulse counter

    sprintf(buffer,"\n");               // send new line to serial
    send_data_serial(buffer);
  } else {
    timer_count++;                      // increase timer multiplier
  }
}


//-------------------------------------------------------------------------
//   LCD routines
//-------------------------------------------------------------------------
void LCD_init()   // 3V
{
    delay_ms(1);
    while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent
    UCB0CTL1 |= UCTR + UCTXSTT;                 // I2C TX, start condition
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x38;                          //write control byte RS=0 & Co=1
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x39;                          //write control data
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x14;                           //Bias and Internal OSC frequency (BS=1 and fosc=347 kHz)
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x70;                           //Contrast(LB)=0b0000
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x54;                          //Power=ON, Boost=1, Contrast(HB)=0b00
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x6F;                           //Follower control Fon=ON, Rab=0b010
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x0C;                           //Display ON/OFF: Display=ON, Cursor=OFF, CurPos=OFF
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x01;                           //Clear Display
    while(!(IFG2 & UCB0TXIFG));
    delay_ms(1);

    UCB0TXBUF = 0x02;                           //Return Home
    while(!(IFG2 & UCB0TXIFG));
    delay_ms(1);

    UCB0CTL1 |= UCTXSTP;                        // stop transmitting
    while(!(IFG2 & UCB0TXIFG));

    return;
}



void LCD_out(short linha,char *imprimir)
{
 /* delay_ms(1);
  UCB0CTL1 |= UCTR + UCTXSTT;       //gera o start bit e envia o endereço
  while(!(IFG2 & UCB0TXIFG));     //aguarda final da transmissão

  UCB0TXBUF = 0x00;                 //write control byte rs=1 Co=0
  while(!(IFG2 & UCB0TXIFG));              //aguarda final da transmissão

  if (linha == 1)
      UCB0TXBUF = 0x80;                 //comando para 1a coluna,primeira linha  (caso fosse 2 linha usar 0xC0)
  else if (linha == 2)
      UCB0TXBUF = 0xC0;
  else
      UCB0TXBUF = 0x80;
  while(!(IFG2 & UCB0TXIFG));              //aguarda final da transmissão

  UCB0CTL1 |= UCTXSTP;              //gera stop bit
  while(!(IFG2 & UCB0TXIFG));              //aguarda final da transmissão

  UCB0CTL1 |= UCTR + UCTXSTT;       // gera o start bit e envia o endereço
  while(!(IFG2 & UCB0TXIFG));     //aguarda final da transmissão
  delay_ms(1);

  UCB0TXBUF = 0x40;                 //write control byte rs=1 Co=0
  while(!(IFG2 & UCB0TXIFG));              //aguarda final da transmissão
  delay_ms(1);

  while (*imprimir)
  {

    UCB0TXBUF = *imprimir;
    while(!(IFG2 & UCB0TXIFG));              //aguarda final da transmissão
    delay_ms(7);
    imprimir++;
  }

  UCB0CTL1 |= UCTXSTP;                     //gera stop bit
  while(!(IFG2 & UCB0TXIFG));              //aguarda final da transmissão

*/
  send_data_serial(imprimir);
  return;
}

void LCD_clear()
{
    delay_ms(1);
    while (UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent
    UCB0CTL1 |= UCTR + UCTXSTT;                 // I2C TX, start condition
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x38;                          //write control byte RS=0 & Co=1
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x39;                          //write control data
    while(!(IFG2 & UCB0TXIFG));

    UCB0TXBUF = 0x01;                           //Clear Display
    while(!(IFG2 & UCB0TXIFG));
    delay_ms(1);

    UCB0TXBUF = 0x02;                           //Return Home
    while(!(IFG2 & UCB0TXIFG));
    delay_ms(1);

    UCB0CTL1 |= UCTXSTP;                        // stop transmitting
    while(!(IFG2 & UCB0TXIFG));

    return;
}
