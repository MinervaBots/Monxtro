#line 1 "C:/Users/chris/Google Drive/Documentos/UFRJ/Minerva Bots/Projetos/1.2_2018.2/Xtronda/Código/Código 2018.2 - 31-10/Whey.c"
#line 29 "C:/Users/chris/Google Drive/Documentos/UFRJ/Minerva Bots/Projetos/1.2_2018.2/Xtronda/Código/Código 2018.2 - 31-10/Whey.c"
unsigned long timeUpSignal1;
unsigned long timeDownSignal1;
unsigned long timeUpSignal2;
unsigned long timeDownSignal2;
unsigned long lastMeasure1;
unsigned long lastMeasure2;
unsigned int numberInterruptionsTimer1 = 0;
unsigned short LowerSignificative8Bits;
unsigned short MoreSignificative8Bits;
unsigned int minDurationCH1 = 1100;
unsigned int maxDurationCH1 = 1900;
unsigned int minDurationCH2 = 1100;
unsigned int maxDurationCH2 = 1900;
unsigned int readDivisionVoltage;
float receivedVoltage;

void SetupPwms(){
 T2CON = 0;
 PR2 = 255;


 CCPTMRS.B1 = 0;
 CCPTMRS.B0 = 0;


 PSTR1CON.B0 = 1;
 PSTR1CON.B1 = 1;
 PSTR1CON.B2 = 0;
 PSTR1CON.B3 = 0;
 PSTR1CON.B4 = 1;

 CCPR1L = 0b11111111;
#line 75 "C:/Users/chris/Google Drive/Documentos/UFRJ/Minerva Bots/Projetos/1.2_2018.2/Xtronda/Código/Código 2018.2 - 31-10/Whey.c"
 CCPTMRS.B3 = 0;
 CCPTMRS.B2 = 0;


 PSTR2CON.B0 = 1;
 PSTR2CON.B1 = 1;
 PSTR2CON.B2 = 0;
 PSTR2CON.B3 = 0;
 PSTR2CON.B4 = 1;


 CCPR2L = 0b11111111;

 CCP2CON = 0b00111100;
 T2CON = 0b00000100;




}

void SetDutyCycle(unsigned int channel, unsigned int duty){
 if(channel == 1)
 CCPR1L = duty;
 if(channel == 2)
 CCPR2L = duty;
}

void PwmSteering(unsigned int channel, unsigned int port){

 if(channel == 1){
 PSTR1CON.B0 = 0;
 PSTR1CON.B1 = 0;
 if(port == 1){
  RA4_bit  = 0;
 PSTR1CON.B0 = 1;
 }
 else if(port == 2){
  RA5_bit  = 0;
 PSTR1CON.B1 = 1;
 }
 }
 else if(channel == 2){
 PSTR2CON.B0 = 0;
 PSTR2CON.B1 = 0;
 if(port == 1){
  RC4_bit  = 0;
 PSTR2CON.B0 = 1;
 }
 else if(port == 2){
  RC5_bit  = 0;
 PSTR2CON.B1 = 1;
 }
 }

}

void SetupTimer1(){

 T1CKPS1_bit = 0x00;
 T1CKPS0_bit = 0x01;
 TMR1CS1_bit = 0x00;
 TMR1CS0_bit = 0x00;
 TMR1ON_bit = 0x01;
 TMR1IE_bit = 0x01;
 TMR1L = 0x00;
 TMR1H = 0x00;



}

unsigned long long Micros(){
 return (TMR1H <<8 | TMR1L)*  1 
 + numberInterruptionsTimer1* 65536 ;
}

void SetupUART() {

 RXDTSEL_bit = 1;
 TXCKSEL_bit = 1;
 UART1_Init(9600);
 Delay_ms(100);
}

void SetupPort(){

 CM1CON0 = 0;
 CM2CON0 = 0;


 P2BSEL_bit = 1;
 CCP2SEL_bit = 1;

 ANSELA = 0;
 ANSELB = 0x10;
 ANSELC = 0;
 ADC_Init();



 TRISA0_bit = 1;
 TRISA1_bit = 1;
 TRISA2_bit = 0;
 TRISA3_bit = 1;
 TRISA4_bit = 0;
 TRISA5_bit = 0;


 TRISB4_bit = 1;
 TRISB5_bit = 0;
 TRISB6_bit = 1;
 TRISB7_bit = 0;


 TRISC0_bit = 1;
 TRISC1_bit = 1;
 TRISC2_bit = 0;
 TRISC3_bit = 1;
 TRISC4_bit = 0;
 TRISC5_bit = 0;
 TRISC6_bit = 1;
 TRISC7_bit = 0;


 GIE_bit = 0X01;
 PEIE_bit = 0X01;
 CCP2IE_bit = 0x01;
 CCP4IE_bit = 0x01;
 CCP2CON = 0x05;
 CCP4CON = 0x05;
}

unsigned BatteryCheck() {
 readDivisionVoltage = ADC_Get_Sample( 10 );
 receivedVoltage = (readDivisionVoltage*5.0)/1023.0;
 return receivedVoltage <  2.85 ;
}

unsigned FailSafeCheck(){

 return ((Micros() - lastMeasure1) >  2000000  || (Micros() - lastMeasure2) >  2000000 );
}
#line 242 "C:/Users/chris/Google Drive/Documentos/UFRJ/Minerva Bots/Projetos/1.2_2018.2/Xtronda/Código/Código 2018.2 - 31-10/Whey.c"
long Map(long x, long in_min, long in_max, long out_min, long out_max){
 return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

void RotateMotor(){
 int duty_cycle1;
 int duty_cycle2;
 unsigned long pulseWidth1;
 unsigned long pulseWidth2;
 pulseWidth1 = timeDownSignal1;
 pulseWidth2 = timeDownSignal2;


 duty_cycle1 = Map(pulseWidth1,minDurationCH1,maxDurationCH1, -255 , 255 );
 duty_cycle2 = Map(pulseWidth2,minDurationCH2,maxDurationCH2, -255 , 255 );


 if(duty_cycle1 <  -255 )
 duty_cycle1 =  -255 ;
 if(duty_cycle1 >  255 )
 duty_cycle1 =  255 ;

 if(duty_cycle2 <  -255 )
 duty_cycle2 =  -255 ;
 if(duty_cycle2 >  255 )
 duty_cycle2 =  255 ;


 if((duty_cycle1 < ( ( 255  + -255 ) / 2  +  50 )) && (duty_cycle1 > ( ( 255  + -255 ) / 2  -  50 )))
 duty_cycle1 =  ( 255  + -255 ) / 2 ;

 if((duty_cycle2 < ( ( 255  + -255 ) / 2  +  50 )) && (duty_cycle2 > ( ( 255  + -255 ) / 2  -  50 )))
 duty_cycle2 =  ( 255  + -255 ) / 2 ;

 if(duty_cycle1 >= 0){
 PwmSteering(1,2);
 SetDutyCycle(1,duty_cycle1);
 }
 else{
 PwmSteering(1,1);
 SetDutyCycle(1,-duty_cycle1);
 }

 if(duty_cycle2 >= 0){
 PwmSteering(2,2);
 SetDutyCycle(2,duty_cycle2);
 }
 else{
 PwmSteering(2,1);
 SetDutyCycle(2,-duty_cycle2);
 }
}






void interrupt(){
 if(TMR1IF_bit)
 {
 TMR1IF_bit = 0;
 numberInterruptionsTimer1++;
 }

 if(CCP2IF_bit && CCP2M0_bit)
 {
 CCP2IE_bit = 0x00;
 CCP4IE_bit = 0x00;
 TMR1ON_bit = 0x00;
 CCP2CON = 0x04;
 timeUpSignal1 = Micros();
 CCP2IF_bit = 0x00;
 TMR1ON_bit = 0x01;
 CCP2IE_bit = 0x01;
 CCP4IE_bit = 0x01;
 }
 else if(CCP2IF_bit)
 {
 CCP2IE_bit = 0x00;
 CCP4IE_bit = 0x00;
 TMR1ON_bit = 0x00;
 CCP2CON = 0x05;
 timeDownSignal1 = Micros() - timeUpSignal1;
 lastMeasure1 = Micros();
 TMR1ON_bit = 0x01;
 CCP2IF_bit = 0x00;
 CCP2IE_bit = 0x01;
 CCP4IE_bit = 0x01;
 }

 if(CCP4IF_bit && CCP4M0_bit)
 {
 CCP2IE_bit = 0x00;
 CCP4IE_bit = 0x00;
 TMR1ON_bit = 0x00;
 CCP4CON = 0x04;
 timeUpSignal2 = Micros();
 CCP4IF_bit = 0x00;
 TMR1ON_bit = 0x01;
 CCP2IE_bit = 0x01;
 CCP4IE_bit = 0x01;
 }
 else if(CCP4IF_bit)
 {
 CCP2IE_bit = 0x00;
 CCP4IE_bit = 0x00;
 TMR1ON_bit = 0x00;
 CCP4CON = 0x05;
 timeDownSignal2 = Micros() - timeUpSignal2;
 lastMeasure2 = Micros();
 TMR1ON_bit = 0x01;
 CCP4IF_bit = 0x00;
 CCP2IE_bit = 0x01;
 CCP4IE_bit = 0x01;
 }
}

void ErrorLedBlink(unsigned time_ms){
 unsigned i;
 time_ms = time_ms/250;
 for(i=0; i< time_ms; i++){
  RC7_bit  = 1;
 delay_ms(200);
  RC7_bit  = 0;
 delay_ms(200);
 }
}

void CalibrationLedBlink(unsigned time_ms){
 unsigned i;
 time_ms = time_ms/250;
 for(i=0; i< time_ms; i++){
  RB7_bit  = 1;
 delay_ms(200);
  RB7_bit  = 0;
 delay_ms(200);
 }
}

void ReadSignalsDataEEPROM(){




 LowerSignificative8Bits = EEPROM_Read(0X00);
 MoreSignificative8Bits = EEPROM_Read(0X01);
 minDurationCH1 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;



 LowerSignificative8Bits = EEPROM_Read(0X02);
 MoreSignificative8Bits = EEPROM_Read(0X03);
 minDurationCH2 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;






 LowerSignificative8Bits = EEPROM_Read(0X04);
 MoreSignificative8Bits = EEPROM_Read(0X05);
 maxDurationCH1 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;



 LowerSignificative8Bits = EEPROM_Read(0X06);
 MoreSignificative8Bits = EEPROM_Read(0X07);
 maxDurationCH2 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;




}

void Calibration(){
 unsigned int signal1_H_value;
 unsigned int signal2_H_value;
 unsigned int signal1_L_value;
 unsigned int signal2_L_value;
 unsigned int signal_T_value;
 unsigned long time_control;

 signal1_L_value = 20000;
 signal2_L_value = 20000;
 signal1_H_value = 0;
 signal2_H_value = 0;
 time_control = Micros();
  RB7_bit  = 1;

 while((Micros() - time_control) < 2000000){
 signal_T_value = (unsigned) timeDownSignal1;
 if(signal_T_value < signal1_L_value)
 signal1_L_value = signal_T_value;

 signal_T_value = (unsigned) timeDownSignal2;
 if(signal_T_value < signal2_L_value)
 signal2_L_value = signal_T_value;
 }



 LowerSignificative8Bits = signal1_L_value & 0xff;
 MoreSignificative8Bits = (signal1_L_value >> 8) & 0xff;
 EEPROM_Write(0X00,LowerSignificative8Bits);
 delay_ms(10);
 EEPROM_Write(0X01,MoreSignificative8Bits);
 delay_ms(10);


 LowerSignificative8Bits = signal2_L_value & 0xff;
 MoreSignificative8Bits = (signal2_L_value >> 8) & 0xff;
 EEPROM_Write(0X02,LowerSignificative8Bits);
 delay_ms(10);
 EEPROM_Write(0X03,MoreSignificative8Bits);
 delay_ms(10);

 CalibrationLedBlink(1600);
 time_control = Micros();
  RB7_bit  = 1;

 while((Micros() - time_control) < 2000000){
 signal_T_value = (unsigned) timeDownSignal1;

 if(signal_T_value > signal1_H_value)
 signal1_H_value = signal_T_value;

 signal_T_value = (unsigned) timeDownSignal2;

 if(signal_T_value > signal2_H_value)
 signal2_H_value = signal_T_value;
 }


 LowerSignificative8Bits = signal1_H_value & 0xff;
 MoreSignificative8Bits = (signal1_H_value >> 8) & 0xff;
 EEPROM_Write(0X04,LowerSignificative8Bits);
 delay_ms(10);
 EEPROM_Write(0X05,MoreSignificative8Bits);
 delay_ms(10);

 LowerSignificative8Bits = signal2_H_value & 0xff;
 MoreSignificative8Bits = (signal2_H_value >> 8) & 0xff;
 EEPROM_Write(0X06,LowerSignificative8Bits);
 delay_ms(10);
 EEPROM_Write(0X07,MoreSignificative8Bits);
 delay_ms(10);

 CalibrationLedBlink(1600);
  RB7_bit  = 0;

 ReadSignalsDataEEPROM();
}

void PrintSignalReceived(){
 char buffer[11];

 UART1_write_text("Sinal 1: ");
 LongWordToStr(timeDownSignal1, buffer);
 UART1_write_text(buffer);
 UART1_write_text("\t");

 UART1_write_text("Sinal 2: ");
 LongWordToStr(timeDownSignal2, buffer);
 UART1_write_text(buffer);
 UART1_write_text("\n");

 delay_ms(100);
}





void main() {

 OSCCON = 0b11110010;
 ADC_Init();
 SetupPort();
 SetupPwms();
 SetupTimer1();
 delay_ms(300);

 if( RA3_bit ==0)
 Calibration();

 while(1){
  RC7_bit  = 0;
 while(FailSafeCheck()) {
  RC7_bit  = 1;
 SetDutyCycle(1, 0);
 SetDutyCycle(2, 0);
 }


 if(BatteryCheck())
 ErrorLedBlink(1600);

 RotateMotor();

 }
}
