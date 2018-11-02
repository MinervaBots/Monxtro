
_SetupPwms:

;Whey.c,45 :: 		void SetupPwms(){
;Whey.c,46 :: 		T2CON = 0;   //desliga o Timer2, timer responsavel pelos PWMS
	CLRF       T2CON+0
;Whey.c,47 :: 		PR2 = 255;
	MOVLW      255
	MOVWF      PR2+0
;Whey.c,50 :: 		CCPTMRS.B1 = 0;    //00 = CCP1 is based off Timer2 in PWM mode
	BCF        CCPTMRS+0, 1
;Whey.c,51 :: 		CCPTMRS.B0 = 0;
	BCF        CCPTMRS+0, 0
;Whey.c,54 :: 		PSTR1CON.B0 = 1;   //1 = P1A pin has the PWM waveform with polarity control from CCP1M<1:0>
	BSF        PSTR1CON+0, 0
;Whey.c,55 :: 		PSTR1CON.B1 = 1;   //1 = P1B pin has the PWM waveform with polarity control from CCP1M<1:0>
	BSF        PSTR1CON+0, 1
;Whey.c,56 :: 		PSTR1CON.B2 = 0;   //0 = P1C pin is assigned to port pin
	BCF        PSTR1CON+0, 2
;Whey.c,57 :: 		PSTR1CON.B3 = 0;   //0 = P1D pin is assigned to port pin
	BCF        PSTR1CON+0, 3
;Whey.c,58 :: 		PSTR1CON.B4 = 1;   //Steering Sync bit, 0 = Output steering update occurs at the beginning of the instruction cycle boundary
	BSF        PSTR1CON+0, 4
;Whey.c,60 :: 		CCPR1L  = 0b11111111; //colocando nivel logico alto nas duas saidas para travar os motores
	MOVLW      255
	MOVWF      CCPR1L+0
;Whey.c,75 :: 		CCPTMRS.B3 = 0;    //00 = CCP2 is based off Timer2 in PWM mode
	BCF        CCPTMRS+0, 3
;Whey.c,76 :: 		CCPTMRS.B2 = 0;
	BCF        CCPTMRS+0, 2
;Whey.c,79 :: 		PSTR2CON.B0 = 1;   //1 = P2A pin has the PWM waveform with polarity control from CCP2M<1:0>
	BSF        PSTR2CON+0, 0
;Whey.c,80 :: 		PSTR2CON.B1 = 1;   //1 = P2B pin has the PWM waveform with polarity control from CCP2M<1:0>
	BSF        PSTR2CON+0, 1
;Whey.c,81 :: 		PSTR2CON.B2 = 0;   //0 = P2C pin is assigned to port pin  (pra que est� sendo usado?)
	BCF        PSTR2CON+0, 2
;Whey.c,82 :: 		PSTR2CON.B3 = 0;   //0 = P2D pin is assigned to port pin  (pra que est� sendo usado?)
	BCF        PSTR2CON+0, 3
;Whey.c,83 :: 		PSTR2CON.B4 = 1;   //Steering Sync bit, 0 = Output steering update occurs at the beginning of the instruction cycle boundary,
	BSF        PSTR2CON+0, 4
;Whey.c,86 :: 		CCPR2L  = 0b11111111;  //colocando nivel logico alto nas duas saidas para travar os motores
	MOVLW      255
	MOVWF      CCPR2L+0
;Whey.c,88 :: 		CCP2CON = 0b00111100; //Mesma configuracao do ECCP1
	MOVLW      60
	MOVWF      CCP2CON+0
;Whey.c,89 :: 		T2CON = 0b00000100;  //pre scaler =  1
	MOVLW      4
	MOVWF      T2CON+0
;Whey.c,94 :: 		}
L_end_SetupPwms:
	RETURN
; end of _SetupPwms

_SetDutyCycle:

;Whey.c,96 :: 		void SetDutyCycle(unsigned int channel, unsigned int duty){ //funcao responsavel por setar o dutycicle nos PWMS, variando de 0 a 255
;Whey.c,97 :: 		if(channel == 1)
	MOVLW      0
	XORWF      FARG_SetDutyCycle_channel+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__SetDutyCycle79
	MOVLW      1
	XORWF      FARG_SetDutyCycle_channel+0, 0
L__SetDutyCycle79:
	BTFSS      STATUS+0, 2
	GOTO       L_SetDutyCycle0
;Whey.c,98 :: 		CCPR1L = duty;
	MOVF       FARG_SetDutyCycle_duty+0, 0
	MOVWF      CCPR1L+0
L_SetDutyCycle0:
;Whey.c,99 :: 		if(channel == 2)
	MOVLW      0
	XORWF      FARG_SetDutyCycle_channel+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__SetDutyCycle80
	MOVLW      2
	XORWF      FARG_SetDutyCycle_channel+0, 0
L__SetDutyCycle80:
	BTFSS      STATUS+0, 2
	GOTO       L_SetDutyCycle1
;Whey.c,100 :: 		CCPR2L = duty;
	MOVF       FARG_SetDutyCycle_duty+0, 0
	MOVWF      CCPR2L+0
L_SetDutyCycle1:
;Whey.c,101 :: 		}
L_end_SetDutyCycle:
	RETURN
; end of _SetDutyCycle

_PwmSteering:

;Whey.c,103 :: 		void PwmSteering(unsigned int channel, unsigned int port){
;Whey.c,105 :: 		if(channel == 1){
	MOVLW      0
	XORWF      FARG_PwmSteering_channel+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PwmSteering82
	MOVLW      1
	XORWF      FARG_PwmSteering_channel+0, 0
L__PwmSteering82:
	BTFSS      STATUS+0, 2
	GOTO       L_PwmSteering2
;Whey.c,106 :: 		PSTR1CON.B0 = 0;   //1 = P1A pin is assigned to port pin
	BCF        PSTR1CON+0, 0
;Whey.c,107 :: 		PSTR1CON.B1 = 0;   //1 = P1B pin is assigned to port pin
	BCF        PSTR1CON+0, 1
;Whey.c,108 :: 		if(port == 1){
	MOVLW      0
	XORWF      FARG_PwmSteering_port+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PwmSteering83
	MOVLW      1
	XORWF      FARG_PwmSteering_port+0, 0
L__PwmSteering83:
	BTFSS      STATUS+0, 2
	GOTO       L_PwmSteering3
;Whey.c,109 :: 		P1B = 0;         //port pin stays at low
	BCF        RA4_bit+0, BitPos(RA4_bit+0)
;Whey.c,110 :: 		PSTR1CON.B0 = 1; //1 = P1A pin has the PWM waveform
	BSF        PSTR1CON+0, 0
;Whey.c,111 :: 		}
	GOTO       L_PwmSteering4
L_PwmSteering3:
;Whey.c,112 :: 		else if(port == 2){
	MOVLW      0
	XORWF      FARG_PwmSteering_port+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PwmSteering84
	MOVLW      2
	XORWF      FARG_PwmSteering_port+0, 0
L__PwmSteering84:
	BTFSS      STATUS+0, 2
	GOTO       L_PwmSteering5
;Whey.c,113 :: 		P1A = 0;         //port pin stays at low
	BCF        RA5_bit+0, BitPos(RA5_bit+0)
;Whey.c,114 :: 		PSTR1CON.B1 = 1; //1 = P1B pin has the PWM waveform
	BSF        PSTR1CON+0, 1
;Whey.c,115 :: 		}
L_PwmSteering5:
L_PwmSteering4:
;Whey.c,116 :: 		}//channel1 if
	GOTO       L_PwmSteering6
L_PwmSteering2:
;Whey.c,117 :: 		else if(channel == 2){
	MOVLW      0
	XORWF      FARG_PwmSteering_channel+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PwmSteering85
	MOVLW      2
	XORWF      FARG_PwmSteering_channel+0, 0
L__PwmSteering85:
	BTFSS      STATUS+0, 2
	GOTO       L_PwmSteering7
;Whey.c,118 :: 		PSTR2CON.B0 = 0;   //1 = P2A pin is assigned to port pin
	BCF        PSTR2CON+0, 0
;Whey.c,119 :: 		PSTR2CON.B1 = 0;   //1 = P2B pin is assigned to port pin
	BCF        PSTR2CON+0, 1
;Whey.c,120 :: 		if(port == 1){
	MOVLW      0
	XORWF      FARG_PwmSteering_port+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PwmSteering86
	MOVLW      1
	XORWF      FARG_PwmSteering_port+0, 0
L__PwmSteering86:
	BTFSS      STATUS+0, 2
	GOTO       L_PwmSteering8
;Whey.c,121 :: 		P2B = 0;         //port pin stays at low
	BCF        RC4_bit+0, BitPos(RC4_bit+0)
;Whey.c,122 :: 		PSTR2CON.B0 = 1; //1 = P2A pin has the PWM waveform
	BSF        PSTR2CON+0, 0
;Whey.c,123 :: 		}
	GOTO       L_PwmSteering9
L_PwmSteering8:
;Whey.c,124 :: 		else if(port == 2){
	MOVLW      0
	XORWF      FARG_PwmSteering_port+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PwmSteering87
	MOVLW      2
	XORWF      FARG_PwmSteering_port+0, 0
L__PwmSteering87:
	BTFSS      STATUS+0, 2
	GOTO       L_PwmSteering10
;Whey.c,125 :: 		P2A = 0;         //port pin stays at low
	BCF        RC5_bit+0, BitPos(RC5_bit+0)
;Whey.c,126 :: 		PSTR2CON.B1 = 1; //1 = P2B pin has the PWM waveform
	BSF        PSTR2CON+0, 1
;Whey.c,127 :: 		}
L_PwmSteering10:
L_PwmSteering9:
;Whey.c,128 :: 		}//channel2 if
L_PwmSteering7:
L_PwmSteering6:
;Whey.c,130 :: 		}
L_end_PwmSteering:
	RETURN
; end of _PwmSteering

_SetupTimer1:

;Whey.c,132 :: 		void SetupTimer1(){
;Whey.c,134 :: 		T1CKPS1_bit = 0x00;                        //Prescaller TMR1 1:2, cada bit do timer1 e correspondente a 1 us
	BCF        T1CKPS1_bit+0, BitPos(T1CKPS1_bit+0)
;Whey.c,135 :: 		T1CKPS0_bit = 0x01;                        //
	BSF        T1CKPS0_bit+0, BitPos(T1CKPS0_bit+0)
;Whey.c,136 :: 		TMR1CS1_bit = 0x00;                        //Clock: Fosc/4 = instruction clock
	BCF        TMR1CS1_bit+0, BitPos(TMR1CS1_bit+0)
;Whey.c,137 :: 		TMR1CS0_bit = 0x00;                        //Clock: Fosc/4 = instruction clock
	BCF        TMR1CS0_bit+0, BitPos(TMR1CS0_bit+0)
;Whey.c,138 :: 		TMR1ON_bit  = 0x01;                        //Inicia a contagem do Timer1
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,139 :: 		TMR1IE_bit  = 0x01;                        //Habilita interrupcoes de TMR1
	BSF        TMR1IE_bit+0, BitPos(TMR1IE_bit+0)
;Whey.c,140 :: 		TMR1L       = 0x00;                        //zera o Timer1
	CLRF       TMR1L+0
;Whey.c,141 :: 		TMR1H       = 0x00;
	CLRF       TMR1H+0
;Whey.c,145 :: 		}
L_end_SetupTimer1:
	RETURN
; end of _SetupTimer1

_Micros:

;Whey.c,147 :: 		unsigned long long Micros(){
;Whey.c,148 :: 		return  (TMR1H <<8 | TMR1L)* TIMER1_CONST     //cada bit do timer 1 vale 1us
	MOVF       TMR1H+0, 0
	MOVWF      R1
	CLRF       R0
	MOVF       TMR1L+0, 0
	IORWF       R0, 0
	MOVWF      R5
	MOVF       R1, 0
	MOVWF      R6
	MOVLW      0
	IORWF       R6, 1
;Whey.c,149 :: 		+ numberInterruptionsTimer1*OVERFLOW_CONST; //numero de interrupcoes vezes o valor maximo do Timer 1 (2^16)
	MOVF       _numberInterruptionsTimer1+1, 0
	MOVWF      R3
	MOVF       _numberInterruptionsTimer1+0, 0
	MOVWF      R2
	CLRF       R0
	CLRF       R1
	MOVF       R5, 0
	ADDWF      R0, 1
	MOVF       R6, 0
	ADDWFC     R1, 1
	MOVLW      0
	ADDWFC     R2, 1
	ADDWFC     R3, 1
;Whey.c,150 :: 		}
L_end_Micros:
	RETURN
; end of _Micros

_SetupUART:

;Whey.c,152 :: 		void SetupUART() {
;Whey.c,154 :: 		RXDTSEL_bit = 1;     //RXDTSEL: RX/DT function is on RA1
	BSF        RXDTSEL_bit+0, BitPos(RXDTSEL_bit+0)
;Whey.c,155 :: 		TXCKSEL_bit = 1;     //TXDTSEL: TX/CK function is on RA0
	BSF        TXCKSEL_bit+0, BitPos(TXCKSEL_bit+0)
;Whey.c,156 :: 		UART1_Init(9600);    //Initialize UART module at 9600 bps      153600 = 9600*16
	BSF        BAUDCON+0, 3
	MOVLW      207
	MOVWF      SPBRG+0
	CLRF       SPBRG+1
	BSF        TXSTA+0, 2
	CALL       _UART1_Init+0
;Whey.c,157 :: 		Delay_ms(100);       //Wait for UART module to stabilize
	MOVLW      2
	MOVWF      R11
	MOVLW      4
	MOVWF      R12
	MOVLW      186
	MOVWF      R13
L_SetupUART11:
	DECFSZ     R13, 1
	GOTO       L_SetupUART11
	DECFSZ     R12, 1
	GOTO       L_SetupUART11
	DECFSZ     R11, 1
	GOTO       L_SetupUART11
	NOP
;Whey.c,158 :: 		}
L_end_SetupUART:
	RETURN
; end of _SetupUART

_SetupPort:

;Whey.c,160 :: 		void SetupPort(){
;Whey.c,162 :: 		CM1CON0       = 0;
	CLRF       CM1CON0+0
;Whey.c,163 :: 		CM2CON0       = 0;
	CLRF       CM2CON0+0
;Whey.c,166 :: 		P2BSEL_bit =  1;    //P2BSEL: 1 = P2B function is on RA4
	BSF        P2BSEL_bit+0, BitPos(P2BSEL_bit+0)
;Whey.c,167 :: 		CCP2SEL_bit =  1;   //CCP2SEL:1 = CCP2/P2A function is on RA5
	BSF        CCP2SEL_bit+0, BitPos(CCP2SEL_bit+0)
;Whey.c,169 :: 		ANSELA     = 0; //Nenhuma porta analogica
	CLRF       ANSELA+0
;Whey.c,170 :: 		ANSELB  = 0x10; //RB4 analogico AN4, ultimo bit do ANSELB. (?)
	MOVLW      16
	MOVWF      ANSELB+0
;Whey.c,171 :: 		ANSELC     = 0; //Nenhuma porta analogica
	CLRF       ANSELC+0
;Whey.c,172 :: 		ADC_Init();     // Initialize ADC module with default settings
	CALL       _ADC_Init+0
;Whey.c,176 :: 		TRISA0_bit = 1; //IS_1B  (Sensor de Corrente)
	BSF        TRISA0_bit+0, BitPos(TRISA0_bit+0)
;Whey.c,177 :: 		TRISA1_bit = 1; //IS_1A  (Sensor de Corrente)
	BSF        TRISA1_bit+0, BitPos(TRISA1_bit+0)
;Whey.c,178 :: 		TRISA2_bit = 0; //INH
	BCF        TRISA2_bit+0, BitPos(TRISA2_bit+0)
;Whey.c,179 :: 		TRISA3_bit = 1; //MCLR
	BSF        TRISA3_bit+0, BitPos(TRISA3_bit+0)
;Whey.c,180 :: 		TRISA4_bit = 0; //PWM_1B  (MotorA)
	BCF        TRISA4_bit+0, BitPos(TRISA4_bit+0)
;Whey.c,181 :: 		TRISA5_bit = 0; //PWM_1A  (MotorA)
	BCF        TRISA5_bit+0, BitPos(TRISA5_bit+0)
;Whey.c,184 :: 		TRISB4_bit = 1; //AN4 (LOW BATTERY)
	BSF        TRISB4_bit+0, BitPos(TRISB4_bit+0)
;Whey.c,185 :: 		TRISB5_bit = 0; //RX
	BCF        TRISB5_bit+0, BitPos(TRISB5_bit+0)
;Whey.c,186 :: 		TRISB6_bit = 1; //Unused grounded pin
	BSF        TRISB6_bit+0, BitPos(TRISB6_bit+0)
;Whey.c,187 :: 		TRISB7_bit = 0; //TX
	BCF        TRISB7_bit+0, BitPos(TRISB7_bit+0)
;Whey.c,190 :: 		TRISC0_bit = 1; //IS_2B  (Sensor de Corrente)
	BSF        TRISC0_bit+0, BitPos(TRISC0_bit+0)
;Whey.c,191 :: 		TRISC1_bit = 1; //IS_2A  (Sensor de Corrente)
	BSF        TRISC1_bit+0, BitPos(TRISC1_bit+0)
;Whey.c,192 :: 		TRISC2_bit = 0; //LED_CALIBRATION
	BCF        TRISC2_bit+0, BitPos(TRISC2_bit+0)
;Whey.c,193 :: 		TRISC3_bit = 1; //CHANNEL_B  (CCP2)
	BSF        TRISC3_bit+0, BitPos(TRISC3_bit+0)
;Whey.c,194 :: 		TRISC4_bit = 0; //PWM_2B  (MotorB)
	BCF        TRISC4_bit+0, BitPos(TRISC4_bit+0)
;Whey.c,195 :: 		TRISC5_bit = 0; //PWM_2A  (MotorB)
	BCF        TRISC5_bit+0, BitPos(TRISC5_bit+0)
;Whey.c,196 :: 		TRISC6_bit = 1; //CHANNEL_A  (CCP4)
	BSF        TRISC6_bit+0, BitPos(TRISC6_bit+0)
;Whey.c,197 :: 		TRISC7_bit = 0; //LED_ERROR
	BCF        TRISC7_bit+0, BitPos(TRISC7_bit+0)
;Whey.c,200 :: 		GIE_bit    = 0X01;   //Habilita a interrupcao Global
	BSF        GIE_bit+0, BitPos(GIE_bit+0)
;Whey.c,201 :: 		PEIE_bit   = 0X01;   //Habilita a interrupcao por perifericos
	BSF        PEIE_bit+0, BitPos(PEIE_bit+0)
;Whey.c,202 :: 		CCP2IE_bit  = 0x01;  //Habilita interrupcoes do modulo CCP2(CHANNEL_B)
	BSF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,203 :: 		CCP4IE_bit  = 0x01;  //Habilita interrupcoes do modulo CCP4(CHANNEL_A)
	BSF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,204 :: 		CCP2CON     = 0x05;  //Configura captura por borda de subida
	MOVLW      5
	MOVWF      CCP2CON+0
;Whey.c,205 :: 		CCP4CON     = 0x05;  //Configura captura por borda de subida
	MOVLW      5
	MOVWF      CCP4CON+0
;Whey.c,206 :: 		}
L_end_SetupPort:
	RETURN
; end of _SetupPort

_BatteryCheck:

;Whey.c,208 :: 		unsigned BatteryCheck() {  // confere tens�o da bateria
;Whey.c,209 :: 		readDivisionVoltage = ADC_Get_Sample(LOW_BAT);
	MOVLW      10
	MOVWF      FARG_ADC_Get_Sample_channel+0
	CALL       _ADC_Get_Sample+0
	MOVF       R0, 0
	MOVWF      _readDivisionVoltage+0
	MOVF       R1, 0
	MOVWF      _readDivisionVoltage+1
;Whey.c,210 :: 		receivedVoltage = (readDivisionVoltage*5.0)/1023.0; /* 5V: maximo que o pino recebe em tensao |  1023: equivalente ao 5V no registrador */
	CALL       _word2double+0
	MOVLW      0
	MOVWF      R4
	MOVLW      0
	MOVWF      R5
	MOVLW      32
	MOVWF      R6
	MOVLW      129
	MOVWF      R7
	CALL       _Mul_32x32_FP+0
	MOVLW      0
	MOVWF      R4
	MOVLW      192
	MOVWF      R5
	MOVLW      127
	MOVWF      R6
	MOVLW      136
	MOVWF      R7
	CALL       _Div_32x32_FP+0
	MOVF       R0, 0
	MOVWF      _receivedVoltage+0
	MOVF       R1, 0
	MOVWF      _receivedVoltage+1
	MOVF       R2, 0
	MOVWF      _receivedVoltage+2
	MOVF       R3, 0
	MOVWF      _receivedVoltage+3
;Whey.c,211 :: 		return receivedVoltage < CUT_VOLTAGE;
	MOVLW      102
	MOVWF      R4
	MOVLW      102
	MOVWF      R5
	MOVLW      54
	MOVWF      R6
	MOVLW      128
	MOVWF      R7
	CALL       _Compare_Double+0
	MOVLW      1
	BTFSC      STATUS+0, 0
	MOVLW      0
	MOVWF      R0
	MOVLW      0
	MOVWF      R1
;Whey.c,212 :: 		}
L_end_BatteryCheck:
	RETURN
; end of _BatteryCheck

_FailSafeCheck:

;Whey.c,214 :: 		unsigned FailSafeCheck(){ //confere se ainda esta recebendo sinal
;Whey.c,216 :: 		return ((Micros() - lastMeasure1) > FAIL_SAFE_TIME || (Micros() - lastMeasure2) > FAIL_SAFE_TIME);
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      R4
	MOVF       R1, 0
	MOVWF      R5
	MOVF       R2, 0
	MOVWF      R6
	MOVF       R3, 0
	MOVWF      R7
	MOVF       _lastMeasure1+0, 0
	SUBWF      R4, 1
	MOVF       _lastMeasure1+1, 0
	SUBWFB     R5, 1
	MOVF       _lastMeasure1+2, 0
	SUBWFB     R6, 1
	MOVF       _lastMeasure1+3, 0
	SUBWFB     R7, 1
	MOVF       R7, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__FailSafeCheck94
	MOVF       R6, 0
	SUBLW      30
	BTFSS      STATUS+0, 2
	GOTO       L__FailSafeCheck94
	MOVF       R5, 0
	SUBLW      132
	BTFSS      STATUS+0, 2
	GOTO       L__FailSafeCheck94
	MOVF       R4, 0
	SUBLW      128
L__FailSafeCheck94:
	BTFSS      STATUS+0, 0
	GOTO       L_FailSafeCheck13
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      R4
	MOVF       R1, 0
	MOVWF      R5
	MOVF       R2, 0
	MOVWF      R6
	MOVF       R3, 0
	MOVWF      R7
	MOVF       _lastMeasure2+0, 0
	SUBWF      R4, 1
	MOVF       _lastMeasure2+1, 0
	SUBWFB     R5, 1
	MOVF       _lastMeasure2+2, 0
	SUBWFB     R6, 1
	MOVF       _lastMeasure2+3, 0
	SUBWFB     R7, 1
	MOVF       R7, 0
	SUBLW      0
	BTFSS      STATUS+0, 2
	GOTO       L__FailSafeCheck95
	MOVF       R6, 0
	SUBLW      30
	BTFSS      STATUS+0, 2
	GOTO       L__FailSafeCheck95
	MOVF       R5, 0
	SUBLW      132
	BTFSS      STATUS+0, 2
	GOTO       L__FailSafeCheck95
	MOVF       R4, 0
	SUBLW      128
L__FailSafeCheck95:
	BTFSS      STATUS+0, 0
	GOTO       L_FailSafeCheck13
	CLRF       R0
	GOTO       L_FailSafeCheck12
L_FailSafeCheck13:
	MOVLW      1
	MOVWF      R0
L_FailSafeCheck12:
	MOVLW      0
	MOVWF      R1
;Whey.c,217 :: 		}
L_end_FailSafeCheck:
	RETURN
; end of _FailSafeCheck

_Map:

;Whey.c,242 :: 		long Map(long x, long in_min, long in_max, long out_min, long out_max){
;Whey.c,243 :: 		return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
	MOVF       FARG_Map_x+0, 0
	MOVWF      R4
	MOVF       FARG_Map_x+1, 0
	MOVWF      R5
	MOVF       FARG_Map_x+2, 0
	MOVWF      R6
	MOVF       FARG_Map_x+3, 0
	MOVWF      R7
	MOVF       FARG_Map_in_min+0, 0
	SUBWF      R4, 1
	MOVF       FARG_Map_in_min+1, 0
	SUBWFB     R5, 1
	MOVF       FARG_Map_in_min+2, 0
	SUBWFB     R6, 1
	MOVF       FARG_Map_in_min+3, 0
	SUBWFB     R7, 1
	MOVF       FARG_Map_out_max+0, 0
	MOVWF      R0
	MOVF       FARG_Map_out_max+1, 0
	MOVWF      R1
	MOVF       FARG_Map_out_max+2, 0
	MOVWF      R2
	MOVF       FARG_Map_out_max+3, 0
	MOVWF      R3
	MOVF       FARG_Map_out_min+0, 0
	SUBWF      R0, 1
	MOVF       FARG_Map_out_min+1, 0
	SUBWFB     R1, 1
	MOVF       FARG_Map_out_min+2, 0
	SUBWFB     R2, 1
	MOVF       FARG_Map_out_min+3, 0
	SUBWFB     R3, 1
	CALL       _Mul_32x32_U+0
	MOVF       FARG_Map_in_max+0, 0
	MOVWF      R4
	MOVF       FARG_Map_in_max+1, 0
	MOVWF      R5
	MOVF       FARG_Map_in_max+2, 0
	MOVWF      R6
	MOVF       FARG_Map_in_max+3, 0
	MOVWF      R7
	MOVF       FARG_Map_in_min+0, 0
	SUBWF      R4, 1
	MOVF       FARG_Map_in_min+1, 0
	SUBWFB     R5, 1
	MOVF       FARG_Map_in_min+2, 0
	SUBWFB     R6, 1
	MOVF       FARG_Map_in_min+3, 0
	SUBWFB     R7, 1
	CALL       _Div_32x32_S+0
	MOVF       FARG_Map_out_min+0, 0
	ADDWF      R0, 1
	MOVF       FARG_Map_out_min+1, 0
	ADDWFC     R1, 1
	MOVF       FARG_Map_out_min+2, 0
	ADDWFC     R2, 1
	MOVF       FARG_Map_out_min+3, 0
	ADDWFC     R3, 1
;Whey.c,244 :: 		}
L_end_Map:
	RETURN
; end of _Map

_RotateMotor:

;Whey.c,246 :: 		void RotateMotor(){
;Whey.c,252 :: 		pulseWidth2 = timeDownSignal2;   //l� o pulso do canal 2
	MOVF       _timeDownSignal2+0, 0
	MOVWF      RotateMotor_pulseWidth2_L0+0
	MOVF       _timeDownSignal2+1, 0
	MOVWF      RotateMotor_pulseWidth2_L0+1
	MOVF       _timeDownSignal2+2, 0
	MOVWF      RotateMotor_pulseWidth2_L0+2
	MOVF       _timeDownSignal2+3, 0
	MOVWF      RotateMotor_pulseWidth2_L0+3
;Whey.c,255 :: 		duty_cycle1 = Map(pulseWidth1,minDurationCH1,maxDurationCH1,MIN_PWM,MAX_PWM);
	MOVF       _timeDownSignal1+0, 0
	MOVWF      FARG_Map_x+0
	MOVF       _timeDownSignal1+1, 0
	MOVWF      FARG_Map_x+1
	MOVF       _timeDownSignal1+2, 0
	MOVWF      FARG_Map_x+2
	MOVF       _timeDownSignal1+3, 0
	MOVWF      FARG_Map_x+3
	MOVF       _minDurationCH1+0, 0
	MOVWF      FARG_Map_in_min+0
	MOVF       _minDurationCH1+1, 0
	MOVWF      FARG_Map_in_min+1
	CLRF       FARG_Map_in_min+2
	CLRF       FARG_Map_in_min+3
	MOVF       _maxDurationCH1+0, 0
	MOVWF      FARG_Map_in_max+0
	MOVF       _maxDurationCH1+1, 0
	MOVWF      FARG_Map_in_max+1
	CLRF       FARG_Map_in_max+2
	CLRF       FARG_Map_in_max+3
	MOVLW      1
	MOVWF      FARG_Map_out_min+0
	MOVLW      255
	MOVWF      FARG_Map_out_min+1
	MOVLW      255
	MOVWF      FARG_Map_out_min+2
	MOVWF      FARG_Map_out_min+3
	MOVLW      255
	MOVWF      FARG_Map_out_max+0
	CLRF       FARG_Map_out_max+1
	CLRF       FARG_Map_out_max+2
	CLRF       FARG_Map_out_max+3
	CALL       _Map+0
	MOVF       R0, 0
	MOVWF      RotateMotor_duty_cycle1_L0+0
	MOVF       R1, 0
	MOVWF      RotateMotor_duty_cycle1_L0+1
;Whey.c,256 :: 		duty_cycle2 = Map(pulseWidth2,minDurationCH2,maxDurationCH2,MIN_PWM,MAX_PWM);
	MOVF       RotateMotor_pulseWidth2_L0+0, 0
	MOVWF      FARG_Map_x+0
	MOVF       RotateMotor_pulseWidth2_L0+1, 0
	MOVWF      FARG_Map_x+1
	MOVF       RotateMotor_pulseWidth2_L0+2, 0
	MOVWF      FARG_Map_x+2
	MOVF       RotateMotor_pulseWidth2_L0+3, 0
	MOVWF      FARG_Map_x+3
	MOVF       _minDurationCH2+0, 0
	MOVWF      FARG_Map_in_min+0
	MOVF       _minDurationCH2+1, 0
	MOVWF      FARG_Map_in_min+1
	CLRF       FARG_Map_in_min+2
	CLRF       FARG_Map_in_min+3
	MOVF       _maxDurationCH2+0, 0
	MOVWF      FARG_Map_in_max+0
	MOVF       _maxDurationCH2+1, 0
	MOVWF      FARG_Map_in_max+1
	CLRF       FARG_Map_in_max+2
	CLRF       FARG_Map_in_max+3
	MOVLW      1
	MOVWF      FARG_Map_out_min+0
	MOVLW      255
	MOVWF      FARG_Map_out_min+1
	MOVLW      255
	MOVWF      FARG_Map_out_min+2
	MOVWF      FARG_Map_out_min+3
	MOVLW      255
	MOVWF      FARG_Map_out_max+0
	CLRF       FARG_Map_out_max+1
	CLRF       FARG_Map_out_max+2
	CLRF       FARG_Map_out_max+3
	CALL       _Map+0
	MOVF       R0, 0
	MOVWF      RotateMotor_duty_cycle2_L0+0
	MOVF       R1, 0
	MOVWF      RotateMotor_duty_cycle2_L0+1
;Whey.c,259 :: 		if(duty_cycle1 < MIN_PWM)
	MOVLW      128
	XORWF      RotateMotor_duty_cycle1_L0+1, 0
	MOVWF      R0
	MOVLW      128
	XORLW      255
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor98
	MOVLW      1
	SUBWF      RotateMotor_duty_cycle1_L0+0, 0
L__RotateMotor98:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor14
;Whey.c,260 :: 		duty_cycle1 = MIN_PWM;
	MOVLW      1
	MOVWF      RotateMotor_duty_cycle1_L0+0
	MOVLW      255
	MOVWF      RotateMotor_duty_cycle1_L0+1
L_RotateMotor14:
;Whey.c,261 :: 		if(duty_cycle1 > MAX_PWM)
	MOVLW      128
	MOVWF      R0
	MOVLW      128
	XORWF      RotateMotor_duty_cycle1_L0+1, 0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor99
	MOVF       RotateMotor_duty_cycle1_L0+0, 0
	SUBLW      255
L__RotateMotor99:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor15
;Whey.c,262 :: 		duty_cycle1 = MAX_PWM;
	MOVLW      255
	MOVWF      RotateMotor_duty_cycle1_L0+0
	CLRF       RotateMotor_duty_cycle1_L0+1
L_RotateMotor15:
;Whey.c,264 :: 		if(duty_cycle2 < MIN_PWM)
	MOVLW      128
	XORWF      RotateMotor_duty_cycle2_L0+1, 0
	MOVWF      R0
	MOVLW      128
	XORLW      255
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor100
	MOVLW      1
	SUBWF      RotateMotor_duty_cycle2_L0+0, 0
L__RotateMotor100:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor16
;Whey.c,265 :: 		duty_cycle2 = MIN_PWM;
	MOVLW      1
	MOVWF      RotateMotor_duty_cycle2_L0+0
	MOVLW      255
	MOVWF      RotateMotor_duty_cycle2_L0+1
L_RotateMotor16:
;Whey.c,266 :: 		if(duty_cycle2 > MAX_PWM)
	MOVLW      128
	MOVWF      R0
	MOVLW      128
	XORWF      RotateMotor_duty_cycle2_L0+1, 0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor101
	MOVF       RotateMotor_duty_cycle2_L0+0, 0
	SUBLW      255
L__RotateMotor101:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor17
;Whey.c,267 :: 		duty_cycle2 = MAX_PWM;
	MOVLW      255
	MOVWF      RotateMotor_duty_cycle2_L0+0
	CLRF       RotateMotor_duty_cycle2_L0+1
L_RotateMotor17:
;Whey.c,270 :: 		if((duty_cycle1 < (CENTER_PWM + DEADZONE)) && (duty_cycle1 > (CENTER_PWM - DEADZONE)))
	MOVLW      128
	XORWF      RotateMotor_duty_cycle1_L0+1, 0
	MOVWF      R0
	MOVLW      128
	XORLW      0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor102
	MOVLW      50
	SUBWF      RotateMotor_duty_cycle1_L0+0, 0
L__RotateMotor102:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor20
	MOVLW      128
	XORLW      255
	MOVWF      R0
	MOVLW      128
	XORWF      RotateMotor_duty_cycle1_L0+1, 0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor103
	MOVF       RotateMotor_duty_cycle1_L0+0, 0
	SUBLW      206
L__RotateMotor103:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor20
L__RotateMotor74:
;Whey.c,271 :: 		duty_cycle1 = CENTER_PWM;
	CLRF       RotateMotor_duty_cycle1_L0+0
	CLRF       RotateMotor_duty_cycle1_L0+1
L_RotateMotor20:
;Whey.c,273 :: 		if((duty_cycle2 < (CENTER_PWM + DEADZONE)) && (duty_cycle2 > (CENTER_PWM - DEADZONE)))
	MOVLW      128
	XORWF      RotateMotor_duty_cycle2_L0+1, 0
	MOVWF      R0
	MOVLW      128
	XORLW      0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor104
	MOVLW      50
	SUBWF      RotateMotor_duty_cycle2_L0+0, 0
L__RotateMotor104:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor23
	MOVLW      128
	XORLW      255
	MOVWF      R0
	MOVLW      128
	XORWF      RotateMotor_duty_cycle2_L0+1, 0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor105
	MOVF       RotateMotor_duty_cycle2_L0+0, 0
	SUBLW      206
L__RotateMotor105:
	BTFSC      STATUS+0, 0
	GOTO       L_RotateMotor23
L__RotateMotor73:
;Whey.c,274 :: 		duty_cycle2 = CENTER_PWM;
	CLRF       RotateMotor_duty_cycle2_L0+0
	CLRF       RotateMotor_duty_cycle2_L0+1
L_RotateMotor23:
;Whey.c,276 :: 		if(duty_cycle1 >= 0){
	MOVLW      128
	XORWF      RotateMotor_duty_cycle1_L0+1, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor106
	MOVLW      0
	SUBWF      RotateMotor_duty_cycle1_L0+0, 0
L__RotateMotor106:
	BTFSS      STATUS+0, 0
	GOTO       L_RotateMotor24
;Whey.c,277 :: 		PwmSteering(1,2);                        //coloca no sentido anti horario de rotacao
	MOVLW      1
	MOVWF      FARG_PwmSteering_channel+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_channel+1
	MOVLW      2
	MOVWF      FARG_PwmSteering_port+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_port+1
	CALL       _PwmSteering+0
;Whey.c,278 :: 		SetDutyCycle(1,duty_cycle1);                     //aplica o duty cycle
	MOVLW      1
	MOVWF      FARG_SetDutyCycle_channel+0
	MOVLW      0
	MOVWF      FARG_SetDutyCycle_channel+1
	MOVF       RotateMotor_duty_cycle1_L0+0, 0
	MOVWF      FARG_SetDutyCycle_duty+0
	MOVF       RotateMotor_duty_cycle1_L0+1, 0
	MOVWF      FARG_SetDutyCycle_duty+1
	CALL       _SetDutyCycle+0
;Whey.c,279 :: 		}
	GOTO       L_RotateMotor25
L_RotateMotor24:
;Whey.c,281 :: 		PwmSteering(1,1);                       //coloca no sentido horario de rotacao
	MOVLW      1
	MOVWF      FARG_PwmSteering_channel+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_channel+1
	MOVLW      1
	MOVWF      FARG_PwmSteering_port+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_port+1
	CALL       _PwmSteering+0
;Whey.c,282 :: 		SetDutyCycle(1,-duty_cycle1);           //aplica o duty cycle
	MOVLW      1
	MOVWF      FARG_SetDutyCycle_channel+0
	MOVLW      0
	MOVWF      FARG_SetDutyCycle_channel+1
	MOVF       RotateMotor_duty_cycle1_L0+0, 0
	SUBLW      0
	MOVWF      FARG_SetDutyCycle_duty+0
	MOVF       RotateMotor_duty_cycle1_L0+1, 0
	BTFSS      STATUS+0, 0
	ADDLW      1
	CLRF       FARG_SetDutyCycle_duty+1
	SUBWF      FARG_SetDutyCycle_duty+1, 1
	CALL       _SetDutyCycle+0
;Whey.c,283 :: 		}
L_RotateMotor25:
;Whey.c,285 :: 		if(duty_cycle2 >= 0){
	MOVLW      128
	XORWF      RotateMotor_duty_cycle2_L0+1, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__RotateMotor107
	MOVLW      0
	SUBWF      RotateMotor_duty_cycle2_L0+0, 0
L__RotateMotor107:
	BTFSS      STATUS+0, 0
	GOTO       L_RotateMotor26
;Whey.c,286 :: 		PwmSteering(2,2);                        //coloca no sentido anti horario de rotacao
	MOVLW      2
	MOVWF      FARG_PwmSteering_channel+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_channel+1
	MOVLW      2
	MOVWF      FARG_PwmSteering_port+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_port+1
	CALL       _PwmSteering+0
;Whey.c,287 :: 		SetDutyCycle(2,duty_cycle2);                      //aplica o duty cycle
	MOVLW      2
	MOVWF      FARG_SetDutyCycle_channel+0
	MOVLW      0
	MOVWF      FARG_SetDutyCycle_channel+1
	MOVF       RotateMotor_duty_cycle2_L0+0, 0
	MOVWF      FARG_SetDutyCycle_duty+0
	MOVF       RotateMotor_duty_cycle2_L0+1, 0
	MOVWF      FARG_SetDutyCycle_duty+1
	CALL       _SetDutyCycle+0
;Whey.c,288 :: 		}
	GOTO       L_RotateMotor27
L_RotateMotor26:
;Whey.c,290 :: 		PwmSteering(2,1);                       //coloca no sentido horario de rotacao
	MOVLW      2
	MOVWF      FARG_PwmSteering_channel+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_channel+1
	MOVLW      1
	MOVWF      FARG_PwmSteering_port+0
	MOVLW      0
	MOVWF      FARG_PwmSteering_port+1
	CALL       _PwmSteering+0
;Whey.c,291 :: 		SetDutyCycle(2,-duty_cycle2);            //aplica o duty cycle
	MOVLW      2
	MOVWF      FARG_SetDutyCycle_channel+0
	MOVLW      0
	MOVWF      FARG_SetDutyCycle_channel+1
	MOVF       RotateMotor_duty_cycle2_L0+0, 0
	SUBLW      0
	MOVWF      FARG_SetDutyCycle_duty+0
	MOVF       RotateMotor_duty_cycle2_L0+1, 0
	BTFSS      STATUS+0, 0
	ADDLW      1
	CLRF       FARG_SetDutyCycle_duty+1
	SUBWF      FARG_SetDutyCycle_duty+1, 1
	CALL       _SetDutyCycle+0
;Whey.c,292 :: 		}
L_RotateMotor27:
;Whey.c,293 :: 		}
L_end_RotateMotor:
	RETURN
; end of _RotateMotor

_interrupt:

;Whey.c,300 :: 		void interrupt(){
;Whey.c,301 :: 		if(TMR1IF_bit)            //interrupcao pelo estouro do Timer1
	BTFSS      TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
	GOTO       L_interrupt28
;Whey.c,303 :: 		TMR1IF_bit = 0;          //Limpa a flag de interrupcao
	BCF        TMR1IF_bit+0, BitPos(TMR1IF_bit+0)
;Whey.c,304 :: 		numberInterruptionsTimer1++;   //incrementa a flag do overflow do timer1
	INCF       _numberInterruptionsTimer1+0, 1
	BTFSC      STATUS+0, 2
	INCF       _numberInterruptionsTimer1+1, 1
;Whey.c,305 :: 		}
L_interrupt28:
;Whey.c,307 :: 		if(CCP2IF_bit && CCP2M0_bit)            //Interrupcao do modulo CCP2 e modo de captura configurado para borda de subida?
	BTFSS      CCP2IF_bit+0, BitPos(CCP2IF_bit+0)
	GOTO       L_interrupt31
	BTFSS      CCP2M0_bit+0, BitPos(CCP2M0_bit+0)
	GOTO       L_interrupt31
L__interrupt76:
;Whey.c,309 :: 		CCP2IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP2
	BCF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,310 :: 		CCP4IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP4
	BCF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,311 :: 		TMR1ON_bit = 0x00;                     //Pausa o TIMER1
	BCF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,312 :: 		CCP2CON     = 0x04;                    //Configura captura por borda de descida
	MOVLW      4
	MOVWF      CCP2CON+0
;Whey.c,313 :: 		timeUpSignal1     = Micros();                //Guarda o valor do timer1 da primeira captura.
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      _timeUpSignal1+0
	MOVF       R1, 0
	MOVWF      _timeUpSignal1+1
	MOVF       R2, 0
	MOVWF      _timeUpSignal1+2
	MOVF       R3, 0
	MOVWF      _timeUpSignal1+3
;Whey.c,314 :: 		CCP2IF_bit  = 0x00;                    //Limpa a flag para nova captura
	BCF        CCP2IF_bit+0, BitPos(CCP2IF_bit+0)
;Whey.c,315 :: 		TMR1ON_bit = 0x01;                     //Retoma a contagem no TIMER1
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,316 :: 		CCP2IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP2
	BSF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,317 :: 		CCP4IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP4
	BSF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,318 :: 		} //end if
	GOTO       L_interrupt32
L_interrupt31:
;Whey.c,319 :: 		else if(CCP2IF_bit)                     //Interrupcao do modulo CCP2?
	BTFSS      CCP2IF_bit+0, BitPos(CCP2IF_bit+0)
	GOTO       L_interrupt33
;Whey.c,321 :: 		CCP2IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP2
	BCF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,322 :: 		CCP4IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP4
	BCF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,323 :: 		TMR1ON_bit = 0x00;                     //Pausa o TIMER1
	BCF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,324 :: 		CCP2CON     = 0x05;                    //Configura captura por borda de subida
	MOVLW      5
	MOVWF      CCP2CON+0
;Whey.c,325 :: 		timeDownSignal1     = Micros() - timeUpSignal1;      //Guarda o valor do timer1 da segunda captura.
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      _timeDownSignal1+0
	MOVF       R1, 0
	MOVWF      _timeDownSignal1+1
	MOVF       R2, 0
	MOVWF      _timeDownSignal1+2
	MOVF       R3, 0
	MOVWF      _timeDownSignal1+3
	MOVF       _timeUpSignal1+0, 0
	SUBWF      _timeDownSignal1+0, 1
	MOVF       _timeUpSignal1+1, 0
	SUBWFB     _timeDownSignal1+1, 1
	MOVF       _timeUpSignal1+2, 0
	SUBWFB     _timeDownSignal1+2, 1
	MOVF       _timeUpSignal1+3, 0
	SUBWFB     _timeDownSignal1+3, 1
;Whey.c,326 :: 		lastMeasure1 = Micros();              //guarda o tempo da ultima medida para o controle fail safe
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      _lastMeasure1+0
	MOVF       R1, 0
	MOVWF      _lastMeasure1+1
	MOVF       R2, 0
	MOVWF      _lastMeasure1+2
	MOVF       R3, 0
	MOVWF      _lastMeasure1+3
;Whey.c,327 :: 		TMR1ON_bit = 0x01;                     //Retoma a contagem no TIMER1
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,328 :: 		CCP2IF_bit  = 0x00;                    //Limpa a flag para nova captura
	BCF        CCP2IF_bit+0, BitPos(CCP2IF_bit+0)
;Whey.c,329 :: 		CCP2IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP2
	BSF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,330 :: 		CCP4IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP4
	BSF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,331 :: 		} //end else
L_interrupt33:
L_interrupt32:
;Whey.c,333 :: 		if(CCP4IF_bit && CCP4M0_bit)            //Interrupcao do modulo CCP4 e modo de captura configurado para borda de subida?
	BTFSS      CCP4IF_bit+0, BitPos(CCP4IF_bit+0)
	GOTO       L_interrupt36
	BTFSS      CCP4M0_bit+0, BitPos(CCP4M0_bit+0)
	GOTO       L_interrupt36
L__interrupt75:
;Whey.c,335 :: 		CCP2IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP2
	BCF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,336 :: 		CCP4IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP4
	BCF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,337 :: 		TMR1ON_bit = 0x00;                     //Pausa o TIMER1
	BCF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,338 :: 		CCP4CON     = 0x04;                    //Configura captura por borda de descida
	MOVLW      4
	MOVWF      CCP4CON+0
;Whey.c,339 :: 		timeUpSignal2     = Micros();                //Guarda o valor do timer1 da primeira captura.
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      _timeUpSignal2+0
	MOVF       R1, 0
	MOVWF      _timeUpSignal2+1
	MOVF       R2, 0
	MOVWF      _timeUpSignal2+2
	MOVF       R3, 0
	MOVWF      _timeUpSignal2+3
;Whey.c,340 :: 		CCP4IF_bit  = 0x00;                    //Limpa a flag para nova captura
	BCF        CCP4IF_bit+0, BitPos(CCP4IF_bit+0)
;Whey.c,341 :: 		TMR1ON_bit = 0x01;                     //Retoma a contagem no TIMER1
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,342 :: 		CCP2IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP2
	BSF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,343 :: 		CCP4IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP4
	BSF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,344 :: 		} //end if
	GOTO       L_interrupt37
L_interrupt36:
;Whey.c,345 :: 		else if(CCP4IF_bit)                     //Interrupcao do modulo CCP4?
	BTFSS      CCP4IF_bit+0, BitPos(CCP4IF_bit+0)
	GOTO       L_interrupt38
;Whey.c,347 :: 		CCP2IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP2
	BCF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,348 :: 		CCP4IE_bit  = 0x00;                    //Desabilita interrupcao do periferico CCP4
	BCF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,349 :: 		TMR1ON_bit = 0x00;                     //Pausa o TIMER1
	BCF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,350 :: 		CCP4CON     = 0x05;                    //Configura captura por borda de subida
	MOVLW      5
	MOVWF      CCP4CON+0
;Whey.c,351 :: 		timeDownSignal2     = Micros() - timeUpSignal2;      //Guarda o valor do timer1 da segunda captura.
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      _timeDownSignal2+0
	MOVF       R1, 0
	MOVWF      _timeDownSignal2+1
	MOVF       R2, 0
	MOVWF      _timeDownSignal2+2
	MOVF       R3, 0
	MOVWF      _timeDownSignal2+3
	MOVF       _timeUpSignal2+0, 0
	SUBWF      _timeDownSignal2+0, 1
	MOVF       _timeUpSignal2+1, 0
	SUBWFB     _timeDownSignal2+1, 1
	MOVF       _timeUpSignal2+2, 0
	SUBWFB     _timeDownSignal2+2, 1
	MOVF       _timeUpSignal2+3, 0
	SUBWFB     _timeDownSignal2+3, 1
;Whey.c,352 :: 		lastMeasure2 = Micros();              //guarda o tempo da ultima medida para o controle fail safe
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      _lastMeasure2+0
	MOVF       R1, 0
	MOVWF      _lastMeasure2+1
	MOVF       R2, 0
	MOVWF      _lastMeasure2+2
	MOVF       R3, 0
	MOVWF      _lastMeasure2+3
;Whey.c,353 :: 		TMR1ON_bit = 0x01;                     //Retoma a contagem no TIMER1
	BSF        TMR1ON_bit+0, BitPos(TMR1ON_bit+0)
;Whey.c,354 :: 		CCP4IF_bit  = 0x00;                    //Limpa a flag para nova captura
	BCF        CCP4IF_bit+0, BitPos(CCP4IF_bit+0)
;Whey.c,355 :: 		CCP2IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP2
	BSF        CCP2IE_bit+0, BitPos(CCP2IE_bit+0)
;Whey.c,356 :: 		CCP4IE_bit  = 0x01;                    //Habilita interrupcao do periferico CCP4
	BSF        CCP4IE_bit+0, BitPos(CCP4IE_bit+0)
;Whey.c,357 :: 		} //end else                                //Sim...
L_interrupt38:
L_interrupt37:
;Whey.c,358 :: 		} //end interrupt
L_end_interrupt:
L__interrupt109:
	RETFIE     %s
; end of _interrupt

_ErrorLedBlink:

;Whey.c,360 :: 		void ErrorLedBlink(unsigned time_ms){
;Whey.c,362 :: 		time_ms = time_ms/250; //4 blinks por segundo
	MOVLW      250
	MOVWF      R4
	CLRF       R5
	MOVF       FARG_ErrorLedBlink_time_ms+0, 0
	MOVWF      R0
	MOVF       FARG_ErrorLedBlink_time_ms+1, 0
	MOVWF      R1
	CALL       _Div_16X16_U+0
	MOVF       R0, 0
	MOVWF      FARG_ErrorLedBlink_time_ms+0
	MOVF       R1, 0
	MOVWF      FARG_ErrorLedBlink_time_ms+1
;Whey.c,363 :: 		for(i=0; i< time_ms; i++){
	CLRF       ErrorLedBlink_i_L0+0
	CLRF       ErrorLedBlink_i_L0+1
L_ErrorLedBlink39:
	MOVF       FARG_ErrorLedBlink_time_ms+1, 0
	SUBWF      ErrorLedBlink_i_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__ErrorLedBlink111
	MOVF       FARG_ErrorLedBlink_time_ms+0, 0
	SUBWF      ErrorLedBlink_i_L0+0, 0
L__ErrorLedBlink111:
	BTFSC      STATUS+0, 0
	GOTO       L_ErrorLedBlink40
;Whey.c,364 :: 		ERROR_LED = 1;
	BSF        RC7_bit+0, BitPos(RC7_bit+0)
;Whey.c,365 :: 		delay_ms(200);
	MOVLW      3
	MOVWF      R11
	MOVLW      8
	MOVWF      R12
	MOVLW      119
	MOVWF      R13
L_ErrorLedBlink42:
	DECFSZ     R13, 1
	GOTO       L_ErrorLedBlink42
	DECFSZ     R12, 1
	GOTO       L_ErrorLedBlink42
	DECFSZ     R11, 1
	GOTO       L_ErrorLedBlink42
;Whey.c,366 :: 		ERROR_LED = 0;
	BCF        RC7_bit+0, BitPos(RC7_bit+0)
;Whey.c,367 :: 		delay_ms(200);
	MOVLW      3
	MOVWF      R11
	MOVLW      8
	MOVWF      R12
	MOVLW      119
	MOVWF      R13
L_ErrorLedBlink43:
	DECFSZ     R13, 1
	GOTO       L_ErrorLedBlink43
	DECFSZ     R12, 1
	GOTO       L_ErrorLedBlink43
	DECFSZ     R11, 1
	GOTO       L_ErrorLedBlink43
;Whey.c,363 :: 		for(i=0; i< time_ms; i++){
	INCF       ErrorLedBlink_i_L0+0, 1
	BTFSC      STATUS+0, 2
	INCF       ErrorLedBlink_i_L0+1, 1
;Whey.c,368 :: 		}
	GOTO       L_ErrorLedBlink39
L_ErrorLedBlink40:
;Whey.c,369 :: 		}
L_end_ErrorLedBlink:
	RETURN
; end of _ErrorLedBlink

_CalibrationLedBlink:

;Whey.c,371 :: 		void CalibrationLedBlink(unsigned time_ms){
;Whey.c,373 :: 		time_ms = time_ms/250; //4 blinks por segundo
	MOVLW      250
	MOVWF      R4
	CLRF       R5
	MOVF       FARG_CalibrationLedBlink_time_ms+0, 0
	MOVWF      R0
	MOVF       FARG_CalibrationLedBlink_time_ms+1, 0
	MOVWF      R1
	CALL       _Div_16X16_U+0
	MOVF       R0, 0
	MOVWF      FARG_CalibrationLedBlink_time_ms+0
	MOVF       R1, 0
	MOVWF      FARG_CalibrationLedBlink_time_ms+1
;Whey.c,374 :: 		for(i=0; i< time_ms; i++){
	CLRF       CalibrationLedBlink_i_L0+0
	CLRF       CalibrationLedBlink_i_L0+1
L_CalibrationLedBlink44:
	MOVF       FARG_CalibrationLedBlink_time_ms+1, 0
	SUBWF      CalibrationLedBlink_i_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__CalibrationLedBlink113
	MOVF       FARG_CalibrationLedBlink_time_ms+0, 0
	SUBWF      CalibrationLedBlink_i_L0+0, 0
L__CalibrationLedBlink113:
	BTFSC      STATUS+0, 0
	GOTO       L_CalibrationLedBlink45
;Whey.c,375 :: 		CALIB_LED = 1;
	BSF        RB7_bit+0, BitPos(RB7_bit+0)
;Whey.c,376 :: 		delay_ms(200);
	MOVLW      3
	MOVWF      R11
	MOVLW      8
	MOVWF      R12
	MOVLW      119
	MOVWF      R13
L_CalibrationLedBlink47:
	DECFSZ     R13, 1
	GOTO       L_CalibrationLedBlink47
	DECFSZ     R12, 1
	GOTO       L_CalibrationLedBlink47
	DECFSZ     R11, 1
	GOTO       L_CalibrationLedBlink47
;Whey.c,377 :: 		CALIB_LED = 0;
	BCF        RB7_bit+0, BitPos(RB7_bit+0)
;Whey.c,378 :: 		delay_ms(200);
	MOVLW      3
	MOVWF      R11
	MOVLW      8
	MOVWF      R12
	MOVLW      119
	MOVWF      R13
L_CalibrationLedBlink48:
	DECFSZ     R13, 1
	GOTO       L_CalibrationLedBlink48
	DECFSZ     R12, 1
	GOTO       L_CalibrationLedBlink48
	DECFSZ     R11, 1
	GOTO       L_CalibrationLedBlink48
;Whey.c,374 :: 		for(i=0; i< time_ms; i++){
	INCF       CalibrationLedBlink_i_L0+0, 1
	BTFSC      STATUS+0, 2
	INCF       CalibrationLedBlink_i_L0+1, 1
;Whey.c,379 :: 		}
	GOTO       L_CalibrationLedBlink44
L_CalibrationLedBlink45:
;Whey.c,380 :: 		}
L_end_CalibrationLedBlink:
	RETURN
; end of _CalibrationLedBlink

_ReadSignalsDataEEPROM:

;Whey.c,382 :: 		void ReadSignalsDataEEPROM(){
;Whey.c,387 :: 		LowerSignificative8Bits = EEPROM_Read(0X00);
	CLRF       FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,388 :: 		MoreSignificative8Bits = EEPROM_Read(0X01);
	MOVLW      1
	MOVWF      FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,389 :: 		minDurationCH1 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;
	MOVF       R0, 0
	MOVWF      _minDurationCH1+1
	CLRF       _minDurationCH1+0
	MOVF       _LowerSignificative8Bits+0, 0
	IORWF       _minDurationCH1+0, 1
	MOVLW      0
	IORWF       _minDurationCH1+1, 1
;Whey.c,393 :: 		LowerSignificative8Bits = EEPROM_Read(0X02);
	MOVLW      2
	MOVWF      FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,394 :: 		MoreSignificative8Bits = EEPROM_Read(0X03);
	MOVLW      3
	MOVWF      FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,395 :: 		minDurationCH2 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;
	MOVF       R0, 0
	MOVWF      _minDurationCH2+1
	CLRF       _minDurationCH2+0
	MOVF       _LowerSignificative8Bits+0, 0
	IORWF       _minDurationCH2+0, 1
	MOVLW      0
	IORWF       _minDurationCH2+1, 1
;Whey.c,402 :: 		LowerSignificative8Bits = EEPROM_Read(0X04);
	MOVLW      4
	MOVWF      FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,403 :: 		MoreSignificative8Bits = EEPROM_Read(0X05);
	MOVLW      5
	MOVWF      FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,404 :: 		maxDurationCH1 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;
	MOVF       R0, 0
	MOVWF      _maxDurationCH1+1
	CLRF       _maxDurationCH1+0
	MOVF       _LowerSignificative8Bits+0, 0
	IORWF       _maxDurationCH1+0, 1
	MOVLW      0
	IORWF       _maxDurationCH1+1, 1
;Whey.c,408 :: 		LowerSignificative8Bits = EEPROM_Read(0X06);
	MOVLW      6
	MOVWF      FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,409 :: 		MoreSignificative8Bits = EEPROM_Read(0X07);
	MOVLW      7
	MOVWF      FARG_EEPROM_Read_Address+0
	CALL       _EEPROM_Read+0
	MOVF       R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,410 :: 		maxDurationCH2 = (MoreSignificative8Bits << 8) | LowerSignificative8Bits;
	MOVF       R0, 0
	MOVWF      _maxDurationCH2+1
	CLRF       _maxDurationCH2+0
	MOVF       _LowerSignificative8Bits+0, 0
	IORWF       _maxDurationCH2+0, 1
	MOVLW      0
	IORWF       _maxDurationCH2+1, 1
;Whey.c,415 :: 		}
L_end_ReadSignalsDataEEPROM:
	RETURN
; end of _ReadSignalsDataEEPROM

_Calibration:

;Whey.c,417 :: 		void Calibration(){
;Whey.c,425 :: 		signal1_L_value = 20000;                    //Tempo maximo, frequencia = 50 ... T=20ms
	MOVLW      32
	MOVWF      Calibration_signal1_L_value_L0+0
	MOVLW      78
	MOVWF      Calibration_signal1_L_value_L0+1
;Whey.c,426 :: 		signal2_L_value = 20000;                    //Tempo maximo, frequencia = 50 ... T=20ms
	MOVLW      32
	MOVWF      Calibration_signal2_L_value_L0+0
	MOVLW      78
	MOVWF      Calibration_signal2_L_value_L0+1
;Whey.c,427 :: 		signal1_H_value = 0;                        //Tempo minimo
	CLRF       Calibration_signal1_H_value_L0+0
	CLRF       Calibration_signal1_H_value_L0+1
;Whey.c,428 :: 		signal2_H_value = 0;                        //Tempo minimo
	CLRF       Calibration_signal2_H_value_L0+0
	CLRF       Calibration_signal2_H_value_L0+1
;Whey.c,429 :: 		time_control = Micros();                    //controla o tempo de captura
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      Calibration_time_control_L0+0
	MOVF       R1, 0
	MOVWF      Calibration_time_control_L0+1
	MOVF       R2, 0
	MOVWF      Calibration_time_control_L0+2
	MOVF       R3, 0
	MOVWF      Calibration_time_control_L0+3
;Whey.c,430 :: 		CALIB_LED = 1;                              //indica a captura do pulso
	BSF        RB7_bit+0, BitPos(RB7_bit+0)
;Whey.c,432 :: 		while((Micros() - time_control) < 2000000){
L_Calibration49:
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      R4
	MOVF       R1, 0
	MOVWF      R5
	MOVF       R2, 0
	MOVWF      R6
	MOVF       R3, 0
	MOVWF      R7
	MOVF       Calibration_time_control_L0+0, 0
	SUBWF      R4, 1
	MOVF       Calibration_time_control_L0+1, 0
	SUBWFB     R5, 1
	MOVF       Calibration_time_control_L0+2, 0
	SUBWFB     R6, 1
	MOVF       Calibration_time_control_L0+3, 0
	SUBWFB     R7, 1
	MOVLW      0
	SUBWF      R7, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration116
	MOVLW      30
	SUBWF      R6, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration116
	MOVLW      132
	SUBWF      R5, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration116
	MOVLW      128
	SUBWF      R4, 0
L__Calibration116:
	BTFSC      STATUS+0, 0
	GOTO       L_Calibration50
;Whey.c,433 :: 		signal_T_value = (unsigned) timeDownSignal1;   //valor da largura do pulso do canal1
	MOVF       _timeDownSignal1+0, 0
	MOVWF      Calibration_signal_T_value_L0+0
	MOVF       _timeDownSignal1+1, 0
	MOVWF      Calibration_signal_T_value_L0+1
;Whey.c,434 :: 		if(signal_T_value < signal1_L_value)
	MOVF       Calibration_signal1_L_value_L0+1, 0
	SUBWF      _timeDownSignal1+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration117
	MOVF       Calibration_signal1_L_value_L0+0, 0
	SUBWF      _timeDownSignal1+0, 0
L__Calibration117:
	BTFSC      STATUS+0, 0
	GOTO       L_Calibration51
;Whey.c,435 :: 		signal1_L_value = signal_T_value;
	MOVF       Calibration_signal_T_value_L0+0, 0
	MOVWF      Calibration_signal1_L_value_L0+0
	MOVF       Calibration_signal_T_value_L0+1, 0
	MOVWF      Calibration_signal1_L_value_L0+1
L_Calibration51:
;Whey.c,437 :: 		signal_T_value = (unsigned) timeDownSignal2;   //valor da largura do pulso do canal2
	MOVF       _timeDownSignal2+0, 0
	MOVWF      Calibration_signal_T_value_L0+0
	MOVF       _timeDownSignal2+1, 0
	MOVWF      Calibration_signal_T_value_L0+1
;Whey.c,438 :: 		if(signal_T_value < signal2_L_value)
	MOVF       Calibration_signal2_L_value_L0+1, 0
	SUBWF      _timeDownSignal2+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration118
	MOVF       Calibration_signal2_L_value_L0+0, 0
	SUBWF      _timeDownSignal2+0, 0
L__Calibration118:
	BTFSC      STATUS+0, 0
	GOTO       L_Calibration52
;Whey.c,439 :: 		signal2_L_value = signal_T_value;
	MOVF       Calibration_signal_T_value_L0+0, 0
	MOVWF      Calibration_signal2_L_value_L0+0
	MOVF       Calibration_signal_T_value_L0+1, 0
	MOVWF      Calibration_signal2_L_value_L0+1
L_Calibration52:
;Whey.c,440 :: 		}
	GOTO       L_Calibration49
L_Calibration50:
;Whey.c,444 :: 		LowerSignificative8Bits = signal1_L_value & 0xff;        //seleciona os 8 bits menos significativos
	MOVLW      255
	ANDWF      Calibration_signal1_L_value_L0+0, 0
	MOVWF      R3
	MOVF       R3, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,445 :: 		MoreSignificative8Bits = (signal1_L_value >> 8) & 0xff; //seleciona os 8 bits mais significativos
	MOVF       Calibration_signal1_L_value_L0+1, 0
	MOVWF      R0
	CLRF       R1
	MOVLW      255
	ANDWF      R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,446 :: 		EEPROM_Write(0X00,LowerSignificative8Bits);
	CLRF       FARG_EEPROM_Write_Address+0
	MOVF       R3, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,447 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration53:
	DECFSZ     R13, 1
	GOTO       L_Calibration53
	DECFSZ     R12, 1
	GOTO       L_Calibration53
	NOP
;Whey.c,448 :: 		EEPROM_Write(0X01,MoreSignificative8Bits);
	MOVLW      1
	MOVWF      FARG_EEPROM_Write_Address+0
	MOVF       _MoreSignificative8Bits+0, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,449 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration54:
	DECFSZ     R13, 1
	GOTO       L_Calibration54
	DECFSZ     R12, 1
	GOTO       L_Calibration54
	NOP
;Whey.c,452 :: 		LowerSignificative8Bits = signal2_L_value & 0xff;        //seleciona os 8 bits menos significativos
	MOVLW      255
	ANDWF      Calibration_signal2_L_value_L0+0, 0
	MOVWF      R3
	MOVF       R3, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,453 :: 		MoreSignificative8Bits = (signal2_L_value >> 8) & 0xff; //seleciona os 8 bits mais significativos
	MOVF       Calibration_signal2_L_value_L0+1, 0
	MOVWF      R0
	CLRF       R1
	MOVLW      255
	ANDWF      R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,454 :: 		EEPROM_Write(0X02,LowerSignificative8Bits);
	MOVLW      2
	MOVWF      FARG_EEPROM_Write_Address+0
	MOVF       R3, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,455 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration55:
	DECFSZ     R13, 1
	GOTO       L_Calibration55
	DECFSZ     R12, 1
	GOTO       L_Calibration55
	NOP
;Whey.c,456 :: 		EEPROM_Write(0X03,MoreSignificative8Bits);
	MOVLW      3
	MOVWF      FARG_EEPROM_Write_Address+0
	MOVF       _MoreSignificative8Bits+0, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,457 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration56:
	DECFSZ     R13, 1
	GOTO       L_Calibration56
	DECFSZ     R12, 1
	GOTO       L_Calibration56
	NOP
;Whey.c,459 :: 		CalibrationLedBlink(1600);                      //indica a captura do valor minimo
	MOVLW      64
	MOVWF      FARG_CalibrationLedBlink_time_ms+0
	MOVLW      6
	MOVWF      FARG_CalibrationLedBlink_time_ms+1
	CALL       _CalibrationLedBlink+0
;Whey.c,460 :: 		time_control = Micros();                    //controla o tempo de captura
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      Calibration_time_control_L0+0
	MOVF       R1, 0
	MOVWF      Calibration_time_control_L0+1
	MOVF       R2, 0
	MOVWF      Calibration_time_control_L0+2
	MOVF       R3, 0
	MOVWF      Calibration_time_control_L0+3
;Whey.c,461 :: 		CALIB_LED = 1;                              //indica a captura do pulso
	BSF        RB7_bit+0, BitPos(RB7_bit+0)
;Whey.c,463 :: 		while((Micros() - time_control) < 2000000){
L_Calibration57:
	CALL       _Micros+0
	MOVF       R0, 0
	MOVWF      R4
	MOVF       R1, 0
	MOVWF      R5
	MOVF       R2, 0
	MOVWF      R6
	MOVF       R3, 0
	MOVWF      R7
	MOVF       Calibration_time_control_L0+0, 0
	SUBWF      R4, 1
	MOVF       Calibration_time_control_L0+1, 0
	SUBWFB     R5, 1
	MOVF       Calibration_time_control_L0+2, 0
	SUBWFB     R6, 1
	MOVF       Calibration_time_control_L0+3, 0
	SUBWFB     R7, 1
	MOVLW      0
	SUBWF      R7, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration119
	MOVLW      30
	SUBWF      R6, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration119
	MOVLW      132
	SUBWF      R5, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration119
	MOVLW      128
	SUBWF      R4, 0
L__Calibration119:
	BTFSC      STATUS+0, 0
	GOTO       L_Calibration58
;Whey.c,464 :: 		signal_T_value = (unsigned) timeDownSignal1;   //valor da largura do pulso do canal1
	MOVF       _timeDownSignal1+0, 0
	MOVWF      Calibration_signal_T_value_L0+0
	MOVF       _timeDownSignal1+1, 0
	MOVWF      Calibration_signal_T_value_L0+1
;Whey.c,466 :: 		if(signal_T_value > signal1_H_value)
	MOVF       _timeDownSignal1+1, 0
	SUBWF      Calibration_signal1_H_value_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration120
	MOVF       _timeDownSignal1+0, 0
	SUBWF      Calibration_signal1_H_value_L0+0, 0
L__Calibration120:
	BTFSC      STATUS+0, 0
	GOTO       L_Calibration59
;Whey.c,467 :: 		signal1_H_value = signal_T_value;
	MOVF       Calibration_signal_T_value_L0+0, 0
	MOVWF      Calibration_signal1_H_value_L0+0
	MOVF       Calibration_signal_T_value_L0+1, 0
	MOVWF      Calibration_signal1_H_value_L0+1
L_Calibration59:
;Whey.c,469 :: 		signal_T_value = (unsigned) timeDownSignal2;   //valor da largura do pulso do canal1
	MOVF       _timeDownSignal2+0, 0
	MOVWF      Calibration_signal_T_value_L0+0
	MOVF       _timeDownSignal2+1, 0
	MOVWF      Calibration_signal_T_value_L0+1
;Whey.c,471 :: 		if(signal_T_value > signal2_H_value)
	MOVF       _timeDownSignal2+1, 0
	SUBWF      Calibration_signal2_H_value_L0+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__Calibration121
	MOVF       _timeDownSignal2+0, 0
	SUBWF      Calibration_signal2_H_value_L0+0, 0
L__Calibration121:
	BTFSC      STATUS+0, 0
	GOTO       L_Calibration60
;Whey.c,472 :: 		signal2_H_value = signal_T_value;
	MOVF       Calibration_signal_T_value_L0+0, 0
	MOVWF      Calibration_signal2_H_value_L0+0
	MOVF       Calibration_signal_T_value_L0+1, 0
	MOVWF      Calibration_signal2_H_value_L0+1
L_Calibration60:
;Whey.c,473 :: 		}
	GOTO       L_Calibration57
L_Calibration58:
;Whey.c,476 :: 		LowerSignificative8Bits = signal1_H_value & 0xff;        //seleciona os 8 bits menos significativos
	MOVLW      255
	ANDWF      Calibration_signal1_H_value_L0+0, 0
	MOVWF      R3
	MOVF       R3, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,477 :: 		MoreSignificative8Bits = (signal1_H_value >> 8) & 0xff; //seleciona os 8 bits mais significativos
	MOVF       Calibration_signal1_H_value_L0+1, 0
	MOVWF      R0
	CLRF       R1
	MOVLW      255
	ANDWF      R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,478 :: 		EEPROM_Write(0X04,LowerSignificative8Bits);
	MOVLW      4
	MOVWF      FARG_EEPROM_Write_Address+0
	MOVF       R3, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,479 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration61:
	DECFSZ     R13, 1
	GOTO       L_Calibration61
	DECFSZ     R12, 1
	GOTO       L_Calibration61
	NOP
;Whey.c,480 :: 		EEPROM_Write(0X05,MoreSignificative8Bits);
	MOVLW      5
	MOVWF      FARG_EEPROM_Write_Address+0
	MOVF       _MoreSignificative8Bits+0, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,481 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration62:
	DECFSZ     R13, 1
	GOTO       L_Calibration62
	DECFSZ     R12, 1
	GOTO       L_Calibration62
	NOP
;Whey.c,483 :: 		LowerSignificative8Bits = signal2_H_value & 0xff;        //seleciona os 8 bits menos significativos
	MOVLW      255
	ANDWF      Calibration_signal2_H_value_L0+0, 0
	MOVWF      R3
	MOVF       R3, 0
	MOVWF      _LowerSignificative8Bits+0
;Whey.c,484 :: 		MoreSignificative8Bits = (signal2_H_value >> 8) & 0xff; //seleciona os 8 bits mais significativos
	MOVF       Calibration_signal2_H_value_L0+1, 0
	MOVWF      R0
	CLRF       R1
	MOVLW      255
	ANDWF      R0, 0
	MOVWF      _MoreSignificative8Bits+0
;Whey.c,485 :: 		EEPROM_Write(0X06,LowerSignificative8Bits);
	MOVLW      6
	MOVWF      FARG_EEPROM_Write_Address+0
	MOVF       R3, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,486 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration63:
	DECFSZ     R13, 1
	GOTO       L_Calibration63
	DECFSZ     R12, 1
	GOTO       L_Calibration63
	NOP
;Whey.c,487 :: 		EEPROM_Write(0X07,MoreSignificative8Bits);
	MOVLW      7
	MOVWF      FARG_EEPROM_Write_Address+0
	MOVF       _MoreSignificative8Bits+0, 0
	MOVWF      FARG_EEPROM_Write_data_+0
	CALL       _EEPROM_Write+0
;Whey.c,488 :: 		delay_ms(10);
	MOVLW      26
	MOVWF      R12
	MOVLW      248
	MOVWF      R13
L_Calibration64:
	DECFSZ     R13, 1
	GOTO       L_Calibration64
	DECFSZ     R12, 1
	GOTO       L_Calibration64
	NOP
;Whey.c,490 :: 		CalibrationLedBlink(1600);                      //indica a captura do valor maximo
	MOVLW      64
	MOVWF      FARG_CalibrationLedBlink_time_ms+0
	MOVLW      6
	MOVWF      FARG_CalibrationLedBlink_time_ms+1
	CALL       _CalibrationLedBlink+0
;Whey.c,491 :: 		CALIB_LED = 0;
	BCF        RB7_bit+0, BitPos(RB7_bit+0)
;Whey.c,493 :: 		ReadSignalsDataEEPROM();
	CALL       _ReadSignalsDataEEPROM+0
;Whey.c,494 :: 		}
L_end_Calibration:
	RETURN
; end of _Calibration

_PrintSignalReceived:

;Whey.c,496 :: 		void PrintSignalReceived(){
;Whey.c,499 :: 		UART1_write_text("Sinal 1: ");
	MOVLW      ?lstr1_Whey+0
	MOVWF      FARG_UART1_Write_Text_uart_text+0
	MOVLW      hi_addr(?lstr1_Whey+0)
	MOVWF      FARG_UART1_Write_Text_uart_text+1
	CALL       _UART1_Write_Text+0
;Whey.c,500 :: 		LongWordToStr(timeDownSignal1, buffer);
	MOVF       _timeDownSignal1+0, 0
	MOVWF      FARG_LongWordToStr_input+0
	MOVF       _timeDownSignal1+1, 0
	MOVWF      FARG_LongWordToStr_input+1
	MOVF       _timeDownSignal1+2, 0
	MOVWF      FARG_LongWordToStr_input+2
	MOVF       _timeDownSignal1+3, 0
	MOVWF      FARG_LongWordToStr_input+3
	MOVLW      PrintSignalReceived_buffer_L0+0
	MOVWF      FARG_LongWordToStr_output+0
	MOVLW      hi_addr(PrintSignalReceived_buffer_L0+0)
	MOVWF      FARG_LongWordToStr_output+1
	CALL       _LongWordToStr+0
;Whey.c,501 :: 		UART1_write_text(buffer);
	MOVLW      PrintSignalReceived_buffer_L0+0
	MOVWF      FARG_UART1_Write_Text_uart_text+0
	MOVLW      hi_addr(PrintSignalReceived_buffer_L0+0)
	MOVWF      FARG_UART1_Write_Text_uart_text+1
	CALL       _UART1_Write_Text+0
;Whey.c,502 :: 		UART1_write_text("\t");
	MOVLW      ?lstr2_Whey+0
	MOVWF      FARG_UART1_Write_Text_uart_text+0
	MOVLW      hi_addr(?lstr2_Whey+0)
	MOVWF      FARG_UART1_Write_Text_uart_text+1
	CALL       _UART1_Write_Text+0
;Whey.c,504 :: 		UART1_write_text("Sinal 2: ");
	MOVLW      ?lstr3_Whey+0
	MOVWF      FARG_UART1_Write_Text_uart_text+0
	MOVLW      hi_addr(?lstr3_Whey+0)
	MOVWF      FARG_UART1_Write_Text_uart_text+1
	CALL       _UART1_Write_Text+0
;Whey.c,505 :: 		LongWordToStr(timeDownSignal2, buffer);
	MOVF       _timeDownSignal2+0, 0
	MOVWF      FARG_LongWordToStr_input+0
	MOVF       _timeDownSignal2+1, 0
	MOVWF      FARG_LongWordToStr_input+1
	MOVF       _timeDownSignal2+2, 0
	MOVWF      FARG_LongWordToStr_input+2
	MOVF       _timeDownSignal2+3, 0
	MOVWF      FARG_LongWordToStr_input+3
	MOVLW      PrintSignalReceived_buffer_L0+0
	MOVWF      FARG_LongWordToStr_output+0
	MOVLW      hi_addr(PrintSignalReceived_buffer_L0+0)
	MOVWF      FARG_LongWordToStr_output+1
	CALL       _LongWordToStr+0
;Whey.c,506 :: 		UART1_write_text(buffer);
	MOVLW      PrintSignalReceived_buffer_L0+0
	MOVWF      FARG_UART1_Write_Text_uart_text+0
	MOVLW      hi_addr(PrintSignalReceived_buffer_L0+0)
	MOVWF      FARG_UART1_Write_Text_uart_text+1
	CALL       _UART1_Write_Text+0
;Whey.c,507 :: 		UART1_write_text("\n");
	MOVLW      ?lstr4_Whey+0
	MOVWF      FARG_UART1_Write_Text_uart_text+0
	MOVLW      hi_addr(?lstr4_Whey+0)
	MOVWF      FARG_UART1_Write_Text_uart_text+1
	CALL       _UART1_Write_Text+0
;Whey.c,509 :: 		delay_ms(100);
	MOVLW      2
	MOVWF      R11
	MOVLW      4
	MOVWF      R12
	MOVLW      186
	MOVWF      R13
L_PrintSignalReceived65:
	DECFSZ     R13, 1
	GOTO       L_PrintSignalReceived65
	DECFSZ     R12, 1
	GOTO       L_PrintSignalReceived65
	DECFSZ     R11, 1
	GOTO       L_PrintSignalReceived65
	NOP
;Whey.c,510 :: 		}
L_end_PrintSignalReceived:
	RETURN
; end of _PrintSignalReceived

_main:

;Whey.c,516 :: 		void main() {
;Whey.c,518 :: 		OSCCON = 0b11110010;
	MOVLW      242
	MOVWF      OSCCON+0
;Whey.c,519 :: 		ADC_Init();
	CALL       _ADC_Init+0
;Whey.c,520 :: 		SetupPort();
	CALL       _SetupPort+0
;Whey.c,521 :: 		SetupPwms();
	CALL       _SetupPwms+0
;Whey.c,522 :: 		SetupTimer1();
	CALL       _SetupTimer1+0
;Whey.c,523 :: 		delay_ms(300);
	MOVLW      4
	MOVWF      R11
	MOVLW      12
	MOVWF      R12
	MOVLW      51
	MOVWF      R13
L_main66:
	DECFSZ     R13, 1
	GOTO       L_main66
	DECFSZ     R12, 1
	GOTO       L_main66
	DECFSZ     R11, 1
	GOTO       L_main66
	NOP
	NOP
;Whey.c,525 :: 		if(CALIB_BUTTON==0)
	BTFSC      RA3_bit+0, BitPos(RA3_bit+0)
	GOTO       L_main67
;Whey.c,526 :: 		Calibration();
	CALL       _Calibration+0
L_main67:
;Whey.c,528 :: 		while(1){
L_main68:
;Whey.c,529 :: 		ERROR_LED = 0;
	BCF        RC7_bit+0, BitPos(RC7_bit+0)
;Whey.c,530 :: 		while(FailSafeCheck()) {
L_main70:
	CALL       _FailSafeCheck+0
	MOVF       R0, 0
	IORWF       R1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main71
;Whey.c,531 :: 		ERROR_LED = 1;
	BSF        RC7_bit+0, BitPos(RC7_bit+0)
;Whey.c,532 :: 		SetDutyCycle(1, 0);
	MOVLW      1
	MOVWF      FARG_SetDutyCycle_channel+0
	MOVLW      0
	MOVWF      FARG_SetDutyCycle_channel+1
	CLRF       FARG_SetDutyCycle_duty+0
	CLRF       FARG_SetDutyCycle_duty+1
	CALL       _SetDutyCycle+0
;Whey.c,533 :: 		SetDutyCycle(2, 0);
	MOVLW      2
	MOVWF      FARG_SetDutyCycle_channel+0
	MOVLW      0
	MOVWF      FARG_SetDutyCycle_channel+1
	CLRF       FARG_SetDutyCycle_duty+0
	CLRF       FARG_SetDutyCycle_duty+1
	CALL       _SetDutyCycle+0
;Whey.c,534 :: 		}
	GOTO       L_main70
L_main71:
;Whey.c,537 :: 		if(BatteryCheck())
	CALL       _BatteryCheck+0
	MOVF       R0, 0
	IORWF       R1, 0
	BTFSC      STATUS+0, 2
	GOTO       L_main72
;Whey.c,538 :: 		ErrorLedBlink(1600);
	MOVLW      64
	MOVWF      FARG_ErrorLedBlink_time_ms+0
	MOVLW      6
	MOVWF      FARG_ErrorLedBlink_time_ms+1
	CALL       _ErrorLedBlink+0
L_main72:
;Whey.c,540 :: 		RotateMotor();
	CALL       _RotateMotor+0
;Whey.c,542 :: 		}
	GOTO       L_main68
;Whey.c,543 :: 		}
L_end_main:
	GOTO       $+0
; end of _main
