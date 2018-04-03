#include "Common.h"

//----------------------Alias_Config----------------------
int I_PLC_TapingStart1		;
int I_PLC_TapingStart2		;
int I_PLC_PreOkuri			;
int I_PLC_BarOpen			;
int I_PLC_MotorReset		;
int I_PLC_ReturnToOrigin	;
//int I_PLC_Spare1			;
int I_PLC_Spare2			;
int O_PLC_TapingFinish		;
int O_PLC_RetToOrigFinish	;
int O_PLC_PreOkuriFinish	;
int O_PLC_BarOpenFinish		;
int O_PLC_MotorAlarm		;
//int O_PLC_Spare1			;
int O_PLC_Spare2			;
//----------------------Logic_Config----------------------
int L_PLC_TapingStart1		;
int L_PLC_TapingStart2		;
int L_PLC_PreOkuri			;
int L_PLC_BarOpen			;
int L_PLC_MotorReset		;
int L_PLC_ReturnToOrigin	;
//int L_PLC_Spare1			;
int L_PLC_Spare2			;

void Init_Interface(void);
void initI(void);
void IOCheck(void);
void Serial_Check(void);
void SignalsRead(void);

void Init_Interface(void) {
	initI();
}

void initI() {
	//----------------------Signals_Alias----------------------
	I_PLC_TapingStart1		= In_PLC_TapingStart1		;
	I_PLC_TapingStart2		= In_PLC_TapingStart2		;
	I_PLC_PreOkuri		    = In_PLC_PreOkuri			;
	I_PLC_BarOpen			= In_PLC_BarOpen			;
	I_PLC_MotorReset		= In_PLC_MotorReset			;
	I_PLC_ReturnToOrigin	= In_PLC_ReturnToOrigin		;
	//I_PLC_Spare1			= In_PLC_Spare1				;
	I_PLC_Spare2			= In_PLC_Spare2				;
	O_PLC_TapingFinish		= Out_PLC_TapingFinish		;
	O_PLC_RetToOrigFinish	= Out_PLC_RetToOrigFinish	;
	O_PLC_PreOkuriFinish	= Out_PLC_PreOkuriFinish	;
	O_PLC_BarOpenFinish		= Out_PLC_BarOpenFinish		;
	O_PLC_MotorAlarm		= Out_PLC_MotorAlarm		;
	//O_PLC_Spare1			= Out_PLC_Spare1			;
	O_PLC_Spare2			= Out_PLC_Spare2			;

	//----------------------Signals_Configration----------------------
	pinMode(I_PLC_TapingStart1    , INPUT_PULLDOWN)	;
	pinMode(I_PLC_TapingStart2    , INPUT_PULLDOWN)	;
	pinMode(I_PLC_PreOkuri		  , INPUT_PULLDOWN)	;
	pinMode(I_PLC_BarOpen         , INPUT_PULLDOWN)	;
	pinMode(I_PLC_MotorReset      , INPUT_PULLDOWN)	;
	pinMode(I_PLC_ReturnToOrigin  , INPUT_PULLDOWN)	;
	//pinMode(I_PLC_Spare1        , INPUT_PULLDOWN)	;
	pinMode(I_PLC_Spare2          , INPUT_PULLDOWN)	;
	pinMode(O_PLC_TapingFinish    , OUTPUT)			;
	pinMode(O_PLC_RetToOrigFinish , OUTPUT)			;
	pinMode(O_PLC_PreOkuriFinish  , OUTPUT)			;
	pinMode(O_PLC_BarOpenFinish   , OUTPUT)			;
	pinMode(O_PLC_MotorAlarm      , OUTPUT)			;
	//pinMode(O_PLC_Spare1        , OUTPUT)			;
	pinMode(O_PLC_Spare2          , OUTPUT)			;

}

//----------------------Communication_with_I/O----------------------
void IOCheck(){
  SignalsRead();

  if(L_PLC_TapingStart1		== HIGH){
    SerialUSB.println("#Y0_#D10_Received_PLC_TapingStart1");
    enc_count = 0;
    Running_Status = START_TAPING;
  }
  if(L_PLC_TapingStart2		== HIGH){
    SerialUSB.println("#Y1_#D11_Received_PLC_TapingStart2");
  }
  if(L_PLC_PreOkuri			== HIGH){
    SerialUSB.println("#Y2_#D12_Received_PLC_PreOkuri");
	Okuri_Start(Delay_Time_Okuri[0]);
    digitalWrite(O_PLC_PreOkuriFinish, HIGH);
	delay(100);
    digitalWrite(O_PLC_PreOkuriFinish, LOW);
  }
  if(L_PLC_BarOpen			== HIGH){
    SerialUSB.println("#Y3_#D13_Received_PLC_BarOpen");
	LeverOpen();
	digitalWrite(O_PLC_BarOpenFinish, HIGH);
    delay(500);
	digitalWrite(O_PLC_BarOpenFinish, LOW);
  }
  if(L_PLC_MotorReset		== HIGH){
    SerialUSB.println("#Y5_#D15_Received_PLC_MotorReset");
  }
  if(L_PLC_ReturnToOrigin   == HIGH){
    SerialUSB.println("#Y6_#D14_Received_PLC_ReturnToOrigin");
    digitalWrite(O_PLC_RetToOrigFinish, HIGH);
	delay(200);
    digitalWrite(O_PLC_RetToOrigFinish, LOW);
  }
  //if(L_PLC_Spare1   == HIGH){
  //  SerialUSB.println("#Y6_#D14_Received_L_PLC_Spare1");
  //}
  if(L_PLC_Spare2			== HIGH){
    SerialUSB.println("#Y7_#D17_Received_PLC_Spare2");
  }
}

//----------------------Communication_with_SerialUSB----------------------
void Serial_Check(){
    
      if(SerialUSB.available() > 0){
          switch(SerialUSB.read()){
			  //'a':	Forwad Ratation
			  //'b':	Breaking
			  //'c':	Reverse Rotation
			  //'d':	Taping Start
			  //'e':	PID_Break
			  //'f':	Okuri_Start
			  //'g':	Cutting
			  //'h':	Cut_On
			  //'i':	Cut_Off
			  //'j':	Only_Motor_Rot
             case 'a':    
                enc_count = 0;
                Forward();
                break;
  
              case 'b':
                
                Break();
                break;
  
              case 'c':
                enc_count = 0;
                Reverse();
                break;
  
              case 'd':
                enc_count = 0;
                Running_Status = START_TAPING;
                break;
  
              case 'e':
                enc_count = 0;
                Break();
                PID_On = 0;
                break;
  
              case 'f':
                Okuri_Start(Delay_Time_Okuri[0]);
                break;
  
              case 'g':
                Cutting(Delay_Time_Cutting);
                break;
  
              case 'h':
                SerialUSB.println("Cutting_On");
                digitalWrite(T_Cut1, LOW);
                digitalWrite(T_Cut2, HIGH);
                break;
  
              case 'i':
                SerialUSB.println("Cutting_Off");
                digitalWrite(T_Cut2, LOW);
                digitalWrite(T_Cut1, HIGH);
                break;

              case 'j':
                enc_count = 0;
                Running_Status = MOTOR_START0;
                break;

              default:
                break;
          }
   }
}

void SignalsRead(){

	L_PLC_TapingStart1   = digitalRead(I_PLC_TapingStart1)	;//#Y0
	L_PLC_TapingStart2   = digitalRead(I_PLC_TapingStart2)	;//#Y1
	L_PLC_PreOkuri		 = digitalRead(I_PLC_PreOkuri)		;//#Y2
	L_PLC_BarOpen        = digitalRead(I_PLC_BarOpen)		;//#Y3
	L_PLC_MotorReset     = digitalRead(I_PLC_MotorReset)	;//#Y5
	L_PLC_ReturnToOrigin = digitalRead(I_PLC_ReturnToOrigin);//#Y6
	//L_PLC_Spare1         = digitalRead(I_PLC_Spare1)		;
	L_PLC_Spare2	     = digitalRead(I_PLC_Spare2)		;//#Y7

}