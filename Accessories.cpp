#include "Common.h"

int T_OkuriMotor		;
int T_OkuriMotor_Middle	;
int T_Cut1				;
int T_Cut2				;
int T_BarOpen			;
int S_COriginSignal		;

void Init_Accessories(void);
void initA(void);
void Cutting(int Delay_Time_Cutting);
void Okuri_Start(int DelayTime);
void LeverOpen(void);
void LeverClose(void);


void Init_Accessories(void) {
	initA();
}

void initA() {
	T_OkuriMotor			 	= Taping_OkuriMotor;
	T_OkuriMotor_Middle	 		= Taping_OkuriMotor_Middle;
	T_Cut1			 			= Taping_Cut1;
	T_Cut2			 			= Taping_Cut2;
	T_BarOpen					= Taping_BarOpen;
    S_COriginSignal	     		= Sensor_COriginSignal;

	pinMode(T_OkuriMotor        , OUTPUT);
	pinMode(T_OkuriMotor_Middle , OUTPUT);
	pinMode(T_Cut1				, OUTPUT);
	pinMode(T_Cut2				, OUTPUT);
	digitalWrite(T_Cut2			, LOW);
	digitalWrite(T_Cut1			, HIGH);
	pinMode(T_BarOpen			, OUTPUT);
	pinMode(S_COriginSignal	    , INPUT_PULLDOWN);

}

void Cutting(int Delay_Time_Cutting){
    
    SerialUSB.println("Cutting_On");
    digitalWrite(T_Cut1, LOW);
    digitalWrite(T_Cut2, HIGH);
    delay(Delay_Time_Cutting);
    digitalWrite(T_Cut2, LOW);
    digitalWrite(T_Cut1, HIGH);
    SerialUSB.println("Cutting_Off");
  
}
  
void Okuri_Start(int DelayTime){
  
    SerialUSB.println("Okuri_Start1_On");
    digitalWrite(T_OkuriMotor, HIGH);
    digitalWrite(T_OkuriMotor_Middle, HIGH);
    delay(DelayTime);
    digitalWrite(T_OkuriMotor, LOW);
    digitalWrite(T_OkuriMotor_Middle, LOW);
    SerialUSB.println("Okuri_Start1_Off");
      
}

void LeverOpen(void){
	digitalWrite(T_BarOpen, HIGH);
    delay(100);
}

void LeverClose(void){
	digitalWrite(T_BarOpen, LOW);
    delay(100);
}

