#include <Common.h>
int Running_Status;

volatile int enc_count        = 0;
const int Target_Count[3]     = {4800,230,4800}; //{j,1,2}
int phase[3]                  = {0,1,2};         //{j,1,2}
const int Delay_Time_Okuri[3] = {700,310,300};   //{f,1,2}
const int Delay_Time_Cutting  = 300;
  
int Debug_On                  = 1;
int Debug_Time                = 500;
int Polling_Time              = 10;
int MicroSec_To_MilliSec      = 1000;
int MilliSec_To_Sec           = 1000;
  
void setup() {
  Running_Status = WAIT_INPUT;
  
  //-----------------Initialize_AllFiles-----------------
  Init_TapingMotor();
  Init_Accessories();
  Init_Interface();
}
  
void loop() {
    //-----------------Taping_Sequence-----------------
    //SerialUSB.println(enc_count);
    switch(Running_Status){
    case START_TAPING:
                  Running_Status = OKURI_START1;
      break;
    case OKURI_START1:
                  SerialUSB.println("OKURI_START1");
                  Okuri_Start(Delay_Time_Okuri[1]);
                  Running_Status = MOTOR_START1;
      break;
    case MOTOR_START1:
                  SerialUSB.println("MOTOR_START1");
                  PID_Start();
                  Running_Status = PID_CONTROL1;
      break;
    case PID_CONTROL1:
                  SerialUSB.println(enc_count);
                  Running_Status = PID_CONTROL1;
                  PID_Control(Target_Count[1],phase[1]);
      break;
    case LEVER_CLOSE:
                  SerialUSB.println("LEVER_CLOSE");
                  LeverClose();
                  Running_Status = CUTTING_START;
                  delay(200);
      break;
    case CUTTING_START:
                  SerialUSB.println("CUTTING");
                  Cutting(Delay_Time_Cutting);
                  Running_Status = MOTOR_START2;
                  delay(200);
      break;
    case MOTOR_START2:
                  SerialUSB.println("MOTOR_START2");
                  PID_Start();
                  Running_Status = PID_CONTROL2;
      break;
    case PID_CONTROL2:
                  SerialUSB.println(enc_count);
                  Running_Status = PID_CONTROL2;
                  PID_Control(Target_Count[2],phase[2]);
      break;
    case OKURI_START2:
                  SerialUSB.println("OKURI_START2");
                  Okuri_Start(Delay_Time_Okuri[2]);
                  Running_Status = LEVER_OPEN;
                  SerialUSB.println(enc_count);
      break;
    case LEVER_OPEN:
                  SerialUSB.println("LEVER_OPEN");
                  LeverOpen();
                  Running_Status = WAIT_INPUT;
                  digitalWrite(O_PLC_TapingFinish, HIGH);
	          delay(200);
                  digitalWrite(O_PLC_TapingFinish, LOW);
      break;
    //----------------------------------
    //-----------------OnlyMotorControl(Input 'j' on Serial Monitor)-----------------
    case MOTOR_START0:
                  PID_Start();
                  Running_Status = PID_CONTROL0;
      break;
    case PID_CONTROL0:
                  SerialUSB.println(enc_count);
                  Running_Status = PID_CONTROL0;
                  PID_Control(Target_Count[0],phase[0]);
    //----------------------------------
      break;
    default:
      break;      
    }
    Serial_Check();
    IOCheck();
  
    delayMicroseconds(Polling_Time);
}
