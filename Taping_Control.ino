#define WAIT_INPUT     100
#define START_TAPING   150
#define OKURI_START1   200
#define MOTOR_START    250
#define PID_CONTROL    300
#define CUTTING_START  350
#define OKURI_START2   400

int Running_Status;
int MicroSec_To_MilliSec      = 1000;
int MilliSec_To_Sec           = 1000;
int Debug_On                  = 1;
int Debug_Time                = 500;
int Polling_Time              = 10;

volatile int enc_count        = 0;
const int target_count        = 4800;
double P                      = 0.03;
double I                      = 0;
double D                      = 0;
double PWM_MAX                = 255;
double PWM_MIN                = 70;
int PID_On                    = 0;

const int M_pinA              = 2; //A相割り込み0
const int M_pinB              = 3; //B相割り込み1
const int M_pinFW             = 8; //Foword Trigger
const int M_pinBR             = 9; //Break Trigger
const int M_pinRE             = 10; //Reverse Trigger
const int M_PWM               = 11; //PWM

const int T_OkuriMotor        = 4; //送りモーター
const int T_OkuriMotor_Middle = 5; //中間送りモーター
const int T_Cut1              = 6; //カット出
const int T_Cut2              = 7; //カット戻
const int T_Bar1              = 12; //バー開
const int T_Bar2              = 13; //バー閉
int Okuri_Delay_Time[2]       = {500,500};


void setup() {
  Running_Status = WAIT_INPUT;

  Serial.begin(9600);
  //
  pinMode(M_pinA              , INPUT);
  pinMode(M_pinB              , INPUT);
  pinMode(M_pinFW             , OUTPUT);
  pinMode(M_pinBR             , OUTPUT);
  pinMode(M_pinRE             , OUTPUT);
  pinMode(M_PWM               , OUTPUT);
  //
  pinMode(T_OkuriMotor        , OUTPUT);
  pinMode(T_OkuriMotor_Middle , OUTPUT);
  pinMode(T_Cut1              , OUTPUT);
  pinMode(T_Cut2              , OUTPUT);
  pinMode(T_Bar1              , OUTPUT);
  pinMode(T_Bar2              , OUTPUT);
  //
  attachInterrupt(0           , enc_changedPinA, CHANGE);//pinAの信号変化に合わせて割り込み処理
  attachInterrupt(1           , enc_changedPinB, CHANGE);//pinBの信号変化に合わせて割り込み処理
}

void loop() {

  //Taping_Sequence//
  switch(Running_Status){
  case START_TAPING:
                Running_Status = OKURI_START1;
    break;
  case OKURI_START1:
                Okuri_Start(Okuri_Delay_Time[0]);
                Running_Status = CUTTING_START;
    break;
  case CUTTING_START:
                delay(400);
                Cutting();
                Running_Status = MOTOR_START;
                delay(400);
    break;
  case MOTOR_START:
                Motor_Start();
                Running_Status = PID_CONTROL;
                delay(100);
    break;
  case PID_CONTROL:
                Running_Status = PID_CONTROL;
                PID_Control();
    break;
  case OKURI_START2:
                Okuri_Start(Okuri_Delay_Time[1]);
                Running_Status = WAIT_INPUT;
                Serial.println(enc_count);
    break;
  default:
    break;
  }
  //Taping_Sequence//
  Serial_Check();

  delayMicroseconds(Polling_Time);
}

//pinAの割り込み処理
void enc_changedPinA(){

  if(digitalRead(M_pinA)){
    if(digitalRead(M_pinB)) --enc_count;
    else ++enc_count;
   }else{
    if(digitalRead(M_pinB)) ++enc_count;
    else --enc_count;
   }
   
}

//pinBの割り込み処理
void enc_changedPinB(){

  if(digitalRead(M_pinB)){
    if(digitalRead(M_pinA)) ++enc_count;
    else --enc_count;
   }else{
    if(digitalRead(M_pinA)) --enc_count;
    else ++enc_count;
   }
   
}

void Forward(){
  
    digitalWrite(M_pinFW, HIGH);
    digitalWrite(M_pinBR, LOW);
    digitalWrite(M_pinRE, LOW);

}

void Break(){
    
    digitalWrite(M_pinFW, LOW);
    digitalWrite(M_pinBR, HIGH);
    digitalWrite(M_pinRE, LOW);
    
}

void Reverse(){
  
    digitalWrite(M_pinFW, LOW);
    digitalWrite(M_pinBR, LOW);
    digitalWrite(M_pinRE, HIGH);

}

void Cutting(){
  
    digitalWrite(T_Cut1, LOW);
    digitalWrite(T_Cut2, HIGH);
    delay(200);
    digitalWrite(T_Cut2, LOW);
    digitalWrite(T_Cut1, HIGH);

}

void Okuri_Start(int Okuri_Delay_Time){

    digitalWrite(T_OkuriMotor, HIGH);
    digitalWrite(T_OkuriMotor_Middle, HIGH);
    delay(Okuri_Delay_Time);
    digitalWrite(T_OkuriMotor, LOW);
    digitalWrite(T_OkuriMotor_Middle, LOW);
    
}

void Motor_Start(){
    //P = (PWM_MAX - PWM_MIN) / target_count;
    P = 0.02;
    PID_On = 1;
    //
    Reverse();
}


void PID_Control(){
  static   int i              = 0;
  volatile int P_Control      = 0;
  volatile int I_Control      = 0;
  volatile int D_Control      = 0;
  float dt, preTime;

  volatile int p_dev          = 0;
  volatile int i_sum          = 0;
  volatile int d_val          = 0;
  volatile int pre_p_dev      = 0;
  
  int Output                  = 0;

  i++;
   //
   if(i == MicroSec_To_MilliSec * Debug_Time && Debug_On ==1){
      Serial.println("enc_count");
      Serial.println(enc_count);
      Serial.println("p_dev");
      Serial.println(p_dev);
      Serial.println("i_sum");
      Serial.println(i_sum);
      Serial.println("P_control");
      Serial.println(P_Control);
      i = 0;
      
   }
   dt = (micros() - preTime) / (MilliSec_To_Sec * MicroSec_To_MilliSec);
   preTime = micros();
   if(PID_On ==1 && enc_count < 0){
      p_dev              = target_count - (-1) * enc_count;
      i_sum             += p_dev        * dt;
      d_val              = (p_dev - pre_p_dev) / dt;
      if(p_dev >= 0){
        P_Control   = P            * p_dev     + PWM_MIN;
        I_Control   = I            * i_sum;
        D_Control   = D            * d_val;
        Output      = P_Control    + I_Control - D_Control;
        Reverse();
      }else if(p_dev <= 0){
              P_Control   = P            * (-1)      * p_dev  + PWM_MIN;
              I_Control   = I            * (-1)      * i_sum;
              D_Control   = D            * (-1)      * d_val;
              Output      = P_Control    + I_Control - D_Control;
              Forward();
      }
   }else if(PID_On ==1 && enc_count > 0){
      p_dev      = target_count        - enc_count;
      i_sum     += p_dev               * dt;
      d_val      = (p_dev - pre_p_dev) / dt;
        if(p_dev >= 0){
          P_Control   = P            * p_dev     + PWM_MIN;
          I_Control   = I            * i_sum;
          D_Control   = D            * d_val;
          Output      = P_Control    + I_Control - D_Control;
          Reverse();
        }else if(p_dev <= 0){
                P_Control   = P            * (-1)      * p_dev  + PWM_MIN;
                I_Control   = I            * (-1)      * i_sum;
                D_Control   = D            * (-1)      * d_val;
                Output      = P_Control    + I_Control - D_Control;
                Forward();
        }
   }
   pre_p_dev = p_dev;
   PWM_OutPutValue_Check(Output);
   //
   COrigin_Check(p_dev);
   //   
  i_sum     = 0;
  d_val     = 0;
  p_dev     = 0;
  pre_p_dev = 0;
  P_Control = 0;
  I_Control = 0;
  D_Control = 0;
  Output    = 0;
}

void PWM_OutPutValue_Check(int Output){
   if(PWM_MIN < Output && Output < PWM_MAX){
       Serial.println("PWM_MIN < Output && Output < PWM_MAX");
       Serial.println(Output);
       analogWrite(M_PWM, Output);
   }else if(Output <= PWM_MIN){
        Output = PWM_MIN;
        Serial.println("Output <= PWM_MIN");
        Serial.println(Output);
        analogWrite(M_PWM, Output);
    }else if(PWM_MAX < Output){
        Output = PWM_MAX;
        Serial.println("PWM_MAX < Output");
        Serial.println(Output);
        analogWrite(M_PWM, Output);
    }
}

void COrigin_Check(int p_deviation){
  #define Origin_Min        -50
  #define Origin_Max        50
  #define Origin_Check_Time 5000
  static int i=0;
  
  if(PID_On == 1 && Origin_Min <= p_deviation && p_deviation <= Origin_Max){
    i++;
    if(i >= Origin_Check_Time){
      Running_Status = OKURI_START2;
      i=0;
      Break();
      PID_On = 0;
    }
  }
}


void Serial_Check(){
  
      if(Serial.available() > 0){
          switch(Serial.read()){
             case 'a':    //Forwad Ratation
                enc_count = 0;
                Forward();
                break;

              case 'b':   //Breaking
              
                Break();
                break;

              case 'c':   //Reverse Rotation
                enc_count = 0;
                Reverse();
                break;

              case 'd':   //PID_Start
                enc_count = 0;
                //Running_Status = START_TAPING;
                Running_Status = MOTOR_START;
                break;

              case 'e':   //PID_Break
                enc_count = 0;
                Break();
                PID_On = 0;
                break;

              case 'f':   //PID_Start
                Okuri_Start(500);
                break;

              default:    //Default Mode
                break;
          }
   }
}






