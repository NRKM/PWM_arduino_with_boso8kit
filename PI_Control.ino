volatile int enc_count        = 0;
const int target_count        = 3600;
double P                      = 0; //0.03
double I                      = 0;
double PWM_MAX                = 255;
double PWM_MIN                = 65;

const int M_pinA              = 2; //A相割り込み0
const int M_pinB              = 3; //B相割り込み1
const int M_pinFW             = 8; //Foword Trigger
const int M_pinBR             = 9; //Break Trigger
const int M_pinRE             = 10; //Reverse Trigger
const int M_PWM               = 11; //バー開

int Debug_On                  = 1;
int Debug_Time                = 500;
int Polling_Time              = 10;
int MicroSec_To_MilliSec      = 1000;
int MilliSec_To_Sec           = 1000;

void setup() {

  Serial.begin(9600);
  pinMode(M_pinA              , INPUT);
  pinMode(M_pinB              , INPUT);
  pinMode(M_pinFW             , OUTPUT);
  pinMode(M_pinBR             , OUTPUT);
  pinMode(M_pinRE             , OUTPUT);
  pinMode(M_PWM               , OUTPUT);
  attachInterrupt(0           , enc_changedPinA, CHANGE);//pinAの信号変化に合わせて割り込み処理
  attachInterrupt(1           , enc_changedPinB, CHANGE);//pinBの信号変化に合わせて割り込み処理
  
}

void loop() {

  PID_Control();
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

void PID_Start(){

    P = (PWM_MAX - PWM_MIN) / target_count;
    //P = 0.03;
    //
    if(Debug_On ==1){
      Serial.println("P_Value");
      Serial.println(P);
    }

    digitalWrite(M_pinFW, LOW);
    digitalWrite(M_pinBR, LOW);
    digitalWrite(M_pinRE, HIGH);

}


void PID_Control(){
  static   int i              = 0;
  static   int PID_On         = 0;
  volatile int P_Control      = 0;
  volatile int I_Control      = 0;
  volatile int enc_deviation  = 0;
  float dt, preTime;

  volatile int i_sum          = 0;
  int Output                  = 0;
  PID_On = 1;

  i++;
   //
   if(i == MicroSec_To_MilliSec *Debug_Time && Debug_On ==1){
      Serial.println("enc_count");
      Serial.println(enc_count);
      Serial.println("enc_deviation");
      Serial.println(enc_deviation);
      Serial.println("i_sum");
      Serial.println(i_sum);
      Serial.println("P_control");
      Serial.println(P_Control);
      i = 0;
   }
   if(PID_On ==1 || enc_count < 0){
    enc_deviation      = target_count + enc_count;
    dt = (micros() - preTime) / (MilliSec_To_Sec * MicroSec_To_MilliSec);
    preTime = micros();
    i_sum             += enc_deviation * dt;
    if(enc_deviation >= 0){
     P_Control   = P            * enc_deviation + PWM_MIN;
     I_Control   = I            * i_sum;
     Output      = P_Control    + I_Control;
     Reverse();
     }else if(enc_deviation <= 0){
       P_Control   = P            * (-1) * enc_deviation  + PWM_MIN;
       I_Control   = I            * (-1) * i_sum;
       Output      = P_Control    + I_Control;
       Forward();
     }
   }else if(PID_On ==1 || enc_count > 0){
    enc_deviation      = target_count - enc_count;
    dt = (micros() - preTime) / (MilliSec_To_Sec * MicroSec_To_MilliSec);
    preTime = micros();
    i_sum             += enc_deviation * dt;
    if(enc_deviation >= 0){
     P_Control   = P            * enc_deviation + PWM_MIN;
     I_Control   = I            * i_sum;
     Output      = P_Control    + I_Control;
     Reverse();
     }else if(enc_deviation <= 0){
       P_Control   = P            * (-1) * enc_deviation + PWM_MIN;
       I_Control   = I            * (-1) * i_sum;
       Output      = P_Control    + I_Control;
       Forward();
     }
   }
   PWM_OutPutValue_Check(Output);
}

void PWM_OutPutValue_Check(int Output){
   if(PWM_MIN < Output || Output < PWM_MAX){
       analogWrite(M_PWM, Output);
   }else if(Output < PWM_MIN){
        Output = PWM_MIN;
        analogWrite(M_PWM, Output);
    }else if(PWM_MAX < Output){
        Output = PWM_MAX;
        analogWrite(M_PWM, Output);
    }
}
void Serial_Check(){
  
      if(Serial.available() > 0){
          switch(Serial.read())
          {
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

              case 'd':   //PWM
                enc_count = 0;
                PID_Start();
                break;

              case 'e':   //PWM
                enc_count = 0;
                break;

              default:    //Default Mode
                break;
          }
   }
}



