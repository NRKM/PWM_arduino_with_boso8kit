  #define WAIT_INPUT     100
  #define START_TAPING   200
  #define OKURI_START1   300
  #define MOTOR_START0   400
  #define MOTOR_START1   500
  #define MOTOR_START2   600
  #define PID_CONTROL0   700
  #define PID_CONTROL1   800
  #define PID_CONTROL2   900
  #define CUTTING_START  1000
  #define OKURI_START2   1100
  
  volatile int enc_count        = 0;
  const int target_count[3]     = {230,230,4800};//{j,1,2,}
  const int delaytime[3]        = {700,310,300};//{f,1,2}
  int phase[3]                  = {0,1,2};
  double P                      = 0.02;
  double I                      = 0;
  double D                      = 0;
  double PWM_MAX                = 255;
  double PWM_MIN                = 170;
  int PID_On                    = 0;
  int Running_Status;
  
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
  
  int Debug_On                  = 1;
  int Debug_Time                = 500;
  int Polling_Time              = 10;
  int MicroSec_To_MilliSec      = 1000;
  int MilliSec_To_Sec           = 1000;
  
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
    
    switch(Running_Status){
    case START_TAPING:
                  Running_Status = OKURI_START1;
      break;
    case OKURI_START1:
                  Okuri_Start(delaytime[1]);
                  Running_Status = MOTOR_START1;
      break;
    case MOTOR_START1:
                  PID_Start();
                  PWM_MIN = 170;
                  Serial.println(PWM_MIN);
                  Running_Status = PID_CONTROL1;
      break;
    case PID_CONTROL1:
                  Running_Status = PID_CONTROL1;
                  PID_Control(target_count[1],phase[1]);
      break;
    case CUTTING_START:
                  Serial.println(enc_count);
                  Cutting();
                  Running_Status = MOTOR_START2;
                  delay(200);
      break;
    case MOTOR_START2:
                  PID_Start();
                  PWM_MIN = 10;
                  Serial.println(PWM_MIN);
                  Running_Status = PID_CONTROL2;
      break;
    case PID_CONTROL2:
                  Running_Status = PID_CONTROL2;
                  PID_Control(target_count[2],phase[2]);
      break;
    case OKURI_START2:
                  Okuri_Start(delaytime[2]);
                  Running_Status = WAIT_INPUT;
                  Serial.println(enc_count);
      break;
    case MOTOR_START0:
                  PID_Start();
                  Running_Status = PID_CONTROL0;
      break;
    case PID_CONTROL0:
                  Running_Status = PID_CONTROL0;
                  PID_Control(target_count[0],phase[0]);
      break;
    default:
      break;      
    }
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
    
      Serial.println("Cutting_On");
      digitalWrite(T_Cut1, LOW);
      digitalWrite(T_Cut2, HIGH);
      //delay(200);
      delay(500);
      digitalWrite(T_Cut2, LOW);
      digitalWrite(T_Cut1, HIGH);
      Serial.println("Cutting_Off");
  
  }
  
  void Okuri_Start(int delaytime){
  
      Serial.println("Okuri_Start1_On");
      digitalWrite(T_OkuriMotor, HIGH);
      digitalWrite(T_OkuriMotor_Middle, HIGH);
      delay(delaytime);
      digitalWrite(T_OkuriMotor, LOW);
      digitalWrite(T_OkuriMotor_Middle, LOW);
      Serial.println("Okuri_Start1_Off");
      
  }
  
  void Okuri_Start2(){
  
      Serial.println("Okuri_Start2_On");
      digitalWrite(T_OkuriMotor, HIGH);
      digitalWrite(T_OkuriMotor_Middle, HIGH);
      delay(300);
      digitalWrite(T_OkuriMotor, LOW);
      digitalWrite(T_OkuriMotor_Middle, LOW);
      Serial.println("Okuri_Start2_Off");
  
  }
  
  void PID_Start(){
      //P = (PWM_MAX - PWM_MIN) / target_count;
      P = 0.02;
      PID_On = 1;
      //
      if(Debug_On ==1){
        Serial.println("P_Value");
        Serial.println(P);
      }
      Forward();
  }
  
  
  void PID_Control(int target_count, int phase){
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
        i_sum             += p_dev * dt;
        d_val              = (p_dev - pre_p_dev) / dt;
        if(p_dev >= 0){
          P_Control   = P            * p_dev + PWM_MIN;
          I_Control   = I            * i_sum;
          D_Control   = D            * d_val;
          Output      = P_Control    + I_Control - D_Control;
          Reverse();
        }else if(p_dev <= 0){
                P_Control   = P            * (-1) * p_dev  + PWM_MIN;
                I_Control   = I            * (-1) * i_sum;
                D_Control   = D            * (-1) * d_val;
                Output      = P_Control    + I_Control - D_Control;
                Forward();
        }
     }else if(PID_On ==1 && enc_count > 0){
        p_dev      = target_count - enc_count;
        i_sum             += p_dev * dt;
        d_val              = (p_dev - pre_p_dev) / dt;
          if(p_dev >= 0){
            P_Control   = P            * p_dev + PWM_MIN;
            I_Control   = I            * i_sum;
            D_Control   = D            * d_val;
            Output      = P_Control    + I_Control - D_Control;
            Reverse();
          }else if(p_dev <= 0){
                  P_Control   = P            * (-1) * p_dev  + PWM_MIN;
                  I_Control   = I            * (-1) * i_sum;
                  D_Control   = D            * (-1) * d_val;
                  Output      = P_Control    + I_Control - D_Control;
                  Forward();
          }
     }
     pre_p_dev = p_dev;
     PWM_OutPutValue_Check(Output);
     COrigin_Check(p_dev,phase,Output);
     
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
  
  void COrigin_Check(int p_deviation,int phase,int output){
    #define Origin_Min -20
    #define Origin_Max 20
    #define Origin_Check_Time 1500
    static int i=0;
    
    if(PID_On == 1 && Origin_Min <= p_deviation && p_deviation <= Origin_Max){
      i++;
      if(i >= Origin_Check_Time){
        i=0;
        Break();
        PID_On = 0;
        if(phase ==1){
          Running_Status = CUTTING_START;
          output = 0;
        }else if(phase ==2){
          Running_Status = OKURI_START2;
          output = 0;
        }else {
          Running_Status = WAIT_INPUT;          
        }
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
                  Running_Status = START_TAPING;
                  break;
  
                case 'e':   //PID_Break
                  enc_count = 0;
                  Break();
                  PID_On = 0;
                  break;
  
                case 'f':   //PID_Break
                  Okuri_Start(delaytime[0]);
                  break;
  
                case 'g':   //Cutting
                  Cutting();
                  break;
  
                case 'h':   //Cut_On
                  Serial.println("Cutting_On");
                  digitalWrite(T_Cut1, LOW);
                  digitalWrite(T_Cut2, HIGH);
                  break;
  
                case 'i':   //Cut_Off
                  Serial.println("Cutting_Off");
                  digitalWrite(T_Cut2, LOW);
                  digitalWrite(T_Cut1, HIGH);
                  break;

                case 'j':   //PID_Start
                  enc_count = 0;
                  Running_Status = MOTOR_START0;
                  break;

                default:    //Default Mode
                  break;
            }
     }
  }
  
  
  
  
  
  
  

