volatile int enc_count      = 0;
const int target_count      = 4800;
//const int target_count    = 24000;
volatile int p_diff         = 0;
volatile int i_sum          = 0;
volatile int P_Control      = 0;
volatile int I_Control      = 0;
int P_control               = 0;
int I_control               = 0;
int Output                  = 0;
int PWM_On                  = 0;
  
const int M_pinA              = 2; //A相割り込み0
const int M_pinB              = 3; //B相割り込み1
const int M_pinFW             = 8; //Foword Trigger
const int M_pinBR             = 9; //Break Trigger
const int M_pinRE             = 10; //Reverse Trigger
const int M_PWM               = 11; //バー開
//const double P                = 0.0275; //1/48
const double P                = 0.03; //1/48
const double I                = 0.027; //1/48


int Running_Status;
int WAIT_INPUT = 100;

void setup() {
  Running_Status = WAIT_INPUT;

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
   //
   Serial.println("Now_Count");
   Serial.println(enc_count);
   Serial.println("p_diff");
   Serial.println(p_diff);
   Serial.println("i_sum");
   Serial.println(i_sum);
   Serial.println("P_control");
   Serial.println(P_Control);
   Serial.println("I_control");
   Serial.println(I_Control);
   //
   if(PWM_On ==1 || enc_count < 0){
    p_diff      = target_count + enc_count;
    if(p_diff >= 0){
     P_Control   = P            * p_diff + 50;
     Output      = P_Control;
     Reverse();
     }else if(p_diff <= 0){
       P_Control   = P            * (-1) * p_diff  + 50;
       Output      = P_Control;
       Forward();      
     }
   }else if(PWM_On ==1 || enc_count > 0){
    p_diff      = target_count - enc_count;
    if(p_diff >= 0){
     P_Control   = P            * p_diff + 50;
     Output      = P_Control;
     Reverse();
     }else if(p_diff <= 0){
       P_Control   = P            * (-1) * p_diff + 50;
       Output      = P_Control;
       Forward();      
     }
   }
   analogWrite(M_PWM, Output);

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
                PWM();
                break;

              case 'e':   //PWM
                enc_count = 0;
                //PWM();
                break;

              default:    //Default Mode
                break;
          }
   }
   

   //if(enc_count >= 4800||enc_count <= -4800){

   //   Break();
   //   Serial.println(enc_count);
      
   //}    
     
   delayMicroseconds(1);
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

void PWM(){
    i_sum          = 0;
    PWM_On         = 1;
    //
    digitalWrite(M_pinFW, LOW);
    digitalWrite(M_pinBR, LOW);
    digitalWrite(M_pinRE, HIGH);

}

