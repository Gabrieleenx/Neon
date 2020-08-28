// motor pins
const int ENA = 5; // Left motor PWM L298N
const int ENB = 6; // Right motor PWM
// direction for left motor 
const int IN1 = 4;
const int IN2 = 7;
// direction for right motor
const int IN3 = 8;
const int IN4 = 9;

// encoder pins
// left
const int CHA_L = 11; // Channel A Left
const int CHB_L = 3; // interrupt pin
// right
const int CHA_R = 10;
const int CHB_R = 2; // interrupt pin

// positive forward
volatile long encoder_count_L = 0; // long to not overflow
volatile long encoder_count_R = 0;

long previous_count_L = 0;
long previous_count_R = 0;

// time
unsigned long previouMicros = 0;

// ste size in meters for encoder 
const float step_size = 0.102*PI/312;

// PI controller error
float error_L = 0; 
float error_R = 0; 

float ref_speed_L = 0.0;
float ref_speed_R = 0.0;

void setup() {
  // put your setup code here, to run once:
  // define pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(CHA_L, INPUT);
  pinMode(CHB_L, INPUT);
  pinMode(CHA_R, INPUT);
  pinMode(CHB_R, INPUT);

  // Internal spinn up the interupt pins
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH); 

  attachInterrupt(digitalPinToInterrupt(CHB_L), flag_L, RISING);
  attachInterrupt(digitalPinToInterrupt(CHB_R), flag_R, RISING);
  
  // Serial monitor
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMicros = micros();

  // read from serial port
  if(Serial.available()){
    // Inspiered from https://eecs.blog/sending-multiple-values-over-serial-to-arduino/
    String recive = "";
    String str_array[2];
    while (Serial.available()){
      delay(2);
      char ch = Serial.read();
      recive += ch;
    }
    int Start_idx = 0;
    int ready_ = 0;
    int array_index = 0;
    
    for (int i = 0; i < recive.length(); i++){
      if (recive.charAt(i) == ';'){
        Start_idx = i+1; // start of message
        ready_ = 1;
      }
      if (ready_ == 1 && recive.charAt(i) == ','){
        str_array[array_index] = recive.substring(Start_idx, i);
        
        Start_idx = i+1;
        array_index++;
        if (array_index >= 2){
          break;
        }
      }
    }
    
    ref_speed_L = str_array[0].toInt() / 1000.0;
    ref_speed_R = str_array[1].toInt() / 1000.0;
  }

  
  float deltaTime = (currentMicros - previouMicros)/(1e6);
  
  // current speed
  float speed_L = (encoder_count_L - previous_count_L)*step_size/deltaTime;
  previous_count_L = encoder_count_L;

  float speed_R = (encoder_count_R - previous_count_R)*step_size/deltaTime;
  previous_count_R = encoder_count_R;
  
  // PI controller
  int pwm_L = (ref_speed_L - speed_L)*1000 + error_L;  
  error_L += 80*(ref_speed_L - speed_L);

  if (ref_speed_L == 0.0){
    pwm_L = 0;
    }
  

  if (error_L > 255){
    error_L = 255;
  }
  else if (error_L < -255){
    error_L = -255;
  }

  if (pwm_L < 0){
    pwm_L = -pwm_L;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  if (pwm_L > 255){
    pwm_L = 255;
  }


  int pwm_R = (ref_speed_R - speed_R)*1000 + error_R;  
  error_R += 80*(ref_speed_R - speed_R);
  if (ref_speed_R == 0.0){
    pwm_R = 0;
  }
  
  if (error_R > 255){
    error_R = 255;
  }
  else if (error_R < -255){
    error_R = -255;
  }

  if (pwm_R < 0){
    pwm_R = -pwm_R;
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else{
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  
  if (pwm_R > 255){
    pwm_R = 255;
  }
  
  analogWrite(ENA, pwm_L);
  analogWrite(ENB, pwm_R);

  // send toserial port 
  Serial.print("HelloNeon ");
  Serial.print(encoder_count_L);
  Serial.print(",");
  Serial.print(encoder_count_R);
  Serial.println();

  // makes it run at about 30 Hz
  int delayTime = 33333 - (currentMicros - previouMicros); 
  if (delayTime > 0){
    delayMicroseconds(delayTime);
    }
  previouMicros = currentMicros;
  
}



void flag_L(){
  // triggerd for interupt
  if (digitalRead(CHA_L)){
    encoder_count_L -= 1;
    }
  else{
    encoder_count_L += 1;
    }
  }


void flag_R(){
  // triggerd for interupt 
  if (digitalRead(CHA_R)){
    encoder_count_R += 1;
    }
  else{
    encoder_count_R -= 1;
    }
  }
