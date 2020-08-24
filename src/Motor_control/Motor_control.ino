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
volatile int encoder_count_L = 0;
volatile int encoder_count_R = 0;


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
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.print("Left ");
  Serial.print(encoder_count_L);
  Serial.print(" Right ");
  Serial.print(encoder_count_R);
  Serial.println();
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
