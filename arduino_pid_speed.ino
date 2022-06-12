#include <LiquidCrystal_I2C.h> 
#include <Wire.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#define cellpin A0 //P 
#define cellpin1 A1 //I
#define cellpin2 A2 //D
float counts =0 ;
const float mvc =0.004912;
double input=0;
float KIP;
int offset =0;
/*
This project implemented by the students listed below:
1.	Ayat Ahmed Khalaf
2.	Intisar Jomaa
3.	Hassan Farhan
Under supervision of
Dr. Raaed Sadoon
in The University of Technology, Control and systems engineering department, Mechatronics Branch – Spring 2022
 */
String mySt = "";
char myChar;
boolean stringComplete = false;  // whether the string is complete
boolean motor_start = false;
const byte pin_a = 2;   //for encoder pulse A
const byte pin_b = 3;   //for encoder pulse B
const byte pin_fwd = 4; //for H-bridge: run motor forward
const byte pin_bwd = 5; //for H-bridge: run motor backward
const byte pin_pwm = 6; //for H-bridge: motor speed
int encoder = 0;
int m_direction = 0;
int sv_speed = 100;     //this value is 0~255
double pv_speed = 0;
double e_speed = 0; //error of speed = pv_speed
double e_speed_pre = 0;  //last error of speed
double e_speed_sum = 0;  //sum error of speed
double pwm_pulse = 0;     //this value is 0~255
double kp = 0;
double ki = 0;
double kd = 0;
int timer1_counter; //for timer
int i=0;


void setup() {
  lcd.init(); //display initialization
  lcd.backlight();
  lcd.begin(16, 4);
  lcd.print("RPM:"); 
  //lcd.print(pv_speed);

  
  //lcd.setCursor(10,0);
pinMode(pin_a,INPUT_PULLUP);
  pinMode(pin_b,INPUT_PULLUP);
  pinMode(pin_fwd,OUTPUT);
  pinMode(pin_bwd,OUTPUT);
  pinMode(pin_pwm,OUTPUT);
  //pinMode(A3, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(pin_a), detect_a, RISING);
  // start serial port at 9600 bps:
  Serial.begin(9600);
  //--------------------------timer setup
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  timer1_counter = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)

  
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
  //--------------------------timer setup
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  analogWrite(pin_pwm,0);   //stop motor
  digitalWrite(pin_fwd,0);  //stop motor
  digitalWrite(pin_bwd,0);  //stop motor
}

void loop() {

 lcd.setCursor(4,0);
 delay(200);
lcd.print(pv_speed);

  if (stringComplete) {
    // clear the string when COM receiving is completed
    mySt = "";  //note: in code below, mySt will not become blank, mySt is blank until '\n' is received
    stringComplete = false;
  }
    digitalWrite(pin_fwd,1);      //run motor run forward
    digitalWrite(pin_bwd,0);
    motor_start = true;
  float voltage = (pv_speed/30); //find the voltage from the motor specification sheet
  KIP = analogRead(A0)*mvc;
  kp=KIP/4.95;
  ki = (analogRead(A1)*mvc)/4.95;
  kd = (analogRead(A2)*mvc)/24.8;
  //kd=input/24.8;
  
  Serial.print("kp=");
Serial.println(String(kp));
  Serial.print("kd=");
Serial.println(String(ki));
Serial.println(String(kd));
  Serial.print("Voltage: ");
  Serial.print(voltage);//print the voltge
  Serial.println("V");
  Serial.print("V=");
  lcd.setCursor(12,0);
  lcd.print("V:");
  lcd.setCursor(14,0);
  lcd.print(voltage);//LCD OUT voltge
  lcd.setCursor(0,1);
  lcd.print("KP");
    lcd.setCursor(4,1);
 lcd.print(kp); 
  lcd.setCursor(0,2);
 lcd.print("KI");
    lcd.setCursor(4,2);
 lcd.print(ki); 
  lcd.setCursor(0,3);
  lcd.print("KD");
  lcd.setCursor(4,3);
  lcd.print(kd); 
}

void detect_a() {
  encoder+=1; //increasing encoder at new pulse
  m_direction = digitalRead(pin_b); //read direction of motor
}
ISR(TIMER1_OVF_vect)        // interrupt service routine - tick every 0.1sec
{
  TCNT1 = timer1_counter;   // set timer
  pv_speed = (600.0*(encoder/200.0)/0.1)/10;  //calculate motor speed, unit is rpm
  encoder=0;
  //print out speed
  if (Serial.available() <= 0) {
    //Serial.print("speed");
    Serial.println(pv_speed);         //Print speed (rpm) value 


     }


   
  //PID program

    e_speed = pv_speed;
      Serial.print("e_speed=");
Serial.println(String(e_speed));

    pwm_pulse = e_speed*kp + e_speed_sum*ki + (e_speed - e_speed_pre)*kd;
          Serial.print("pwm_pulse=");
Serial.println(String(pwm_pulse));
    e_speed_pre = e_speed;  //save last (previous) error
    e_speed_sum += e_speed; //sum of error
           Serial.print("e_speed_pre=");
Serial.println(String(e_speed_pre)); 
Serial.print("e_speed_sum=");
Serial.println(String(e_speed_sum));   
    if (e_speed_sum >2500) e_speed_sum = 2500;
    if (e_speed_sum <-2500) e_speed_sum = -2500;


  
  //update new speed
  if (pwm_pulse <255 & pwm_pulse >0){
    analogWrite(pin_pwm,pwm_pulse);  //set motor speed  
  }
  else{
    if (pwm_pulse>255){
      analogWrite(pin_pwm,255);
    }
    else{
      analogWrite(pin_pwm,0);
    }
  }

}
 
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    if (inChar != '\n') {
      mySt += inChar;
    }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;

    }
  }
}
