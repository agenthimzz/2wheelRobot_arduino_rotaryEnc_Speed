
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

unsigned long previousMillis = 0;
const long interval = 20;

int diffPosition0,diffPosition1;
int prevPosition0,prevPosition1;

#define encoder0PinA 3
#define encoder0PinB 10
#define encoder1PinA 2
#define encoder1PinB 13

#define in1A 7
#define in1B 8 
#define pwm1 5
#define in2A 9
#define in2B 4
#define pwm2 6

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

int read1;
int read2;
char ident;

//Right motor speed control

double Pk1 = 40;  //speed it gets there
double Ik1 = 10;
double Dk1 = 0.0001;
double Setpoint1, Input1, Output1, Output1a;    // PID variables velocity0

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

//Right motor speed control

double Pk2 = 40;  //speed it gets there
double Ik2 = 10;
double Dk2 = 0.0001;
double Setpoint2, Input2, Output2, Output2a;    // PID variables velocity1

PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

void setup() {

  Serial.begin (57600);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(20);

  pinMode(in1A, OUTPUT);   //  motor PWMs
  pinMode(in1B, OUTPUT);
  pinMode(pwm1, OUTPUT);   
  pinMode(in2A, OUTPUT);
  pinMode(in2B, OUTPUT); 
  pinMode(pwm2, OUTPUT);

  pinMode(encoder0PinA, INPUT_PULLUP); 
  pinMode(encoder0PinB, INPUT_PULLUP); 
  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP); 
 
// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoder0A, CHANGE);
  
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoder0B, CHANGE);  

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);  
    
}

void loop(){ //Do stuff here

       unsigned long currentMillis = millis();
       if (currentMillis - previousMillis >= interval) {  // start timed event
          previousMillis = currentMillis;

          Serial.print(encoder0Pos);
          Serial.print(" , ");  
          Serial.print(encoder1Pos);
//          Serial.print(encoder1Pos);
//          Serial.print(" , ");

          // calculate velocity - change in encoder counts over time

          diffPosition0 = ((encoder0Pos - prevPosition0));       // calc change over time and make it always position with abs         
          prevPosition0 = encoder0Pos; 
          diffPosition1 = ((encoder1Pos - prevPosition1));       // calc change over time and make it always position with abs         
          prevPosition1 = encoder1Pos;          
          Serial.print(" , ");                   
          Serial.print(diffPosition0);
          Serial.print(" , "); 
          Serial.print(diffPosition1);

          
          if (Serial.available()>2) {  // check for enough serial data
              // read the incoming byte:
              read1 = Serial.parseInt();   // vel0
              read2 = Serial.parseInt();   // vel1
              ident = Serial.read();         

              if (ident == 'a') {
                  Setpoint1 = read1;
                  Serial.print("input0 = ");
                  Serial.println(read1);
                  Setpoint2 = read2;
                  Serial.print("input1 = ");
                  Serial.println(read2);
              }
              
          } // end of serial read

          Input1 = diffPosition0;
          PID1.Compute();

          Input2 = diffPosition1;
          PID2.Compute();

          Serial.print(" , ");
          Serial.print(Output1);

          Serial.print(" , ");
          Serial.println(Output2);

          // drive motor0
          
          if (Output1 < 0)                                       // decide which way to turn the motor
          {
            //Output1a = abs(Output1);
            //analogWrite(9, Output1a);                           // set PWM pins 
            //analogWrite(10, 0);
            analogWrite(pwm1, abs(Output1));
            analogWrite(in1A, 0);
            analogWrite(in1B, 1023);
          }
          else if (Output1 > 0)                                  // decide which way to turn the motor
          { 
            //Output1a = abs(Output1);
            //analogWrite(10, Output1a);  
            //analogWrite(9, 0);
            analogWrite(pwm1, abs(Output1));
            analogWrite(in1A, 1023);
            analogWrite(in1B, 0);
          } 
          else if (Input1==0)
          {
            analogWrite(pwm1, 0);
            analogWrite(in1A, 0);
            analogWrite(in1B, 0);
          }
          else
          {
            analogWrite(pwm1, 0);
            analogWrite(in1A, 0);
            analogWrite(in1B, 0);
          } 


          // drive motor1
          if (Output2 < 0)                                       // decide which way to turn the motor
          {
            //Output2a = abs(Output2);
            //analogWrite(9, Output1a);                           // set PWM pins 
            //analogWrite(10, 0);
            analogWrite(pwm2, abs(Output2));
            analogWrite(in2A, 0);
            analogWrite(in2B, 1023);
          }
          else if (Output1 > 0)                                  // decide which way to turn the motor
          { 
            //Output2a = abs(Output2);
            //analogWrite(10, Output1a);  
            //analogWrite(9, 0);
            analogWrite(pwm2, abs(Output2));
            analogWrite(in2A, 1023);
            analogWrite(in2B, 0);
          } 
          else if (Input2==0)
          {
            analogWrite(pwm2, 0);
            analogWrite(in2A, 0);
            analogWrite(in2B, 0);
          }
          else
          {
            analogWrite(pwm2, 0);
            analogWrite(in2A, 0);
            analogWrite(in2B, 0);
          }                 
             
                      
      }      // end of timed event
  
}  // end of main loop



// encoder 1

void doEncoder0A(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos - 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos - 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;          // CCW
    }
  } 
}

void doEncoder0B(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos - 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos - 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos + 1;          // CCW
    }
  } 
}


// encoder 2

void doEncoder1A(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  } 
}

void doEncoder1B(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos + 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  } 
}
