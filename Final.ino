
#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

unsigned long previousMillis = 0;
const long interval = 20;

int diffPosition0,diffPosition1;
int prevPosition0,prevPosition1;

#define encoder0PinA 2
#define encoder0PinB 13
#define encoder1PinA 3
#define encoder1PinB 10

volatile long encoder0Pos = 0;
volatile long encoder1Pos = 0;

int read1;
int read2;
char ident;

//Right motor speed control

double Pk1 = 60;  //speed it gets there
double Ik1 = 4;
double Dk1 = 0.01;
double Setpoint1, Input1, Output1, Output1a;    // PID variables position

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

//Right motor speed control

double Pk2 = 60;  //speed it gets there
double Ik2 = 4;
double Dk2 = 0.01;
double Setpoint2, Input2, Output2, Output2a;    // PID variables velocity

PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);

void setup() {

  Serial.begin (57600);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(20);

  pinMode(9, OUTPUT);   //  motor PWMs
  pinMode(10, OUTPUT);

  pinMode(encoder0PinA, INPUT_PULLUP); 
  pinMode(encoder0PinB, INPUT_PULLUP); 
  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP); 
 
// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(2), doEncoder0A, CHANGE);
  
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(13), doEncoder0B, CHANGE);  

// encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(3), doEncoder1A, CHANGE);
  
// encoder pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(10), doEncoder1B, CHANGE);  
    
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

          diffPosition0 = abs(encoder0Pos - prevPosition0);       // calc change over time and make it always position with abs         
          prevPosition0 = encoder0Pos;
          diffPosition1 = abs(encoder1Pos - prevPosition1);       // calc change over time and make it always position with abs         
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
                  Setpoint2 = read2;
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
            Output1a = abs(Output1);
            //analogWrite(9, Output1a);                           // set PWM pins 
            //analogWrite(10, 0);
            analogWrite(6, Output1a);
            analogWrite(4, 0);
            analogWrite(9, 1023);
          }
          else if (Output1 > 0)                                  // decide which way to turn the motor
          { 
            Output1a = abs(Output1);
            //analogWrite(10, Output1a);  
            //analogWrite(9, 0);
            analogWrite(6, Output1a);
            analogWrite(4, 1023);
            analogWrite(9, 0);
          } 
          else if (Input1==0)
          {
            analogWrite(6, 0);
            analogWrite(4, 0);
            analogWrite(9, 0);
          }
          else
          {
            analogWrite(6, 0);
            analogWrite(4, 0);
            analogWrite(9, 0);
          } 


          // drive motor1
          if (Output2 < 0)                                       // decide which way to turn the motor
          {
            Output2a = abs(Output2);
            //analogWrite(9, Output1a);                           // set PWM pins 
            //analogWrite(10, 0);
            analogWrite(5, Output1a);
            analogWrite(8, 0);
            analogWrite(7, 1023);
          }
          else if (Output1 > 0)                                  // decide which way to turn the motor
          { 
            Output2a = abs(Output2);
            //analogWrite(10, Output1a);  
            //analogWrite(9, 0);
            analogWrite(5, Output1a);
            analogWrite(8, 1023);
            analogWrite(7, 0);
          } 
          else if (Input2==0)
          {
            analogWrite(5, 0);
            analogWrite(8, 0);
            analogWrite(7, 0);
          }
          else
          {
            analogWrite(5, 0);
            analogWrite(8, 0);
            analogWrite(7, 0);
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
