#include <Servo.h> //servo library

#define send
#define TIMEOUT_VAL   25

// IR settings
//IRrecv irrecv(12);
//decode_results results;
//unsigned long val;

Servo myservo; // create servo object to control servo
int timeout;      // Create a timer object for timeouts


int Echo = A4;  
int Trig = A5;


// Odometry functionality (in mm)
float x_pos = 0.0;
float y_pos = 0.0;

float angle = 90;

int y_mult = 1;
int x_mult = 1;
int timer = 0;

float rate = 28.4;      // cm / s approx
float ang_rate = .094;  // Degrees / ms approx


int servoPosition = 100;
boolean servoRight = false;

// Pins and other constants
int in1 = 6;
int in2 = 7;
int in3 = 8;
int in4 = 9;
int ENA = 5;
int ENB = 11;
int SPEED = 100;
int ROT = 120;
int rightDistance = 0,leftDistance = 0,middleDistance = 0 ;

// Helper function for sign
int sign(float num) {
  if (num < 0) return -1;
  if (num > 0) return 1;

  return 0;
}

void _mForward()
{
  analogWrite(ENA,SPEED);
  analogWrite(ENB,SPEED);
  digitalWrite(in1,HIGH);//digital output
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  Serial.println("go forward!");
}

void _mBack()
{
  analogWrite(ENA,SPEED);
  analogWrite(ENB,SPEED);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  Serial.println("go back!");
}

void _mleft()
{
  analogWrite(ENA,ROT);
  analogWrite(ENB,ROT);
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW); 
  Serial.println("go left!");
}

void _mright()
{
  analogWrite(ENA,ROT);
  analogWrite(ENB,ROT);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
  Serial.println("go right!");
} 
void _mStop()
{
  digitalWrite(ENA,LOW);
  digitalWrite(ENB,LOW);
  Serial.println("Stop!");
} 

// For simplicity's sake, this will rotate right
void _mRotateRight(float ang) {
  float d = ang / ang_rate;
  _mright();
  delay(d);
  _mStop();
}


// This will rotate left
void _mRotateLeft(float ang) {
  float d = ang / ang_rate;
  _mleft();
  delay(d);
  _mStop();
}

void rotate_to_angle(float ang) {
  float ang_diff = angle - ang;
  _mright();
  while (abs(ang_diff) > 10) {
    delay(20);
    updateAngle(1, 20);
    ang_diff = angle - ang;
  }
}

 /*Ultrasonic distance measurement Sub function*/
int Distance_test()   
{
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);   
  float Fdistance = pulseIn(Echo, HIGH);  
  Fdistance= Fdistance/58;       
  return (int)Fdistance;
}  

void setup() 
{ 
  myservo.attach(3);// attach servo on pin 3 to servo object
  Serial.begin(9600);     
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  timer = millis();
  _mStop();
} 

// Subroutine to find the best direction of travel
int  optimalPath() {

  int max_dist = 0;
  int best_dir = 0;
  for (int i = 10; i < 180; i += 10) {
    myservo.write(i);
    delay(200);
    int dist = Distance_test();     // Check distances of ultrasonic sensor at increments of 10 degrees
    Serial.println(dist);
    if (dist > max_dist) {          // Find the farthest distance
      max_dist = dist;
      best_dir = i;
    }
  }
  return best_dir;
}

void updateOdometry() {
  x_pos += sin(angle) * (rate / 2.0) * x_mult;
  y_pos += cos(angle) * (rate / 2.0) * y_mult;

  Serial.println(x_pos);
  
}

void updateAngle(int dir, int wait) {
  
  angle += (wait * ang_rate) * dir;
  if (angle < 0) {
    angle = angle + 359;
  } else if (angle > 359) {
    angle = angle - 360; 
  }

  // y_mult
  if (angle <= 90 && angle >= 0 || angle >= 270) {
    y_mult = -1;
  } else {
    y_mult = 1;
  }

  if (angle >= 0 && angle <= 180) {
    x_mult = 1;
  } else {
    x_mult = -1;
  }

  Serial.print("Angle value: ");
  Serial.println(angle);
  
}


// Subroutine to rotate the bot towards it's origin
void turn_to_origin() {

  Serial.println("Turn to angle of origin!");
  float angle_of_origin = atan(x_pos / y_pos);
  if (angle >= 90 && angle <= 270) {
    rotate_to_angle(359 - angle_of_origin);
  } else {
    rotate_to_angle(angle_of_origin);
  }
  
}

void loop() 
{ 
    myservo.write(servoPosition);//setservo position according to scaled value
    
    delay(500); // Half second delay
    if (millis() - timer >= 50000) {
      // turn_to_origin();
      timer = millis();
      // _mForward();
    }
    
    // Update odometry
    updateOdometry();
    
    middleDistance = Distance_test();
    #ifdef send
    Serial.print("middleDistance=");
    Serial.println(middleDistance);
    #endif

    Serial.println(timeout);
    if (timeout >= TIMEOUT_VAL) {
      _mBack();
      delay(480);
      middleDistance = 0;
    }
    if(middleDistance<=40)
    {
      timeout = 0;
      _mStop();
//      delay(500);	  
//      myservo.write(20);//10°-180°          
//      delay(1000);      
//      rightDistance = Distance_test();

      #ifdef send
      Serial.print("rightDistance=");
      Serial.println(rightDistance);
      #endif

//      delay(500);
//       myservo.write(90);              
//      delay(1000);                                                  
//      myservo.write(180);              
//      delay(1000); 
//      leftDistance = Distance_test();

      int best_dir = optimalPath();
      Serial.print("Best Direction:");
      Serial.println(best_dir);

      int rotateAngle = 100 - best_dir;

      if (rotateAngle < 0) {

        _mRotateLeft(abs(rotateAngle));
      } else {
        _mRotateRight(abs(rotateAngle));
      };
//      #ifdef send
//      Serial.print("leftDistance=");
//      Serial.println(leftDistance);
//      #endif
//
//      delay(500);
//      myservo.write(90);              
//      delay(1000);
//      if(rightDistance>leftDistance)  
//      {
//        _mright();
//        updateAngle(1, 860);
//        delay(360);
//       }
//       else if(rightDistance<leftDistance)
//       {
//        _mleft();
//        updateAngle(-1, 860);
//        delay(360);
//       }
//       else if((rightDistance<=40)||(leftDistance<=40))
//       {
//        _mBack();
//        delay(500);
//       }
//       else
//       {
//        _mForward();
//        
//       }
   }  
    else
        _mForward();

    // This rotates the ultrasonic sensor to better detect oncoming collisions 
    if (servoRight == false) {
      servoPosition += 20;                  
    } else {
      servoPosition -= 20;
    }

    if (servoPosition >= 110) {
      servoRight = true;
    } else if (servoPosition <= 90) {
      servoRight = false;
    }
    timeout += 1;
}


