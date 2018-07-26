#include <Servo.h> //define the servo library

#define trigPin1 11
#define echoPin1 12
#define trigPin2 6
#define echoPin2 7
#define trigPin3 8
#define echoPin3 9


Servo ssm; //create an instance of the servo object called ssm
Servo esc; //create an instance of the servo object called esc

int steering=90,velocity=90; //defining global variables to use later

//Kalman parameters for sensor fusion left
float P_fusionn = 1;   // Prediction covariance 
float P_fusion;        // Correctied covariance
float R_fusion = 0.9;  // Measurement noise
float y_fusion, y_fusionn;  //Distance correction & Distance prediction
float Q_fusion = 2;    // Processing noise

//Kalman parameters for sensor front
float P_frontn = 10;  // Prediction covariance 
float P_front;        // Correctied covariance
float R_front=89.176; // Measurement noise
float y_front;        // Filtered time duration
float duration_front, duration_frontn; // Duration correction & Duration prediction
float Q_front = (0.01 +0.9787)/0.0176 ;  // Processing noise

//Velocity PID parameters
float e_v=0, e_v1=0, e_v2=0;

float kp_v=2.1, ki_v=0, kd_v=2.8;

float u_v=0;
float k1_v=kp_v+ki_v+kd_v;
float k2_v=-kp_v-2*kd_v;
float k3_v=kd_v;

// Steering PID parameters
float e_s=0, e_s1=0, e_s2=0;

float kp_s=1.8, ki_s=0, kd_s=2.25;

float u_s=0;
float k1_s=kp_s+ki_s+kd_s;
float k2_s=-kp_s-2*kd_s;
float k3_s=kd_s;

//Dummy variable for convergence
int i=1;

void setup() {
  
  Serial.begin(9600);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  ssm.attach(2);    //define that ssm is connected at pin 10
  esc.attach(3);     //define that esc is connected at pin 11
  setVehicle(steering, velocity);
  delay(2000);

}

void loop() {
  //Local variable  
  float duration_1, duration_2, duration_3, distance_1, distance_2, distance_3;

  // Ultrasonic sensor data extract
  
  digitalWrite(trigPin1, LOW);
  digitalWrite(trigPin2, LOW);
  digitalWrite(trigPin3, LOW);
  
  delayMicroseconds(2);

  //Sensor1
  digitalWrite(trigPin1, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(trigPin1, LOW);
  
  duration_1=pulseIn(echoPin1, HIGH, 25000);
  
  delayMicroseconds(5);

  //Sensor3
  digitalWrite(trigPin3, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(trigPin3, LOW);
  
  duration_3=pulseIn(echoPin3, HIGH, 25000);

  delayMicroseconds(5);

  //Sensor2
  digitalWrite(trigPin2, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(trigPin2, LOW);

  duration_2=pulseIn(echoPin2, HIGH, 25000);

  // Calibration
  
  distance_1=duration_1*0.0177-1.9183;
  distance_2=duration_2*0.0175-0.8925;
  distance_3=duration_3*0.0177-1.4034;

  // Initialization to the first value for faster convergence
  
  if(i==1){
    y_front=duration_2;
    i++;
  }
  
  Kalman_fusion(distance_1,distance_3);  // Call Kalman fusion function for left side
  
  Kalman_front(duration_2);              // Call Kalman filter function for front
  
  y_front = duration_front*0.0175 - 0.8925;  // Calibration for front 
 
//Velocity PID controller
  e_v2=e_v1;

  e_v1=e_v;
  
  e_v=(y_front)-31.5;

  u_v=u_v+k1_v*e_v+k2_v*e_v1+k3_v*e_v2;


//Steering PID controller
  e_s2=e_s1;

  e_s1=e_s;
  
  e_s=31.5-y_fusion;

  u_s=u_s+k1_s*e_s+k2_s*e_s1+k3_s*e_s2;


//Control vehicle

  // Set velocity limit
  
  if((90+u_v) > 98 || y_front>=300){
    
    setVehicle((90+u_s), 98);
    
    }

  // Condition for larger distance
  else if(y_front<1){
    
    setVehicle(90+u_s,98);
  
  }

  // Set velocity limit for reverse
  
  else if((90+u_v) < 80){
    
    setVehicle((90+u_s), 80);
    
    }

  // Set velocity limit when the distance range is 30cm to 80cm
    
  else if(y_front>=30 && y_front<=80){
    
    setVehicle((90+u_s),min(97,(90+u_v)));
    
    }

  // Normal operation set by PID
    
  else{
      setVehicle(90+u_s, (90+u_v));
    }
  

//PRINT STATEMENTS
  
//  Serial.print(" ");
//  Serial.print(e_v);
//  Serial.print(" cm");
//  Serial.print("    ");
//  Serial.print(e_s);
//  Serial.print(" cm");
//  Serial.print(" ");
//  Serial.print(y_fusion);
//  Serial.print(" ");
//  Serial.println(y_front);
//  Serial.print(" ");
//  Serial.println(distance_3);
//  Serial.print(" ");
//  Serial.print(90+u_s);
//  Serial.print(" ");
//  Serial.println(90+u_v);
   delay(100);

}

//********************** Vehicle Control **********************//
//***************** Do not change below part *****************//
void setVehicle(int s, int v) 
{
  s=min(max(0,s),180);  //saturate steering command
  v=min(max(75,v),105); //saturate velocity command
  ssm.write(s); //write steering value to steering servo
  esc.write(v); //write velocity value to the ESC unit
}
//***************** Do not change above part *****************//

//KALMAN FILTER FOR SENSOR FUSION
void Kalman_fusion(float x, float y)
{
  float K_fusion;
  P_fusion = P_fusionn + Q_fusion;
  y_fusionn = x;
  K_fusion = (P_fusionn)/(P_fusionn + R_fusion);
  y_fusion = y_fusionn + K_fusion *(y - y_fusionn);
  P_fusion=(1-K_fusion)*P_fusionn;
  y_fusionn = y_fusion;
  P_fusionn = P_fusion;
   
}

//KALMAN FILTER for FRONT SENSOR
void Kalman_front(float x)
{
  float K_front;
  P_front = P_frontn + Q_front;
  duration_frontn = x;
  K_front = (P_frontn)/(P_frontn + R_front);
  duration_front = duration_frontn + K_front *(x - duration_frontn);
  P_front=(1-K_front)*P_front;
  duration_frontn = duration_front;
  P_frontn = P_front;
}

