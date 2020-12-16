#include <AccelStepper.h>
// Define some steppers and the pins the will use
AccelStepper stepper1(AccelStepper::DRIVER, 5,4); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper stepper2(AccelStepper::DRIVER, 10,9);
AccelStepper stepper3(AccelStepper::DRIVER, 7,6);

const int ms3Pin   = 11;   // Ms3
const int ms2Pin   = 12;   // Ms2
const int ms1Pin   = 13;   // Ms1

float offset = 0;//((5/6)*PI);//((5/6)*PI);//
float rover_radius = 13;
float wheel_radius = 5;
float wheel_perimeter = wheel_radius * 2 * PI;
float cur_x = 0;
float cur_y = 0;
float cur_angle = offset;
float w = 0;
float v_n = 0;
float v_x = 0;
float v_y = 0;
float vxconst = 0;
float vyconst = 0;
float wconst = 0;
float v0 = 0;
float v1 = 0;
float v2 = 0;
float pos_0 = 0;
float pos_1 = 0;
float pos_2 = 0;
float time_limit = 0;
float time_accel = 0;
double time_pre = micros();
double timer_start = millis();
float total_elapsed = 0;
double time_elapsed = 0;
double time_cur =0;
float e_x = 0;
float e_y = 0;
float e_angle = 0;
float des_x = 0;
float des_y = 0;
float des_angle = 0;
float pre_x = 0;
float pre_y = 0;
float pre_angle = 0;
float kp = 0.1;
float ki = 0;
float kd = 0;
float e_lim = 0;
float pre_v_x = 0;
float pre_v_y = 0;
float pre_w = 0;
float v_max = 4;
float v_xtemp, v_ytemp, w_temp;
//float cerrorx, cerrory, cerrora;

void setup()
{  
    stepper1.setMaxSpeed(5000);
    stepper2.setMaxSpeed(5000);
    stepper3.setMaxSpeed(5000);
  
    stepper1.setSpeed(0);
    stepper2.setSpeed(0);
    stepper3.setSpeed(0); 

    Serial.begin(9600);

    pinMode(ms3Pin,OUTPUT);  
    pinMode(ms2Pin,OUTPUT);  
    pinMode(ms1Pin,OUTPUT); 

    digitalWrite(ms1Pin, HIGH);
    digitalWrite(ms2Pin, HIGH); 
    digitalWrite(ms3Pin, HIGH); 

 
}

void loop()
{   
    //delay(0.005);
    des_x = 50;  
    des_y = -50;      
    des_angle = 0; // PI/2;   

    if (cur_angle > PI) {
        cur_angle = cur_angle - 2 * PI;
    }

    time_cur = micros();
    
    time_elapsed = (time_cur - time_pre)/1000000;
    //Serial.println(time_elapsed);
    //Serial.println(time);
    time_pre = time_cur;
    
    cur_x = cur_x + (pre_v_x+v_x)/2 * time_elapsed;
    cur_y = cur_y + (pre_v_y+v_y)/2 * time_elapsed;
    cur_angle = cur_angle + (pre_w+w)/2 * time_elapsed + offset;
    //cur_angle = 0;

    //v_x = (pre_x - cur_x) /time_elapsed;
    //v_y = (pre_y - cur_y) /time_elapsed;
    //w   = (pre_angle - cur_angle) /time_elapsed;
    //pre_x     = cur_x;
    //pre_y     = cur_y;
    //pre_angle = cur_angle;
    
    e_x = des_x - cur_x;
    e_y = des_y - cur_y;
    e_angle = des_angle - cur_angle;
    //cerrorx = cerrorx + (e_x*time_elapsed);
    //cerrory = cerrory + (e_y*time_elapsed);
    //cerrora = cerrora + (e_angle*time_elapsed);

    pre_v_x = v_x;
    pre_v_y = v_y;
    pre_w = w;

    //v_x = e_x / time_elapsed;

    v_x = kp*e_x +  kd*v_x;
    v_y = kp*e_y +  kd*v_y;
    w =   kp*e_angle +  kd*w;
    //w = 0.1;
    Serial.println(v_y);
    
    //v_x = min(v_max,v_xtemp);
    //v_y = min(v_max,v_ytemp);
    //w = min(v_max,w_temp);
    
    v0 = -sin(cur_angle)        * v_x + cos(cur_angle)        * v_y + rover_radius * w;
    v1 = -sin(PI/3 - cur_angle) * v_x - cos(PI/3 - cur_angle) * v_y + rover_radius * w;
    v2 =  sin(PI/3 + cur_angle) * v_x - cos(PI/3 + cur_angle) * v_y + rover_radius * w;
    //Serial.println(v0);
    //v0 = -sin(PI/3)* v_x + cos(PI/3)* v_y + rover_radius*w;
    //v1 = 0 - 1*v_x + rover_radius*w;
    //v2 = sin(PI/3)*v_x + cos(PI/3)*v_y + rover_radius*w;

    //v0 = (v_x*(sqrt(3)*cos(cur_angle)-sin(cur_angle)) + v_y*(cos(cur_angle)-sqrt(3)*sin(cur_angle)) + w*2*rover_radius)/2;
    //v1 = v_x*sin(cur_angle) - v_y*cos(cur_angle) + w*rover_radius;
    //v2 = (v_x*(sqrt(3)*cos(cur_angle)-sin(cur_angle)) + v_y*(sqrt(3)*sin(cur_angle)+cos(cur_angle)) + w*2*rover_radius)/2;
    
  
    //Serial.println(v_x);
    //Serial.print(", ");
    //Serial.print(cur_y);
    //Serial.print(", ");r
    //Serial.print(cur_angle);
    //Serial.println(";");
    v0 = v0;
    v1 = v1;
    v2 = v2;
    
  
    stepper1.setSpeed(((v0)/(2*PI*5.2))*3200);//*3200*64
    stepper2.setSpeed(((v1)/(2*PI*5.2))*3200);
    stepper3.setSpeed(((v2)/(2*PI*5.2))*3200); 
    
    stepper1.runSpeed();
    stepper2.runSpeed();
    stepper3.runSpeed();

    if (abs(e_x) <= e_lim && abs(e_y) <= e_lim && abs(e_angle) <= e_lim){
        stepper1.setSpeed(0);
        stepper2.setSpeed(0);
        stepper3.setSpeed(0); 
    
        stepper1.runSpeed();
        stepper2.runSpeed();
        stepper3.runSpeed();

    } 

    

}
