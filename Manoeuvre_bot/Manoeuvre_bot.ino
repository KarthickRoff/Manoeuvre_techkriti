
#include <Ps3Controller.h>
#include<ESP32Servo.h>
#define enable_A_front_right 32
#define IN1_front_right 18
#define IN2_front_right 05
#define IN3_front_left 04
#define IN4_front_left 19
#define enable_B_front_left 33
#define enable_A_back_right 25
#define IN1_back_right 26
#define IN2_back_right 27
#define IN3_back_left 14
#define IN4_back_left 12
#define enable_B_back_left 13
#define Led1 23
#define Led2 22
#define Led3 21
#define Led4 3
#define precise_delay 100
#define general_delay 100
#define Max_speed 200
#define precise_speed 100
Servo claw_servo;
Servo lift_servo;
int LX_value; // left joystick wheel sliding control x-axis
int LY_value; // left joystick claw up and down control y-axis
int RX_value;// right joystick wheel  control x-axis
int RY_value; // right joystick wheel control y-axis
int lift_servo_value;
bool claw_servo_state = false;
int i=0;
int j=0;
int battery = 0;
void notify()
{
LX_value = (Ps3.data.analog.stick.lx);
LY_value = (Ps3.data.analog.stick.ly);
RX_value = (Ps3.data.analog.stick.rx);
RY_value = (Ps3.data.analog.stick.ry);
lift_servo_value = map(LY_value,-127,127,0,180);
analogWrite(enable_A_front_right,Max_speed);
analogWrite(enable_B_front_left,Max_speed);
analogWrite(enable_A_back_right,Max_speed);
analogWrite(enable_B_back_left,Max_speed);
  if (RY_value <= -50)  //Forward
  {
    Forward_Motion();
  }
  else if (RY_value >= 50) // Backward
  {
    Backward_Motion();
  }
  else if (RX_value >= 50)  // Rotation Clockwise
  {
    Rotation_Clockwise();
  }
  else if (RX_value <= -50)  // Rotation Counterclockwise
  {
   Rotation_Counterclockwise();
  }
  else   //Stop 
  {
    Stop();
  } 
  
  if( abs(Ps3.event.analog_changed.button.r2) ){
       i=(Ps3.data.analog.button.r2,DEC);
       Forward_Motion_with_SC();
   }
  if( abs(Ps3.event.analog_changed.button.l2) ){
      j=(Ps3.data.analog.button.l2,DEC);
      Backward_Motion_with_SC();
   }

   if (LX_value >= 50)  
  {
    Sideways_Motion_Right();
  }
  else if (LX_value <= -50) 
  {
   Sideways_Motion_Left();
  }
lift_servo.write(lift_servo_value);
  if( Ps3.event.button_down.r1 )
   {
    claw_servo_state=!claw_servo_state;
    if(claw_servo_state==true)
     {
      claw_servo.write(0);
     }
    else
     {
      claw_servo.write(180);
     }
    }
  if( Ps3.event.button_down.l1 )
  {
    Stop();
  } 
  if( Ps3.event.button_down.up )
  {
    Diagonal_Forward_Right();
    delay(precise_delay);
  }
  if( Ps3.event.button_down.right )
  {
    Diagonal_Backward_Right();
    delay(precise_delay);
  }
  if( Ps3.event.button_down.down )
  {
    Diagonal_Backward_Left();
    delay(precise_delay);
  }   
  if( Ps3.event.button_down.left )
  {
    Diagonal_Forward_Left();
    delay(precise_delay);
  }
    // precise controls
  if( Ps3.event.button_down.cross )
  {
    Backward_Motion_Precise();
    delay(precise_delay);
  }
  if( Ps3.event.button_down.square )
  {
   Rotation_Clockwise_Precise();
   delay(precise_delay); 
  }
  if( Ps3.event.button_down.triangle )
  {
    Forward_Motion_Precise();
    delay(precise_delay);
  }
  if( Ps3.event.button_down.circle )
  {
    Rotation_Counterclockwise_Precise();
    delay(precise_delay);
  }
  if( Ps3.event.button_down.select )
  {
   Sideways_Motion_Left_Precise();
   delay(precise_delay); 
  }
  if( Ps3.event.button_down.start )
  {
   Sideways_Motion_Right_Precise();
   delay(precise_delay);
  }
  if( battery != Ps3.data.status.battery )
 {
  battery = Ps3.data.status.battery;
  Serial.print("The controller battery is ");
  if( battery == ps3_status_battery_full )
  {
    Ps3.setPlayer(4);    
  }
  else if( battery == ps3_status_battery_high )
  {
    Ps3.setPlayer(3);     
  }
  else if( battery == ps3_status_battery_low) 
  {
    Ps3.setPlayer(2);
  }
  else if( battery == ps3_status_battery_dying )
  {
    Ps3.setPlayer(1); 
  }
  else if( battery == ps3_status_battery_shutdown ) 
  {
    Ps3.setPlayer(0);
    Ps3.setRumble(50.0, 1000);
    delay(1000);
  }
  }
}
void onConnect(){
    Serial.println("Connected.");
    digitalWrite(Led1,HIGH);
    digitalWrite(Led2,HIGH);
    digitalWrite(Led3,HIGH);
    digitalWrite(Led4,HIGH);
    Ps3.setRumble(50.0, 1000);
    delay(1000);
    Ps3.setRumble(50.0, 1000);
}
void onDisConnect()
{
  digitalWrite(Led1,LOW);
  digitalWrite(Led2,LOW);
  digitalWrite(Led3,LOW);
  digitalWrite(Led4,LOW);
}
void setup()
{
  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.attachOnDisconnect(onDisConnect);
  Ps3.begin();
  Serial.println("Ready.");
  pinMode(enable_A_front_right,OUTPUT);
  pinMode(IN1_front_right,OUTPUT); 
  pinMode(IN2_front_right,OUTPUT);
  pinMode(IN3_front_left,OUTPUT);
  pinMode(IN4_front_left,OUTPUT); 
  pinMode(enable_B_front_left,OUTPUT);
  pinMode(enable_A_back_right,OUTPUT);
  pinMode(IN1_back_right,OUTPUT); 
  pinMode(IN2_back_right,OUTPUT);
  pinMode(IN3_back_left,OUTPUT);
  pinMode(IN4_back_left,OUTPUT); 
  pinMode(enable_B_back_left,OUTPUT); 
  pinMode(Led1,OUTPUT);
  pinMode(Led2,OUTPUT);
  pinMode(Led3,OUTPUT);
  pinMode(Led4,OUTPUT);
  claw_servo.attach(2);
  lift_servo.attach(15);
}

void loop() {
  
}
void Forward_Motion()
 {
  digitalWrite(IN1_front_right,HIGH);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,HIGH);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,HIGH);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,HIGH);
  digitalWrite(IN4_front_left,LOW);      
 }
void Backward_Motion()
 {
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,HIGH);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,HIGH);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,HIGH);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,HIGH);
 }
 
void Sideways_Motion_Right()
 {  
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,HIGH);
  digitalWrite(IN3_front_left,HIGH);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,HIGH);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,HIGH); 
 }
 
void  Sideways_Motion_Left()
 {
  digitalWrite(IN1_front_right,HIGH);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,HIGH);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,HIGH);
  digitalWrite(IN3_back_left,HIGH);
  digitalWrite(IN4_front_left,LOW); 
 }
 
void Diagonal_Forward_Right()
 {
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,HIGH);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,HIGH);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,LOW);
 }
 
void Diagonal_Backward_Right() 
 {
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,HIGH);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,HIGH);
 }  
 
void Diagonal_Forward_Left()
 {
  digitalWrite(IN1_front_right,HIGH);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,HIGH);
  digitalWrite(IN4_front_left,LOW);  
 }
    
void Diagonal_Backward_Left()
 {
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,HIGH);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,HIGH);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,LOW); 
 }
   
void Rotation_Clockwise()
 {
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,HIGH);
  digitalWrite(IN3_front_left,HIGH);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,HIGH);
  digitalWrite(IN3_back_left,HIGH);
  digitalWrite(IN4_front_left,LOW); 
 }
    
void  Rotation_Counterclockwise()
  {
  digitalWrite(IN1_front_right,HIGH);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,HIGH);
  digitalWrite(IN1_back_right,HIGH);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,HIGH);
  }
  
void Stop()
 {
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,LOW);
 }
 
void Forward_Motion_with_SC() // speed control
 {
  analogWrite(IN1_front_right,i);
  digitalWrite(IN2_front_right,LOW);
  analogWrite(IN3_front_left,i);
  digitalWrite(IN4_front_left,LOW);
  analogWrite(IN1_back_right,i);
  digitalWrite(IN2_back_right,LOW);
  analogWrite(IN3_back_left,i);
  digitalWrite(IN4_front_left,LOW);      
  }
  
void Backward_Motion_with_SC() // Speed control
 {
  digitalWrite(IN1_front_right,LOW);
  analogWrite(IN2_front_right,j);
  digitalWrite(IN3_front_left,LOW);
  analogWrite(IN4_front_left,j);
  digitalWrite(IN1_back_right,LOW);
  analogWrite(IN2_back_right,j);
  digitalWrite(IN3_back_left,LOW);
  analogWrite(IN4_front_left,j);
  }
  
void Forward_Motion_Precise() // precise control
 {
  analogWrite(IN1_front_right,precise_speed);
  digitalWrite(IN2_front_right,LOW);
  analogWrite(IN3_front_left,precise_speed);
  digitalWrite(IN4_front_left,LOW);
  analogWrite(IN1_back_right,precise_speed);
  digitalWrite(IN2_back_right,LOW);
  analogWrite(IN3_back_left,precise_speed);
  digitalWrite(IN4_front_left,LOW);      
  }

void Backward_Motion_Precise() // Precise control
 {
  digitalWrite(IN1_front_right,LOW);
  analogWrite(IN2_front_right,precise_speed);
  digitalWrite(IN3_front_left,LOW);
  analogWrite(IN4_front_left,precise_speed);
  digitalWrite(IN1_back_right,LOW);
  analogWrite(IN2_back_right,precise_speed);
  digitalWrite(IN3_back_left,LOW);
  analogWrite(IN4_front_left,precise_speed);
  }  
  
void Rotation_Clockwise_Precise() // precise control
 {
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,precise_speed);
  digitalWrite(IN3_front_left,precise_speed);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,precise_speed);
  digitalWrite(IN3_back_left,precise_speed);
  digitalWrite(IN4_front_left,LOW); 
 } 

void  Rotation_Counterclockwise_Precise() // precise control
  {
  digitalWrite(IN1_front_right,precise_speed);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,precise_speed);
  digitalWrite(IN1_back_right,precise_speed);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,precise_speed);
  }

void Sideways_Motion_Right_Precise() // precise control
 {  
  digitalWrite(IN1_front_right,LOW);
  digitalWrite(IN2_front_right,precise_speed);
  digitalWrite(IN3_front_left,precise_speed);
  digitalWrite(IN4_front_left,LOW);
  digitalWrite(IN1_back_right,precise_speed);
  digitalWrite(IN2_back_right,LOW);
  digitalWrite(IN3_back_left,LOW);
  digitalWrite(IN4_front_left,precise_speed); 
 }
 
void  Sideways_Motion_Left_Precise() // precise control
 {
  digitalWrite(IN1_front_right,precise_speed);
  digitalWrite(IN2_front_right,LOW);
  digitalWrite(IN3_front_left,LOW);
  digitalWrite(IN4_front_left,precise_speed);
  digitalWrite(IN1_back_right,LOW);
  digitalWrite(IN2_back_right,precise_speed);
  digitalWrite(IN3_back_left,precise_speed);
  digitalWrite(IN4_front_left,LOW); 
 }     
