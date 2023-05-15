#include <ros.h>
#include <std_msgs/Int32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

const int MOE = 8; //Margin of error in degrees the thrusers must fall within
int des_angle, des_angle2; //Make varables local to main when possible. Currently breaks code.
int count = 0;
int prev_left = 0;
int prev_right = 0; 


//initialize the liquid crystal library
//the first parameter is the I2C address
//the second parameter is how many rows are on your screen
//the third parameter is how many columns are on your screen
LiquidCrystal_I2C lcd(0x27, 20, 4);


// Ros Stuff
///////////////////////////////////////////////////////
ros::NodeHandle nh;
int thruster_value_right; //handles the right thruster value
int thruster_value_left; //handles the left thruster value
int angle_value_right; //handles the right angle value
int angle_value_left; //handles the left angle value
int left_angle; //handles the left angle value
int right_angle; //handles the right angle value
int port_auto_speed;
int stbd_auto_speed;
int auto_thrusters_on = 0; //handles the ramping case in auto... rmaps up signal to not lock 3-phase DC trolling motor

std_msgs::Int32 angle_msg;
std_msgs::Int32 angle_msg1;
std_msgs::Int32 thrust_msg;
std_msgs::Int32 thrust_msg1;


void thruster_sub_right(const std_msgs::Int32& thruster_msg_right){ //function that subscribes to get the STBD thruster value
  thruster_value_right = thruster_msg_right.data;
}

void angle_sub_right(const std_msgs::Int32& angle_msg_right){ //function that subscribes to get the STBD angle value
  angle_value_right = angle_msg_right.data;
}

void thruster_sub_left(const std_msgs::Int32& thruster_msg_left){ //function that subscribes to get the PORT thruster value
  thruster_value_left = thruster_msg_left.data;
}

void angle_sub_left(const std_msgs::Int32& angle_msg_left){ //function that subscribes to get the PORT angle value
  angle_value_left = angle_msg_left.data;
}

//SUBSCRIBER DECLARATIONS FOR AUTO MODE FROM JETSON 192.186.1.68
ros::Subscriber<std_msgs::Int32> sub("/thruster_int_right", &thruster_sub_right); //input from jetson for the stbd thruster
ros::Subscriber<std_msgs::Int32> sub1("/angle_int_right", &angle_sub_right); //input from jetson for stbd angle
ros::Subscriber<std_msgs::Int32> sub2("/thruster_int_left", &thruster_sub_left);//input from jetson for the port thruster
ros::Subscriber<std_msgs::Int32> sub3("/angle_int_left", &angle_sub_left);//input from jetson for the port angle 
//PUBLISHERS 
ros::Publisher pub("angle_tune_left", &angle_msg);
ros::Publisher pub1("angle_tune_right", &angle_msg1);
ros::Publisher pub2("teensy_thrust_left", &thrust_msg);
ros::Publisher pub3("teensy_thrust_right", &thrust_msg1);

void setup() {
  nh.initNode(); //initializes node 
  nh.subscribe(sub);
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.advertise(pub);
  nh.advertise(pub1);
  nh.advertise(pub2);
  nh.advertise(pub3);

  //adjust frequency of port and stbd thrust values to help with the motors   
  analogWriteFrequency(24,480);
  analogWriteFrequency(25,480);
  //initialize lcd screen
  lcd.init();
  // turn on the backlight
  lcd.backlight();
  //Start of new pin outputs for teensy 4.1
  pinMode(15, INPUT); //Potentiometer pin stbd04
  pinMode(20, INPUT); //Potentiometer pin port
  pinMode(21,INPUT);   //Stbd speed control 
  pinMode(22,INPUT);   //Port speed control
  pinMode(23,INPUT); //Auto-RC switch
  pinMode(25,OUTPUT); //PWM out to port motor
  pinMode(24,OUTPUT); //PWM out to stbd motor
  pinMode(38,OUTPUT); //P1 Steering Relays
  pinMode(39,OUTPUT); //P2 Steering Relays 
  pinMode(40,OUTPUT); //S1 Steerinf Relays
  pinMode(41,OUTPUT); //S2 Steering Relays
  pinMode(13, OUTPUT);//Onboard LED Pin
}

//Start of loop that controls the auto and manual modes and spins the subsrcibers 
void loop() { 

  digitalWrite(13,HIGH); // LED on
  LCD_Screen();

  if (Switch_read(23) == 1)
  {
    auto_control();
  }
  else if (Switch_read(23) == 0)
  {
    auto_thrusters_on = 0;
    prev_left = 0;
    prev_right = 0;
    RC_control();
  }
  else
  {
    //Serial.println("Error, Auto-Manual Switch not detected, verify RC controller is on. Pin 9 on RC receiver, pin 23 on Teensy.");  
    left_angle = Potentiometer_read(20);
    right_angle = Potentiometer_read(15);
    analogWrite(25,0); // Turns off all outputs
    analogWrite(24,0); 
    digitalWrite(38,LOW);
    digitalWrite(39,LOW);
    digitalWrite(40,LOW);
    digitalWrite(41,LOW);
  }


  // Slow down publish rate
  if (count == 1)
  {
    // Sends ros thruster angles
    angle_msg.data = left_angle;
    pub.publish(&angle_msg);
    angle_msg1.data = right_angle;
    pub1.publish(&angle_msg1);
  }
  else if (count == 10)
  {
    count = 0;
  }
  
  count= count + 1;
  nh.spinOnce();  
  //publish the values recieved from the jetson to have confirmation of a good transfer
} // End of loop


//Returns a degree value given the input pin of a potentiometer
//Dealing with somewhat significant noise in the braided cables
float Potentiometer_read(int pin){
  float raw_pot, raw_avg, angle, i;
  raw_pot = 0;
  for(i=0; i < 250; i++)
  {
    raw_pot = raw_pot + analogRead(pin);
  }
  raw_avg = raw_pot / (float)i;
  angle = (raw_avg*(900.0/1023.0)) - 270; //Baised on 4:1 ratio with 10 turn pot
  return(round(-angle)); //Negative angle due to pot wiring
}

//Turns port motor to desired angle
int Motor_turn_port(int left_angle, int des_angle2){
  if(left_angle <= (des_angle2 + MOE) && left_angle >= (des_angle2 - MOE))
  {
    digitalWrite(38,LOW);
    digitalWrite(39,LOW);
    return(1); //port at desired angle bit
  }
  else
  {
    if(left_angle < des_angle2 - MOE)
    {
      digitalWrite(39,HIGH);
      digitalWrite(38,LOW);
    }
    if(left_angle > des_angle2 + MOE)
    {
      digitalWrite(38,HIGH);
      digitalWrite(39,LOW);
    }
    return(0);
  }
}
////////////////////////////////////////////////

//Turns stbd motor to desired angle
int Motor_turn_stbd(int right_angle, int des_angle){
  if(right_angle <= (des_angle + MOE) && right_angle >= (des_angle - MOE))
  {
    digitalWrite(40,LOW);
    digitalWrite(41,LOW);
    return(1); //port at desired angle bit
  }
  else
  {
    if(right_angle < des_angle - MOE)
    {
      digitalWrite(41,HIGH);
      digitalWrite(40,LOW);
    }
    if(right_angle > des_angle + MOE)
    {
      digitalWrite(40,HIGH);
      digitalWrite(41,LOW);
    }
    return(0);
  }

}

//Returns switch state (-1 = no input, 0 = low, 1 = high) given input pin of switch
int Switch_read(int pin){
  int pwm_in;
  pwm_in = pulseIn(pin,HIGH);
  if (pwm_in> 1901 && pwm_in < 1951)
  {
    return(0);
  }
  else if (pwm_in < 1143 && pwm_in > 1043)
  {
    return(1);
  }
  else
  {
    return(-1);
  }
}
////////////////////////////////////////////////////////////////////
//RC Operation
void RC_control(){

  int port_AA, stbd_AA;
  int left_angle, right_angle, port_RC_speed, stbd_RC_speed;


///////////////////////////////////////////////////
//Port thruster RC control
  left_angle = Potentiometer_read(20);

  //0 when the stick is in the middle -100 at the bottom 100 at the top
  //PWM value is not symetrical nessitating the if statements below
  port_RC_speed = pulseIn(22,HIGH);

  if (port_RC_speed >= 1550)
  {
    port_RC_speed = (port_RC_speed-1550)/3.73;
  }
  else
  {
    port_RC_speed = (port_RC_speed-1550)/4.63;
  }
  
  if (port_RC_speed <= 4 && port_RC_speed >= -4)
  {
    port_RC_speed = 0;
  }


  //If the stick is in its sprung back location the thruster will keep its previous orientation
  if(port_RC_speed >= 5) //forward case
  {
    des_angle2 = 0;
  }
  if(port_RC_speed < -5) //backward case
  {
    des_angle2 = -180;
  }

  port_AA = Motor_turn_port(left_angle, des_angle2); //Calls Motor turn function
   
  if(port_AA == 1)
  {
   port_RC_speed = abs(port_RC_speed);
   //Eliminates motor controler deadzone between 0 and abs(140) by normalizing between 140 and 255
   //140 = 2.75v and 255 = 5v after running through the 3.3v to 5v board and PWM analog converter
   if (port_RC_speed < 140 && port_RC_speed > 5)
   {
     port_RC_speed = round(port_RC_speed * 1.15) + 140;
   }
   analogWrite(25,port_RC_speed);
  }

  else
  {
   analogWrite(25,0);
  }


////////////////////////////////////////////////////////
  // Stbd thruster RC

  
    right_angle = Potentiometer_read(15);
    angle_msg.data = left_angle;
    pub.publish(&angle_msg);
    angle_msg1.data = right_angle;
    pub1.publish(&angle_msg1);

  //0 when the stick is in the middle -100 at the bottom 100 at the top
  //PWM value is not symetrical nessitating the if statements below
  stbd_RC_speed = pulseIn(21,HIGH);

  if (stbd_RC_speed >= 1500)
  {
    stbd_RC_speed = (stbd_RC_speed-1500)/4.23;
  }
  else
  {
    stbd_RC_speed = (stbd_RC_speed-1500)/4.13;
  }
  
  if (stbd_RC_speed <= 4 && stbd_RC_speed >= -4)
  {
    stbd_RC_speed = 0;
  }
  
  //If the stick is in its sprung back location the thruster will keep its previous orientation
  if(stbd_RC_speed >= 5) //forward case
  {
    des_angle = 0;
  }
  if(stbd_RC_speed < -5) //backward case
  {
    des_angle = 180;
  }

  stbd_AA = Motor_turn_stbd(right_angle, des_angle); //Calls Motor turn function
   
  if(stbd_AA == 1)
  {
   stbd_RC_speed = abs(stbd_RC_speed);
   //Eliminates motor controler deadzone between 0 and abs(140) by normalizing between 140 and 255
   //140 = 2.75v and 255 = 5v after running through the 3.3v to 5v board and PWM analog converter
   if (stbd_RC_speed < 140 && stbd_RC_speed > 5)
   {
     stbd_RC_speed = round(stbd_RC_speed * 1.15) + 140;
   }
   analogWrite(24,stbd_RC_speed);
  }
  else
  {
   analogWrite(24,0);
  }
}



////////////////////////////////////////////////
// Autonomus Control
void auto_control(){

  //Reads the motors curent location and the desired location then moves to
  //that location. The nested if statements give the motors +/- 200 degrees of rotation.
  left_angle = Potentiometer_read(20);
//Port side for truning motors based on relay outputs
  if(angle_value_left <= 180)
  {
    if (angle_value_left >= 160 && left_angle < 0)
    {
      angle_value_left = angle_value_left - 360;
      Motor_turn_port(left_angle, angle_value_left);
    }
    else
    {
      Motor_turn_port(left_angle, angle_value_left);
    }
  }
  else
  {
    if (angle_value_left <= 200 && left_angle > 0)
    {
      Motor_turn_port(left_angle, angle_value_left);
    }
    else
    {
      angle_value_left = angle_value_left - 360;
      Motor_turn_port(left_angle, angle_value_left);
    }
  }
//,stbd side for turning motors based on relays
  right_angle = Potentiometer_read(15);
  if(angle_value_right <= 180)
  {
    if (angle_value_right >= 160 && right_angle < 0)
    {
      angle_value_right = angle_value_right - 360;
      Motor_turn_stbd(right_angle, angle_value_right);
    }
    else
    {
      Motor_turn_stbd(right_angle, angle_value_right);
    }
  }
  else
  {
    if (angle_value_right <= 200 && right_angle > 0)
    {
      Motor_turn_stbd(right_angle, angle_value_right);
    }
    else
    {
      angle_value_right = angle_value_right - 360;
      Motor_turn_stbd(right_angle, angle_value_right);
    }
  }
//publishing the angles for the potentiometers
  angle_msg.data = right_angle;
  pub.publish(&angle_msg);
  angle_msg1.data = left_angle;
  pub1.publish(&angle_msg1);
//turns off motors if not to desrired angle within a certain margin of error defined at the top
  if ((abs(angle_value_right - right_angle) >= MOE) || (abs(angle_value_left - left_angle) >= MOE))
  {
    prev_left = 0;
    prev_right = 0;
    analogWrite(25,0); // outputs port thruster value.
    analogWrite(24,0); // outputs stbd thruster value.
    auto_thrusters_on = 0;
  } 
//thrusters are at the desired angle and will output input thrust
  else
  {
    thruster_value_right = (int)round(((double)125/255)*(double)thruster_value_right + 130);
    thruster_value_left = (int)round(((double)125/255)*(double)thruster_value_left + 130);

    thrust_msg.data = thruster_value_left;
    pub2.publish(&thrust_msg);
    thrust_msg1.data = thruster_value_right;
    pub3.publish(&thrust_msg1);

    analogWrite(25,thruster_value_right);
    analogWrite(24,thruster_value_left);
/* //start of ramp signal created
    if (auto_thrusters_on == 0)
    {    
      for (int i = 0; ((i < thruster_value_right) || (i<thruster_value_left));i++)
        {
          delay(1);
          if (thruster_value_left > i)
            {
              analogWrite(24,i);
            }
          if (thruster_value_right > i)
            {
              analogWrite(25,i);
            }
          } 
          auto_thrusters_on = 1;
    }
if (auto_thrusters_on == 1)
  {
     // analogWrite(24,thruster_value_left);
      //analogWrite(25,thruster_value_right);
      if ((prev_left != thruster_value_left) || (prev_right != thruster_value_right))
        {
          if (prev_left < thruster_value_left)
            {
              delay(1);
              prev_left += 1;
              analogWrite(24,prev_left);
            }
          if (prev_right < thruster_value_right)
            {
              delay(1);
              prev_right += 1;
              analogWrite(25,prev_right);
            }
          if (prev_left > thruster_value_left)
          {
            prev_left = thruster_value_left;
            analogWrite(24,thruster_value_left);
          }
          if (prev_right > thruster_value_right)
          {
            prev_right = thruster_value_right;
            analogWrite(25,thruster_value_right);
          }
        }
    }*/
  }
}


//////////////////////////////////////////////////////////
// Prints angles to LCD Screen
void LCD_Screen()
{
  
  int port, stbd;
  port = Potentiometer_read(20);
  stbd = Potentiometer_read(15);

  //Reads the switch mode an LCD prints the mode on the screen
      if (Switch_read(23) == 1)
    {
      lcd.setCursor(0,0);
      lcd.print("                   ");
      lcd.setCursor(0,0);
      lcd.print("AUTONOMOUS MODE");
    }
    else if (Switch_read(23) == 0)
    {
      lcd.setCursor(0,0);
      lcd.print("                   ");
      lcd.setCursor(0,0);
      lcd.print("RC MODE");
    }
    else
    {
      lcd.setCursor(0,0);
      lcd.print("                   ");
      lcd.setCursor(0,0);
      lcd.print("ERROR. NO RC");
    }
  


    //Prints the STBD and PORT angles on the screen as three digits with a +/-
    lcd.setCursor(0,1);
    lcd.print("STBD ANGLE:");
    lcd.setCursor(12,1);
    if (stbd > 0)
    {
      lcd.print("+");
      if (stbd < 100 && stbd >=10)
      {
        lcd.print("0");
        lcd.print(stbd);
      }
      else if (stbd < 10)
      {
        lcd.print("00");
        lcd.print(stbd);    
      }
      else
      {
        lcd.setCursor(13,1);
        lcd.print(stbd);        
      }
    }
    else
    {
      lcd.print("-");
      stbd = abs(stbd);
      
      if (stbd < 100 && stbd >= 10)
      {
        lcd.print("0");
        lcd.print(stbd);
      }
      else if (stbd < 10)
      {
        lcd.print("00");
        lcd.print(stbd);    
      }
      else
      {
        lcd.setCursor(13,1);
        lcd.print(stbd);        
      }      
    }
    lcd.print((char)223); // Degree symbol

    lcd.setCursor(0,2);
    lcd.print("PORT ANGLE:");
    lcd.setCursor(12,2);
    if (port > 0)
    {
      lcd.print("+");
      if (port < 100 && port >=10)
      {
        lcd.print("0");
        lcd.print(port);
      }
      else if (port < 10)
      {
        lcd.print("00");
        lcd.print(port);    
      }
      else
      {
        lcd.setCursor(13,2);
        lcd.print(port);        
      }
    }
    else
    {
      lcd.print("-");
      port = abs(port);
      
      if (port < 100 && port >= 10)
      {
        lcd.print("0");
        lcd.print(port);
      }
      else if (port < 10)
      {
        lcd.print("00");
        lcd.print(port);    
      }
      else
      {
        lcd.setCursor(13,2);
        lcd.print(port);        
      }      
    }
    lcd.print((char)223); //Degree symbol

}
