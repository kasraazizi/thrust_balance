
#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>


MPU6050 mpu(Wire);
Servo ESC1;     // left
Servo ESC2;     //right

unsigned short int leftMotorPWM=0, rightMotorPWM=0;
String pcCommand;

void setup() {
  Serial.begin(9600);    
  Serial.setTimeout(10);              
 
  ESC1.attach(5,1000,2000);                                                            // (pin, min pulse width, max pulse width in microseconds) 
  ESC2.attach(6,1000,2000);                                                            // (pin, min pulse width, max pulse width in microseconds)   
  motor_off();
  delay(100);

  Wire.begin();
  Serial.println(F("wire initiated"));
 
  byte status = mpu.begin();
  Serial.println(F("imu initiated"));
   
  while(status!=0){  Serial.println(F("MPU6050 not found")); }                           // stop everything if could not connect to MPU6050


  /* ==========================imu calibration==============================*/
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(500);
  mpu.calcOffsets(); // gyro and accelero
  delay(500);
  Serial.println("imu OK\n"); 
  /* -----------------------------------------------------------------------*/  

  /* ==========================esc calibration==============================*/
  
  /* -----------------------------------------------------------------------*/  
}
 
void loop() 
{
  mpu.update();    
  if (Serial.available() > 0) 
    {    
    pcCommand = Serial.readString(); 
       
    if (pcCommand.startsWith("L") && pcCommand.endsWith("R\n") )              // check the structure of massage with "L??-??R/n"
      {
      pc_to_arduino(pcCommand);
      }     

    else if ( pcCommand == "motor off\n" )
      {                                               
      motor_off();
      Serial.println(F("motor off"));
      
      } 
    else  
      { Serial.println(F("wrong")); motor_off(); }                                             // input of motor command function 
    delay(1);
   }       
}

        
void pc_to_arduino(String motorCommand){    
  
                                                                      
  short unsigned int  leftPose_start = motorCommand.indexOf("L") +  1;             // start index of left and right pwm 
  short unsigned int  leftPose_stop  = motorCommand.indexOf("-");                  // start index of left and right pwm
  short unsigned int  rightPose_stop = motorCommand.indexOf("R"); 

  /* ---------------------------MOTOR--------------------------- */                                                 
  if( leftPose_stop <= leftPose_start+2  && rightPose_stop <= leftPose_stop+3 )    // check digits number of left pwm, otherwise shutdown the motor      
   {          
   leftMotorPWM  = motorCommand.substring(leftPose_start, leftPose_stop).toInt();  // excratcing left motor pwm from input data        
   rightMotorPWM = motorCommand.substring(leftPose_stop+1, rightPose_stop).toInt(); // excratcing right motor pwm from input data        
   motor_command_to_esc(leftMotorPWM, rightMotorPWM);   

    /* ---------------------*/

    double roll = mpu.getAngleX();
    Serial.print( String(roll) + motorCommand  );    
        
    delay(1);            

    
   }
  else { motor_off(); }
      
}            
      
void motor_command_to_esc(short unsigned int leftpwm, short unsigned int rightpwm) {
//  short unsigned int left_esc_command
//  short unsigned int right_esc_command 
  
  if (leftpwm == 0) {  ESC1.write(0); }
  else
  {
    ESC1.write(map(leftpwm,  0, 99, 10, 100) );             // pwm map to esc frame
  }
  
 if (rightpwm == 0) {  ESC2.write(0); }
  else
  {
     ESC2.write(map(rightpwm,  0, 99, 10, 100) );              // pwm map to esc frame
  }


  
//  ESC1.write(left_esc_command);
//  ESC2.write(right_esc_command);
  delay(1);
//  Serial.print(F("left ESC = "));   
//  Serial.print(left_esc_command);
//  Serial.print(F("    right ESC = "));      
//  Serial.println(right_esc_command);           
    
}
         


void motor_off(void)
{
  ESC1.write(0);
  ESC2.write(0);
  delay(100);  
}
