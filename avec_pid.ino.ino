#include <Wire.h>
#include <Servo.h>


Servo right_prop;
Servo left_prop;

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];




float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

double kp=4;
double ki=0.03;
double kd=1;

double throttle=1100; //initial value of throttle to the motors
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady


void setup() {
  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);
  right_prop.attach(10); //attatch the right motor to pin 9
  left_prop.attach(9);  //attatch the left motor to pin 10

  time = millis(); //Start counting time in milliseconds
  /*In order to start up the ESCs we have to send a min value
   * of PWM to them before connecting the battery. Otherwise
   * the ESCs won't start up or enter in the configure mode.
   * The min value is 1000us and max is 2000us, REMEMBER!*/
  left_prop.writeMicroseconds(1000); 
  right_prop.writeMicroseconds(1000);
  delay(7000); 

}//end of setup void

void loop() {

/////////////////////////////I M U/////////////////////////////////////
    timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000; 
  
  /*The tiemStep is the time that elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. We work in ms so we haveto divide the value by 1000 
   to obtain seconds*/

  /*Reed the values that the accelerometre gives.
   * We know that the slave adress for this IMU is 0x68 in
   * hexadecimal. For that in the RequestFrom and the 
   * begin functions we have to put this value.*/
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
   
  



    
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();

 
 



     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 



    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/seconda we have to divide first
   the raw value by 131 because that's the value that the datasheet gives us*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;




   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*Now we have our angles in degree and values from -10º0 to 100º aprox*/
    //Serial.println(Total_angle[1]);

   



error = Total_angle[1] - desired_angle;



//proportionnel

pid_p = kp*error;

//integrateur 

if(error > 10)
{
  pid_i = 0;  
}
else if(error < -10)
{
  pid_i = 0;  
}
else {
  pid_i +=(ki*error); //0.1s    
}

//derivateur
pid_d = kd*((error - previous_error)/elapsedTime);


PID = pid_p + pid_i + pid_d;




if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}


pwmLeft = throttle + PID;
pwmRight = throttle - PID;



//Right
if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}
//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}




left_prop.writeMicroseconds(pwmLeft);
right_prop.writeMicroseconds(pwmRight);


Serial.print("error : ");
Serial.println(error);
Serial.print("motor2 : ");
Serial.println(pwmLeft);
Serial.print("motor1 : ");
Serial.println(pwmRight);
previous_error = error; 

}//end of loop void
