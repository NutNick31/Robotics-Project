#include <Servo.h>
#include <math.h>
#include <Wire.h>

unsigned char ok_flag;
unsigned char fail_flag;

unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;



Servo myServo1;
Servo myServo2;
Servo myServo3;
Servo myServo4;


int lEDpin=13;
int iRsensorpin=3;
int val1,val2,val3;
float ang1,ang2,ang3;
float x,y,z;


float l1=500.00;
float l2=500.00;

void setup(){
  
 Wire.begin(); 
pinMode(lEDpin,OUTPUT);
pinMode(iRsensorpin,INPUT);
  
myServo1.attach(11);
myServo2.attach(9);
myServo3.attach(6);
myServo4.attach(13);
Serial.begin(9600);

}

void loop(){
 int xl=ReadDistance();
/*detection has to start before motion*/


Serial.println("Enter the x coordinate upto which to travel");
Serial.printf("%d",x);
int h;/* height of object*/
Serial.printf("Enter the height of object:%d",h);
y=h;




ang2=acos((x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2)) * 180/3.14;
  
ang1=atan(y/x)-atan((l2*sin(ang2))/(l1+l2*cos(ang2))) * 180/3.14;  
  
 ang3=atan(z/x)*180/3.14; 
 
 float defcoordinate,theta1,theta2;
 int c=0;
 for(;;){
  if(digitalRead(iRsensorpin) == HIGH){
   
    digitalWrite(lEDpin,HIGH);/*hole detected*/
    break;
    delay(10);
  }else
  {
    
    digitalWrite(lEDpin,LOW);/* no hole detected */
    delay(10);
  }

 myServo1.write(ang1/100); 
 myServo2.write(ang2/100); 
  c=c+1;
 }
 theta1=(ang1/100)*c;
 theta2=(ang2/100)*c;

float x1=l1*(cos(theta1)) + l2*(cos(theta2));
y1=l1/2;
float angl2=acos((x1*x1 + y1*y1 - l1*l1 - l2*l2)/(2*l1*l2)) * 180/3.14;
  
float angl1=atan(y1/x1)-atan((l2*sin(ang2))/(l1+l2*cos(ang2))) * 180/3.14; 
myServo1.write(angl1);
myServo2.write(angl2);
myServo4.write(30);/* gripper arm opens up*/
myServo4.write(0);


z=x1;
float ang13=atan(z/x1)*180/3.14; 
myServo3.write(angl3);
myServo4.write(30);

  
 

 
  
 /* Serial.println(x);
  Serial.println(x);
*/  
delay(15);

}

int serial_putc( char c, struct __file * )
{
  Serial.write( c );
  return c;
}

void printf_begin(void)
{
  fdevopen( &serial_putc, 0 );
}



void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52)
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}

int ReadDistance(){
    SensorRead(0x00,i2c_rx_buf,2);
    lenth_val=i2c_rx_buf[0];
    lenth_val=lenth_val<<8;
    lenth_val|=i2c_rx_buf[1];
    delay(300); 
    return lenth_val;
}
