#include "astorino.h"

astorino r(Serial1); // use Serial1 for UART communication
astorino::RetVal ret; //return value1

void setup() {
  Serial.begin(115200);

  delay(1000);
  if(r.Connect() == 0) //open connection
  {
	  r.setUartTimeout(10); //set connection timeout to 1000 ms
	  r.setMotorOn(); //turn on MOTORS
	  delay(1000);
	  r.Zero(); // start zeroing procedure
	  Serial.println("Zeroing done");
	  r.HOME(80,90,90); //go to HOME position
	  Serial.println("In HOME");
	  ret = r.Pose(); // read current position
	  if (ret.returnCode == 0)
	  {
		  Serial.print(ret.values[0]); //x
		  Serial.print(" ");
		  Serial.print(ret.values[1]); //y
		  Serial.print(" ");
		  Serial.println(ret.values[2]); //z
	  }
	  ret = r.selectedProgram(); // read currently selected program
	  if (ret.returnCode == 0)
	  {
		  Serial.println(ret.name);
	  }
	  delay(1000);
	  delay(1000);
	  delay(1000);
	 
	  r.setMotorOff(); // turn off MOTORS
	  Serial.println("MOTORS are OFF");
	  delay(1000);
	  r.Disconnect(); // close connection
  }
  else
  {
	  //communication failed
	  while(true)
	  {
		  delay(1);
	  }
  }
}

void loop() 
{
	//do nothing
    delay(100);
}
