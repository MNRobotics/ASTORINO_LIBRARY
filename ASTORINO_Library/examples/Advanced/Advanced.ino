#include "astorino.h"

/*
	Analog Input to RTC data
	Demonstrates analog input by reading an analog joystick on analog pin 0 and 1, and
	sending that data to RTC processor, RTC data then is used to modify robots path.
	

	The circuit:
	* Potentiometer attached to analog input 0
	* center pin of the potentiometer to the analog pin
	* one side pin (either one) to ground
	* the other side pin to +3.3V


	To make this code work astorino must be READY to workand in HOME position (0,0,-90,0,-90,0), and listed below program must be turned ON in REPEAT MODE:
	
	Robot position:
     ____
 	|	 |
		 |
		_|_
	
	Program code:
	
	.PROGRAM RTC_EXAMPLE
		WHILE(1==1) DO
			DELAY 10
		END
	.END
	
	
	Code below does not incomporate any error handling.

	Created by Marek Niewiadomski 23.08.2024
 
*/

astorino r(Serial1); // use Serial1 for UART communication
astorino::RetVal ret; //return value1

void setup() {
  Serial.begin(115200);
	
  delay(1000);
  if(r.Connect() == 0) //open connection
  {
	  r.setUartTimeout(10); //set connection timeout to 1000 ms
	  r.RTC_ON(); // turn on Real Time Control
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
	ret = r.Pose(); // read and print to the terminal current position
	if (ret.returnCode == 0)
	{
	  Serial.print(ret.values[0]);
	  Serial.print(" ");
	  Serial.print(ret.values[1]);
	  Serial.print(" ");
	  Serial.println(ret.values[2]);
	}
	int xValue = analogRead(A0);    
	int yValue = analogRead(A1); 
	
	r.setRTC_OffsetData((double)xValue, (double)yValue, 0, 0, 0, 0);
	
	delay(2); //short delay for loop stability
	
	if(Serial.available())
	{
		char comm = Serial.read();
		if(comm == "e")
		{
			r.RTC_OFF();
			r.Disconnect(); // close connection
			while(1==1)
			{
				delay(1); // do nothing
			}
		}
	}
}
