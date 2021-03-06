/***************************************************************************/
//	Function: Measure the distance to obstacles in front and print the distance
//			  value to the serial terminal.The measured distance is from
//			  the range 0 to 400cm(157 inches).
//	Hardware: Ultrasonic Range sensor
//	Arduino IDE: Arduino-1.0
//	Author:	 LG
//	Date: 	 Jan 17,2013
//	Version: v1.0 modified by FrankieChu
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
//
/*****************************************************************************/
#include "Arduino.h"
#include <RadioShackRobotics.h>
class Ultrasonic
{
	public:
		Ultrasonic(int pin);
        void DistanceMeasure(void);
		long microsecondsToCentimeters(void);
		long microsecondsToInches(void);
	private:
		int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        long duration;// the Pulse time received;
};
Ultrasonic::Ultrasonic(int pin)
{
	_pin = pin;
}
/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
    pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pin,LOW);
	pinMode(_pin,INPUT);
	duration = pulseIn(_pin,HIGH);
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
	return duration/29/2;
}
/*The measured distance from the range 0 to 157 Inches*/
long Ultrasonic::microsecondsToInches(void)
{
	return duration/74/2;
}

int redPin = 11;
int greenPin = 10;
int bluePin = 9;

Ultrasonic ultrasonic(7);
RadioShackRobotics wheels;//declare object
bool do_turn_left = true;

int state = 0; // for switching the turn state
int speed = 100; // for how fast to turn the wheels
int blocked_count = 0; // how many times we have been redirected due to an obstacle in the way
long cm = 0; // the current distance measurement
int start_time = 0; // when we started driving
int max_time = 60; // only run for 60 seconds
bool stopped = false;
int factor = 1;
bool use_random = true;
double speed_factor = 1;

void setup()
{
	Serial.begin(10420);                     //tell the Arduino to communicate with PCB
  delay(500);                              //delay 500ms
	wheels.all_stop();               //all motors stop
	//Serial.begin(9600);

	pinMode(redPin, OUTPUT);
	pinMode(greenPin, OUTPUT);
	pinMode(bluePin, OUTPUT);
	start_time = millis();
}

void loop()
{
	speed_factor = 1;
	// wait for button to be pressed
	// after button pressed, run for 20 seconds
	// ping distance in front of it
	factor = 1;
	ultrasonic.DistanceMeasure();// get the current signal time;
	cm = ultrasonic.microsecondsToCentimeters();
	distanceToColor(cm);

	if(millis() - start_time > (max_time*1000))
	{
		// if we have been running too long, go into a stopped state
		stopped = true;
	}

	if(stopped)
	{
		wheels.move_stop();

		// flash the light;
		distanceToColor(((state++)%4)*20);
		delay(400);
	}
	else if(cm < 2)
	{
		stopped = true;
	}
	else if(cm < 20)
	{
		blocked_count++;
		//if(blocked_count < 4)
		{
			/*if((state/4)%2)
			{
				// every 4 switches, switch into reversing??
				speed *= -1;
			}*/
			if(use_random)
			{
				do_turn_left = random(0,1 + 1);
				if(do_turn_left)
					wheels.turn_left(speed*speed_factor);
				else
					wheels.turn_right(speed*speed_factor);
				factor = random(8,20 + 1);
			}
			else{

				factor = 10;
				switch(state++%6)
				{
					case 0:
					case 3:
						wheels.turn_left(speed*speed_factor);
						break;
					case 1:
					case 4:
						wheels.turn_right(speed*speed_factor);
						break;
					case 2:
						// turn around
						speed_factor = 1.5;
						wheels.go_backward(speed*speed_factor);
						delay(400);
						wheels.turn_right(speed*speed_factor);
						factor = 40;
						break;
					case 5:
						// turn around
						speed_factor = 1.5;
						wheels.go_backward(speed*speed_factor);
						delay(400);
						wheels.turn_left(speed*speed_factor);
						factor = 40;
					/*case 3:
					default:
						wheels.turn_left(speed*speed_factor);
						break;*/
				}

			}
		}
		/*else if(blocked_count < 20)
		{
			// we have been blocked for a while, take a break
			wheels.move_stop();
		}
		else
		{
			stopped = true;
		}*/
	}
	else if( cm > 100 && cm < 110)
	{
		// Use the catapult!
		wheels.all_stop();           //all motors stop
		// 5 second count down
		for(int i = 0; i < 10; i++)
		{
			if(i %2 == 0)
				setColor(0,0,0);
			else
				setColor(0,255,0,1);
			delay(500);                   //delay 1000ms
		}
		wheels.catapult_head_throw(255);  //throw (wired backwards :(   )
		delay(1000);                   //delay 1000ms
		setColor(0,0,255,6);// Set to Bright Blue
		wheels.catapult_head_pull(60);  //pull catapult arm back (wired backwards :(   )
		delay(1000);                   //delay 1000ms
		wheels.all_stop();           //all motor stop
		setColor(0,0,0);// Set to Bright Blue

		if(cm < 500)
			speed_factor = 1 + ((double) cm - 20)/500.;
		wheels.go_forward(abs(speed*speed_factor));

		// drive for at least a second
		int seconds = 2;
		factor = 10*seconds*speed_factor;
	}
	else
	{
		/*if(blocked_count > 0)
			blocked_count--;*/
		if(cm < 500)
			speed_factor = 1 + ((double) cm - 20)/500.;
		wheels.go_forward(abs(speed*speed_factor));

		// drive for at least a second
		int seconds = 1;
		factor = 10*seconds*speed_factor;
	}

//	void go_forward(int speed*speed_factor);		//left and right motors (M1 and M2) forward
//	void go_backward(int speed*speed_factor);	//left and right motors (M1 and M2) forward
//	void turn_left(int speed*speed_factor);		//left motor (M1) backward and right motor (M2) forward
//	void turn_right(int speed*speed_factor);		//left motor (M1) forward and right motor (M2) backward
//	void turn_front_left(int speed*speed_factor);	//left and right motors (M1 and M2) forward with right motor (M2) at twice the speed of left motor (M1)
//	void turn_front_right(int speed*speed_factor);   //left and right motors (M1 and M2) forward with left motor (M1) at twice the speed of right motor (M2)
//	void move_stop();			////left and right motors (M1 and M2) stop
//void RadioShackRobotics::catapult_head_pull(int speed)
//{
//  m4_action(BW,speed);
//}

//void RadioShackRobotics::catapult_head_throw(int speed)
//{
//  m4_action(FW,speed);
//}

//catapult.all_stop();           //all motors stop
//catapult.catapult_head_throw(255);  //throw
//delay(1000);                   //delay 1000ms
//catapult.catapult_head_pull(60);  //pull catapult arm back
//delay(1000);                   //delay 1000ms
//catapult.all_stop();           //all motor stop

	delay(100*(double) factor/speed_factor);
}

void distanceToColor(long cm)
{
	if(cm < 5)
		setColor(255,0,0);// RED
	else if(cm < 10)
		setColor(255,127,0);// ORANGE
	else if(cm < 20)
		setColor(255,255,0);// YELLOW
	else if(cm < 30)
		setColor(127,255,0);// YELLOW green
	else if(cm < 40)
		setColor(0,255,0);// green
	else if(cm < 50)
		setColor(0,255,127);// BLUE green
	else if(cm < 60)
		setColor(0,255,255);// CYAN
	else if(cm < 70)
		setColor(0,127,255);// LIGHT BLUE
	else if(cm < 80)
		setColor(0,0,255);// BLUE
	else if(cm < 90)
		setColor(127,0,255);// PURPLE
	else if(cm < 100)
		setColor(255,0,255);// MAGENTA
	else if(cm < 110)
		setColor(255,0,127);// PINK
	/*else if(cm > 200 && cm < 250)
		setColor(0,255,0, 1);// BRIGHT Green*/
	else
		setColor(0,0,0);// BLACK
}

void setColor(int red, int green, int blue)
{
	setColor(red, green, blue, 12);
}

void setColor(int red, int green, int blue, int divisor)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red/divisor);
  analogWrite(greenPin, green/divisor);
  analogWrite(bluePin, blue/divisor);
}

/*void hsvToRgb(int h_i, int s_i, int v_i){
    int r, g, b;
	double h = (double)(h_i)/255.;
	double s = (double)(s_i)/255.;
	double v = (double)(v_i)/255.;

    double i = Math.floor(h * 6);
    double f = h * 6 - i;
    double p = v * (1 - s);
    double q = v * (1 - f * s);
    double t = v * (1 - (1 - f) * s);

    switch(i % 6){
        case 0: r = v, g = t, b = p; break;
        case 1: r = q, g = v, b = p; break;
        case 2: r = p, g = v, b = t; break;
        case 3: r = p, g = q, b = v; break;
        case 4: r = t, g = p, b = v; break;
        case 5: r = v, g = p, b = q; break;
    }

    return [r * 255, g * 255, b * 255];
	}*/
