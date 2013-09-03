/*
***************************************************************************
* This is an application to control the servo and the camera movement.
* 
* The code is based on the sample of Phidget-c-samples, Advanced_servo.
* Once the servo is initiated, the camera will be set at a position as
* (0,115), (1, 115); 0 and 1 referring to the device index. A position
* is approximately vertical to the horizon.
*  
* Since there are two servos, in order to control its movement, we use
* 'a'and 'd' to control the up and down movement; 
* 's' and 'w' to control the left and right movement;
* 'r' for reset.
*
* For more information about libphidget21, go to the website below:
*
*
* Nov 16, 2011 @Robotics Lab, CSIS, Pace University
*
* For more information, send email to:
* NORAYX.LIN@GMAIL.COM
**************************************************************************
*/

//#include <stdio.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <assert.h>
#include <stdio.h>
#include <phidget21.h>
#include <ncurses.h>

// get the attached servo serial number-- from AdvancedServo.c
int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	printw("%s %10d attached!\n", name, serialNo);
	refresh();

	return 0;
}

// get the detach device serial number -- from AdvancedServo.c
int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
	printw("%s %10d detached!\n", name, serialNo);
	refresh();

	return 0;
}

// print error if there is no attached device-- from AdvancedServo.c
int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{
	printw("Error handled. %d - %s\n", ErrorCode, Description);
	refresh();
	return 0;
}

// show current position when initiate -- from AdvancedServo.c
int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
//	printf("Motor: %d > Current Position: %f\n", Index, Value);
	return 0;
}

//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number and version of the attached device.
//--from AdvancedServo.c
int display_properties(CPhidgetAdvancedServoHandle phid)
{
	int serialNo, version, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetAdvancedServo_getMotorCount (phid, &numMotors);

	printw("%s\n", ptr);
	printw("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);
	refresh();

	return 0;
}

int main(int argc, char* argv[])
{
	initscr();		// Start curses mode
	raw();			// Line buffering disabled
	keypad(stdscr, TRUE);	// We get F1, F2 etc..
	noecho();		// Don't echo() while we do getch
	
	int c;
	CPhidgetAdvancedServoHandle servo1;
	CPhidgetAdvancedServoHandle servo2;
	int result1, result2;
	const char *err1;
	const char *err2;
	double curr_pos1;
	double curr_pos2;
	double dst_pos1;
	double dst_pos2;
	double minAccel1, maxVel1;
	double minAccel2, maxVel2;
	
	servo1 = 0;
	servo2 = 0;

	//create the advanced servo object
	CPhidgetAdvancedServo_create(&servo1);
	CPhidgetAdvancedServo_create(&servo2);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo1, AttachHandler, NULL);
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo2, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo1, DetachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo2, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo1, ErrorHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo2, ErrorHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo1, PositionChangeHandler, NULL);
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo2, PositionChangeHandler, NULL);

	//open the device for connections
	CPhidget_open((CPhidgetHandle)servo1, -1);
	CPhidget_open((CPhidgetHandle)servo2, -1);

	//get the program to wait for an advanced servo device to be attached
	printw("Waiting for Phidget 0 to be attached....");
	refresh();
	if((result1 = CPhidget_waitForAttachment((CPhidgetHandle)servo1, 10000))) {
		CPhidget_getErrorDescription(result1, &err1);
		printw("Problem waiting for attachment 0: %s\n", err1);
		refresh();
		return 0;
	}
	printw("Waiting for Phidget 1 to be attached....");
	refresh();
	if((result2 = CPhidget_waitForAttachment((CPhidgetHandle)servo2, 10000))) {
		CPhidget_getErrorDescription(result2, &err2);
		printw("Problem waiting for attachment 1: %s\n", err2);
		refresh();
		return 0;
	}

	//Display the properties of the attached device
	display_properties(servo1);
	display_properties(servo2);

	//read event data
	printw("Reading.....\n");
	refresh();
	
	//This example assumes servo motor is attached to index 0

	//Set up some initial acceleration and velocity values
	CPhidgetAdvancedServo_getAccelerationMin(servo1, 0, &minAccel1);
	CPhidgetAdvancedServo_getAccelerationMin(servo2, 0, &minAccel2);
	CPhidgetAdvancedServo_setAcceleration(servo1, 0, minAccel1*2);
	CPhidgetAdvancedServo_setAcceleration(servo2, 0, minAccel2*2);
	CPhidgetAdvancedServo_getVelocityMax(servo1, 0, &maxVel1);
	CPhidgetAdvancedServo_getVelocityMax(servo2, 0, &maxVel2);
	CPhidgetAdvancedServo_setVelocityLimit(servo1, 0, maxVel1/2);
	CPhidgetAdvancedServo_setVelocityLimit(servo2, 0, maxVel2/2);

	//display current motor position
	if(CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_pos1) == EPHIDGET_OK){
		printw("Motor 1 Current Position: %f\n", curr_pos1);
		refresh();
	}
	if(CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_pos2) == EPHIDGET_OK){
		printf("Motor 2 Current Position: %f\n", curr_pos2);
		refresh();
	}

	//keep displaying servo event data until user input is read
	printw("Press any key to continue\n");
	refresh();
	getch();

	//change the motor position
	//valid range is -23 to 232, but for most motors ~30-210
	//we'll set it to a few random positions to move it around

	printw("Move motor 1 to position 115.00 and engage. Press any key to Continue\n");
	printw("Move motor 2 to position 115.00 and engage. Press any key to Continue\n");
	refresh();
	getch();

	CPhidgetAdvancedServo_setPosition (servo1, 0, 115.00);
	CPhidgetAdvancedServo_setPosition (servo2, 0, 115.00);

	CPhidgetAdvancedServo_setEngaged(servo1, 0, 1);
	CPhidgetAdvancedServo_setEngaged(servo2, 0, 1);
	
	printw("Press any key to start motor control...\n");
	refresh();
	getch();
	printw("Motor control started...\n");
	printw("Use 'a, s, d, w' to control motors. 'r' to reset position.\n");
	refresh();

	CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_pos1);
	CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_pos2);
	
	while(1){
		c = getch();	// If raw() hadn't been called
				// we have to press enter before it
				// gets to the program
		if(c == 115){
//			printw("s pressed\n");
//			printw("Current position is %f\n", curr_pos1);
			refresh();
			dst_pos1 = curr_pos1 - 5;
			CPhidgetAdvancedServo_setPosition (servo1, 0, dst_pos1);
//			CPhidgetAdvancedServo_setPosition (servo1, 0, 100);
			curr_pos1 = dst_pos1;
			continue;
		}
		else if(c == 119){
//			printw("w pressed\n");
			refresh();
			dst_pos1 = curr_pos1 + 5;
			CPhidgetAdvancedServo_setPosition (servo1, 0, dst_pos1);
//			CPhidgetAdvancedServo_setPosition (servo1, 0, 140);
			curr_pos1 = dst_pos1;
			continue;
		}
		else if(c == 97){
//			printw("a pressed\n");
			refresh();
			dst_pos2 = curr_pos2 - 5;
			CPhidgetAdvancedServo_setPosition (servo2, 0, dst_pos2);
//			CPhidgetAdvancedServo_setPosition (servo2, 0, 90);
			curr_pos2 = dst_pos2;
			continue;
		}
		else if(c == 100){
//			printw("d pressed\n");
			refresh();
			dst_pos2 = curr_pos2 + 5;
			CPhidgetAdvancedServo_setPosition (servo2, 0, dst_pos2);
//			CPhidgetAdvancedServo_setPosition (servo2, 0, 140);
			curr_pos2 = dst_pos2;
			continue;
		}
		else if(c == 114){
//			printw("r pressed\n");
			refresh();
			CPhidgetAdvancedServo_setPosition (servo1, 0, 115);
			CPhidgetAdvancedServo_setPosition (servo2, 0, 115);
			curr_pos1 = 115.00;
			curr_pos2 = 115.00;
			continue;
		}
		else if(c == 27){
			printw("esc pressed\n");
			refresh();		
			break;
		}
		else {
			printw("Command not recognized!!\n");
			refresh();
			continue;
		}
	}
	
	printw("Disengage Servo. Press any key to Continue\n");
	refresh();
	getch();

	CPhidgetAdvancedServo_setEngaged(servo1, 0, 0);
	CPhidgetAdvancedServo_setEngaged(servo2, 0, 0);

	printw("Press any key to end\n");
	refresh();
	getch();

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printw("Closing...\n");
	refresh();
	CPhidget_close((CPhidgetHandle)servo1);
	CPhidget_close((CPhidgetHandle)servo2);
	CPhidget_delete((CPhidgetHandle)servo1);
	CPhidget_delete((CPhidgetHandle)servo2);

	endwin();	// End curses mode

	//all done, exit
	return 0;
}

