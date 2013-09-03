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

//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>
//#include <assert.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <phidget21.h>
#include <ncurses.h>

using namespace std;
using namespace cv;


// get the attached servo serial number-- from AdvancedServo.c
int CCONV AttachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
        printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

// get the detach device serial number -- from AdvancedServo.c
int CCONV DetachHandler(CPhidgetHandle ADVSERVO, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (ADVSERVO, &name);
	CPhidget_getSerialNumber(ADVSERVO, &serialNo);
        printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

// print error if there is no attached device-- from AdvancedServo.c
int CCONV ErrorHandler(CPhidgetHandle ADVSERVO, void *userptr, int ErrorCode, const char *Description)
{
        printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}

// show current position when initiate -- from AdvancedServo.c
int CCONV PositionChangeHandler(CPhidgetAdvancedServoHandle ADVSERVO, void *usrptr, int Index, double Value)
{
//        printf("Motor: %d > Current Position: %f\n", Index, Value);
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

        printf("%s\n", ptr);
        printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

	return 0;
}

int main(int argc, char* argv[])
{
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
        printf("Waiting for Phidget 0 to be attached....");
        if((result1 = CPhidget_waitForAttachment((CPhidgetHandle)servo1, 10000))) {
		CPhidget_getErrorDescription(result1, &err1);
                printf("Problem waiting for attachment 0: %s\n", err1);
                return 0;
	}
        printf("Waiting for Phidget 1 to be attached....");
        if((result2 = CPhidget_waitForAttachment((CPhidgetHandle)servo2, 10000))) {
		CPhidget_getErrorDescription(result2, &err2);
                printf("Problem waiting for attachment 1: %s\n", err2);
                return 0;
	}

	//Display the properties of the attached device
	display_properties(servo1);
	display_properties(servo2);

	//read event data
        printf("Reading.....\n");

	//This example assumes servo motor is attached to index 0

	//Set up some initial acceleration and velocity values
	CPhidgetAdvancedServo_getAccelerationMin(servo1, 0, &minAccel1);
	CPhidgetAdvancedServo_getAccelerationMin(servo2, 0, &minAccel2);
        CPhidgetAdvancedServo_setAcceleration(servo1, 0, minAccel1*4);
        CPhidgetAdvancedServo_setAcceleration(servo2, 0, minAccel2*4);
	CPhidgetAdvancedServo_getVelocityMax(servo1, 0, &maxVel1);
	CPhidgetAdvancedServo_getVelocityMax(servo2, 0, &maxVel2);
//	CPhidgetAdvancedServo_setVelocityLimit(servo1, 0, maxVel1/2);
//	CPhidgetAdvancedServo_setVelocityLimit(servo2, 0, maxVel2/2);
        CPhidgetAdvancedServo_setVelocityLimit(servo1, 0, maxVel1*2);
        CPhidgetAdvancedServo_setVelocityLimit(servo2, 0, maxVel2*2);

	//display current motor position
//	if(CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_pos1) == EPHIDGET_OK){
//                printf("Motor 1 Current Position: %f\n", curr_pos1);
//	}
//	if(CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_pos2) == EPHIDGET_OK){
//		printf("Motor 2 Current Position: %f\n", curr_pos2);
//	}

	//keep displaying servo event data until user input is read

	//change the motor position
	//valid range is -23 to 232, but for most motors ~30-210
	//we'll set it to a few random positions to move it around
        CPhidgetAdvancedServo_setEngaged(servo1, 0, 0);
        CPhidgetAdvancedServo_setEngaged(servo2, 0, 0);
        printf("Move motor 1 to position 115.00 and engage. Press any key to Continue\n");
        printf("Move motor 2 to position 115.00 and engage. Press any key to Continue\n");
        getchar();

        CPhidgetAdvancedServo_setPosition (servo1, 0, 135.00);
        CPhidgetAdvancedServo_setPosition (servo2, 0, 115.00);

	CPhidgetAdvancedServo_setEngaged(servo1, 0, 1);
	CPhidgetAdvancedServo_setEngaged(servo2, 0, 1);

        printf("Press any key to start tracking...");
        getchar();
        CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_pos1);
        CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_pos2);

        int x, y, x_diff, y_diff;

        int x_prev, x_curr, y_prev, y_curr;

        FILE * center_file;

        int idle_count = 0;

        while(1){
//            double vel1, vel2;
//            CPhidgetAdvancedServo_getVelocity(servo1, 0, &vel1);
//            CPhidgetAdvancedServo_getVelocity(servo1, 0, &vel2);

//            if(vel1 != 0 || vel2 != 0) {
                center_file = fopen ("face_center.txt","r");
                if(center_file) {
                    char line[128];
                    while(fgets(line, sizeof(line), center_file) != NULL) {
                        sscanf (line,"%d %*d", &x);
                        sscanf (line,"%*d %d", &y);
                    }

                    printf("x:%d\n", x);
                    printf("y:%d\n", y);
                    fclose(center_file);
                }else {
                    printf("cannot read file\n");
                    continue;
                }

                x_curr = x;
                y_curr = y;

//                if((x_curr - x_prev) == 0 && (y_curr - y_prev) == 0) {
//                    idle_count++;
//                    printf("idle_count:%d\n", idle_count);
//                    if(idle_count > 100) {
//                        CPhidgetAdvancedServo_setPosition (servo1, 0, 135.00);
//                        CPhidgetAdvancedServo_setPosition (servo2, 0, 115.00);
//                    }
//                }

                x_diff = 160 - x;
                y_diff = 120 - y;

                if(abs(x_diff) <=50 && abs(y_diff) <= 50)
                    continue;

                dst_pos1 = curr_pos1 + (y_diff * 0.05);
                printf("dst_pos1_org: %f\n", dst_pos1);
                if(dst_pos1 > 160)
                    dst_pos1 = 160;
                if(dst_pos1 < 120)
                    dst_pos1 = 120;
                CPhidgetAdvancedServo_setPosition (servo1, 0, dst_pos1);
                CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_pos1);
                printf("dst_pos1: %f\n", dst_pos1);

                dst_pos2 = curr_pos2 - (x_diff * 0.07);
                if(dst_pos2 > 160)
                    dst_pos2 = 160;
                if(dst_pos2 < 70)
                    dst_pos2 = 70;
                CPhidgetAdvancedServo_setPosition (servo2, 0, dst_pos2);
                CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_pos2);
                printf("dst_pos2: %f\n", dst_pos2);

                x_prev = x_curr;
                y_prev = y_curr;

//                usleep(2000);
//            }else
//                continue;

//            if(getchar())
//                break;
        }
	
        printf("Disengage Servo. Press any key to Continue\n");

	CPhidgetAdvancedServo_setEngaged(servo1, 0, 0);
	CPhidgetAdvancedServo_setEngaged(servo2, 0, 0);

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
        printf("Closing...\n");
        CPhidget_close((CPhidgetHandle)servo1);
	CPhidget_close((CPhidgetHandle)servo2);
	CPhidget_delete((CPhidgetHandle)servo1);
	CPhidget_delete((CPhidgetHandle)servo2);

	//all done, exit
	return 0;
}

