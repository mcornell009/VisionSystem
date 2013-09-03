#include <iostream>
#include <stdio.h>
#include <phidget21.h>


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

class MotorControl
{
    private:
		
	public:
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
		bool motorsOn;
        MotorControl();
        void stop();
        void moveToXY(int &x, int &y);
        void moveToXY(Point2f& coor);

};

MotorControl::MotorControl()
{
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
                
	}
        printf("Waiting for Phidget 1 to be attached....");
        if((result2 = CPhidget_waitForAttachment((CPhidgetHandle)servo2, 10000))) {
		CPhidget_getErrorDescription(result2, &err2);
                printf("Problem waiting for attachment 1: %s\n", err2);
                
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

        CPhidgetAdvancedServo_setPosition (servo1, 0, 135.00);
        CPhidgetAdvancedServo_setPosition (servo2, 0, 115.00);

	CPhidgetAdvancedServo_setEngaged(servo1, 0, 1);
	CPhidgetAdvancedServo_setEngaged(servo2, 0, 1);
}

void MotorControl::moveToXY(int& x, int& y)
{
    // I estimated roughly that the conversion between rotation value was 40 motor positions horizontial for the left to right
    // and then 30 vertically by the ratio
    double diffx = ((double)x - 320.0) * (40.0 / 640.0), curr_posX = 0;
    CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_posX);
	CPhidgetAdvancedServo_setPosition (servo2, 0, curr_posX + diffx);
	
	double diffy = ((double)y - 240.0) * (30.0 / 480.0), curr_posY = 0;
	CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_posY);
    CPhidgetAdvancedServo_setPosition (servo1, 0, curr_posY + diffy);
    //cout << "x / curr_posX / diffx " << x << "\t" << curr_posX << "\t" << diffx << endl;
    //cout << "y / curr_posY / diffy " << y << "\t" << curr_posY << "\t" << diffy << endl;
    x = 320, y = 240;
}

void MotorControl::moveToXY(Point2f& coor)
{
    // I estimated roughly that the conversion between rotation value was 40 motor positions horizontial for the left to right
    // and then 30 vertically by the ratio
    double x = coor.x, y = coor.y;
    double diffx = (x - 320.0) * (40.0 / 640.0), curr_posX = 0;
    CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_posX);
	CPhidgetAdvancedServo_setPosition (servo2, 0, curr_posX + diffx);
	
	double diffy = -(y - 240.0) * (30.0 / 480.0), curr_posY = 0;
	CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_posY);
    CPhidgetAdvancedServo_setPosition (servo1, 0, curr_posY + diffy);
    //cout << "x / curr_posX / diffx " << x << "\t" << curr_posX << "\t" << diffx << endl;
    //cout << "y / curr_posY / diffy " << y << "\t" << curr_posY << "\t" << diffy << endl;
    x = 320, y = 240;
}

void MotorControl::stop()
{
	CPhidgetAdvancedServo_setEngaged(servo1, 0, 0);
	CPhidgetAdvancedServo_setEngaged(servo2, 0, 0);

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
        printf("Closing...\n");
        CPhidget_close((CPhidgetHandle)servo1);
	CPhidget_close((CPhidgetHandle)servo2);
	CPhidget_delete((CPhidgetHandle)servo1);
	CPhidget_delete((CPhidgetHandle)servo2);

}
 
 
//*/
