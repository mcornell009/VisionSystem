#include "ssc.h"
#include <stdio.h>
#include <pthread.h>
#include "highgui.h"
#include "cv.h"
#include "cvaux.h"
#include <stdlib.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#define		PAN		7
#define		TILT	0
#define		UP		254
#define		DOWN	74
#define		LEFT	254
#define		RIGHT	0
#define		SLEEP	500000



// Sleep time affects movement speed, anything below 2400 is to fast
// for the serial connection
#define SLEEP_TIME      2400
#define SERVO_MAX		254		// If you want to limit servos motion
#define SERVO_MIN		0


void thread(void){
	ssc_init("/dev/ttyUSB0", 9600);
/*	ssc_pos(TILT, UP);
	ssc_pos(PAN, LEFT);
	usleep(SLEEP);
	ssc_pos(TILT, DOWN);
	usleep(SLEEP);
	ssc_pos(PAN, RIGHT);
	ssc_pos(TILT, UP);
	usleep(SLEEP);
	ssc_pos(TILT, DOWN);
	usleep(SLEEP);
	ssc_pos(PAN, LEFT);
	ssc_pos(TILT, UP);
	usleep(SLEEP);

	ssc_pos(PAN, RIGHT);
	usleep(SLEEP);
	ssc_pos(TILT, 224);
	ssc_pos(PAN, LEFT);
	usleep(SLEEP);
	ssc_pos(TILT, 194);
	ssc_pos(PAN, RIGHT);
	usleep(SLEEP);
	ssc_pos(TILT, 164);
	ssc_pos(PAN, LEFT);
	usleep(SLEEP);
	ssc_pos(TILT, 134);
	ssc_pos(PAN, RIGHT);
	usleep(SLEEP);
	ssc_pos(TILT, 104);
	ssc_pos(PAN, LEFT);
	usleep(SLEEP);
	ssc_pos(TILT, 74);(void *(*)(void *))
	ssc_pos(PAN, RIGHT);
	usleep(SLEEP);
	
	ssc_pos(PAN, 127);
	ssc_pos(TILT, 127);
*/



	int i;
	int j=0;
	float pan;
	float tilt;

	

	// figure eight motion
	while (j<1){
		// top-left -> bottom-left
		pan = 254.0;
		tilt = 254.0;
		i = 0;
		usleep(100000);
		ssc_pos(PAN, (int)pan);
		while (i<180){
			ssc_pos(TILT, (int)tilt);
			usleep(100000);
			tilt -= 1.0;
			i++;
		}
		pan = 254.0;
		tilt = 74.0;
		i = 0;
		ssc_pos(TILT,127);
		ssc_pos(PAN, 127);
		usleep(SLEEP);
		// bottom-left -> top-right
		while (i<180){
			ssc_pos(TILT, (int)tilt);
			ssc_pos(PAN, (int)pan);
			usleep(100000);
			pan -= 1.412;
			tilt += 1.0;
			i++;
		}
		pan = 0.0;
		tilt = 254.0;
		i = 0;
		
		// top-right -> bottom-right
		ssc_pos(PAN, (int)pan);
		while (i<180){
			ssc_pos(TILT, (int)tilt);
			usleep(100000);
			tilt -= 1.0;
			i++;
		}
		pan = 0.0;
		tilt = 74.0;
		i = 0;

		// bottom-right -> top-left
		while (i<180){
			ssc_pos(TILT, (int)tilt);
			ssc_pos(PAN, (int)pan);
			usleep(100000);
			pan += 1.412;
			tilt += 1.0;
			i++;
		}
		
		j++;
	}

	// side to side motion
	i = 0;
	j = 0;
	pan = 254.0;
	tilt = 254.0;
	ssc_pos(TILT,127);
	ssc_pos(PAN, 127);
	usleep(SLEEP);
	ssc_pos(PAN, (int)pan);
	ssc_pos(PAN, (int)pan);
	
	while(j<3) {
		i = 0;
		pan = 254.0;
		ssc_pos(TILT, (int)tilt);
		
		// left to right
		while (i<180) {
			ssc_pos(PAN, (int)pan);
			usleep(100000);
			pan -= 1.412;
			i++;
		}
		
		i = 0;
		tilt -= 30.0;
		ssc_pos(TILT, (int)tilt);
		
		// right to left
		while (i<180) {
			ssc_pos(PAN, (int)pan);
			usleep(100000);
			pan += 1.412;
			i++;
		}
		
		tilt -= 30.0;
		j++;
		
	}
	
	i = 0;
	pan = 254.0;
	ssc_pos(TILT, (int)tilt);
	while (i<180) {ssc_pos(TILT,127);
		ssc_pos(PAN, 127);
		usleep(SLEEP);
		ssc_pos(PAN, (int)pan);
		usleep(100000);
		pan -= 1.412;
		i++;
	}

	
	ssc_pos(TILT, 127);
	ssc_pos(PAN, 127);

	ssc_close();
}

int main(int argc, char** argv){
	pthread_t id;
	int ret;
	int done = 0;

	ret=pthread_create(&id,NULL,(void *(*)(void *))&thread,NULL);
	
	if(ret!=0){
		printf ("Create pthread error!\n");
		exit (1);
	}

	cvNamedWindow("Left Frame",CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Left Frame", 100, 100);
	cvNamedWindow("Right Frame",CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Right Frame", 420, 100);
	
	//CvCapture* capture;
	CvCapture* captureLeft;
	CvCapture* captureRight;
	
	if(argc == 1) {
	//capture = cvCreateCameraCapture(0);
		captureLeft = /*cvCaptureFromCAM(0);*/cvCreateCameraCapture(0);
		captureRight = /*cvCaptureFromCAM(1);*/cvCreateCameraCapture(1); 
		cvSetCaptureProperty(captureLeft, CV_CAP_PROP_FRAME_WIDTH, 320);
		cvSetCaptureProperty(captureLeft, CV_CAP_PROP_FRAME_HEIGHT, 240); 
		cvSetCaptureProperty(captureRight, CV_CAP_PROP_FRAME_WIDTH, 320);
		cvSetCaptureProperty(captureRight, CV_CAP_PROP_FRAME_HEIGHT, 240);
		if( !captureLeft ){
			printf( "Cannot open initialize WebCam 1\n" );
			return 1;
		}
		if( !captureRight ){
			printf( "Cannot open initialize WebCam 2\n" );
			return 1;
		} 
	} 
	else {
		captureLeft = cvCreateFileCapture(argv[1]);
	}
	
	IplImage* leftFrame;
	IplImage* rightFrame;
	
	while(1) {
		leftFrame = cvQueryFrame(captureLeft);
		rightFrame = cvQueryFrame(captureRight);
		if(!leftFrame) break;
		if(!rightFrame) break;
		cvShowImage("Right Frame",rightFrame);
		cvShowImage("Left Frame",leftFrame);
		char c = cvWaitKey(10);
		if(c==27){ done = 1; break;}
		if(done == 1) {
			cvWaitKey(0);
			cvReleaseImage(&leftFrame);
			cvReleaseImage(&rightFrame);
			cvDestroyWindow("Left Frame");
			cvDestroyWindow("Right Frame");
		}
	}

	pthread_join(id,NULL);

	cvWaitKey(0);
	cvReleaseCapture(&captureLeft);
	cvReleaseCapture(&captureRight);
	
	return (0);
} 

int fd;						// File descriptor for serial port
struct termios options;		// Serial port options

// open serial port, returns 0 if successful, 1 if error
// port: serial port device to open
// speed: port speed: 2400 or 9600 depending on jumper 'B'
int ssc_init(char *port, int speed)
{
	if (!( speed == 2400 || speed == 9600 )) {
		return EINVAL;
	}
	
    if (fd > 0) {
    	close(fd);
    }
    
    // open serial port
	fd = open(port, O_RDWR | O_NOCTTY | O_APPEND | O_NDELAY);
    
	if (fd < 0) {
		perror(port);
		return errno;
    }
	
    // following options don't seem to work with USB/Serial.
	if (!strstr (port, "USB")) {
		options.c_cflag |= CLOCAL;			// Local commms
		options.c_cflag |= CRTSCTS;			// hardware flow control
	}
	
	// set port speed
	if (speed == 9600) { // 9600
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
	} else {	// 2400 - default unjumpered
		cfsetispeed(&options, B2400);
		cfsetospeed(&options, B2400);
	}
	
	options.c_cflag &= ~PARENB;			// unset pairity : no pairity
	options.c_cflag &= ~CSTOPB;			// unset stop bit: 1 stop bit (or 2)
	options.c_cflag &= ~CSIZE;			// unset character size mask
	options.c_cflag |= CS8;				// 8 bits
	tcsetattr(fd, TCSANOW, &options);	// set attributes of port
	tcflush(fd, TCIOFLUSH);				// flush port
	return 0;
}

// reset and close port
int ssc_close() {
    cfmakeraw(&options);				// reset options
	tcsetattr(fd, TCSANOW, &options);	// apply options
    tcflush(fd, TCIOFLUSH);				// flush port
    return close(fd);					// close
}


// set position of a servo, returns 0 if successful, 1 if error.
// servo: servo number: 0-7 on one board
// position: position of that servo: 32-222

int ssc_pos(int servo, int position) {
	if (position < SERVO_MIN)
		position = SERVO_MIN;

	if (position > SERVO_MAX)
		position = SERVO_MAX;
	
	// 3 byte command
    char command[3];
    command[0] = 255;			// 1st byte must be 255
    command[1] = servo;			// then servo num
    command[2] = position;		// then the position
    
	if (write(fd, command, 3) == -1) {
		perror("Can't set position");
		return errno;
    }
    
	// need to sleep a bit between commands
	usleep(SLEEP_TIME);
    return 0;
}
