#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include "highgui.h"
#include "cv.h"
#include "cvaux.h"
// Sleep time affects movement speed, anything below 2400 is to fast
// for the serial connection
#define SLEEP_TIME      2400
#define SERVO_MAX	254		// If you want to limit servos motion
#define SERVO_MIN	0

// define PAN and TILT POSITION
#define		PAN	7
#define		TILT	0
#define		UP	254
#define		DOWN	74
#define		LEFT	254
#define		RIGHT	0

//define intervals between PAN and TILT
#define		SLEEP	500000

//declare the ssc_init, ssc_pos, ssc_close
int ssc_init(char *port, int speed);
int ssc_close();
int ssc_pos(int servo, int position);
CvMemStorage* storage;
CvHaarClassifierCascade* cascade;
int pan_pos = 128, tilt_pos = 128;
void pan(int p);
void tilt(int p);

int main() {

	ssc_init("/dev/ttyUSB0", 9600);
	pan( 0 );
	tilt( 0 );

	cascade = (CvHaarClassifierCascade*)cvLoad(
		"cascade.xml", 0, 0, 0 );
	if( !cascade )
    {
        fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
        return 1;
    }
	storage = cvCreateMemStorage(0);

	struct camera *cam = init_camera( LCAM );
	load_params( cam );

	cvNamedWindow( "Face", CV_WINDOW_AUTOSIZE );

	CvPoint pt1, pt2;
	CvRect *r;
	CvSeq *faces;
	float multip = 0.1;
	int x_diff, y_diff;

	while( cvWaitKey( 10 ) == -1 ) {

		cap_frame( cam );
		undistort( cam );
		
		cvClearMemStorage( storage );
		faces = cvHaarDetectObjects( cam->undist, cascade, storage,
        	1.1, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(40, 40) );

		if (faces->total > 0) {
;
			r = (CvRect*)cvGetSeqElem( faces, 0 );
			pt1.x = r->x;
			pt2.x = r->x + r->width;
			pt1.y = r->y;
			pt2.y = r->y + r->height;
			cvRectangle( cam->undist, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
			
			x_diff = (cam->frame->width / 2) - (r->x + (r->width/2)) ;
            y_diff = (cam->frame->height / 2) - (r->y + (r->height/2)) ;

            pan( -(int)(x_diff * multip) );
            tilt( (int)(y_diff * multip) );

		}

		cvShowImage( "Face", cam->undist );

	}
	cvDestroyAllWindows();
	cvReleaseMemStorage( &storage );
	free_camera( cam );
}

void pan( int p ){
    if ((pan_pos+p) > 255)
        pan_pos = 255;
    else if ((pan_pos+p) < 0)
        pan_pos = 0;
    else
        pan_pos += p;
    ssc_pos(7, pan_pos);
}

void tilt( int p ){
    if ((tilt_pos+p) > 255)
        tilt_pos = 255;
    else if ((tilt_pos+p) < 75)
        tilt_pos = 75;
    else
        tilt_pos += p;
    ssc_pos(0, tilt_pos);
}

int fd;				// File descriptor for serial port
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
	tcsetattr(fd, TCSANOW, &options);		// apply options
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


