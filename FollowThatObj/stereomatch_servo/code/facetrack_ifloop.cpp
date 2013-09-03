
#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include "assert.h"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

CvHaarClassifierCascade *cascadeLeft;
CvHaarClassifierCascade *cascadeRight;
CvMemStorage            *storageLeft;
CvMemStorage            *storageRight;
IplImage* leftFrame;
IplImage* rightFrame;
 
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

void detectFaces( IplImage *img, CvHaarClassifierCascade *cascade, CvMemStorage *storage, char *framename );
int ssc_init(char *port, int speed);
int ssc_close();
int ssc_pos(int servo, int position);
int pan_pos = 128, tilt_pos = 128;
void pan(int p);
void tilt(int p);

int main( int argc, char** argv )

{
    ssc_init("/dev/ttyUSB0", 9600);
    CvCapture* captureLeft;
    CvCapture* captureRight;
    
    int       key;
    char      *filename1 = "/home/robotlab/opencv/OpenCV-2.1.0/data/haarcascades/haarcascade_frontalface_alt.xml";
    char      *filename2 = "/home/robotlab/opencv/OpenCV-2.1.0/data/haarcascades/haarcascade_frontalface_alt.xml";
    /* load the classifier
       note that I put the file in the same directory with this code 
    */

    cascadeLeft = ( CvHaarClassifierCascade* )cvLoad(filename1, 0, 0, 0 );
    cascadeRight = ( CvHaarClassifierCascade* )cvLoad(filename2, 0, 0, 0 );
    /* setup memory buffer; needed by the face detector */
    storageLeft = cvCreateMemStorage( 0 );
    storageRight = cvCreateMemStorage( 0 );
    /* initialize cameras */
    captureLeft = /*cvCaptureFromCAM(0);*/cvCreateCameraCapture(0);
    captureRight = /*cvCaptureFromCAM(1);*/cvCreateCameraCapture(1);

    cvSetCaptureProperty(captureLeft, CV_CAP_PROP_FRAME_WIDTH, 320);
    cvSetCaptureProperty(captureLeft, CV_CAP_PROP_FRAME_HEIGHT, 240);

    cvSetCaptureProperty(captureRight, CV_CAP_PROP_FRAME_WIDTH, 320);
    cvSetCaptureProperty(captureRight, CV_CAP_PROP_FRAME_HEIGHT, 240);
    /* always check */
    assert( cascadeLeft && storageLeft && captureLeft );
    assert( cascadeRight && storageRight && captureRight );
    /* create windows */
    cvNamedWindow("Left Frame",CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Left Frame", 100, 100);
    cvNamedWindow("Right Frame",CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Right Frame", 420, 100);
    while( key != 'q' ) {
        /* get a frame */
		leftFrame = cvQueryFrame(captureLeft);
		rightFrame = cvQueryFrame(captureRight);
        /* always check */
        if(!leftFrame) break;
		if(!rightFrame) break;
        
		cvClearMemStorage( storageLeft );
		cvClearMemStorage( storageRight );
		/* 'fix' frame */
        cvFlip( leftFrame, leftFrame, -1 );
		cvFlip( rightFrame, rightFrame, -1 );
        leftFrame->origin = 0;
		rightFrame->origin=0;
        /* detect faces and display video */
        detectFaces( leftFrame, cascadeLeft, storageLeft, "Left Frame" );
		detectFaces( rightFrame, cascadeRight, storageRight, "Right Frame" );
        /* quit if user press 'q' */
        key = cvWaitKey( 10 );
    }

    /* free memory */
    cvReleaseImage(&leftFrame);
    cvReleaseImage(&rightFrame);
    cvDestroyWindow("Left Frame");
    cvDestroyWindow("Right Frame");
    cvReleaseCapture(&captureLeft);
    cvReleaseCapture(&captureRight);
    cvReleaseHaarClassifierCascade( &cascadeLeft );
    cvReleaseHaarClassifierCascade( &cascadeRight );
    cvReleaseMemStorage( &storageLeft );
    cvReleaseMemStorage( &storageRight );
    return 0;
}

void detectFaces( IplImage *img, CvHaarClassifierCascade *cascade, CvMemStorage *storage, char *framename )
{
    	//int i;
	float multip = 0.1;
	int x_diff, y_diff;

    /* detect faces */
    CvSeq *faces = cvHaarDetectObjects(img, cascade, storage, 1.1, 3, 0 /*CV_HAAR_DO_CANNY_PRUNNING*/, cvSize( 40, 40 ) );
    
	
	/* for each face found, draw a red box */
/*    for( i = 0 ; i < ( faces ? faces->total : 0 ) ; i++ ) {
		CvRect *r = ( CvRect* )cvGetSeqElem( faces, i );
        cvRectangle( img, cvPoint( r->x, r->y ),
		cvPoint( r->x + r->width, r->y + r->height ),
		CV_RGB( 255, 0, 0 ), 1, 8, 0 );
	
		x_diff = (img->width / 2) - (r->x + (r->width/2)) ;
		y_diff = (img->height / 2) - (r->y + (r->height/2)) ;

		pan( -(int)(x_diff * multip) );
		tilt( (int)(y_diff * multip) );
    }
*/	
    
	//Or try this loop
	if (faces->total > 0) {
		CvRect *r = (CvRect*)cvGetSeqElem( faces, 0 );
		cvRectangle( img, cvPoint( r->x, r->y ), cvPoint( r->x + r->width, r->y + r->height ), CV_RGB( 255, 0, 0 ), 1, 8, 0 );
			
		x_diff = (img->width / 2) - (r->x + (r->width/2)) ;
        y_diff = (img->height / 2) - (r->y + (r->height/2)) ;

        pan( -(int)(x_diff * multip) );
        tilt( (int)(y_diff * multip) );

	}

	
	/* display video */
    cvShowImage(framename,img);
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

int ssc_init(char *port, int speed)
{	
	
	if (!( speed == 2400 || speed == 9600 )) {return EINVAL;}
	if (fd > 0) {close(fd);}
    
    // open serial port
	fd = open(port, O_RDWR | O_NOCTTY | O_APPEND | O_NDELAY);
    
	if (fd < 0){
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
	if (position < SERVO_MIN){position = SERVO_MIN;}
	if (position > SERVO_MAX){position = SERVO_MAX;}
	
	// 3 byte command
	char command[3];
	     command[0] = 255;			// 1st byte must be 255
	     command[1] = servo;			// then servo num
	     command[2] = position;		// then the position
	    
	if (write(fd, command, 3) == -1) {
		perror("Can't set position");
		// printf("can't set position");
		return errno; 
    		}
    
	// need to sleep a bit between commands
	usleep(SLEEP_TIME);
    return 0;
}

