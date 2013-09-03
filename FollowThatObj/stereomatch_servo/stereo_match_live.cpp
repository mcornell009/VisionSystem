#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <pthread.h>
#include <phidget21.h>

#include <stdio.h>
#include <iostream>

using namespace cv;

void print_help()
{
    printf("\nDemo stereo matching converting L and R images into disparity and point clouds\n");
    printf("\nUsage: stereo_match <left_image> <right_image> [--algorithm=bm|sgbm|hh] [--blocksize=<block_size>]\n"
           "[--max-disparity=<max_disparity>] [-i <intrinsic_filename>] [-e <extrinsic_filename>]\n"
           "[--no-display] [-o <disparity_image>] [-p <point_cloud_file>]\n");
}

//void saveXYZBGR(const char* filename, const Mat& mat)
// Mat& img is the parameter for getting the color information from one of the paired images
void saveXYZ(const char* filename, const Mat& mat, const Mat& img)
{

    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            /* Retrieve RGB information, OpenCV stores RGB in BGR order*/
            // Stored the RGB information in char format
            Vec3b colorPoint = img.at<Vec3b>(y,x); // vector char vec3b; vetor int vec3i;
            unsigned char B = colorPoint[0];
            unsigned char G = colorPoint[1];
            unsigned char R = colorPoint[2];

            Vec3f point = mat.at<Vec3f>(y, x);

//            std::cout << point[0] << std::endl;

            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            // get the 3D points info
            //fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
            // get 3D points with RGB info
              fprintf(fp, "%f %f %f %d %d %d \n", point[0], point[1], point[2], R, G, B);
        }
    }
    fclose(fp);
}

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

        printf("%s\n", ptr);
        printf("Serial Number: %10d\nVersion: %8d\n# Motors: %d\n", serialNo, version, numMotors);

        return 0;
}

//CPhidgetAdvancedServoHandle servo1 = 0;
//CPhidgetAdvancedServoHandle servo2 = 0;
//int result1, result2;
//const char *err1;
//const char *err2;
//double curr_pos1;
//double curr_pos2;
//double dst_pos1;
//double dst_pos2;
//double minAccel1, maxVel1;
//double minAccel2, maxVel2;
//
//void thread(void) {
//    //create the advanced servo object
//    CPhidgetAdvancedServo_create(&servo1);
//    CPhidgetAdvancedServo_create(&servo2);
//
//    //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
//    CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo1, AttachHandler, NULL);
//    CPhidget_set_OnAttach_Handler((CPhidgetHandle)servo2, AttachHandler, NULL);
//    CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo1, DetachHandler, NULL);
//    CPhidget_set_OnDetach_Handler((CPhidgetHandle)servo2, DetachHandler, NULL);
//    CPhidget_set_OnError_Handler((CPhidgetHandle)servo1, ErrorHandler, NULL);
//    CPhidget_set_OnError_Handler((CPhidgetHandle)servo2, ErrorHandler, NULL);
//
//    //Registers a callback that will run when the motor position is changed.
//    //Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
//    CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo1, PositionChangeHandler, NULL);
//    CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo2, PositionChangeHandler, NULL);
//
//    //open the device for connections
//    CPhidget_open((CPhidgetHandle)servo1, -1);
//    CPhidget_open((CPhidgetHandle)servo2, -1);
//
//    //get the program to wait for an advanced servo device to be attached
//    printf("Waiting for Phidget 0 to be attached....");
//    if((result1 = CPhidget_waitForAttachment((CPhidgetHandle)servo1, 10000))) {
//            CPhidget_getErrorDescription(result1, &err1);
//            printf("Problem waiting for attachment 0: %s\n", err1);
////            return 0;
//    }
//    printf("Waiting for Phidget 1 to be attached....");
//    if((result2 = CPhidget_waitForAttachment((CPhidgetHandle)servo2, 10000))) {
//            CPhidget_getErrorDescription(result2, &err2);
//            printf("Problem waiting for attachment 1: %s\n", err2);
////            return 0;
//    }
//    printf("*************************\n");
//
//    //Display the properties of the attached device
//    display_properties(servo1);
//    display_properties(servo2);
//
//    //read event data
//    printf("Reading.....\n");
//
//    //This example assumes servo motor is attached to index 0
//
//    //Set up some initial acceleration and velocity values
//    CPhidgetAdvancedServo_getAccelerationMin(servo1, 0, &minAccel1);
//    CPhidgetAdvancedServo_getAccelerationMin(servo2, 0, &minAccel2);
//    CPhidgetAdvancedServo_setAcceleration(servo1, 0, minAccel1*2);
//    CPhidgetAdvancedServo_setAcceleration(servo2, 0, minAccel2*2);
//    CPhidgetAdvancedServo_getVelocityMax(servo1, 0, &maxVel1);
//    CPhidgetAdvancedServo_getVelocityMax(servo2, 0, &maxVel2);
//    CPhidgetAdvancedServo_setVelocityLimit(servo1, 0, maxVel1/2);
//    CPhidgetAdvancedServo_setVelocityLimit(servo2, 0, maxVel2/2);
//
//    //display current motor position
//    if(CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_pos1) == EPHIDGET_OK){
//            printf("Motor 1 Current Position: %f\n", curr_pos1);
//    }
//    if(CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_pos2) == EPHIDGET_OK){
//            printf("Motor 2 Current Position: %f\n", curr_pos2);
//    }
//
//    //keep displaying servo event data until user input is read
//    printf("Press any key to continue\n");
////    fflush(stdout);
////    waitKey();
//    printf("\n");
//
//    //change the motor position
//    //valid range is -23 to 232, but for most motors ~30-210
//    //we'll set it to a few random positions to move it around
//
//    printf("Move motor 1 to position 115.00 and engage. Press any key to Continue\n");
//    printf("Move motor 2 to position 115.00 and engage. Press any key to Continue\n");
////    fflush(stdout);
////    waitKey();
//    printf("\n");
//
//    CPhidgetAdvancedServo_setPosition (servo1, 0, 115.00);
//    CPhidgetAdvancedServo_setPosition (servo2, 0, 115.00);
//
//    CPhidgetAdvancedServo_setEngaged(servo1, 0, 1);
//    CPhidgetAdvancedServo_setEngaged(servo2, 0, 1);
//
//    printf("Press any key to start motor control...\n");
////    fflush(stdout);
////    waitKey();
//    printf("\n");
//    printf("Motor control started...\n");
//    printf("Use 'a, s, d, w' to control motors. 'r' to reset position.\n");
//
//    CPhidgetAdvancedServo_getPosition(servo1, 0, &curr_pos1);
//    CPhidgetAdvancedServo_getPosition(servo2, 0, &curr_pos2);
//}



int main(int argc, char** argv){
//    pthread_t id;
//    int ret;
//
//    ret=pthread_create(&id,NULL,(void *(*)(void *))&thread,NULL);
//
//    if(ret!=0){
//            printf ("Create pthread error!\n");
//            exit (1);
//    }

    const char* algorithm_opt = "--algorithm=";
    const char* maxdisp_opt = "--max-disparity=";
    const char* blocksize_opt = "--blocksize=";
    const char* nodisplay_opt = "--no-display=";

    if(argc < 3)
    {
        print_help();
        return 0;
    }
    const char* intrinsic_filename = 0;
    const char* extrinsic_filename = 0;
    const char* disparity_filename = 0;
    const char* point_cloud_filename = 0;

    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2 };
    int alg = STEREO_SGBM;
    int SADWindowSize = 0, numberOfDisparities = 0;
    bool no_display = false;

    StereoBM bm;
    StereoSGBM sgbm;

    for( int i = 1; i < argc; i++ )
    {
        if( strncmp(argv[i], algorithm_opt, strlen(algorithm_opt)) == 0 )
        {
            char* _alg = argv[i] + strlen(algorithm_opt);
            alg = strcmp(_alg, "bm") == 0 ? STEREO_BM :
                  strcmp(_alg, "sgbm") == 0 ? STEREO_SGBM :
                  strcmp(_alg, "hh") == 0 ? STEREO_HH : -1;
            if( alg < 0 )
            {
                printf("Command-line parameter error: Unknown stereo algorithm\n\n");
                print_help();
                return -1;
            }
        }
        else if( strncmp(argv[i], maxdisp_opt, strlen(maxdisp_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(maxdisp_opt), "%d", &numberOfDisparities ) != 1 ||
                numberOfDisparities < 1 || numberOfDisparities % 16 != 0 )
            {
                printf("Command-line parameter error: The max disparity (--maxdisparity=<...>) must be a positive integer divisible by 16\n");
                print_help();
                return -1;
            }
        }
        else if( strncmp(argv[i], blocksize_opt, strlen(blocksize_opt)) == 0 )
        {
            if( sscanf( argv[i] + strlen(blocksize_opt), "%d", &SADWindowSize ) != 1 ||
                SADWindowSize < 1 || SADWindowSize % 2 != 1 )
            {
                printf("Command-line parameter error: The block size (--blocksize=<...>) must be a positive odd number\n");
                return -1;
            }
        }
        else if( strcmp(argv[i], nodisplay_opt) == 0 )
            no_display = true;
        else if( strcmp(argv[i], "-i" ) == 0 )
            intrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-e" ) == 0 )
            extrinsic_filename = argv[++i];
        else if( strcmp(argv[i], "-o" ) == 0 )
            disparity_filename = argv[++i];
        else if( strcmp(argv[i], "-p" ) == 0 )
            point_cloud_filename = argv[++i];
        else
        {
            printf("Command-line parameter error: unknown option %s\n", argv[i]);
            return -1;
        }
    }

    if( (intrinsic_filename != 0) ^ (extrinsic_filename != 0) )
    {
        printf("Command-line parameter error: either both intrinsic and extrinsic parameters must be specified, or none of them (when the stereo pair is already rectified)\n");
        return -1;
    }

    if( extrinsic_filename == 0 && point_cloud_filename )
    {
        printf("Command-line parameter error: extrinsic and intrinsic parameters must be specified to compute the point cloud\n");
        return -1;
    }

//    int color_mode = alg == STEREO_BM ? 0 : -1;

    //set up the resolution
//        int frameW  = 640;
//        int frameH  = 480;
    int frameW  = 320;
    int frameH  = 240;

    //create windows for left and right webcam, depthmap and fake-color image
    namedWindow("Left Frame", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Left Frame", 50, 100);
    namedWindow("Right Frame", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Right Frame", 400, 100);

    namedWindow("Disparity", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("Disparity", 800,100);

    //create webcam input/capture
    CvCapture* captureLeft;
    CvCapture* captureRight;

    IplImage* leftFrame = 0;
    IplImage* rightFrame = 0;

    captureLeft = cvCreateCameraCapture( 0 ); //capture frames from cam on index 0: /dev/video0/
    captureRight = cvCreateCameraCapture( 1 ); //capture frames from cam on index 1: /dev/video1

    //set the resolution of LeftCam
    cvSetCaptureProperty(captureLeft, CV_CAP_PROP_FRAME_WIDTH, frameW);
    cvSetCaptureProperty(captureLeft, CV_CAP_PROP_FRAME_HEIGHT, frameH);

    //set the resolution of RightCam
    cvSetCaptureProperty(captureRight, CV_CAP_PROP_FRAME_WIDTH, frameW);
    cvSetCaptureProperty(captureRight, CV_CAP_PROP_FRAME_HEIGHT, frameH);

    assert( captureLeft ); // assert for testing leftcam is working or not
    assert( captureRight ); // assert for testing rightcam is working or not

    //read frames from cache
    leftFrame = cvQueryFrame(captureLeft);
    rightFrame = cvQueryFrame(captureRight);

    Mat img1, img2;
    if(alg == STEREO_BM){
        cvtColor(leftFrame, img1, CV_RGB2GRAY);
        cvtColor(rightFrame, img2, CV_RGB2GRAY);
    }
    else{
        img1 = leftFrame;
        img2 = rightFrame;
    }

    Size img_size = img1.size();

    Mat map11, map12, map21, map22;

    Rect roi1, roi2;
    Mat Q;

    if( intrinsic_filename )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename);
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        fs.open(extrinsic_filename, CV_STORAGE_READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename);
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, -1, img_size, &roi1, &roi2 );

        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
    }

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : img_size.width/8;

    bm.state->roi1 = roi1;
    bm.state->roi2 = roi2;
    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
    bm.state->minDisparity = 0;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;

    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = img1.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = bm.state->speckleWindowSize;
    sgbm.speckleRange = bm.state->speckleRange;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = alg == STEREO_HH;

    Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    Mat img1r, img2r;

    if( !no_display ){
        while(1){
            remap(img1, img1r, map11, map12, INTER_LINEAR);
            remap(img2, img2r, map21, map22, INTER_LINEAR);

            if( alg == STEREO_BM )
                bm(img1, img2, disp);
            else
                sgbm(img1, img2, disp);

            //disp = dispp.colRange(numberOfDisparities, img1p.cols);
            disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

            imshow("Left Frame", img1);
            imshow("Right Frame", img2);
            imshow("Disparity", disp8);

            leftFrame = cvQueryFrame(captureLeft);
            rightFrame = cvQueryFrame(captureRight);

            if(alg == STEREO_BM){
                cvtColor(leftFrame, img1, CV_RGB2GRAY);
                cvtColor(rightFrame, img2, CV_RGB2GRAY);
            }
            else{
                img1 = leftFrame;
                img2 = rightFrame;
            }

            int c = cvWaitKey( 10 );
            if( c == 27 )
                break;
        }
    }

//    pthread_join(id,NULL);

    if(disparity_filename)
        imwrite(disparity_filename, disp8);

    if(point_cloud_filename)
    {
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename, xyz, img1);
        printf("\n");
    }

    return 0;
}
