#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <boost/thread.hpp>
#include <iostream>
#include <stdio.h>


//#include <math.h>

using namespace cv;
using namespace std;


class Disparity {
private:
	Mat_ cameraMatrix[2], distCoeffs[2];	

public:
	void pointDisparity(Point2f point);
	Disparity();
	
};

Disparity::Disparity()
{
	string intrinsic = "../calibData/intrinsics.yml";
    string extrinsic = "../calibData/extrinsics.yml";
    FileStorage fs(intrinsic, CV_STORAGE_READ);

    if(!fs.isOpened())
    {
        printf("Failed to open intrinsic file\n"); //" %s\n", intrinsic);
        fs.release();
    }

	Mat cameraMatrixOrig[2], distCoeffsOrig[2];	
    fs["M1"] >> cameraMatrixOrig[0];
    fs["D1"] >> distCoeffsOrig[0];
    fs["M2"] >> cameraMatrixOrig[1];
    fs["D2"] >> distCoeffsOrig[1];
    
    fs.release();
    
    cameraMatrix = cameraMatrixOrig;
    distCoeffs = distCoeffs;
}

void Disparity::pointDisparity(Point2f point)
{
//	atan2(px-x,fx)
//    float px0 = cameraMatrix[0][0][2];
	cout << cameraMatrix[0].at(0,2) << endl;
//    atan2(cameraMatrix[0][0][2]-point.x, cameraMatrix[0][0][0] )
//    atan2(cameraMatrix[0][1][2]-point.x, cameraMatrix[0][0][0] )
//	atan2(py-y,fy)
}

main()
{
	Disparity disp;
	Point2f testpoint;
	testpoint.x = 160, testpoint.y = 120;
	disp.pointDisparity(testpoint);
	//cout << "X/Y distance" << 
}
