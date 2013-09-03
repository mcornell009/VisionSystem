#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

void help(char** argv)
{
	cout << "\nDemonstrate mean-shift based color segmentation in spatial pyramid.\n"
    << "Call:\n   " << argv[0] << " image\n"
    << "This program allows you to set the spatial and color radius\n"
    << "of the mean shift window as well as the number of pyramid reduction levels explored\n"
    << endl;
}

//This colors the segmentations
void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
            }
        }
    }
}

string LwinName = "Left meanshift", RwinName = "Right meanshift", CwinName = "Control Window";
int spatialRad, colorRad, maxPyrLevel;
Mat Limg, Lres, Rimg, Rres;

void meanShiftSegmentation( int, void* )
{        
    pyrMeanShiftFiltering( Limg, Lres, spatialRad, colorRad, maxPyrLevel );
    floodFillPostprocess( Lres, Scalar::all(2) );
    //imshow( LwinName, Lres );
    
    pyrMeanShiftFiltering( Rimg, Rres, spatialRad, colorRad, maxPyrLevel );
    floodFillPostprocess( Rres, Scalar::all(2) );
    //imshow( RwinName, Rres );
}

int main(int argc, char** argv)
{
 /*   if( argc !=2 )
    {
    	help(argv);
        return -1;
    }

    img = imread( argv[1] );
    if( img.empty() )
        return -1;
*/

    spatialRad = 4;
    colorRad = 20;
    maxPyrLevel = 2;

    namedWindow( LwinName, CV_WINDOW_AUTOSIZE );
    namedWindow( RwinName, CV_WINDOW_AUTOSIZE );
    namedWindow( CwinName, CV_WINDOW_AUTOSIZE );
    namedWindow(CwinName, CV_WINDOW_AUTOSIZE);
    cvMoveWindow(LwinName.c_str(), 0,0);
    cvMoveWindow(RwinName.c_str(), 640,0);

    createTrackbar( "spatialRad", CwinName, &spatialRad, 80, meanShiftSegmentation );
    createTrackbar( "colorRad", CwinName, &colorRad, 60, meanShiftSegmentation );
    createTrackbar( "maxPyrLevel", CwinName, &maxPyrLevel, 5, meanShiftSegmentation );
    
    VideoCapture camera1 = VideoCapture(0); 
    VideoCapture camera2 = VideoCapture(1);
    if(!camera1.isOpened()) // check if we succeeded
    {
	cout << "Camera 0 not attached" << endl;
        return -1;
    }
    if(!camera1.isOpened()) // check if we succeeded
    {
    	cout << "Camera 1 not attached" << endl;
        return -1;
    }
    bool running = true;
    while(running)
    {
        camera1 >> Limg;
        camera2 >> Rimg;
    
        meanShiftSegmentation(0,0);
        
        imshow(LwinName, Lres);
        imshow(RwinName, Rres);   
    
         switch ( (char)waitKey(10) )
	    {
	    case 27: case 'q':  case 'Q':
	       	running = false;
	        break;
	    }
    }
    return 0;
}
