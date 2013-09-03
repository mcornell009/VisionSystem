#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

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

int thresh = 100;
int max_thresh = 255;
Mat original;
void meanShiftSegmentation( int, void* );
bool bOrig;
//This colors the segmentations
void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
    
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat src_gray, canny_output;
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
    cvtColor( img, src_gray, CV_BGR2GRAY );
    Canny( src_gray, canny_output, thresh, thresh*2, 3 );
    copyMakeBorder( canny_output, mask, 1, 1, 1, 1, BORDER_REPLICATE, 1 );
    
    int numregions = 0;
    vector<Rect> regions;
    Rect region;
    Mat temp2;
    original.copyTo(temp2);
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, &region, colorDiff, colorDiff, 4 );

                if ((region.height * region.width > 250))
                {
                rectangle( temp2, Point(region.x,region.y), Point(region.x + region.width, region.y + region.height), Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ), 2, 8, 0 );
                regions.push_back(region);                
                numregions++;
                }
                
            }
        }
    }
    cout << "Number of regions: " << numregions << "\tSize of Region Vector: " << regions.size() << endl;
    imshow( "meanshift", img );
    imshow( "Regions ROIs", temp2);
    
    createTrackbar( " Canny thresh:", "Mask", &thresh, max_thresh, meanShiftSegmentation );
    
    vector<Rect>::iterator it;
    
    for ( it = regions.begin() ; it < regions.end() ; )
    {
              
        switch ( (char)waitKey(10) )
	    {
	    case 27: case 'q':  case 'Q':
	       	it = regions.end();
	        break;
	    case 'i': case 'I':
	        //imshow("Original", img);
	        imshow("ROI", img( *it ));
	        Mat temp;
	        original.copyTo(temp);
	        region = *it;
	        cout << "Region[" << numregions << "] Values (x,y,row,cols): " << region.x << "\t\t" << region.y << "\t\t" << region.height << "\t\t" << region.width << endl;
	        rectangle( temp, Point(region.x,region.y), Point(region.x + region.width, region.y + region.height), Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ), 2, 8, 0 );
	        imshow( "meanshift", temp );
	        it++;
	        break;
	    /*case 'o': case 'O':
	        bOrig ? destroyWindow("Original") : imshow( "Original", original );
	        bOrig ? bOrig = false : bOrig = true;
	        break;*/
	    
	    }
    
    }
}

string winName = "meanshift";
int spatialRad, colorRad, maxPyrLevel, threshold1, threshold2;
Mat img, res, src_gray;
RNG rng(12345);
Mat drawing;

CvSeq *comp;
CvMemStorage *storage;

void thresh_callback(int, void* )
{
  Mat canny_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using canny
  Canny( src_gray, canny_output, thresh, thresh*2, 3 );
  imshow( "Source", canny_output );
  /// Find contours  
  findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Get the moments
  vector<Moments> mu(contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mu[i] = moments( contours[i], false ); }

  ///  Get the mass centers: 
  vector<Point2f> mc( contours.size() );
  for( int i = 0; i < contours.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  /// Draw contours
  drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     { 
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       if( mu[i].m00 > 5 ) drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() ); 
       //circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }
  /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
  printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );  
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      // drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() ); 
      // circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }

}


void findContours()
{

  /// Convert image to gray and blur it
  cvtColor( res, src_gray, CV_BGR2GRAY );
  //blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
//  char* source_window = "Source";
//  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  

  
  thresh_callback( 0, 0 );

}

void meanShiftSegmentation( int, void* )
{
    cout << "spatialRad=" << spatialRad << "; "
         << "colorRad=" << colorRad << "; "
         << "maxPyrLevel=" << maxPyrLevel << endl;
    pyrMeanShiftFiltering( img, res, spatialRad, colorRad, maxPyrLevel );
    floodFillPostprocess( res, Scalar::all(2) );
    //cvPyrSegmentation(res, res, storage, &comp, maxPyrLevel, threshold1+1, threshold2+1);
    imshow( winName, res );
    //findContours();    
    //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    //imshow( "Contours",  drawing);
}


int main(int argc, char** argv)
{

    if( argc !=2 )
    {
    	help(argv);
        return -1;
    }
    
    img = imread( argv[1] );
    if( img.empty() )
        return -1;

    img.copyTo(original);
    spatialRad = 4;
    colorRad = 20;
    maxPyrLevel = 2;
    bOrig = true;
    //imshow( "Original", original );

    namedWindow( winName, CV_WINDOW_AUTOSIZE );
    
    threshold1 =255;
    threshold2 =30;

    createTrackbar( "spatialRad", winName, &spatialRad, 80, meanShiftSegmentation );
    createTrackbar( "colorRad", winName, &colorRad, 60, meanShiftSegmentation );
    createTrackbar( "maxPyrLevel", winName, &maxPyrLevel, 5, meanShiftSegmentation );
    //createTrackbar("Threshold1", winName, &threshold1, 255, meanShiftSegmentation);
    //createTrackbar("Threshold2", winName,  &threshold2, 255, meanShiftSegmentation);


    meanShiftSegmentation(0, 0);
    waitKey();
    return 0;
}
