#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

Mat src; Mat src_gray; Mat grad;
int thresh = 100, picSwitchTC = 0;
int max_thresh = 255;
RNG rng(12345);
bool picSwitch;
float maxArea = 20.0f;

/// Function header
void thresh_callback(int, void* );

/** @function main */
int main( int argc, char** argv )
{
    /// Load source image and convert it to gray
    src = imread( argv[1], 1 );


    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    picSwitch = true;
    int c;


    /// Convert image to gray and blur it
    cvtColor( src, src_gray, CV_BGR2GRAY );
    blur( src_gray, src_gray, Size(3,3) );


    /// Create window
    //namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

    /// Create Window
    char* source_window = "Source";
    namedWindow( source_window, CV_WINDOW_AUTOSIZE );
    imshow( source_window, grad );

    createTrackbar( "Canny thresh: ", "Source", &thresh, max_thresh, thresh_callback );
    createTrackbar( "Pic Switch: ", "Source", &picSwitchTC, 1, thresh_callback );
    thresh_callback( 0, 0 );

    waitKey(0);
    return(0);
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
    Mat canny_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    picSwitchTC == 0 ? picSwitch = false : picSwitch = true;
    /// Detect edges using canny
    picSwitch ? Canny( grad, canny_output, thresh, thresh*2, 3 ) : Canny( src_gray, canny_output, thresh, thresh*2, 3 );
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
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 ), contours_poly;
    int i = 0;
    bool first = true;
    for( ; i < contours.size(); )
    {
        if (first)
        {
            cout << "running through loop" << endl;
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            approxPolyDP( Mat(contours[i]), contours_poly, 3, true );
            drawContours( drawing, contours, i, color, CV_FILLED, 8, hierarchy, 0, Point() );

            //fillPoly( drawing , const Point** pts, const int* npts, int ncontours, const Scalar& color, int lineType=8, int shift=0, Point offset=Point() )
            //cvDrawContours( drawing, contours, color, color, 0, 1, 8)
            //circle( drawing, mc[i], 4, color, -1, 8, 0 );

            /// Calculate the distances to the contour
            Mat raw_dist( src.size(), CV_32FC1 );
            
            for( int j = 0; j < src.rows; j++ )
            { for( int i2 = 0; i2 < src.cols; i2++ )
                { raw_dist.at<float>(j,i2) = pointPolygonTest( contours[i], Point2f(i2,j), true ); }
            }

            double minVal; double maxVal;
            minMaxLoc( raw_dist, &minVal, &maxVal, 0, 0, Mat() );
            minVal = abs(minVal); maxVal = abs(maxVal);

            /// Depicting the  distances graphically
            Mat drawing = Mat::zeros( src.size(), CV_8UC3 );

            for( int j = 0; j < src.rows; j++ )
            {
                for( int i2 = 0; i2 < src.cols; i2++ )
                {
                    if( raw_dist.at<float>(j,i2) < 0 )
                    { drawing.at<Vec3b>(j,i2)[0] = 255 - (int) abs(raw_dist.at<float>(j,i2))*255/minVal; }
                    else if( raw_dist.at<float>(j,i2) > 0 )
                    { drawing.at<Vec3b>(j,i2)[2] = 255 - (int) raw_dist.at<float>(j,i2)*255/maxVal; }
                    else
                    { drawing.at<Vec3b>(j,i2)[0] = 255; drawing.at<Vec3b>(j,i2)[1] = 255; drawing.at<Vec3b>(j,i2)[2] = 255; }
                }
            }

            /// Create Window and show your results
            //char* source_window = "Source";
            //namedWindow( source_window, CV_WINDOW_AUTOSIZE );
            //imshow( source_window, src );
            namedWindow( "Distance", CV_WINDOW_AUTOSIZE );
            imshow( "Distance", drawing );
            namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
            imshow( "Contours", drawing );
        }
        first = false;

        switch ( (char)waitKey(10) )
        {
        case 'i': case 'I':
            cout << i << endl;
            first = true;
            i++;
            break;
        }
        //waitKey(1000);
        
    }

    /// Show in a window


    /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
    /*printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours.size(); i++ )
     {
     if (mu[i].m00 > maxArea)
       {
       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
       //circle( drawing, mc[i], 4, color, -1, 8, 0 );
       }
     }*/
}
