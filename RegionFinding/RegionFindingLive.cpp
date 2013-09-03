#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>

#include "ObjRecClassRF.cpp"

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
bool running, display, rects, paused;
Mat original;
int thresh, lineFinder, fileCount;
void meanShiftSegmentation( int, void* );
ObjectRecognition ObjRec;

//This colors the segmentations
void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{    
    
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat src_gray, canny_output, mask;
    cvtColor( img, src_gray, CV_BGR2GRAY );
    
    Canny( src_gray, canny_output, thresh, 2*thresh, 3 );
    
    
    /*//If other line finders uncomment the switch below
    Mat dst;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    int kernel_size = 3; 
    	    Laplacian( src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
    	    convertScaleAbs( dst, canny_output );
    /*
    
    switch (lineFinder)
    {
        case '0': 
            Canny( src_gray, canny_output, thresh, 2*thresh, 3 );
            break;
    
    // Other Line finding For mask ... Not useful but looks cool
      // Laplacian
        case '1':
            Laplacian( src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT );
    	    convertScaleAbs( dst, canny_output );
    	    break;
    	case '2':   
    	    //Sobel
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
	    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, canny_output );
	    break;
    }
    */
    
    copyMakeBorder( canny_output, mask, 1, 1, 1, 1, BORDER_REPLICATE, 1 );
    int numregions = 0;
    vector<Rect> regions;
    vector<Mat> Masks; 
    Rect region;
    //Mat temp2;
    //original.copyTo(temp2);
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, &region, colorDiff, colorDiff,  4 );  //FLOODFILL_MASK_ONLY +
                if ((region.height * region.width > 250))
                {
                if(rects)  rectangle( img, Point(region.x,region.y), Point(region.x + region.width, region.y + region.height), Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ), 2, 8, 0 );
                regions.push_back(region);
                Masks.push_back(mask);                
                numregions++;
                }
                
            }
        }
    }
    //cout << "Number of regions: " << numregions << "\tSize of Region Vector: " << regions.size() << endl;
    //imshow( "meanshift", img );
    //imshow( "Regions ROIs", temp2);
    
//    createTrackbar( " Canny thresh:", "Mask", &thresh, max_thresh, meanShiftSegmentation );
    if(display)
    {
        //mask = Scalar::all(0);
        //imshow("mask", mask);
        
        Mat temp2;
        original.copyTo( temp2, canny_output);
        //imshow("Mask", temp2);
        //imshow("Mask After", mask);
        vector<Rect>::iterator it;
        string filename;
        vector<Mat>::iterator itt;
        itt = Masks.begin();
    	
    	
    	it = regions.begin();
    	//just to show first ROI
    	Mat temp;
    	imshow("ROI", img( *it ));
	original.copyTo(temp);
	cout << "Region[" << numregions << "] Values (x,y,row,cols): " << it->x << "\t\t" << it->y << "\t\t" << it->height << "\t\t" << it->width << endl;
	rectangle( temp, Point(it->x,it->y), Point(it->x + it->width, it->y + it->height), Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ), 2, 8, 0 );
	imshow( "meanshift", temp );
	moveWindow("meanshift", 40,40);
	moveWindow("ROI",700,40);
    	
    	for ( ; it+1 < regions.end() ; )
    	{
        
	switch ( (char)waitKey(10) )
	    {
	    case 27: 
		it = regions.end();
		running = false;
		break;
	    case 'q':  case 'Q':
	       	it = regions.end();
	       	display = false;
	        cvDestroyWindow("meanshift");
	        cvDestroyWindow("ROI");
		break;
	    case 'i': case 'I':
	        it++;
		itt++;
		imshow("ROI", img( *it ));
		original.copyTo(temp);
		cout << "Region[" << numregions << "] Values (x,y,row,cols): " << it->x << "\t\t" << it->y << "\t\t" << it->height << "\t\t" << it->width << endl;
		rectangle( temp, Point(it->x,it->y), Point(it->x + it->width, it->y + it->height), Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) ), 2, 8, 0 );
		imshow( "meanshift", temp );
		//imshow( "Masks", *itt );
		break;
	    case 's': case 'S':
	        cout << "Name of New file: " << endl;
	        cin >> filename;
	        filename = "./data/imageDatabase/" + filename + ".png"; 
		if (imwrite( filename, original(*it)) )
		    cout << "File: \"" << filename << "\" is saved!" << endl;
		else cout << "File Save Error" << endl;
		break;
	    case 'o': case 'O':
	        string name;
	        ObjRec.matchObsvToDB( original(*it), name );
	        break;
	    }
	} 
    }
}

string LwinName = "Left meanshift", RwinName = "Right meanshift", CwinName = "Control Window";
int spatialRad, colorRad, maxPyrLevel;
Mat Limg, Lres, Rimg, Rres;

void meanShiftSegmentation( int, void* )
{        
    if (display) Limg.copyTo(original);
    pyrMeanShiftFiltering( Limg, Lres, spatialRad, colorRad, maxPyrLevel );
    floodFillPostprocess( Lres, Scalar::all(2) );
    
    if (display) Rimg.copyTo(original);
    pyrMeanShiftFiltering( Rimg, Rres, spatialRad, colorRad, maxPyrLevel );
    floodFillPostprocess( Rres, Scalar::all(2) );
    
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
    lineFinder = 0;
    spatialRad = fileCount = 4;
    colorRad = 20;
    maxPyrLevel = 2;
    thresh = 100;
    running = true;
    display = rects = paused = false;
    namedWindow( LwinName, CV_WINDOW_AUTOSIZE );
    namedWindow( RwinName, CV_WINDOW_AUTOSIZE );
    namedWindow( CwinName, CV_WINDOW_AUTOSIZE );
    moveWindow(LwinName.c_str(), 0, 0);
    moveWindow(RwinName.c_str(), 640, 0);
    moveWindow(CwinName.c_str(), 450, 584);
    createTrackbar( "spatialRad", CwinName, &spatialRad, 80, meanShiftSegmentation );
    createTrackbar( "colorRad", CwinName, &colorRad, 60, meanShiftSegmentation );
    createTrackbar( "maxPyrLevel", CwinName, &maxPyrLevel, 5, meanShiftSegmentation );
    createTrackbar( "cannythresh", CwinName, &thresh, 200, meanShiftSegmentation );
    

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
    


    while(running)
    {
        if ( !paused )
        {
        camera1 >> Limg;
        camera2 >> Rimg;
    
        meanShiftSegmentation(0,0);
        
        imshow(LwinName, Lres);
        imshow(RwinName, Rres);   
        }
        
         switch ( (char)waitKey(1) )
	    {
	    case 27: case 'q':  case 'Q':
	       	running = false;
	        break;
	    case 'd': case 'D':
	        display ? display = false : display = true;
	        break;
	    case 'r': case 'R':
	        rects ? rects = false : rects = true;
	        break;
	    case 'p': case 'P':
	        paused ? paused = false : paused = true;
	        break;
	    case 'l': case 'L':
	        lineFinder++;
	        if (lineFinder > 2 ) lineFinder = 0; 
	        break;
	    }
    }
    return 0;
}
