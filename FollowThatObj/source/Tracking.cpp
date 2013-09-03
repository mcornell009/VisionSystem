#include "opencv2/highgui/highgui.hpp"
#include "ObjRec.cpp"
#include "motor_control.cpp"

#include <vector>
#include <string>
#include <iostream>
#include <boost/thread.hpp>

using namespace cv;
using namespace std;

class FollowThatObj
{
    private:
    int x, y;
    Mat Limg, Rimg;
    Mat LimgDisp, RimgDisp;
    string LwinName, RwinName;
    bool running, canRun, cameraSwitch, cameraON; 
    bool tracking, KFon, motorsOn, displayPoints;
    VideoCapture camera0, camera1;
    ObjectRecognition ObjRec;
    MotorControl motors;
    Point2f coorThatObj, coorPrevious;

    bool trackObj();
    void mainLoop();
    void keyboardControl();
    void displayImages();    

    public:
    FollowThatObj();
    void getNextImage();
    void start();
    void stop();

};

FollowThatObj::FollowThatObj()
{
    cameraSwitch = true, cameraON = false;
    
    tracking = true; 
    KFon = motorsOn = displayPoints = false;       
    x = 320, y = 240;
    coorPrevious.x = coorThatObj.x = x, coorPrevious.y = coorThatObj.y = y;
    
    //  Init Cameras
    camera0 = VideoCapture(0); 
    camera1 = VideoCapture(1);
    if(camera0.isOpened() && camera1.isOpened()) canRun = true; // check if we succeeded 
    else
    {
    	cout << "Cameras are not attached" << endl;
    	canRun = false;
    }
    
    // Create Windows for Display
    LwinName = "Left Camera", RwinName = "Right Camera";
    namedWindow( LwinName, CV_WINDOW_AUTOSIZE );
    namedWindow( RwinName, CV_WINDOW_AUTOSIZE );

    moveWindow(LwinName.c_str(), 0, 0);
    moveWindow(RwinName.c_str(), 640, 0);

    canRun = ObjRec.loadImageDB();
    canRun ? cout << "Can Run, starting program" << endl : cout << "Cannot run (canRun bool false), exiting program" << endl;  
}

void FollowThatObj::start()
{

    if(canRun)
    {
    running = true;
    mainLoop();
    }
    else stop();
}

void FollowThatObj::stop()
{
    running = false;
    motors.stop();
}


void FollowThatObj::getNextImage()
{
    Mat _Limg, _Rimg;
    
    while(running)
    {
        if(cameraSwitch)
     	{ 
    	    camera0 >> _Limg;
    	    camera1 >> _Rimg;
        } 
        else
        {
    	    camera0 >> _Rimg;
    	    camera1 >> _Limg;	
        }
        
        _Limg.copyTo(Limg);
        _Rimg.copyTo(Rimg);
        
        cameraON = true;
    }
}

void FollowThatObj::displayImages()
{
    imshow( LwinName, LimgDisp );
    imshow( RwinName, RimgDisp );       
}

void FollowThatObj::keyboardControl()
{

    switch ( (char)waitKey(5) )
    {
	case 27: case 'q':  case 'Q':
	    running = false;
	    break;
	case 'a': case 'A':
	    x = 160;
	    motors.moveToXY(x,y);
	    break;
	case 'd': case 'D':
	    x += 160;
	    motors.moveToXY(x,y);
	    break;
	case 'w': case 'W':
	    y += 120;
	    motors.moveToXY(x,y);
	    break;
	case 's': case 'S':
	    y -= 120;
	    motors.moveToXY(x,y);
	    break;
	case 'v': case 'V':
	    tracking ? tracking = false : tracking = true;
	    tracking ? cout << "Tracking on" << endl : cout << "Tracking off" << endl;
	    break;
	case 'b': case 'B':
	    KFon ? KFon = false : KFon = true;
	    KFon ? cout << "Keypoint finding on" << endl : cout << "Keypoint finding off" << endl;
	    break;
	case 'm': case 'M':
	    motorsOn ? motorsOn = false : motorsOn = true;
	    motorsOn ? cout << "motors on" << endl : cout << "Motors off" << endl; 
	    break;
	case 'o': case 'O':
	    ObjRec.track ? ObjRec.track = false : ObjRec.track = true;
	    break;
	case 'z': case 'Z':
	    displayPoints ? displayPoints = false : displayPoints = true;
	    break;
	    
    }
}

void FollowThatObj::mainLoop()
{    
    boost::thread getNextImages(boost::bind(&FollowThatObj::getNextImage,this));
    
    vector<Rect> L_ROIs, R_ROIs;
    vector<Mat> L_ROIMasks, R_ROIMasks;
    Mat _Limg, _Rimg;
    TickMeter tm;       
    while(running)
    {
 
    tm.start();
        
        if (cameraON)
        {
            Limg.copyTo(_Limg);
            Rimg.copyTo(_Rimg);
            
            
            
            /*
  	        boost::thread L_RF(boost::bind(&RegionFinding::findRegions, &RegFind, _Limg));
            L_ROIs = RegFind.regions;
            L_ROIMasks = RegFind.masks;
            boost::thread R_RF(boost::bind(&RegionFinding::findRegions, &RegFind, _Rimg));
            R_ROIs = RegFind.regions;
            R_ROIMasks = RegFind.masks;
            L_RF.join();
            R_RF.join();
            */

		    if (tracking)
		    {
		        trackObj();
		    	
		    }
		    	
            _Limg.copyTo(LimgDisp);
            _Rimg.copyTo(RimgDisp);
            
            displayImages();
            keyboardControl(); 
            tm.stop();
            //cout << "Full loop in " << tm.getTimeMilli() << " ms" << endl; 
            tm.reset();
        }
    }
}

bool FollowThatObj::trackObj()
{
    Point2f KFcoor;
    bool finding = false;
    // Find coorinates from KeyPoint Object Matching
    if (KFon) 
    {
    ObjRec.matchObsvToDB(Limg, KFcoor);
    finding = true;
    coorThatObj = KFcoor;
	}

	// Rolling average to merge previous data with new data
	if (finding)
	{
		coorThatObj.x = (coorThatObj.x + coorPrevious.x) / 2;
		coorThatObj.y = (coorThatObj.y + coorPrevious.y) / 2;
		coorPrevious = coorThatObj;
	}
	//cout << "Finding, KFon, Motorson" << finding << KFon << motorsOn << endl;
    // Showing points 
	if (displayPoints && finding) cout << "XY of obj\t" << coorThatObj.x << "\t" << coorThatObj.y << endl;
	if (motorsOn && finding ) motors.moveToXY(coorThatObj);
	
	//    			

}

//*/
int main(int argc, char** argv)
{
    FollowThatObj tracking;
    tracking.start();
}

//*/
