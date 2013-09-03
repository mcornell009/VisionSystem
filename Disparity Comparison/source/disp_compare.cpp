#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <stdexcept>

#include "typedefs.h"

using namespace cv;
using namespace std;

/***** To compile in terminal use command: "g++ camcap_liveRectify.cpp -o camcap_liveRectify `pkg-config --cflags --libs opencv pcl-1.5`"
 Recommended to compile using CMAKE and make command to specify different OpenCV versions - go to documentation to see how or just figure it out like I did

*/


struct App
{
    Rect validRoi[3];
    VideoCapture camera1; 
    VideoCapture camera2;
    
    StereoBM bm;
    StereoSGBM sgbm;
    StereoVar svar;

    void BMdisp();    
    void updateBM();
    void controlWindowBM();
    
    void SGBMdisp();
    void updateSGBM();
    void controlWindowSGBM();
    
    void SVARdisp();
    void updateSVAR();
    void controlWindowSVAR();
    
    void handleKey();
    void direct();
    void dispar();
    void run();
    void visualizer();
    void createPCD();
    void close(); 
    
    string method_str() const
    {
        switch (method)
        {
        case BM: return "BM";
        case SGBM: return "SGBM";
        case SVAR: return "SVAR";
        }
        return "";
    }
    
    enum {BM, SGBM, SVAR} method;

    private:
    //Settings for the two Block Matching 
    int nDis, sadWindow, uRatio, sWindows, minnDis, sRange, TThresh, preFilterType1, preFilterSize1, preFilterCap1, sWindowSize, MaxDiff, p1, p2;
    // Settings for Stereo Var
    int levels, nIt, pyrScale, poly_n,  poly_sigma, fi, lambda, fileCount;
    //float fi, lambda;
    //double poly_sigma, pyrScale;
    Mat leftFrame, rightFrame, disp,  rmap[2][2], Q;
    bool loop, running, camswitch;    
    PointCloudPtr metacloud;// (new PointCloud);
    Mat left, right;
};

//need it global for trackbar
App app;

int main(int argc, char** argv) {
    app.run();   
}
/*
    //    camera1.set(CV_CAP_PROP_FRAME_WIDTH, frameW);
    //    camera1.set(CV_CAP_PROP_FRAME_HEIGHT, frameH);

    //    camera2.set(CV_CAP_PROP_FRAME_WIDTH, frameW);
    //    camera2.set(CV_CAP_PROP_FRAME_HEIGHT, frameH);
*/

void App::run()
{
    validRoi[0] = { 13, 32, 564, 421 }; 
    validRoi[1] = { 51, 32, 571, 428 };
    // Define Rect manually (need to fix to put it in intrinsic or redefine each time)
    camera1 = VideoCapture(0); 
    camera2 = VideoCapture(1);
    
    if(!camera1.isOpened()) // check if we succeeded
    {
	cout << "Camera 0 not attached" << endl;
        close();
    } else running = true;
    if(!camera1.isOpened()) // check if we succeeded
    {
    	cout << "Camera 1 not attached" << endl;
        close();
    } else running = true;
    
    //Init
    camswitch = true, fileCount=0;
    
    //Main loop
    while (running)
    {
        if(running) dispar();
	if(running) direct();
    }    
}

void App::direct() 
{
    cout << "Doing Direct Feed from Stereo Cameras" << endl;

    namedWindow("Left Frame", CV_WINDOW_AUTOSIZE);
    namedWindow("Right Frame", CV_WINDOW_AUTOSIZE);

    cvMoveWindow("Left Frame", 0,0);
    cvMoveWindow("Right Frame", 640,0);

    loop = true;
    while(loop)
    {
        camswitch ? camera1 >> leftFrame : camera1 >> rightFrame ;
        camswitch ? camera2 >> rightFrame : camera2 >> leftFrame ;

        /// Show your results
        imshow("Left Frame", leftFrame);
        imshow("Right Frame", rightFrame);

        handleKey();
    }
    destroyWindow("Left Frame");
    destroyWindow("Right Frame");
}


//Can't be part of struct or wont call correctly
void on_trackbar(int, void*)
{
//    cout << "Updated Disparity" << endl;
    switch (app.method)
        {
        case 0: 
            app.updateBM();
            break;
        case 1: 
            app.updateSGBM();
            break;
        case 2: 
            app.updateSVAR();
            break;
        default:;
        }  

}


void App::dispar()
{

    cout << "Doing Disparity Mat" << endl;

    int cn = leftFrame.channels();
    nDis = 7, sadWindow = 2, uRatio = 40, sWindows = 0, minnDis = 0, sRange = 0, TThresh = 0, preFilterType1 = 0, preFilterSize1 = 5, preFilterCap1 = 1, sWindowSize = 0;
    p1 = 8*cn*80*80, p2 = 32*cn*80*80; MaxDiff = 1;
    levels = 3, nIt = 25, pyrScale = 50, poly_n = 3, poly_sigma = 11, fi = 15.0f, lambda = 3;
    
    

    namedWindow("Disparity", CV_WINDOW_AUTOSIZE);
    namedWindow("Control Window", CV_WINDOW_AUTOSIZE);

    
    switch (method)
    {
    case BM:
        controlWindowBM();
        break;
    case SGBM:
        controlWindowSGBM();
        break;
    case SVAR:
        controlWindowSVAR();
        break;
    }

    cvMoveWindow("Disparity", 0,0);
    cvMoveWindow("Control Window", 640, 0);


    string intrinsic = "../calibData/intrinsics.yml";
    string extrinsic = "../calibData/extrinsics.yml";
    FileStorage fs(intrinsic, CV_STORAGE_READ);

    if(!fs.isOpened())
    {
        printf("Failed to open intrinsic file\n"); //" %s\n", intrinsic);
        fs.release();
        close();
    }


    Mat cameraMatrix[2], distCoeffs[2];
    fs["M1"] >> cameraMatrix[0];
    fs["D1"] >> distCoeffs[0];
    fs["M2"] >> cameraMatrix[1];
    fs["D2"] >> distCoeffs[1];

    fs.open(extrinsic, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        printf("Failed to open extrinsic file");//" %s\n", extrinsic);
        close();
    }

    Mat R, T, R1, P1, R2, P2;

    fs["R"] >> R;
    fs["T"] >> T;
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;
    //fs["Roi1"] >> validRoi[0];
    //fs["Roi2"] >> validRoi[1];

    fs.release();
    Size imageSize = cv::Size(640, 480);

//    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1], imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
        //validRoi[3] = validRoi[1] & validRoi[0];
//	cout << validRoi[0].x << "\t\t" << validRoi[0].y << "\t\t" << validRoi[0].width << "\t\t" << validRoi[0].height << "\n" << 
//	validRoi[1].x << "\t\t" << validRoi[1].y << "\t\t" << validRoi[1].width << "\t\t" << validRoi[1].height << endl;
    
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    loop = true;
    while(loop)
    {
        
        camswitch ? camera1 >> leftFrame : camera1 >> rightFrame ;
        camswitch ? camera2 >> rightFrame : camera1 >> leftFrame ;
        
        Mat rimg1;
        remap(leftFrame, rimg1, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
        leftFrame = rimg1;

        Mat rimg2;
        remap(rightFrame, rimg2, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
        rightFrame = rimg2;
        
        
        
        switch (method)
        {
        case BM:
            BMdisp();
            break;
        case SGBM:
            SGBMdisp();
            break;
        case SVAR: 
            SVARdisp();
            break;
        }
        /// Show your results
        imshow("Disparity", disp);
        handleKey();
    }
    destroyWindow("Disparity");
    destroyWindow("Control Window");

}

void App::BMdisp()
{
    cvtColor(leftFrame, leftFrame, CV_BGR2GRAY);
    cvtColor(rightFrame, rightFrame, CV_BGR2GRAY);
    bm(leftFrame, rightFrame, disp, CV_32F);
}

void App::updateBM()
{
    bm.state -> roi1 = validRoi[0];
    bm.state -> roi2 = validRoi[1];
    bm.state->numberOfDisparities = nDis * 16 + 16;
    bm.state->SADWindowSize = sadWindow * 2 + 5;
    bm.state->uniquenessRatio = uRatio;
    //            bm.state->minDisparity = nDis * 16 + 16;

               bm.state->preFilterCap = preFilterCap1;
            bm.state->preFilterSize = preFilterSize1;
            bm.state->preFilterType = preFilterType1;
            bm.state->minDisparity = minnDis;
            bm.state->textureThreshold = TThresh;
            bm.state->uniquenessRatio = uRatio;
            bm.state->speckleWindowSize = sWindowSize;
            bm.state->speckleRange = sRange;
            bm.state->disp12MaxDiff = 1;  
}

void App::controlWindowBM()
{
    createTrackbar("Number of Disparities", "Control Window", &nDis, 20, on_trackbar);
    createTrackbar("Uniqueness Ratio", "Control Window", &uRatio, 100, on_trackbar);
    createTrackbar("SAD Window Size", "Control Window", &sadWindow, 30, on_trackbar);
    updateBM();
}

void App::SGBMdisp()
{
    Mat blankdisp;
    cvtColor(leftFrame, left, CV_BGR2GRAY);
    cvtColor(rightFrame, right, CV_BGR2GRAY);
        	
    //disp.assignTo(disp, CV_32F);
    /*
     cout << " left frame info (size, type): " << left.cols << "\t\t" << left.rows << "\t\t" << left.type() << 
            "\n right frame info (size, type): " << right.cols << "\t\t" << right.rows << "\t\t" << right.type() << 
	    "\n Disp frame info (size, type): " << blankdisp.cols << "\t\t" << blankdisp.rows << "\t\t" << blankdisp.type() << endl;
	    */
	    
    //imshow("left",leftFrame);
    //imshow("right",rightFrame);
     sgbm(left, right, blankdisp);
    
    
    Mat disproi(blankdisp, getValidDisparityROI(validRoi[0], validRoi[1], sgbm.minDisparity, sgbm.numberOfDisparities, sgbm.SADWindowSize));
	//imshow("Disp", disproi);
        disproi.assignTo(disproi, CV_32F);
        disp = disproi;
    //disp.assignTo(disp, CV_32F);
    /*
cout << " left frame info (size, type): " << left.cols << "\t\t" << left.rows << "\t\t" << left.type() << 
            "\n right frame info (size, type): " << right.cols << "\t\t" << right.rows << "\t\t" << right.type() << 
	    "\n Disp frame info (size, type): " << disp.cols << "\t\t" << disp.rows << "\t\t" << disp.type() << "\n\n" << endl;
*/
    //		disp(disp, getValidDisparityROI(validRoi[0], validRoi[1], minnDis, nDis, sadWindow));
    //Mat disproi(disp, getValidDisparityROI(validRoi[0], validRoi[1], minnDis, nDis, sadWindow));
    
    
    //disp = disproi;
    
    
}


void App::updateSGBM()
{ 
    //cout << "Updating SGBM" << endl;   
    sadWindow = sadWindow * 2 + 5;
    nDis = nDis * 16 + 16;
    
    
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = sadWindow > 0 ? sadWindow : 3;

    int cn = leftFrame.channels();
    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = minnDis;
    sgbm.numberOfDisparities = nDis;
    sgbm.uniquenessRatio = uRatio;
    sgbm.speckleWindowSize = sWindowSize;
    sgbm.speckleRange = sRange;
    sgbm.disp12MaxDiff = MaxDiff;
    sgbm.fullDP = 1;
    
    //Change variables back to trackbar values
    
    /*
            
    
    
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = sadWindow > 0 ? sadWindow : 3;
    
    int cn = leftFrame.channels();
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2 };
    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = minnDis;
    sgbm.numberOfDisparities = nDis;
    sgbm.uniquenessRatio = uRatio;
    sgbm.speckleWindowSize = bm.state->speckleWindowSize;
    sgbm.speckleRange = bm.state->speckleRange;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = STEREO_HH;
    */
    sadWindow = (sadWindow-5) / 2;
    nDis = (nDis - 16) / 16;
    
    //cout << 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize << "\t\t" << 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize  << "\t\t " << cn << endl;
}

void App::controlWindowSGBM()
{
    createTrackbar("Disparities", "Control Window", &nDis, 15, on_trackbar);
    createTrackbar("Uniqueness Ratio", "Control Window", &uRatio, 100, on_trackbar);
    createTrackbar("SAD Window Size", "Control Window", &sadWindow, 10, on_trackbar);
    createTrackbar("P1", "Control Window", &p1, 10000, on_trackbar);
    createTrackbar("P2", "Control Window", &p2, 10000, on_trackbar);
//    sgbm.speckleWindowSize = bm.state->speckleWindowSize;
//    sgbm.speckleRange = bm.state->speckleRange;
    createTrackbar("Max Diff", "Control Window", &MaxDiff, 300, on_trackbar);
    updateSGBM();
}

void App::SVARdisp()
{
    Mat blankdisp;
    svar(leftFrame, rightFrame, blankdisp);
    disp = blankdisp;
}

void App::updateSVAR()
{
//    levels, pyrScale, nIt, minDisp, maxDisp, poly_n, poly_sigma, fi, lambda, penalization, cycle, flags;
    disp.assignTo(disp, CV_8UC1);
    
//    StereoVar(levels, pyrScale, nIt, minDisp, maxDisp, poly_n, poly_sigma, fi, lambda, penalization, cycle, flags)
//  createTrackbar("Number of Disparities", "Control Window", &nDis, 20, on_trackbar);

        svar.levels = levels;						// ignored with USE_AUTO_PARAMS
	svar.pyrScale = (float)pyrScale/100;				// ignored with USE_AUTO_PARAMS
	svar.nIt = nIt;
	svar.minDisp = minnDis;	
	svar.maxDisp = (nDis * 16 + 16);
	svar.poly_n = poly_n*2 + 3;
	svar.poly_sigma = (float)poly_sigma/100;
	svar.fi = (float)fi/10;
	svar.lambda = (float)lambda/100;
	svar.penalization = svar.PENALIZATION_TICHONOV; 		// ignored with USE_AUTO_PARAMS
	svar.cycle = svar.CYCLE_V;					// ignored with USE_AUTO_PARAMS
	svar.flags = svar.USE_SMART_ID | svar.USE_INITIAL_DISPARITY | svar.USE_MEDIAN_FILTERING ;
//| svar.USE_AUTO_PARAMS


/*  Example from StereoMatch

    var.levels = 3;									// ignored with USE_AUTO_PARAMS
	var.pyrScale = 0.5;								// ignored with USE_AUTO_PARAMS
	var.nIt = 25;
	var.minDisp = -numberOfDisparities;	
	var.maxDisp = 0;
	var.poly_n = 3;
	var.poly_sigma = 0.0;
	var.fi = 15.0f;
	var.lambda = 0.03f;
	var.penalization = var.PENALIZATION_TICHONOV;	// ignored with USE_AUTO_PARAMS
	var.cycle = var.CYCLE_V;						// ignored with USE_AUTO_PARAMS
	var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;
*/

}

void App::controlWindowSVAR()
{
    createTrackbar("Disparities", "Control Window", &nDis, 15, on_trackbar);
    createTrackbar("levels", "Control Window", &levels, 10, on_trackbar);
    createTrackbar("Pyramid Scale/100", "Control Window", &pyrScale, 100, on_trackbar);
    createTrackbar("num it", "Control Window", &nIt, 50, on_trackbar);
    createTrackbar("Poly n(only odd)", "Control Window", &nDis, 5, on_trackbar);
    createTrackbar("Poly Sigma/100 ", "Control Window", &nDis, 100, on_trackbar);
    createTrackbar("Fi/10", "Control Window", &nDis, 200, on_trackbar);
    createTrackbar("Lambda/100", "Control Window", &lambda, 100, on_trackbar);
    
    
    updateSVAR();
}


void App::handleKey()
{
    switch ( (char)waitKey(3) )
    {
    case 27: case 'q':  case 'Q':
       	close();
        break;
    case 'z': case 'Z':
        cout << "Switching Cameras" << endl;
	camswitch ? camswitch = false : camswitch = true; 
     	break;
    case 'w': case 'W':
        loop = false;
        break;
    case 'm': case 'M':
        destroyWindow("Control Window");
        namedWindow("Control Window", CV_WINDOW_AUTOSIZE);
        switch (method)
        {
        case BM: 
            cout << "Computing stereo correspondence using the semi-global block matching algorithm" << endl;
            controlWindowSGBM();
            method = SGBM;
            break;
        case SGBM: 
            cout << "Computing stereo correspondence using the variational matching algorithm" << endl;
            controlWindowSVAR();
            method = SVAR;
            break;
        case SVAR:
            cout << "Computing stereo correspondence using the block matching algorithm" << endl;
            controlWindowBM();
            method = BM;
            break;
        }
        break;
    case 'v': case 'V':
            createPCD();
            //visualizer();
            break;   
    
    case 'd': case 'D':
        sadWindow = sadWindow * 2 + 5;
        nDis = nDis * 16 + 16;

    	 switch (method)
        {
        case BM: 
               cout << "\nDisplaying Diparity Parameters State:  " <<
	    "\nTry Smaller Windows:  " << sWindows <<
	    "\nPre-Filter Type: " << preFilterType1 <<
	    "\nPre-Filter Size: " << preFilterSize1 <<
	    "\nPre-Filter Cap: " << preFilterCap1 <<
	    "\nSAD Window Size:  " << sadWindow <<
	    "\nMaximum Disparities:  " << nDis <<
	    "\nMinimum Disparities:  " << minnDis <<
	    "\nSpeckle Ratio:  " << sRange <<
	    "\nSpeckle Window Size:  " << sWindowSize <<
	    "\nTexture Threshold:  " << TThresh <<
	    "\nUniqueness Ratio:  " << uRatio << endl;
            break;
        case SGBM: 
            /*   cout << "\nDisplaying Diparity Parameters State:  " <<
	    "\nTry Smaller Windows:  " << sWindows <<
	    "\nPre-Filter Type: " << preFilterType1 <<
	    "\nPre-Filter Size: " << preFilterSize1 <<
	    "\nPre-Filter Cap: " << preFilterCap1 <<
	    "\nSAD Window Size:  " << sadWindow <<
	    "\nMaximum Disparities:  " << nDis <<
	    "\nMinimum Disparities:  " << minnDis <<
	    "\nSpeckle Ratio:  " << sRange <<
	    "\nSpeckle Window Size:  " << sWindowSize <<
	    "\nTexture Threshold:  " << TThresh <<
	    "\nUniqueness Ratio:  " << uRatio << endl;
	    */
	    cout << "\nDisplaying Diparity Parameters State:  " <<
	    "\nPre-Filter Cap: " << sgbm.preFilterCap <<
	    "\nSAD Window Size:  " <<sgbm.SADWindowSize <<
	    "\nMaximum Disparities:  " << sgbm.numberOfDisparities <<
	    "\nMinimum Disparities:  " << sgbm.minDisparity <<
	    "\nSpeckle Ratio:  " << sgbm.speckleRange <<
	    "\nSpeckle Window Size:  " << sgbm.speckleWindowSize <<
	    "\nUniqueness Ratio:  " << sgbm.uniquenessRatio << 
	    "\nP1: " << sgbm.P1 <<
	    "\nP2: " << sgbm.P2 <<
	    "\nDisp12MaxDiff: " << sgbm.disp12MaxDiff <<
	    "\n FullDP: " << sgbm.fullDP <<
	    "\n left frame info (size, type): " << left.cols << "\t\t" << left.rows << "\t\t" << left.type() << 
	    "\n Disp frame info (size, type): " << disp.cols << "\t\t" << disp.rows << "\t\t" << disp.type() <<
	    endl;
	    imshow("left", left); imshow("right", right);
            break;
        case SVAR:
            cout << "Displaying Diparity Parameters" << endl;
            break;
        }
	    sadWindow = (sadWindow - 5)  / 2;
           nDis = (nDis - 16) / 16;

	    break;
	}
}

void App::createPCD()
{
    printf("storing the point cloud...");
    fflush(stdout);
    Mat xyz;
    reprojectImageTo3D(disp, xyz, Q, true);
    string NamePCD = "/home/robotlab/robot/PCDLab/PLYout";
    stringstream fileCountStr;
    fileCountStr << fileCount;
    NamePCD += fileCountStr.str();
    NamePCD += ".pcd";
    fileCount++;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int numPoints = xyz.rows * xyz.cols;
    cloud->width = numPoints;
    cloud->height = 1;
    cloud->points.resize(numPoints);
    int i = 0;
    const double max_z = 1.0e4;
    //FILE* fp = fopen(filename, "wt");
    cout << "Going into creation loop" << endl;
    for(int y = 0; y < xyz.rows; y++)
    {
        for(int x = 0; x < xyz.cols; x++)
        {
        
            Vec3f point = xyz.at<Vec3f>(y, x);
            Vec3i colorPoint = leftFrame.at<Vec3b>(y,x); // vector char vec3b; vetor int vec3i;
            unsigned int B = colorPoint[0];
            unsigned int G = colorPoint[1];
            unsigned int R = colorPoint[2];
           
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            
            //if(point[2] > -500)
            {
            cloud->points[i].x = -1*point[0];
            cloud->points[i].y = point[1];
            cloud->points[i].z = point[2];
            cloud->points[i].r = R;
            cloud->points[i].g = G;
            cloud->points[i].b = B;
            
            cout << "Point " << i << ": " << cloud->points[i].x << "\t\t" << cloud->points[i].y << "\t\t" << cloud->points[i].z << "\t\t" << cloud->points[i].r << "\t\t" << cloud->points[i].g << "\t\t" << cloud->points[i].b << "\t\t" << endl;
            ++i;
            }
            
            //fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
            //fprintf(fp, "%f %f %f %d %d %d \n", point[0], point[1], point[2], R, G, B);
            
        }
    }
    cloud->points.resize(i);
    ///fclose(fp);
    printf("done\n");
    pcl::console::print_info ("Starting visualizer... Close window to exit.\n");
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud (cloud);
    vis.resetCamera ();
    vis.spin ();
    cout << "I'm called" << endl;
    // vis.close ();
}

void App::visualizer()
{
    pcl::console::print_info ("Starting visualizer... Close window to exit.\n");
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud (metacloud);
    vis.resetCamera ();
    vis.spin ();
}

void App::close()
{
    running = false;
    loop = false;
    //destroyAllWindows();
}
