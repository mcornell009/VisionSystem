#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

//************ Classes ************//


class DBobj
{
    public:
         string name;
         Mat img;
         vector<KeyPoint> keypoints;
         Mat description;
         //Mat Mask (if floodfill ever works right)
         
         //DBObj(Mat img);
};



class ObjectRecognition
{ 
/* ********** 
To use 
    1. create an object  ex > ObjectRecognition ObjRec;
        creates feature (also called keypoint) detector, feature detector, and feature matcher (DDM)
    2. Load db.  Either load images from default database stored in DBdirName or add a string to the call to call from another dir
               ex> loadImageDB(); or loadImageDB(./data/databaseImages); 
    3.  Match by calling matchObsvToDB(image to be matched, returned name) and will return closes image match name
  Parameters information can be found at " http://opencv.itseez.com/modules/nonfree/doc/feature_detection.html?highlight=surf#SURF : public Feature2D " 
   ********** */
    private:    
        Ptr<FeatureDetector> detector;
        Ptr<DescriptorExtractor> descriptor;
        Ptr<DescriptorMatcher> matcher;
        Mat observedImage;        
        vector<DBobj> DB;
        
        string dirName, params_name, detector_name, descriptor_name, matcher_name, DBdirName;
        bool createDetectorDescriptorMatcher( );
        void saveloadDDM( const string& params_filename );
        void detectKeypointsSingle( const Mat& Image, vector<KeyPoint>& Keypoints);
        void computeDescriptorsSingle( const Mat& observedImage, vector<KeyPoint>& observedKeypoints, Mat& observedDescriptors);
        int  getdir ( const string &dir, vector<string> &files);
        void matchObsvToDB(const Mat &img, vector<KeyPoint> &imgKp, vector<DMatch> &matches);
       
        // Below is unused but might be useful
        void maskMatchesByTrainImgIdx( const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask );
    public:
        string observedImageName;
        void matchObsvToDB(const Mat &img, string &name);
        void matchObsvToDB(const Mat &img, Point2f &coor);
        bool loadImageDB(string &dir, string &className);
        bool loadImageDB (string &dir);
        bool loadImageDB ();
        bool track;
        ObjectRecognition();  // Contructor for Init
        void showDatabase ();  // show items of database
        
        
        bool saveDatabaseOfDescriptors(const string& dirName,const vector<Mat>& databaseDescriptors, vector<string> files);
        
        
};

//************ Constructors  ************//

ObjectRecognition::ObjectRecognition()
{
     //cout << "I am a created object" << endl;
     dirName = "../data", detector_name = "SURF", descriptor_name = "SURF", matcher_name = "FlannBased", DBdirName = dirName + "/imageDatabase/";
     observedImageName = DBdirName + "/me1.png", params_name = detector_name + "_" + descriptor_name + "_" + matcher_name;
     track = false;
     if ( !createDetectorDescriptorMatcher () )
     {
       cout << "Error Creating DDM" << endl;
       //Should Prevent the class from being created
     }
     track = false;
     saveloadDDM( dirName + "/params/" + params_name + ".yml" );
     
     
}

void ObjectRecognition::showDatabase ()
{
    if(!DB.empty())
    {
        cout << "Showing Database which has a size of " << DB.size() << " entries." << endl;
	for ( vector<DBobj>::iterator DBiter = DB.begin(); DBiter != DB.end() ; DBiter++)
	{
	    Mat outImage = DBiter->img;
	    drawKeypoints(DBiter->img, DBiter->keypoints, outImage, Scalar(255, 0, 0), 1 );
	    imshow("Image with Keypoints", outImage);
	    cout << "Name: \"" << DBiter->name << "\"\tNum of Keypoints: " << DBiter->keypoints.size() << endl;
	    waitKey(0); 
	} 
    
    } else cout << "Database is empty" << endl;
    
}


//************ Writing then Saving Functions  ************//

void ObjectRecognition::saveloadDDM( const string& params_filename)
{
    FileStorage fs(params_filename, FileStorage::READ);
    if( fs.isOpened() )
    {
        detector->read(fs["detector"]);
        descriptor->read(fs["descriptor"]);
        /// *** need to see about applying params to matcher
       // matcher->read(fs["matcher"]);
    }
    else
    {
        fs.open(params_filename, FileStorage::WRITE);
        fs << "detector" << "{";
        detector->write(fs);
        fs << "}" << "descriptor" << "{";
        descriptor->write(fs);
        fs << "}" << "matcher" << "{";
        matcher->write(fs);
        fs << "}";
    }
}

bool ObjectRecognition::loadImageDB()
{
   return loadImageDB(DBdirName);
}

bool ObjectRecognition::loadImageDB(string &dir)
{
   string className = "\t/\t";
   return loadImageDB(dir, className);
}

bool ObjectRecognition::loadImageDB(string &dir, string &className)
{
   TickMeter tm;
   tm.start();  
   vector<string> files;
   
   getdir(dir,files);
   
   string extention = ".png"; 

   vector<string>::iterator it = files.begin();
   vector<Mat> descriptorDatabase;
   for (unsigned int i = 0;i < files.size();i++) 
   {
       if ( 
       	    files[i].size() > 4 && 
       	    files[i].compare( files[i].size() - 4, 4 , extention) == 0 &&
       	    (files[i].compare( 0, className.size(), className) == 0 || className.compare("\t/\t") == 0 )
       	  )
       {
           DBobj DBentry;
           DBentry.name = files[i];
           if(dir.compare( dir.size()-1, dir.size(), "/") != 0)  dir.append("/"); 
           cout << dir + files[i] << endl;
     	   DBentry.img = imread( dir + files[i] );
           if( DBentry.img.empty() )  cout << "Image: " << files[i] << " can not be read or has no information." << endl;
           
           DBentry.img.assignTo(DBentry.img, CV_8U);
           //cout << files[i]  << "\tRows" << DBentry.img.rows << "\t Cols" << DBentry.img.cols << "\t Type/Depth: " << DBentry.img.depth() << endl;    
           
           detectKeypointsSingle(DBentry.img, DBentry.keypoints );
           //cout << files[i] << "\t# Keypoints:" << DBentry.keypoints.size() << endl;
           if (DBentry.keypoints.size() > 9)
           {
           
               computeDescriptorsSingle(DBentry.img, DBentry.keypoints, DBentry.description);
               //cout << files[i] << "\t# of Descriptors: " << DBentry.description.rows << "\t# of Dimensions for descriptor: " << DBentry.description.cols  
               //      << "\tType/depth: " << DBentry.description.type() << " | " << DBentry.description.depth() << endl;
           
               descriptorDatabase.push_back(DBentry.description);
               DB.push_back( DBentry );
               
           }
       }
       it++;
   }
   // Add Database to matcher program.
   matcher->add(descriptorDatabase);
   matcher->train();
   
   tm.stop();
   if (DB.size() > 0)
   { 
   cout << "End reading directory in " << tm.getTimeMilli() << " ms, of size " << DB.size() << endl;
   return true;
   } 
   else 
   {
   cout << "Error reading Database ... ending program" << endl; 
   return false;
   }
}

int ObjectRecognition::getdir ( const string &dir, vector<string> &files)
{
    DIR *dp;
    struct dirent *dirp;
    
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    
    sort(files.begin(), files.end());
    
    closedir(dp);
    return 0;
}
//************ DDM Functions  ************//


bool ObjectRecognition::createDetectorDescriptorMatcher( )
{
    cout << "\"" << detector_name << "\"" << endl;
    cout << "\"" << descriptor_name << "\"" << endl;
    detector = FeatureDetector::create( detector_name );
    descriptor = DescriptorExtractor::create( descriptor_name );
    matcher = DescriptorMatcher::create( matcher_name );

    bool isCreated = !( detector.empty() || descriptor.empty() || matcher.empty() );
    if( !isCreated )
    {
        if ( detector.empty() ) cout << "Cannot Create Keypoint Detector" << endl;
        if ( descriptor.empty() ) cout << "Cannot Create Keypoint Descriptor" << endl;
	if ( matcher.empty() ) cout << "Cannot Create Keypoint Matcher" << endl;
        //cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << endl;
    }
    return isCreated;
}

void ObjectRecognition::detectKeypointsSingle( const Mat& Image, vector<KeyPoint>& Keypoints)
{
    detector->detect( Image, Keypoints );
}


void ObjectRecognition::computeDescriptorsSingle( const Mat& observedImage, vector<KeyPoint>& observedKeypoints, Mat& observedDescriptors)
{
    descriptor->compute( observedImage, observedKeypoints, observedDescriptors );
    //cout << "Observed Image descriptors count: " << observedDescriptors.rows << endl;
}

void ObjectRecognition::matchObsvToDB(const Mat &img, Point2f& coor)
{
    string ObjName;
    vector<KeyPoint> imgKp;
    vector<DMatch> matches;
    matchObsvToDB(img, imgKp, matches);
    //for finding which picture has most matches
    
    int numMatchesToDB [(int)DB.size()], bestMatchIdx=0;
    Point2f avg_points [(int)DB.size()];
    
    for (int i = 0; i < (int)DB.size(); i++)  
    {
    avg_points[i].x = avg_points[i].y = 0.0;
    numMatchesToDB[i] = 0;
    }
    //bin for finding which pic has most matches
    
    vector<KeyPoint>::iterator KPiter = imgKp.begin();
    for (vector<DMatch>::iterator DMiter = matches.begin(); DMiter != matches.end(); DMiter++)
    {
        numMatchesToDB[DMiter->imgIdx]++;
        
        avg_points[DMiter->imgIdx].x += KPiter->pt.x;
        avg_points[DMiter->imgIdx].y += KPiter->pt.y;
        
        //cout << "X,Y,total x, y\t" << KPiter->pt.x << "\t" << KPiter->pt.y << "\t" << avg_points[DMiter->imgIdx].x << "\t" << avg_points[DMiter->imgIdx].y << endl;
        
        
        if ( numMatchesToDB[bestMatchIdx] < numMatchesToDB[DMiter->imgIdx] ) bestMatchIdx = DMiter->imgIdx;
        
        //cout << "bestMatchIdx: " << numMatchesToDB[bestMatchIdx] << "\t" << numMatchesToDB[DMiter->imgIdx] << "\t" << bestMatchIdx << "\t" << DMiter->imgIdx << endl;
        //cout << "Match information (queryIDx/trainIDx/imgIDx/distance): " << 
        //    DMiter->queryIdx << "\t" << DMiter->trainIdx << "\t" << DMiter->imgIdx << "\t" << DMiter->distance << endl;
        
        KPiter++;
    }
    //cout << "Match time: " << matchTime << " ms with the best match at " <<  DB.at(bestMatchIdx).name << " with " << numMatchesToDB[bestMatchIdx] << " matching keypoints" << endl;
    //cout << "Keypoint db/ matches db size:\t" << imgKp.size() << "\t" << matches.size() << endl; 
    
    //cout << "Observed Descriptors " << imgDesc.rows << " and number of matches " << (int)matches.size() << endl;
    
    ObjName = DB.at(bestMatchIdx).name;
        
    //for (int i = 0; i < (int)DB.size(); i++)  
    	
    //cout << "Total x,y / number of matches / avg x,y \t" <<   
    //		avg_points[bestMatchIdx].x << ", " << avg_points[bestMatchIdx].y << "\t" << numMatchesToDB[bestMatchIdx] << "\t" 
    //		<< avg_points[bestMatchIdx].x / numMatchesToDB[bestMatchIdx] << ", " << avg_points[bestMatchIdx].y / numMatchesToDB[bestMatchIdx] << endl; 
    
    
    coor.x = avg_points[bestMatchIdx].x / numMatchesToDB[bestMatchIdx];
    coor.y = avg_points[bestMatchIdx].y / numMatchesToDB[bestMatchIdx];
    
    
    float stdevx = 0.0, stdevy = 0.0;
    KPiter = imgKp.begin();
    for (vector<DMatch>::iterator DMiter = matches.begin(); DMiter != matches.end(); DMiter++)
    {
        if ( DMiter->imgIdx == bestMatchIdx)
        {
        stdevx += pow((KPiter->pt.x - coor.x),2);
        stdevy += pow((KPiter->pt.y - coor.y),2);
        }
        
        KPiter++;
    }
    stdevx = sqrt(stdevx / avg_points[bestMatchIdx].x);
    stdevy = sqrt(stdevy / avg_points[bestMatchIdx].y);
    
    //cout << "standard deviation x,y\t" << stdevx << "\t" << stdevy << endl; 

    int inliers = 0;
    float stdevFromMean = 3.5, xval = 0.0, yval = 0.0;
    KPiter = imgKp.begin();
    for (vector<DMatch>::iterator DMiter = matches.begin(); DMiter != matches.end(); DMiter++)
    {
        if ( DMiter->imgIdx == bestMatchIdx)
        {
        	if ( ((KPiter->pt.x - coor.x) / stdevx < stdevFromMean) 
        	  && ((KPiter->pt.y - coor.y) / stdevx < stdevFromMean) )
        	  {
        	  		xval += KPiter->pt.x;
        	  		yval += KPiter->pt.y;
        	    	inliers++;
        	  }
        }
        
        KPiter++;
    }
    coor.x = xval / inliers, coor.y = yval / inliers;
    //cout << "Number of Points, Inliers:\t" << numMatchesToDB[bestMatchIdx] << "\t" << inliers << endl;

    //*/ Show only bestMatchIdx pic
    if (track)
    {
    //preparing mask so not all keypoints are shown, only links where imgIdx (DB image position) 
    vector<char> mask;
    mask.resize( matches.size() );
    //fill( mask.begin(), mask.end(), 0 );
    for( size_t i = 0; i < matches.size(); i++ )
    {
        if( matches[i].imgIdx == bestMatchIdx )
        {
        	cout << "Distance:\t" << matches[i].distance << endl; 
            mask[i] = 1;
        }
        else mask[i] = 0;
    }
    
    
    Mat drawImg;
    drawMatches( img, imgKp, DB.at(bestMatchIdx).img, DB.at(bestMatchIdx).keypoints, matches, drawImg, Scalar(255, 0, 0), Scalar(0, 255, 255), mask ); 
    
    imshow("Object Finding", drawImg);
    //waitKey();
    }
    //*/
        

    /*/  Show each match by pic    
    bool running = true;
    Mat drawImg;
    vector<char> mask;
    vector<DBobj>::iterator DBiter = DB.begin();   
    for( size_t i = 0; running ;  )
    {
        maskMatchesByTrainImgIdx( matches, (int)i, mask );
        drawMatches( img, imgKp, DBiter->img, DBiter->keypoints, matches, drawImg, Scalar(255, 0, 0), Scalar(0, 255, 255), mask );
        imshow("Matchs", drawImg);
                    
        switch ( (char) waitKey(5))
        {
           case 'q': case 'Q':
               running = false; 
               break;
           case 'i': case 'I':
                //cout <<  (bool) DBiter != DB.end()  << endl;
                if (( DBiter != DB.end() ) && ( i < DB.size()-1 )) 
        	{
            	DBiter++;
            	i++;
            	}
            	else
            	{
           	DBiter = DB.begin();
            	i = 0;
            	}
                cout << DBiter->name << "\tNumber of Matches: " << numMatchesToDB[i] << endl;
                break;       
        }
    }
    //*/
}


void ObjectRecognition::matchObsvToDB(const Mat &img, string& ObjName)
{
    vector<KeyPoint> imgKp;
    vector<DMatch> matches;
    matchObsvToDB(img, imgKp, matches);
    //for finding which picture has most matches
    
    int numMatchesToDB [(int)DB.size()], bestMatchIdx=0;
    for (int i = 0; i < (int)DB.size(); i++)  numMatchesToDB[i] = 0;
    //bin for finding which pic has most matches
    for (vector<DMatch>::iterator DMiter = matches.begin(); DMiter != matches.end(); DMiter++)
    {
        numMatchesToDB[DMiter->imgIdx]++; 
        if ( numMatchesToDB[bestMatchIdx] < numMatchesToDB[DMiter->imgIdx] ) bestMatchIdx = DMiter->imgIdx;
        //cout << "bestMatchIdx: " << numMatchesToDB[bestMatchIdx] << "\t" << numMatchesToDB[DMiter->imgIdx] << "\t" << bestMatchIdx << "\t" << DMiter->imgIdx << endl;
        //cout << "Match information (queryIDx/trainIDx/imgIDx/distance): " << 
        //    DMiter->queryIdx << "\t" << DMiter->trainIdx << "\t" << DMiter->imgIdx << "\t" << DMiter->distance << endl;
    }
    //cout << "Match time: " << matchTime << " ms with the best match at " <<  DB.at(bestMatchIdx).name << " with " << numMatchesToDB[bestMatchIdx] << " matching keypoints" << endl;
    
    
    //cout << "Observed Descriptors " << imgDesc.rows << " and number of matches " << (int)matches.size() << endl;

    
    
    ObjName = DB.at(bestMatchIdx).name;
    
    
    //*/ Show only bestMatchIdx pic
    
    //preparing mask so not all keypoints are shown, only links where imgIdx (DB image position) 
    vector<char> mask;
    mask.resize( matches.size() );
    //fill( mask.begin(), mask.end(), 0 );
    for( size_t i = 0; i < matches.size(); i++ )
    {
        if( matches[i].imgIdx == bestMatchIdx && matches[i].distance < .4) 
        {
//        cout << "Distance:\t" << matches[i].distance << endl; 
        mask[i] = 1;
        }
        else mask[i] = 0;
    }
    
    
    Mat drawImg;
    drawMatches( img, imgKp, DB.at(bestMatchIdx).img, DB.at(bestMatchIdx).keypoints, matches, drawImg, Scalar(255, 0, 0), Scalar(0, 255, 255), mask ); 
    
    imshow(DB.at(bestMatchIdx).name, drawImg);
    waitKey();
    
    //*/
        

    /*/  Show each match by pic    
    bool running = true;
    Mat drawImg;
    vector<char> mask;
    vector<DBobj>::iterator DBiter = DB.begin();   
    for( size_t i = 0; running ;  )
    {
        maskMatchesByTrainImgIdx( matches, (int)i, mask );
        drawMatches( img, imgKp, DBiter->img, DBiter->keypoints, matches, drawImg, Scalar(255, 0, 0), Scalar(0, 255, 255), mask );
        imshow("Matchs", drawImg);
                    
        switch ( (char) waitKey(5))
        {
           case 'q': case 'Q':
               running = false; 
               break;
           case 'i': case 'I':
                //cout <<  (bool) DBiter != DB.end()  << endl;
                if (( DBiter != DB.end() ) && ( i < DB.size()-1 )) 
        	{
            	DBiter++;
            	i++;
            	}
            	else
            	{
           	DBiter = DB.begin();
            	i = 0;
            	}
                cout << DBiter->name << "\tNumber of Matches: " << numMatchesToDB[i] << endl;
                break;       
        }
    }
    //*/
}

void ObjectRecognition::matchObsvToDB(const Mat &img, vector<KeyPoint>& imgKp, vector<DMatch>& matches)
{
    TickMeter tm;
    tm.start();
    Mat imgDesc;
    detectKeypointsSingle(img, imgKp);
    computeDescriptorsSingle(img, imgKp, imgDesc);
    matcher->match( imgDesc, matches );
    
    tm.stop();
    double matchTime = tm.getTimeMilli();
    CV_Assert( imgDesc.rows == (int)matches.size() || matches.empty() );   
    
}

void ObjectRecognition::maskMatchesByTrainImgIdx( const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask )
{
    mask.resize( matches.size() );
    fill( mask.begin(), mask.end(), 0 );
    for( size_t i = 0; i < matches.size(); i++ )
    {
        if( matches[i].imgIdx == trainImgIdx )
            mask[i] = 1;
    }
}

/* Main for debugging 
int main(int argc, char** argv)
{
    ObjectRecognition debug;
    debug.loadImageDB();
    string name;
    Mat img = imread(debug.observedImageName);
    debug.matchObsvToDB(img, name);
    cout << "Closes match is file: " << name << endl;
 //   debug.showDatabase();
}//*/
