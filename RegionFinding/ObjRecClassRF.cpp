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
{ // To use create an object which grabs data from DBdirName (where db is stored) loads it into matcher.  Then call matchObsvToDB(image) and will return closes image match
    private:    
        Ptr<FeatureDetector> detector;
        Ptr<DescriptorExtractor> descriptor;
        Ptr<DescriptorMatcher> matcher;
        Mat observedImage;
        
        vector<DBobj> DB;
        
        //vector<Mat> databaseImages;
        //vector<vector<KeyPoint> > databaseKeypoints;
        
        string dirName, params_name, detector_name, descriptor_name, matcher_name, DBdirName;
        bool createDetectorDescriptorMatcher( );
        void saveloadDDM( const string& params_filename );
        void detectKeypointsSingle( const Mat& Image, vector<KeyPoint>& Keypoints);
        void computeDescriptorsSingle( const Mat& observedImage, vector<KeyPoint>& observedKeypoints, Mat& observedDescriptors);
        int getdir ( const string &dir, vector<string> &files);
        bool loadImageDB ();
       
        void maskMatchesByTrainImgIdx( const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask );
        bool readDatabase(const string& dir, vector<Mat>& databaseDescriptors, vector<string>& files);
    public:
        string observedImageName;
        void matchObsvToDB(const Mat &img, string &name);
            
        ObjectRecognition();  // Contructor for Init
        void showDatabase ();  // show items of database
        
        
        bool saveDatabaseOfDescriptors(const string& dirName,const vector<Mat>& databaseDescriptors, vector<string> files);
        
        
};

//************ Constructors  ************//

ObjectRecognition::ObjectRecognition()
{
     //cout << "I am a created object" << endl;
     dirName = "./data", params_name = "SURF_FLANN", detector_name = "SURF", descriptor_name = "SURF", matcher_name = "FlannBased", DBdirName = dirName + "/imageDatabase/";
     observedImageName = dirName + "/query.png";
     
     if ( !createDetectorDescriptorMatcher () )
     {
       cout << "Error Creating DDM" << endl;
       //Should Prevent the class from being created
     }
     
     saveloadDDM( dirName + "/params/" + params_name + ".yml" );
     
     loadImageDB();
     
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
        fs << "}";
        // << "matcher" << "{";
        //matcher->write(fs);
        //fs << "}";
    }
}
/*
bool ObjectRecognition::saveDatabaseOfDescriptors(const string& dirName,const vector<Mat>& databaseDescriptors, vector<string> files)
{
    // Save discriptors
    
    cout << "Saving Database of Discriptors" << endl;
    TickMeter tm;
    tm.start();

    int fileCount = 0;
    string fileExtention = ".png";

    vector<Mat>::const_iterator it;    
    for ( it=databaseDescriptors.begin() ; it < databaseDescriptors.end(); it++ )
    {
        string fileName;
        fileName = dirName;
        stringstream fileCountStr;
        fileCountStr << fileCount;
        if (fileCount < 10) fileName.append("0");
        fileName.append(fileCountStr.str() + fileExtention);
        //cout << "Picture " << fileName << "\tcols: " << it->cols << " rows: " << it->rows << "\t\t";// << endl;
        // if ( temp.cols > 0 && temp.rows > 0 ) 
        //cout << imwrite( fileName, *it) << endl;
        fileCount++;
    }
    tm.stop();
    double savedTime = tm.getTimeMilli();
    cout << "Save Time: " << savedTime << " ms" << endl;
}*/

bool ObjectRecognition::loadImageDB()
{
   TickMeter tm;
   tm.start();  
   vector<string> files;
   
   getdir(DBdirName,files);
   
   string extention = ".png"; 

   vector<string>::iterator it = files.begin();
   vector<Mat> descriptorDatabase;
   for (unsigned int i = 0;i < files.size();i++) 
   {
       if ( files[i].size() > 4 && files[i].compare( files[i].size() - 4, 4 , extention) == 0)
       {
           DBobj DBentry;
           DBentry.name = files[i];
     	   DBentry.img = imread( DBdirName + files[i] );
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
   cout << "End reading directory in " << tm.getTimeMilli() << " ms, of size " << DB.size() << endl;
   return true;
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

bool ObjectRecognition::readDatabase(const string& dir, vector<Mat>& databaseDescriptors, vector<string>& files)
{
   TickMeter tm;
   tm.start();  
   getdir(dir,files);
   string outString = "Start Reading Directory.png";
   cout << outString << endl;
   
   string extention = ".png"; 
   vector<string>::iterator it = files.begin();
   for (unsigned int i = 0;i < files.size();i++) 
   {
       if ( files[i].size() > 4 && files[i].compare( files[i].size() - 4, 4 , extention) == 0)
       {
     	   Mat img = imread( dir + files[i] , CV_LOAD_IMAGE_GRAYSCALE );
           //if( img.empty() )  cout << "Database descriptor " << files[i] << " can not be read or has no information." << endl;
           
           //cout << files[i]  << "\tRows" << img.rows << "\t Cols" << img.cols << "\t Type/Depth: " << img.depth() << endl;    
           img.assignTo(img, 5);
           
           databaseDescriptors.push_back( img );
       }
       it++;
   }
   tm.stop();
   cout << "End reading directory in " << tm.getTimeMilli() << " ms, of size " << DB.size() << endl;
   return true;
}

//************ DDM Functions  ************//


bool ObjectRecognition::createDetectorDescriptorMatcher( )
{
    //cout << "< \nCreating feature detector, descriptor extractor and descriptor matcher ..." << endl;
    detector = FeatureDetector::create( detector_name );
    descriptor = DescriptorExtractor::create( descriptor_name );
    matcher = DescriptorMatcher::create( matcher_name );
    //cout << ">" << endl;

    bool isCreated = !( detector.empty() || descriptor.empty() || matcher.empty() );
    if( !isCreated )
        cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << ">" << endl;

    return isCreated;
}

void ObjectRecognition::detectKeypointsSingle( const Mat& Image, vector<KeyPoint>& Keypoints)
{
    //cout << endl << "< \nExtracting keypoints from image..." << endl;
    detector->detect( Image, Keypoints );
    //cout << ">" << endl;
}


void ObjectRecognition::computeDescriptorsSingle( const Mat& observedImage, vector<KeyPoint>& observedKeypoints, Mat& observedDescriptors)
{
    //cout << "< \nComputing descriptors for keypoints..." << endl;
    descriptor->compute( observedImage, observedKeypoints, observedDescriptors );

    //cout << "Observed Image descriptors count: " << observedDescriptors.rows << endl;
    //cout << ">" << endl;
}

//Match observed to database batch above single below.  Since they need different init i just comment one out
/*void ObjectRecognition::matchObsvToDB(Mat &img)
{
    img.assignTo(img, CV_8U);
    vector<DMatch> matches;
    TickMeter tm;
    tm.start();
    vector<KeyPoint> imgKp;
    Mat imgDesc;
    detectKeypointsSingle(img, imgKp);
    computeDescriptorsSingle(img, imgKp, imgDesc);
    matcher->match( imgDesc, matches );
    tm.stop();
    double matchTime = tm.getTimeMilli();
    
    
    
    cout << "Number of matches: " << matches.size() << endl;
    cout << "Match time: " << matchTime << " ms" << endl;
    
    cout << "Observed Descriptors " << imgDesc.rows << " and number of matches " << (int)matches.size() << endl;
    CV_Assert( imgDesc.rows == (int)matches.size() || matches.empty() );
    
}*/

void ObjectRecognition::matchObsvToDB(const Mat &img, string& ObjName)
{
    //img.assignTo(img, CV_8U);
    vector<DMatch> matches;
    vector<vector<DMatch> > total_matches;
    TickMeter tm;
    tm.start();
    vector<KeyPoint> imgKp;
    Mat imgDesc;
    detectKeypointsSingle(img, imgKp);
    computeDescriptorsSingle(img, imgKp, imgDesc);
    matcher->match( imgDesc, matches );
    
    
    
    /*/ 
     //Match each item in database to pic (problem is it then matches to best keypoint and need to find a way to see which image in database is best)
         // I tried variance of distances but that wasn't reliable, didn't try finding var of angle but probably wouldn't be reliable either
    for ( vector<DBobj>::iterator DBiter = DB.begin() ; DBiter != DB.end(); DBiter++ )
    {
  	matcher->match( imgDesc, DBiter->description, matches);
        total_matches.push_back(matches);
        
        float mean = 0, var = 0;
        
        for (vector<DMatch>::iterator DMiter = matches.begin(); DMiter != matches.end(); DMiter++) mean += DMiter->distance; 
        mean = mean / matches.size();
        for (vector<DMatch>::iterator DMiter = matches.begin(); DMiter != matches.end(); DMiter++) var += (DMiter->distance - mean) * (DMiter->distance - mean);
        cout << "# of Observed Matches to " << DBiter->name << " is " << matches.size() << " with a sd of: " << var <<  endl; 
    } /*/
    
    tm.stop();
    double matchTime = tm.getTimeMilli();
   
    
    //for finding which picture has most matches
    
    int numMatchesToDB [(int)DB.size()], bestMatchIdx=0;
    //init array
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
    
    cout << "Match time: " << matchTime << " ms with the best match at " << DB.at(bestMatchIdx).name << " with " << numMatchesToDB[bestMatchIdx] << " matching keypoints" << endl;
    
    
    //cout << "Observed Descriptors " << imgDesc.rows << " and number of matches " << (int)matches.size() << endl;
    CV_Assert( imgDesc.rows == (int)matches.size() || matches.empty() );
    
    
    ObjName = DB.at(bestMatchIdx).name;
    
    
    //*/ Show only bestMatchIdx pic
    
    //preparing mask so not all keypoints are shown, only links where imgIdx (DB image position) 
    vector<char> mask;
    mask.resize( matches.size() );
    fill( mask.begin(), mask.end(), 0 );
    for( size_t i = 0; i < matches.size(); i++ )
    {
        if( matches[i].imgIdx == bestMatchIdx )
            mask[i] = 1;
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
    string name;
    Mat img = imread(debug.observedImageName);
    debug.matchObsvToDB(img, name);
    cout << "Closes match is file: " << name << endl;
 //   debug.showDatabase();
}//*/
