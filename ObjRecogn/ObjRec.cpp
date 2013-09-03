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

//************ Print Functions ************//

void printPrompt( const string& applName )
{
      cout << "Usage:  Really... Just run the program... (no params necessary)" << endl;
//    cout << "Usage: ObjRec <params file location/name>\n" << 
//            "Default params location ./data/SURF" << endl;
    
}

void showDatabase(const vector<Mat>& databaseImages, vector<vector<KeyPoint> >& databaseKeypoints, vector<Mat>& databaseDescriptors)
{
    int count =0;
    vector<vector<KeyPoint> >::iterator kpIter = databaseKeypoints.begin();
    vector<Mat>::const_iterator tdIter = databaseDescriptors.begin();
    cout << "Displaying Database\tdbImages Size: " << databaseImages.size() << "\tdbKeypoints Size:" << databaseKeypoints.size() 
    		<< "\tdbDescriptors Size: " << databaseDescriptors.size() <<  endl; 
    for (; kpIter != databaseKeypoints.end() || tdIter != databaseDescriptors.end() ; )
    {
        cout << "Picture " << count << " has \t" << kpIter->size() << " keypoints and  \t" <<  tdIter->rows << " descriptors for those key points" << endl;
        count++;
        kpIter++;
        tdIter++;        
    }
    cout << endl;
}


//************ Writing then Saving Functions  ************//

static void saveloadDDM( const string& params_filename,
                         Ptr<FeatureDetector>& detector,
                         Ptr<DescriptorExtractor>& descriptor,
                         Ptr<DescriptorMatcher>& matcher )
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

bool saveDatabaseOfDescriptors(const string& dirName,const vector<Mat>& databaseDescriptors, vector<string> files)
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
}



void readobservedFilenames( const string& filename, string& dirName, vector<string>& observedFilenames )
{
    observedFilenames.clear();

    ifstream file( filename.c_str() );
    if ( !file.is_open() )
        return;

    size_t pos = filename.rfind('\\');
    char dlmtr = '\\';
    if (pos == String::npos)
    {
        pos = filename.rfind('/');
        dlmtr = '/';
    }
    dirName = pos == string::npos ? "" : filename.substr(0, pos) + dlmtr;

    while( !file.eof() )
    {
        string str; getline( file, str );
        if( str.empty() ) break;
        observedFilenames.push_back(str);
    }
    file.close();
}


bool readImages( const string& observedImageName, const string& observedFilename,
                 Mat& observedImage, vector <Mat>& databaseImages, vector<string>& observedImageNames )
{
    cout << "< \nReading the images..." << endl;
    observedImage = imread( observedImageName, CV_LOAD_IMAGE_GRAYSCALE);
    if( observedImage.empty() )
    {
        cout << "Observed image can not be read." << endl << ">" << endl;
        return false;
    }
    
    string observedDirName;
    readobservedFilenames( observedFilename, observedDirName, observedImageNames );
    if( observedImageNames.empty() )
    {
        cout << "Database image filenames can not be read." << endl << ">" << endl;
        return false;
    }
    sort(observedImageNames.begin(),observedImageNames.end());
    
    int readImageCount = 0;
    for( size_t i = 0; i < observedImageNames.size(); i++ )
    {
        string filename = observedDirName + observedImageNames[i];
        Mat img = imread( filename, CV_LOAD_IMAGE_GRAYSCALE );
        cout << "Database image " << filename ;
        if( img.empty() )
            cout << " can not be read." << endl;
        else
            readImageCount++;
            cout << endl;
        databaseImages.push_back( img );
    }
    if( !readImageCount )
    {
        cout << "All database images can not be read." << endl << ">" << endl;
        return false;
    }
    else
        cout << readImageCount << " database images were read." << endl;
    cout << ">" << endl;

    return true;
}

int getdir ( const string &dir, vector<string> &files)
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

bool readDatabase(const string& dir, vector<Mat>& databaseDescriptors, vector<string>& files)
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
   cout << "End reading directory in " << tm.getTimeMilli() << "ms" << endl;
   return true;
}

//************ DDM Functions  ************//


bool createDetectorDescriptorMatcher( const string& detectorType, const string& descriptorType, const string& matcherType,
                                      Ptr<FeatureDetector>& featureDetector,
                                      Ptr<DescriptorExtractor>& descriptorExtractor,
                                      Ptr<DescriptorMatcher>& descriptorMatcher )
{
    cout << "< \nCreating feature detector, descriptor extractor and descriptor matcher ..." << endl;
    featureDetector = FeatureDetector::create( detectorType );
    descriptorExtractor = DescriptorExtractor::create( descriptorType );
    descriptorMatcher = DescriptorMatcher::create( matcherType );
    cout << ">" << endl;

    bool isCreated = !( featureDetector.empty() || descriptorExtractor.empty() || descriptorMatcher.empty() );
    if( !isCreated )
        cout << "Can not create feature detector or descriptor extractor or descriptor matcher of given types." << endl << ">" << endl;

    return isCreated;
}

void detectKeypointsSingle( const Mat& Image, vector<KeyPoint>& Keypoints,
                      Ptr<FeatureDetector>& featureDetector )
{
    cout << endl << "< \nExtracting keypoints from image..." << endl;
    featureDetector->detect( Image, Keypoints );
    cout << ">" << endl;
}

void detectKeypointsMulti( const vector<Mat>& databaseImages, vector<vector<KeyPoint> >& databaseKeypoints,
                      Ptr<FeatureDetector>& featureDetector )
{
    cout << endl << "< \nExtracting keypoints from images..." << endl;
    featureDetector->detect( databaseImages, databaseKeypoints );
    cout << ">" << endl;
}


void computeDescriptorsSingle( const Mat& observedImage, vector<KeyPoint>& observedKeypoints, Mat& observedDescriptors,
                        Ptr<DescriptorExtractor>& descriptorExtractor )
{
    cout << "< \nComputing descriptors for keypoints..." << endl;
    descriptorExtractor->compute( observedImage, observedKeypoints, observedDescriptors );

    cout << "Observed Image descriptors count: " << observedDescriptors.rows << endl;
    cout << ">" << endl;
}

void computeDescriptorsMulti( const vector<Mat>& databaseImages, vector<vector<KeyPoint> >& databaseKeypoints, vector<Mat>& databaseDescriptors,
                         Ptr<DescriptorExtractor>& descriptorExtractor )
{
    cout << "< \nComputing descriptors for database keypoints..." << endl;
    descriptorExtractor->compute( databaseImages, databaseKeypoints, databaseDescriptors );
    
    int totalDatabaseDesc = 0;
    for( vector<Mat>::const_iterator tdIter = databaseDescriptors.begin(); tdIter != databaseDescriptors.end(); tdIter++ )
        {
        cout << tdIter->type() << endl;
        totalDatabaseDesc += tdIter->rows;
        }

    cout << "Total observed descriptors count: " << totalDatabaseDesc << endl;
    cout << ">" << endl;
}

void matchDescriptors( const Mat& observedDescriptors, const vector<Mat>& databaseDescriptors,
                       vector<DMatch>& matches, Ptr<DescriptorMatcher>& descriptorMatcher )
{

    cout << "< Match observed image to database of images..." << endl;
    TickMeter tm;
    
    //*
    //  Batch Matcher that doesn't work with brute force
    tm.start();
    descriptorMatcher->add( databaseDescriptors );
    //descriptorMatcher->train();
    tm.stop();
    double buildTime = tm.getTimeMilli();

    tm.start();
    descriptorMatcher->match( observedDescriptors, matches );
    tm.stop();
    double matchTime = tm.getTimeMilli();
    
    
    /*/
    tm.start();
    vector<Mat>::const_iterator it;
    for ( it=databaseDescriptors.begin() ; it < databaseDescriptors.end(); it++ )
    {
        //DMatch temp;
  	descriptorMatcher->match( *it, observedDescriptors, matches);
        //matches.push_back(temp);
    }
    tm.stop();
    double matchTime = tm.getTimeMilli();
   /*/
    
    cout << "Number of matches: " << matches.size() << endl;
    cout << "Match time: " << matchTime << " ms" << endl;
    cout << ">" << endl;    
    
    cout << "Observed Descriptors " << observedDescriptors.rows << " and number of matches " << (int)matches.size() << endl;
    CV_Assert( observedDescriptors.rows == (int)matches.size() || matches.empty() );
    
}

void maskMatchesByTrainImgIdx( const vector<DMatch>& matches, int trainImgIdx, vector<char>& mask )
{
    mask.resize( matches.size() );
    fill( mask.begin(), mask.end(), 0 );
    for( size_t i = 0; i < matches.size(); i++ )
    {
        if( matches[i].imgIdx == trainImgIdx )
            mask[i] = 1;
    }
}


//************ Main ************//


int main(int argc, char** argv)
{

   
   string name = "SURF_FLANN";
   string detector_name = "SURF";
   string descriptor_name = "SURF";
   string matcher_name = "FlannBased";
   
   string dirName = "./data/descriptorDatabase/";
   
   Ptr<FeatureDetector> detector;
   Ptr<DescriptorExtractor> descriptor;
   Ptr<DescriptorMatcher> matcher;
   
   if ( !createDetectorDescriptorMatcher ( detector_name, descriptor_name, matcher_name, detector, descriptor, matcher) )
   {
       printPrompt( argv[0] );
       return -1;
   }
   
   //saveloadDDM( string(argv[1]), detector, descriptor, matcher );
   saveloadDDM( "./data/SURF_BF_params.yml", detector, descriptor, matcher );


   string fileWithDatabaseImages = "./data/train/trainImages.txt", observedImageName = "./data/query.png";   	
   
   Mat observedImage;
   vector<Mat> databaseImages;
   vector<string> databaseImagesNames;

   if( !readImages( observedImageName, fileWithDatabaseImages, observedImage, databaseImages, databaseImagesNames ) )
   {
       printPrompt( argv[0] );
       return -1;
   }
   
  //  detect database keypoints 
   
   vector<vector<KeyPoint> > databaseKeypoints;
   vector<KeyPoint>  observedKeypoints;
   
   detectKeypointsSingle ( observedImage, observedKeypoints, detector );
   detectKeypointsMulti ( databaseImages, databaseKeypoints, detector );
 
 //  compute descriptors for database keypoints and build table of values 
    
    Mat observedDescriptors;
    vector<Mat> databaseDescriptors;
    vector<string> files = vector<string>();


    //*/
    string dbDiscDir = string("./data/descriptorDatabase/");
    
    
    if (!readDatabase( dbDiscDir, databaseDescriptors, files))
    {
        printPrompt( argv[0] );
        return -1;
    }
    
     // Show total number of keypoints and descriptors for each image 
     //showDatabase( databaseImages, databaseKeypoints, databaseDescriptors);

    
    computeDescriptorsSingle (observedImage, observedKeypoints, observedDescriptors, descriptor);
      // only when adding to database, otherwise, skip and just load from last time
    //computeDescriptorsMulti (databaseImages, databaseKeypoints, databaseDescriptors, descriptor);
 
 
 //  Matching: compare query image to database (one at at time for now) and display strenght of connection
    vector<DMatch> matches;
    matchDescriptors( observedDescriptors, databaseDescriptors, matches, matcher);
 
    


    
    // Iterate through images and show matches
    bool running = true;
    Mat drawImg;
    vector<char> mask;
    for( size_t i = 0; running ;  )
    {
        if( !databaseImages[i].empty() )
        {
            if (i >= databaseImages.size() -1 ) i = 0;
            maskMatchesByTrainImgIdx( matches, (int)i, mask );
            drawMatches( observedImage, observedKeypoints, databaseImages[i], databaseKeypoints[i],
                         matches, drawImg, Scalar(255, 0, 0), Scalar(0, 255, 255), mask );
            imshow("Matchs", drawImg);
            
            //Mat outImage = databaseImages[i];
            //drawKeypoints(databaseImages[i], databaseKeypoints[i], outImage, Scalar(255, 0, 0), 1 );
            //imshow("Keypoints", databaseImages[i]);
            
            switch ( (char) waitKey(5))
            {
               case 'q': case 'Q':
                   running = false; 
                   break;
               case 'i': case 'I':
                   i++;
                   cout << "picture: " << i <<  endl;
            
                   break;
               
            }
        }
    }
 // Save database of descriptors   
    saveDatabaseOfDescriptors( dirName, databaseDescriptors, files);
    
}

/*/ 
    vector<Mat> databaseDescriptors2;
    int count = 0;
    vector<Mat>::iterator tdIter = databaseDescriptors.begin();
    vector<Mat>::iterator tdIter2 = databaseDescriptors2.begin();
    for (; tdIter != databaseDescriptors.end() && tdIter2 != databaseDescriptors2.end() ; )
    {
        cout << "Picture " << count << " has \t" << tdIter->rows << " descriptors for ddb1 and  \t" <<  tdIter2->rows << " descriptors for ddb2" << endl;
        count++;
        tdIter++;
        tdIter2++;        
    }
    cout << endl;
    /*/
