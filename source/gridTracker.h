#include <opencv2\opencv.hpp>
#include <opencv2\video\tracking.hpp>

#define VARIANCE 4
#define MAXTRACKS 1000
#define GRIDSIZE 8
#define MASK_RADIUS 15

using namespace std;
using namespace cv;

#ifndef __GRIDTRACKER__
#define __GRIDTRACKER__

inline bool rule(const KeyPoint& p1, const KeyPoint& p2) 
{
  return p1.response > p2.response;
}

class gridTracker
{
  public:
    
	//mask used for ignoring regions of the image in the detector and for maintaining minimal feature distance
    cv::Mat curMask; 
    
    //tracked feas of current frame
    cv::vector<cv::Point2f> points1;
    cv::vector<cv::Point2f> trackedFeas;//matched Feas of currFrame
    
    //all feas of current frame
    cv::vector<cv::Point2f> allFeas;
	cv::vector<cv::Point2f> preFeas;//matched Feas of preFrame
    
    //num of feas from last frame
    int numActiveTracks;
    
    int TRACKING_HSIZE, LK_PYRAMID_LEVEL, MAX_ITER, fealimitGrid;
    
    float ACCURACY, LAMBDA;
    
    float usableFrac;
    
    //store image pyramid for re-utilization
    std::vector<cv::Mat> prevPyr;
    
	//int overflow;
    int MaxTracks;

	//grids devision
    cv::Point2i hgrids;
    
    //records feature number of each grids
    vector<int> feanumofGrid;
    
    float minAddFrac, minToAdd;
    
    int unusedRoom, gridsHungry;
    
    cv::vector<int> lastNumDetectedGridFeatures;
    
    //feature detector for Fast feature dtection one each gridCols
    vector<cv::FastFeatureDetector> detector;
	
    //thresholds for Fast detection on each grid
    vector<int> hthresholds;
    
    float DETECT_GAIN;
    
    //constructor
	gridTracker();
	
	bool maskPoint(float x, float y);
       
    bool Update(cv::Mat& im0, cv::Mat& im1);
    
    bool trackerInit(cv::Mat& im);
};

std::vector<cv::Mat> ReadVideoFastTracker(char* name);

std::vector<cv::Mat> ReadVideoFastTracker(char* videoName,char* featurefolderName);


#endif
