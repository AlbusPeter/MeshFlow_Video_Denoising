#include "Fast_klt.h"

void myKLT(const cv::Mat source, const cv::Mat target, vector<cv::Point2f> &sourceFeatures, vector<cv::Point2f> &targetFeatures){

	cv::Mat img0Gray = cv::Mat::zeros(source.rows, source.cols, CV_8UC1);
	cv::Mat curImgGray = cv::Mat::zeros(target.rows, target.cols, CV_8UC1);
	cvtColor(source, img0Gray, CV_RGB2GRAY);
	cvtColor(target, curImgGray, CV_RGB2GRAY);

	vector<cv::Point2f> featurePtSet0;
	int maxNum = 10000;
	goodFeaturesToTrack(img0Gray, featurePtSet0, maxNum, 0.05, 5);
	cornerSubPix(img0Gray, featurePtSet0, cv::Size(15, 15), cv::Size(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

	vector<cv::Point2f> curfeaturePtSet;
	vector<uchar> curFlag;
	vector<float> curErrSet;
	calcOpticalFlowPyrLK(img0Gray, curImgGray, featurePtSet0, curfeaturePtSet, curFlag, curErrSet, cv::Size(15, 15));
	for (int p = 0; p < curErrSet.size(); p++)
		if (curErrSet[p] > 100 || curfeaturePtSet[p].x < 0 || curfeaturePtSet[p].y < 0 || curfeaturePtSet[p].x > img0Gray.cols || curfeaturePtSet[p].y > img0Gray.rows)
			curFlag[p] = 0;

	for (int i = 0; i<curFlag.size(); i++){
		if (curFlag.at(i) == 1){
			sourceFeatures.push_back(featurePtSet0.at(i));
			targetFeatures.push_back(curfeaturePtSet.at(i));
		}
	}
}
