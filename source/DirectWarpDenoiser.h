#include "MeshFlow.h"
#include "VideoIO.h"
#include "gridTracker.h"
#include "time.h"
#include "MotionDenoiser.h"

#ifndef __DirectWarpDenoiser__
#define __DirectWarpDenoiser__
class DirectWarpDenoiser{

private:
	int m_height;
	int m_width;
	int m_frameNum;
	cv::Size m_size;
	double m_fps;

private:
	vector<cv::Mat> m_frames;
	vector<cv::Mat> map_X, map_Y;
	vector<cv::Mat> temp_map_X, temp_map_Y;

	cv::Mat m_dst;
	cv::Mat m_mask;
	cv::Mat m_dst_temp;
	cv::Mat m_diff;
	cv::Mat m_temp;
	cv::Mat m_mapedX, m_mapedY;

	cv::Mat m_Counter_adder, m_mask_temp;
	cv::Mat formatX, formatY;

private:
	void MotionEstimation(int targetframeindex);
	void AbsoluteMotion(int reference);
	void TargetFrameBuild(int reference, cv::Mat &dst);

public:
	DirectWarpDenoiser(char* name);
	void Execute();
	void SaveResult(char* name);
};
#endif