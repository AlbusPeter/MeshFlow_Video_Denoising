#include "VideoIO.h"


vector<cv::Mat> GetFrames(char* name, double &fps){
	printf("Read Video\n");
	cv::VideoCapture capture(name);
	// check if video successfully opened
	if (!capture.isOpened())
	{
		cerr << "The video can not open!";
		exit(0);
	}


	fps = capture.get(CV_CAP_PROP_FPS);
	//int m_video_height = (int)capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	//int m_video_width = (int)capture.get(CV_CAP_PROP_FRAME_WIDTH);
	cv::Mat frame, frame_copy; // current video frame
	vector<cv::Mat> dst;
	printf("extract frames...");
	int frame_count = 0;
	capture.read(frame);
	while (capture.read(frame))
	{
		frame_copy = frame.clone();
		dst.push_back(frame_copy);
		//frame_count++;
	}
	//printf("%d\n", frame_count);
	capture.release();
	printf("Video Capture Done!\n");

	return dst;
}

void WriteFrames(vector<cv::Mat> Frames){
	cv::VideoWriter vw;
	int fps = 15;
	cv::Size S = Frames[0].size();
	vw.open("Results\\output.avi", CV_FOURCC('X', 'V', 'I', 'D'), fps, S);

	for (int i = 0; i < Frames.size(); i++){
		Frames[i].convertTo(Frames[i], CV_8UC3);
		vw << Frames[i];
	}

	vw.release();
}
