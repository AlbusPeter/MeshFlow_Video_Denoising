#include "MotionDenoiser.h"

MotionDenoiser::MotionDenoiser(char* name){
	
	m_frames = GetFrames(name, m_fps);
	m_size = m_frames[0].size();
	m_height = m_size.height;
	m_width = m_size.width;
	m_frameNum = m_frames.size();

	dst.resize(m_frameNum);
	for (int i = 0; i < dst.size(); i++) dst[i].create(m_size, CV_8UC3);

	map_X.resize(m_frameNum - 1);
	for (int i = 0; i < map_X.size(); i++) map_X[i].create(m_size, CV_32F);

	map_Y.resize(m_frameNum - 1);
	for (int i = 0; i < map_Y.size(); i++) map_Y[i].create(m_size, CV_32F);

	temp_map_X.resize(2 * N);
	for (int i = 0; i < temp_map_X.size(); i++){
		temp_map_X[i].create(m_size, CV_32F);
		temp_map_X[i].setTo(1);
	}

	temp_map_Y.resize(2 * N);
	for (int i = 0; i < temp_map_Y.size(); i++){
		temp_map_Y[i].create(m_size, CV_32F);
		temp_map_Y[i].setTo(1);
	}

	m_mask.create(m_size, CV_32FC1);
	
	m_dst_temp = cv::Mat::zeros(m_size, CV_32FC3);
	m_diff = cv::Mat::zeros(m_size, CV_32FC1);

	m_temp = cv::Mat::zeros(m_size, CV_8UC3);
	m_mapedX = cv::Mat::zeros(m_size, CV_32FC3);
	m_mapedY = cv::Mat::zeros(m_size, CV_32FC3);
	
	m_Counter_adder = cv::Mat::ones(m_size, CV_32F);
	m_mask_temp=cv::Mat::ones(m_size, CV_32FC1);

	formatX = cv::Mat::zeros(m_size, CV_32F);
	for (int i = 0; i < formatX.rows; i++)
		for (int j = 0; j < formatX.cols; j++)
			formatX.at<float>(i, j) = j;
	
	formatY = cv::Mat::zeros(m_size, CV_32F);
	for (int i = 0; i < formatY.rows; i++)
		for (int j = 0; j < formatY.cols; j++)
			formatY.at<float>(i, j) = i;
}

void MotionDenoiser::MotionEstimation(){
	
	MeshFlow meshflow(m_width,m_height);
	vector<cv::Point2f> sourceFeatures, targetFeatures;

	for (int i = 1; i < m_frameNum; i++){
		meshflow.ReInitialize();
		sourceFeatures.clear();
		targetFeatures.clear();
		
		myKLT(m_frames[i - 1], m_frames[i], sourceFeatures, targetFeatures);
		meshflow.SetFeature(sourceFeatures, targetFeatures);

		meshflow.Execute();
		meshflow.GetMotions(map_X[i-1], map_Y[i-1]);
		printf("%03d\b\b\b", i);
	}
	printf("[DONE]\n");
}

void MotionDenoiser::Execute(){
	
	clock_t clockBegin, clockEnd;
	clockBegin = clock();
	MotionEstimation();
	for (int i = 0; i < m_frameNum; i++){
		AbsoluteMotion(i);
		TargetFrameBuild(i, dst[i]);
		printf("%03d\b\b\b", i);
		m_Counter_adder.setTo(1); 
	}
	clockEnd = clock();
	printf("\n%d\n", (clockEnd - clockBegin) / m_frameNum);
}

void MotionDenoiser::AbsoluteMotion(int reference){
	//left part 0 1 2 3 ,4, 5 6 7 8   0 1 2 ,3, 4 5 6 7
	//			 0 1 2 3   4 5 6 7     0 1 2   3 4 5 6
	//char filename[30];
	for (int i = reference - N, k = 0; i < reference && k < N; i++,k++){
		if (i >= 0){
			temp_map_X[k] = map_X[i];
			temp_map_Y[k] = map_Y[i];
			for (int j = i + 1 ; j < reference; j++){
				temp_map_X[k] += map_X[j];
				temp_map_Y[k] += map_Y[j];
			}
		}
	}
	//right part
	for (int i = reference + N - 1, k = 2 * N - 1; i >= reference&&k >= N; i--, k--){
		if (i < m_frames.size() - 1){
			temp_map_X[k] = -map_X[i];
			temp_map_Y[k] = -map_Y[i];
			for (int j = i - 1; j >= reference; j--){
				temp_map_X[k] -= map_X[j];
				temp_map_Y[k] -= map_Y[j];
			}
		}
	}
}

void MotionDenoiser::TargetFrameBuild(int reference, cv::Mat &dst){
	m_frames[reference].convertTo(m_dst_temp, CV_32FC3);

	//left part
	for (int k = reference - N, m = 0; k < reference&&m < N; k++, m++){
		if (k >= 0){
			m_mapedX = temp_map_X[m] + formatX;
			m_mapedY = temp_map_Y[m] + formatY;
			remap(m_frames[k], m_temp, m_mapedX, m_mapedY, cv::INTER_LINEAR);

			for (int i = 0; i < m_height; i++){
				for (int j = 0; j < m_width; j++){
					int a = abs(m_frames[reference].at<cv::Vec3b>(i, j)[1] - m_temp.at<cv::Vec3b>(i, j)[1]);
					float b = 0;
					a > 20 ? b = 0 : b = 1;

					m_dst_temp.at<cv::Vec3f>(i, j)[0] += b * m_temp.at<cv::Vec3b>(i, j)[0];
					m_dst_temp.at<cv::Vec3f>(i, j)[1] += b * m_temp.at<cv::Vec3b>(i, j)[1];
					m_dst_temp.at<cv::Vec3f>(i, j)[2] += b * m_temp.at<cv::Vec3b>(i, j)[2];
					m_Counter_adder.at<float>(i, j) += b;

				}
			}
			
		}
	}

	//right part
	for (int k = reference + 1, m = N; k < reference + N + 1 && m < 2 * N; k++, m++){
		if (k < m_frames.size()){
			m_mapedX = temp_map_X[m] + formatX;
			m_mapedY = temp_map_Y[m] + formatY;
			remap(m_frames[k], m_temp, m_mapedX, m_mapedY, cv::INTER_LINEAR);

			for (int i = 0; i < m_height; i++){
				for (int j = 0; j < m_width; j++){
					int a = abs(m_frames[reference].at<cv::Vec3b>(i, j)[1] - m_temp.at<cv::Vec3b>(i, j)[1]);
					float b = 0;
					a > 20 ? b = 0 : b = 1;
					
					m_dst_temp.at<cv::Vec3f>(i, j)[0] += b * m_temp.at<cv::Vec3b>(i, j)[0];
					m_dst_temp.at<cv::Vec3f>(i, j)[1] += b * m_temp.at<cv::Vec3b>(i, j)[1];
					m_dst_temp.at<cv::Vec3f>(i, j)[2] += b * m_temp.at<cv::Vec3b>(i, j)[2];
					m_Counter_adder.at<float>(i, j) += b; 
				}
			}
		}
	}

	for (int i = 0; i < m_height; i++){
		for (int j = 0; j < m_width; j++){
			float d = m_Counter_adder.at<float>(i, j);
			dst.at<cv::Vec3b>(i, j)[0] = m_dst_temp.at<cv::Vec3f>(i, j)[0] / d;
			dst.at<cv::Vec3b>(i, j)[1] = m_dst_temp.at<cv::Vec3f>(i, j)[1] / d;
			dst.at<cv::Vec3b>(i, j)[2] = m_dst_temp.at<cv::Vec3f>(i, j)[2] / d;
		}
	}
}

void MotionDenoiser::SaveResult(char* name){
	
	cv::VideoWriter outVideoWriter;
	outVideoWriter.open(name, CV_FOURCC('X', 'V', 'I', 'D'), m_fps,m_size);
	
	for (int i = 0; i < m_frameNum; i++){
		outVideoWriter << dst[i];
	}
	outVideoWriter.~VideoWriter();
}
