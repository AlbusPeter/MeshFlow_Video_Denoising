#include "DirectWarpDenoiser.h"

DirectWarpDenoiser::DirectWarpDenoiser(char* name){
	m_frames = GetFrames(name, m_fps);
	m_size = m_frames[0].size();
	m_height = m_size.height;
	m_width = m_size.width;
	m_frameNum = m_frames.size();

	m_dst.create(m_size, CV_8UC3);

	map_X.resize(m_frameNum);
	for (int i = 0; i < map_X.size(); i++) map_X[i].create(m_size, CV_32F);

	map_Y.resize(m_frameNum);
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
	m_mask_temp = cv::Mat::ones(m_size, CV_32FC1);

	formatX = cv::Mat::zeros(m_size, CV_32F);
	for (int i = 0; i < formatX.rows; i++)
		for (int j = 0; j < formatX.cols; j++)
			formatX.at<float>(i, j) = j;

	formatY = cv::Mat::zeros(m_size, CV_32F);
	for (int i = 0; i < formatY.rows; i++)
		for (int j = 0; j < formatY.cols; j++)
			formatY.at<float>(i, j) = i;
}

void DirectWarpDenoiser::MotionEstimation(int targetframeindex){
	
	MeshFlow meshflow(m_width, m_height);
	

	for (int i = 0; i < m_frameNum; i++){
		if (i != targetframeindex){
			
			meshflow.ReInitialize();
			gridTracker gt;
			gt.trackerInit(m_frames[targetframeindex]);
			gt.Update(m_frames[targetframeindex], m_frames[i]);
			
			/*std::vector<cv::Point2f> features1, features2;
			mySURF(m_frames[targetframeindex], m_frames[i], features1, features2);
			meshflow.SetFeature(features1,features2);*/
			
			meshflow.SetFeature(gt.trackedFeas, gt.preFeas);
			meshflow.Execute();
			meshflow.GetMotions(map_X[i], map_Y[i]);
			printf("%03d\b\b\b", i);
		}
	}

	printf("[DONE]\n");
}


void DirectWarpDenoiser::Execute(){
	
	int reference = 5;
	MotionEstimation(reference);
	TargetFrameBuild(reference, m_dst);
}


void DirectWarpDenoiser::TargetFrameBuild(int reference, cv::Mat &dst){
	m_frames[reference].convertTo(m_dst_temp, CV_32FC3);

	char name[1024];
	for (int i = 0; i < m_frameNum; i++){
		if (i != reference){
			m_mapedX = map_X[i] + formatX;
			m_mapedY = map_Y[i] + formatY;
			remap(m_frames[i], m_temp, m_mapedX, m_mapedY, cv::INTER_LINEAR);
			
			for (int i = 0; i < m_height; i++){
				for (int j = 0; j < m_width; j++){
					int a = abs(m_frames[reference].at<cv::Vec3b>(i, j)[1] - m_temp.at<cv::Vec3b>(i, j)[1]);
					float b = 1;
					
					//a > 10 ? b = 0 : b = 1;

					m_dst_temp.at<cv::Vec3f>(i, j)[0] += b * m_temp.at<cv::Vec3b>(i, j)[0];
					m_dst_temp.at<cv::Vec3f>(i, j)[1] += b * m_temp.at<cv::Vec3b>(i, j)[1];
					m_dst_temp.at<cv::Vec3f>(i, j)[2] += b * m_temp.at<cv::Vec3b>(i, j)[2];
					m_Counter_adder.at<float>(i, j) += b;

				}
			}
		}
	}


	/*
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
					a > 20 ? b = 1 : b = 1;

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
					a > 20 ? b = 1 : b = 1;

					m_dst_temp.at<cv::Vec3f>(i, j)[0] += b * m_temp.at<cv::Vec3b>(i, j)[0];
					m_dst_temp.at<cv::Vec3f>(i, j)[1] += b * m_temp.at<cv::Vec3b>(i, j)[1];
					m_dst_temp.at<cv::Vec3f>(i, j)[2] += b * m_temp.at<cv::Vec3b>(i, j)[2];
					m_Counter_adder.at<float>(i, j) += b;
				}
			}
		}
	}*/

	for (int i = 0; i < m_height; i++){
		for (int j = 0; j < m_width; j++){
			float d = m_Counter_adder.at<float>(i, j);
			dst.at<cv::Vec3b>(i, j)[0] = m_dst_temp.at<cv::Vec3f>(i, j)[0] / d;
			dst.at<cv::Vec3b>(i, j)[1] = m_dst_temp.at<cv::Vec3f>(i, j)[1] / d;
			dst.at<cv::Vec3b>(i, j)[2] = m_dst_temp.at<cv::Vec3f>(i, j)[2] / d;
		}
	}
}


void DirectWarpDenoiser::SaveResult(char* name){
	cv::imwrite(name,m_dst);
}