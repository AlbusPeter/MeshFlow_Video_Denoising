#include "MeshFlow.h"

MeshFlow::MeshFlow(int width, int height){
	
	m_height = height;
	m_width = width;
	m_quadWidth = 1.0*m_width/pow(2.0,3);
	m_quadHeight = 1.0*m_height/pow(2.0,3);

	m_mesh = new Mesh(m_height,m_width,1.0*m_quadWidth,1.0*m_quadHeight);
	m_warpedmesh = new Mesh(m_height,m_width,1.0*m_quadWidth,1.0*m_quadHeight);
	m_meshheight = m_mesh->height;
	m_meshwidth = m_mesh->width;
	m_vertexMotion.resize(m_meshheight*m_meshwidth);
}

void MeshFlow::ReInitialize(){
	for (int i = 0; i < m_meshheight*m_meshwidth; i++) m_vertexMotion[i].x = m_vertexMotion[i].y = 0;
	n.features.clear();
	n.motions.clear();
}

void MeshFlow::SetFeature(vector<cv::Point2f> &spt, vector<cv::Point2f> &tpt){

	n.features = tpt;
	n.motions.resize(tpt.size());

	for (int i = 0; i < tpt.size(); i++){
		n.motions[i] = spt[i] - tpt[i];
	}
	m_globalHomography = cv::findHomography(cv::Mat(spt), cv::Mat(tpt), CV_LMEDS);
}

void MeshFlow::Execute(){
	
	DistributeMotion2MeshVertexes_MedianFilter();  //the first median filter
	SpatialMedianFilter(); //the second median filter
	WarpMeshbyMotion();
}

void MeshFlow::DistributeMotion2MeshVertexes_MedianFilter(){
	
	for(int i=0;i<m_meshheight;i++){
		for(int j=0;j<m_meshwidth;j++){
			cv::Point2f pt = m_mesh->getVertex(i,j);
			cv::Point2f pttrans = Trans(m_globalHomography,pt);
			m_vertexMotion[i*m_meshwidth+j].x = pt.x - pttrans.x;
			m_vertexMotion[i*m_meshwidth+j].y = pt.y - pttrans.y;
		}
	}
	
	vector<vector<float>> motionx,motiony;
	motionx.resize(m_meshheight*m_meshwidth);
	motiony.resize(m_meshheight*m_meshwidth);

	//distribute features motion to mesh vertexes
	for(int i=0;i<m_meshheight;i++){
		for(int j=0;j<m_meshwidth;j++){
			cv::Point2f pt = m_mesh->getVertex(i,j);

			for(int k=0;k<n.features.size();k++){
				cv::Point2f pt2 = n.features[k];
				float dis = sqrt((pt.x - pt2.x)*(pt.x - pt2.x) + (pt.y - pt2.y)*(pt.y - pt2.y));
				
				float w = 1.0/dis;
				if(dis<RADIUS){
					motionx[i*m_meshwidth+j].push_back(n.motions[k].x);
					motiony[i*m_meshwidth+j].push_back(n.motions[k].y);
				}
			}
		}
	}

	for(int i=0;i<m_meshheight;i++){
		for(int j=0;j<m_meshwidth;j++){
			if(motionx[i*m_meshwidth+j].size()>1){
				myQuickSort(motionx[i*m_meshwidth+j],0,motionx[i*m_meshwidth+j].size()-1);
				myQuickSort(motiony[i*m_meshwidth+j],0,motiony[i*m_meshwidth+j].size()-1);
				m_vertexMotion[i*m_meshwidth+j].x = motionx[i*m_meshwidth+j][motionx[i*m_meshwidth+j].size()/2];
				m_vertexMotion[i*m_meshwidth+j].y = motiony[i*m_meshwidth+j][motiony[i*m_meshwidth+j].size()/2];
			} 
		}
	}
}

void MeshFlow::SpatialMedianFilter(){
	VecPt2f tempVertexMotion(m_meshheight*m_meshwidth);
	for(int i=0;i<m_meshheight;i++){
		for(int j=0;j<m_meshwidth;j++){
			tempVertexMotion[i*m_meshwidth+j] = m_vertexMotion[i*m_meshwidth+j];
		}
	}

	int radius = 5;
	for(int i=0;i<m_meshheight;i++){
		for(int j=0;j<m_meshwidth;j++){
			
			vector<float> motionx;
			vector<float> motiony;
			for(int k=-radius;k<=radius;k++){
				for(int l=-radius;l<=radius;l++){
					if(k>=0 && k<m_meshheight && l>=0 && l<m_meshwidth){
						motionx.push_back(tempVertexMotion[i*m_meshwidth+j].x);
						motiony.push_back(tempVertexMotion[i*m_meshwidth+j].y);
					}			   
				}
			}
			myQuickSort(motionx,0,motionx.size()-1);
			myQuickSort(motiony,0,motiony.size()-1);
			m_vertexMotion[i*m_meshwidth+j].x = motionx[motionx.size()/2];
			m_vertexMotion[i*m_meshwidth+j].y = motiony[motiony.size()/2];
		}
	}
}

void MeshFlow::WarpMeshbyMotion(){

	for(int i=0;i<m_meshheight;i++){
		for(int j=0;j<m_meshwidth;j++){
			cv::Point2f s = m_mesh->getVertex(i,j);
			s += m_vertexMotion[i*m_meshwidth+j];
			m_warpedmesh->setVertex(i,j,s);
		}
	}
}

void MeshFlow::GetMotions(cv::Mat &mapX, cv::Mat &mapY){

	vector<cv::Point2f> source(4);
	vector<cv::Point2f> target(4);
	cv::Mat H;

	for (int i = 1; i < m_mesh->height; i++)
	{
		for (int j = 1; j < m_mesh->width; j++)
		{
			Quad s = m_mesh->getQuad(i, j);
			Quad t = m_warpedmesh->getQuad(i, j);

			source[0] = s.V00;
			source[1] = s.V01;
			source[2] = s.V10;
			source[3] = s.V11;

			target[0] = t.V00;
			target[1] = t.V01;
			target[2] = t.V10;
			target[3] = t.V11;

			H = cv::findHomography(source, target, 0);

			for (int ii = source[0].y; ii<source[3].y; ii++){
				for (int jj = source[0].x; jj<source[3].x; jj++){
					double x = 1.0*jj;
					double y = 1.0*ii;

					double X = H.at<double>(0, 0) * x + H.at<double>(0, 1) * y + H.at<double>(0, 2);
					double Y = H.at<double>(1, 0) * x + H.at<double>(1, 1) * y + H.at<double>(1, 2);
					double W = H.at<double>(2, 0) * x + H.at<double>(2, 1) * y + H.at<double>(2, 2);

					W = W ? 1.0 / W : 0;
					mapX.at<float>(ii, jj) = X*W - jj;
					mapY.at<float>(ii, jj) = Y*W - ii;
				}
			}
		}
	}
}


void MeshFlow::GetWarpedSource(cv::Mat &dst, cv::Mat &mapX, cv::Mat &mapY){
	meshWarpRemap(m_source, dst, mapX,mapY,*m_mesh, *m_warpedmesh);
}

cv::Point2f MeshFlow::Trans(cv::Mat H,cv::Point2f &pt){
	cv::Point2f result;

	double a = H.at<double>(0,0) * pt.x + H.at<double>(0,1) * pt.y + H.at<double>(0,2) ;
	double b = H.at<double>(1,0) * pt.x + H.at<double>(1,1) * pt.y + H.at<double>(1,2) ;
	double c = H.at<double>(2,0) * pt.x + H.at<double>(2,1) * pt.y + H.at<double>(2,2) ;

	result.x = a/c;
	result.y = b/c;

	return result;
}