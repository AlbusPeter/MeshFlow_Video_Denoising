#include <opencv2/opencv.hpp>
#include "MeshFlow.h"
#include "gridTracker.h"
#include "MotionDenoiser.h"
#include "DirectWarpDenoiser.h"
#include <time.h>


void main(){


	//MotionDenoiser denoiser("C:\\Users\\shuaicheng\\Desktop\\ICIP2017\\VideoDenoise\\ref\\burstdata\\data\\Square\\0.avi");
	//denoiser.Execute();
	//denoiser.SaveResult("C:\\Users\\shuaicheng\\Desktop\\ICIP2017\\VideoDenoise\\ref\\burstdata\\data\\Square\\0_.avi");


	//DirectWarpDenoiser denoiser("C:\\Users\\shuaicheng\\Desktop\\ICIP2017\\VideoDenoise\\ref\\burstdata\\data\\Toy\\0.avi");
	//denoiser.Execute();
	//denoiser.SaveResult("C:\\Users\\shuaicheng\\Desktop\\ICIP2017\\VideoDenoise\\ref\\burstdata\\results\\Toy\\our.png");


   vector<char*> names;
   vector<char*> outNames;
   
   for (int i = 1; i <= 1; i++){
	   char* name = new char[1024];
	   char* outname = new char[1024];

	   sprintf(name, "16.avi");
	   sprintf(outname, "16_out.avi");

	   names.push_back(name);   
	   outNames.push_back(outname);
	}

	for (int i = 0; i < names.size(); i++){
	   MotionDenoiser denoiser(names[i]);
	   denoiser.Execute();
	   denoiser.SaveResult(outNames[i]);
   }

	getchar();
}