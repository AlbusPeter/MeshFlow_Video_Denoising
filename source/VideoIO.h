#include <opencv2/opencv.hpp>
#include <vector>
using namespace std;

vector<cv::Mat> GetFrames(char* name, double &fps);

void WriteFrames(vector<cv::Mat> Frames);