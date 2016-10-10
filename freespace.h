#include <vector>
#include <opencv2/opencv.hpp>

class FreeSpace
{
public:
	void compute(const cv::Mat& disp, std::vector<int>& bounds);
	cv::Mat score;
	float fu, fv, u0, v0, baseline, camerah, tilt;
	float paramO, paramR;
};