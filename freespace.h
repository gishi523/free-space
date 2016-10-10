#include <vector>
#include <opencv2/opencv.hpp>

class FreeSpace
{
public:
	FreeSpace() = delete;
	FreeSpace(float fu, float fv, float u0, float v0, float baseline, float camerah, float tilt);
	void compute(const cv::Mat& disp, std::vector<int>& bounds, float paramO = 1.0f, float paramR = 1.0f);

	cv::Mat score_;
	float fu_, fv_, u0_, v0_, baseline_, camerah_, tilt_;
};