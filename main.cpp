#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "freespace.h"

static void help()
{
	std::string hotkeys =
		"\n\nHot keys: \n"
		"\tESC - quit the program\n"
		"\tt - toggle mode\n"
		"\tp - pause video\n";

	std::cout << hotkeys;
}

int main(int argc, char *argv[])
{
	if (argc < 4)
	{
		std::cout << "usage: " << argv[0] << " left-image-format right-image-format camera.xml" << std::endl;
		return -1;
	}

	int wsize = 11;
	cv::Ptr<cv::StereoSGBM> ssgbm = cv::StereoSGBM::create(0, 64, wsize, 8 * wsize * wsize, 32 * wsize * wsize);

	// input camera parameters
	cv::FileStorage cvfs(argv[3], CV_STORAGE_READ);
	if (!cvfs.isOpened())
	{
		std::cerr << "open camera.xml failed." << std::endl;
		return -1;
	}

	cv::FileNode node(cvfs.fs, NULL);
	float fu = node["FocalLengthX"];
	float fv = node["FocalLengthY"];
	float u0 = node["CenterX"];
	float v0 = node["CenterY"];
	float baseline = node["BaseLine"];
	float height = node["Height"];
	float tilt = node["Tilt"];

	FreeSpace freespace(fu, fv, u0, v0, baseline, height, tilt);
	int mode = FreeSpace::MODE_DP;

	help();
	for (int frameno = 1;; frameno++)
	{
		char bufl[256], bufr[256];
		sprintf(bufl, argv[1], frameno);
		sprintf(bufr, argv[2], frameno);

		cv::Mat left = cv::imread(bufl, -1);
		cv::Mat right = cv::imread(bufr, -1);

		if (left.empty() || right.empty())
		{
			std::cerr << "imread failed." << std::endl;
			break;
		}

		CV_Assert(left.size() == right.size() && left.type() == right.type());

		switch (left.type())
		{
		case CV_8U:
			// nothing to do
			break;
		case CV_16U:
			// conver to CV_8U
			double maxval;
			cv::minMaxLoc(left, NULL, &maxval);
			left.convertTo(left, CV_8U, 255 / maxval);
			right.convertTo(right, CV_8U, 255 / maxval);
			break;
		default:
			std::cerr << "unsupported image type." << std::endl;
			return -1;
		}

		// calculate dispaliry
		cv::Mat disp;
		ssgbm->compute(left, right, disp);
		disp.convertTo(disp, CV_32F, 1.0 / 16);

		// calculate free space
		std::vector<int> bounds;
		freespace.compute(disp, bounds, 1, 1, mode);

		// draw free space
		cv::Mat draw;
		cv::cvtColor(left, draw, cv::COLOR_GRAY2BGRA);
		for (int u = 0; u < left.cols; u++)
			for (int v = bounds[u]; v < left.rows; v++)
				draw.at<cv::Vec4b>(v, u) += cv::Vec4b(0, 0, 255, 0);

		cv::imshow("result", draw);
		cv::imshow("disp", disp / 64);

		cv::normalize(freespace.score_, freespace.score_, 0, 1, cv::NORM_MINMAX);
		cv::imshow("score", freespace.score_);

		char c = cv::waitKey(1);
		if (c == 27)
			break;
		if (c == 'p')
			cv::waitKey(0);
		if (c == 't')
			mode = !mode;
	}

	return 0;
}