#include "freespace.h"

namespace {

	void freeSpaceScore(const cv::Mat& disp, const std::vector<float>& roaddisp, cv::Mat& score,
		float fu, float fv, float baseline, float camerah, float paramO, float paramR)
	{
		const int SCORE_INV = 9999;
		score.create(disp.size(), CV_32F);
		for (int vb = 0; vb < disp.rows; vb++)
		{
			if (roaddisp[vb] > 0.f)
			{
				// calculate the number of pixels to determine the object score
				float hc = 20.f;
				float ho = (roaddisp[vb] / baseline) * (fv / fu) * camerah;
				int hv = static_cast<int>(hc + ho + 0.5f);

				for (int u = 0; u < disp.cols; ++u)
				{
					// calculate the object score
					float objectscore = 0.f;
					for (int v = std::max(vb - hv, 0); v < vb; ++v)
						objectscore += fabsf(disp.at<float>(v, u) - roaddisp[vb]);

					// calculate the road score
					float roadscore = fabsf(disp.at<float>(vb, u) - roaddisp[vb]);

					score.at<float>(vb, u) = paramO * objectscore + paramR * roadscore;
				}
			}
			else
			{
				score.row(vb) = SCORE_INV;
			}
		}
	}

	void freeSpacePath(const cv::Mat& score, std::vector<int>& path)
	{
		path.resize(score.cols);
		for (int u = 0; u < score.cols; u++)
		{
			float minscore = FLT_MAX;
			int minv = 0;
			for (int v = 0; v < score.rows; v++)
			{
				if (score.at<float>(v, u) < minscore)
				{
					minscore = score.at<float>(v, u);
					minv = v;
				}
			}
			path[u] = minv;
		}
	}
}

FreeSpace::FreeSpace(float fu, float fv, float u0, float v0, float baseline, float camerah, float tilt)
	: fu_(fu), fv_(fv), u0_(u0), v0_(v0), baseline_(baseline), camerah_(camerah), tilt_(tilt)
{
}

void FreeSpace::compute(const cv::Mat& disp, std::vector<int>& bounds, float paramO, float paramR)
{
	CV_Assert(disp.type() == CV_32F);
	
	// calculate road disparity
	std::vector<float> roaddisp(disp.rows);
	for (int v = 0; v < disp.rows; v++)
		roaddisp[v] = (baseline_ / camerah_) * (fu_ * sinf(tilt_) + v * cosf(tilt_));

	// calculate score image
	freeSpaceScore(disp, roaddisp, score_, fu_, fv_, baseline_, camerah_, paramO, paramR);

	// extract the optimal free space path
	freeSpacePath(score_, bounds);
}