#include "freespace.h"

namespace {

	const float SCORE_INV = -1.f;
	const float SCORE_DEFAULT = 1.f;

	void freeSpaceScore(const cv::Mat& disp, const std::vector<float>& roaddisp, cv::Mat& score,
		float fu, float fv, float baseline, float objecth, float paramO, float paramR)
	{
		score.create(disp.size(), CV_32F);

		int vt = 0;
		for (int v = disp.rows - 1; v >= 0; v--)
		{
			if (roaddisp[v] < 0.f)
			{
				vt = v + 1;
				break;
			}
		}

		for (int vb = 0; vb < vt; vb++)
			score.row(vb) = SCORE_INV;

		for (int vb = vt; vb < disp.rows; vb++)
		{
			// calculate the number of pixels to determine the object score
			float hc = 20.f;
			float ho = (roaddisp[vb] / baseline) * (fv / fu) * objecth;
			int hv = static_cast<int>(hc + ho + 0.5f);

			for (int u = 0; u < disp.cols; ++u)
			{
				// calculate the object score
				float objectscore = 0.f;
				for (int v = std::max(vb - hv, 0); v < vb; ++v)
					objectscore += disp.at<float>(v, u) > 0.f ? fabsf(disp.at<float>(v, u) - roaddisp[vb]) : SCORE_DEFAULT;

				// calculate the road score
				float roadscore = disp.at<float>(vb, u) > 0.f ? fabsf(disp.at<float>(vb, u) - roaddisp[vb]) : SCORE_DEFAULT;

				score.at<float>(vb, u) = paramO * objectscore + paramR * roadscore;
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
				if (score.at<float>(v, u) != SCORE_INV && score.at<float>(v, u) < minscore)
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
	for (int v = disp.rows - 1; v >= 0; v--)
		roaddisp[v] = (baseline_ / camerah_) * (fu_ * sinf(tilt_) + (v - v0_) * cosf(tilt_));

	// calculate score image
	freeSpaceScore(disp, roaddisp, score_, fu_, fv_, baseline_, 0.5f, paramO, paramR);

	// extract the optimal free space path
	freeSpacePath(score_, bounds);
}