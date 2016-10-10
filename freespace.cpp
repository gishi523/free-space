#include "freespace.h"

namespace {

	const float SCORE_INV = -1.f;
	const float SCORE_DEFAULT = 1.f;

	void freeSpaceScore(const cv::Mat& disp, const std::vector<float>& roaddisp, cv::Mat& score,
		float fu, float fv, float baseline, float objecth, float paramO, float paramR, int vt = 0)
	{
		score.create(disp.size(), CV_32F);

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
				float roadscore = 0.f;
				for (int v = vb; v < disp.rows - 50; v++)
					roadscore += disp.at<float>(v, u) > 0.f ? fabsf(disp.at<float>(v, u) - roaddisp[v]) : SCORE_DEFAULT;

				score.at<float>(vb, u) = paramO * objectscore + paramR * roadscore;
			}
		}
	}

	void freeSpacePath(const cv::Mat& score, std::vector<int>& path, int vt = 0)
	{
		path.resize(score.cols);
		for (int u = 0; u < score.cols; u++)
		{
			float minscore = FLT_MAX;
			int minv = 0;
			for (int v = vt; v < score.rows; v++)
			{
				if (score.at<float>(v, u) == SCORE_INV)
					continue;

				if (score.at<float>(v, u) < minscore)
				{
					minscore = score.at<float>(v, u);
					minv = v;
				}
			}
			path[u] = minv;
		}
	}

	void freeSpacePathDP(const cv::Mat& disp, const cv::Mat& score, std::vector<int>& path,
		float P1, float P2, int vt = 0)
	{

		cv::Mat dpscore = score.clone();
		cv::Mat pathimg = cv::Mat::zeros(dpscore.size(), CV_32S);

		// forward path
		for (int u = 1; u < dpscore.cols; u++)
		{
			for (int v = vt; v < dpscore.rows; v++)
			{
				float minscore = FLT_MAX;
				int minpath = 0;

				int vvt = std::max(v - 2, vt);
				int vvb = std::min(v + 2, dpscore.rows);

				for (int vv = vvt; vv < vvb; vv++)
				{
					float jump = fabsf(disp.at<float>(vv, u - 1) - disp.at<float>(v, u));
					float penalty = std::min(P1 * jump, P1 * P2);
					float s = dpscore.at<float>(vv, u - 1) + penalty;
					if (s < minscore)
					{
						minscore = s;
						minpath = vv;
					}
				}

				dpscore.at<float>(v, u) += minscore;
				pathimg.at<int>(v, u) = minpath;
			}
		}

		// backward path
		path.resize(dpscore.cols);
		float minscore = FLT_MAX;
		int minv = 0;
		for (int v = vt; v < dpscore.rows; v++)
		{
			if (dpscore.at<float>(v, dpscore.cols - 1) == SCORE_INV)
				continue;

			if (dpscore.at<float>(v, dpscore.cols - 1) < minscore)
			{
				minscore = dpscore.at<float>(v, dpscore.cols - 1);
				minv = v;
			}
		}
		for (int u = pathimg.cols - 1; u >= 0; u--)
		{
			path[u] = minv;
			minv = pathimg.at<int>(minv, u);
		}
	}
}

FreeSpace::FreeSpace(float fu, float fv, float u0, float v0, float baseline, float camerah, float tilt)
	: fu_(fu), fv_(fv), u0_(u0), v0_(v0), baseline_(baseline), camerah_(camerah), tilt_(tilt)
{
}

void FreeSpace::compute(const cv::Mat& disp, std::vector<int>& bounds, float paramO, float paramR, int mode)
{
	CV_Assert(disp.type() == CV_32F);

	// calculate road disparity
	std::vector<float> roaddisp(disp.rows);
	for (int v = 0; v < disp.rows; v++)
		roaddisp[v] = (baseline_ / camerah_) * (fu_ * sinf(tilt_) + (v - v0_) * cosf(tilt_));

	// search v from which road dispaliry becomes negative
	int vt = 0;
	for (int v = disp.rows - 1; v >= 0; v--)
	{
		if (roaddisp[v] < 0.f)
		{
			vt = v + 1;
			break;
		}
	}

	// calculate score image
	freeSpaceScore(disp, roaddisp, score_, fu_, fv_, baseline_, 0.3f, paramO, paramR, vt);

	// extract the optimal free space path
	if (mode == MODE_DP)
	{
		freeSpacePathDP(disp, score_, bounds, 1, 30, vt);
	}
	else if (mode == MODE_MIN)
	{
		freeSpacePath(score_, bounds, vt);
	}
}