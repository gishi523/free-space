#include "freespace.h"

namespace {

	const float SCORE_INV = -1.f;
	const float SCORE_DEFAULT = 1.f;

	void freeSpaceScore(const cv::Mat& disp, const std::vector<float>& roaddisp, cv::Mat& score,
		float fu, float fv, float baseline, float objecth, float paramO, float paramR, int vt = 0)
	{
		score.create(disp.size(), CV_32F);

		for (int vb = 0; vb < vt; vb++)
			score.col(vb) = SCORE_INV;

		int V = disp.cols - 50;

		// make integral image
		cv::Mat integral;
		integral.create(disp.size(), CV_32F);
		for (int u = 0; u < disp.rows; u++)
		{
			integral.at<float>(u, vt) = disp.at<float>(u, vt) > 0.f ? fabsf(disp.at<float>(u, vt) - roaddisp[vt]) : SCORE_DEFAULT;
			for (int v = vt + 1; v < disp.cols; v++)
			{
				integral.at<float>(u, v) = integral.at<float>(u, v - 1)
					+ (disp.at<float>(u, v) > 0.f ? fabsf(disp.at<float>(u, v) - roaddisp[v]) : SCORE_DEFAULT);
			}
		}

		for (int u = 0; u < disp.rows; u++)
		{
			for (int vb = vt; vb < disp.cols; vb++)
			{
				// calculate the number of pixels to determine the object score
				float hc = 20.f;
				float ho = (roaddisp[vb] / baseline) * (fv / fu) * objecth;
				int hv = static_cast<int>(hc + ho + 0.5f);

				// calculate the object score
				float objectscore = 0.f;
				for (int v = std::max(vb - hv, 0); v < vb; ++v)
					objectscore += disp.at<float>(u, v) > 0.f ? fabsf(disp.at<float>(u, v) - roaddisp[vb]) : SCORE_DEFAULT;

				// calculate the road score
				float roadscore = integral.at<float>(u, V - 1) - integral.at<float>(u, vb - 1);
				roadscore = std::max(roadscore, 0.f);
				score.at<float>(u, vb) = paramO * objectscore + paramR * roadscore;
			}
		}
	}

	void freeSpacePath(const cv::Mat& score, std::vector<int>& path, int vt = 0)
	{
		path.resize(score.rows);
		for (int u = 0; u < score.rows; u++)
		{
			float minscore = FLT_MAX;
			int minv = 0;
			for (int v = vt; v < score.cols; v++)
			{
				if (score.at<float>(u, v) == SCORE_INV)
					continue;

				if (score.at<float>(u, v) < minscore)
				{
					minscore = score.at<float>(u, v);
					minv = v;
				}
			}
			path[u] = minv;
		}
	}

	void freeSpacePathDP(const cv::Mat& disp, const cv::Mat& score, std::vector<int>& path,
		float P1, float P2, int vt = 0, int maxpixjumb = 5)
	{

		cv::Mat dpscore = score.clone();
		cv::Mat pathimg = cv::Mat::zeros(dpscore.size(), CV_32S);

		// forward path
		for (int u = 1; u < dpscore.rows; u++)
		{
			for (int v = vt; v < dpscore.cols; v++)
			{
				float minscore = FLT_MAX;
				int minpath = 0;

				int vvt = std::max(v - maxpixjumb, vt);
				int vvb = std::min(v + maxpixjumb + 1, dpscore.cols);

				for (int vv = vvt; vv < vvb; vv++)
				{
					float dispjump = fabsf(disp.at<float>(u - 1, vv) - disp.at<float>(u, v));
					float penalty = std::min(P1 * dispjump, P1 * P2);
					float s = dpscore.at<float>(u - 1, vv) + penalty;
					if (s < minscore)
					{
						minscore = s;
						minpath = vv;
					}
				}

				dpscore.at<float>(u, v) += minscore;
				pathimg.at<int>(u, v) = minpath;
			}
		}

		// backward path
		path.resize(dpscore.rows);
		float minscore = FLT_MAX;
		int minv = 0;
		for (int v = vt; v < dpscore.cols; v++)
		{
			if (dpscore.at<float>(dpscore.rows - 1, v) == SCORE_INV)
				continue;

			if (dpscore.at<float>(dpscore.rows - 1, v) < minscore)
			{
				minscore = dpscore.at<float>(dpscore.rows - 1, v);
				minv = v;
			}
		}
		for (int u = pathimg.rows - 1; u >= 0; u--)
		{
			path[u] = minv;
			minv = pathimg.at<int>(u, minv);
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

	cv::Mat dispt = disp.t();

	// calculate score image
	freeSpaceScore(dispt, roaddisp, score_, fu_, fv_, baseline_, 0.3f, paramO, paramR, vt);

	// extract the optimal free space path
	if (mode == MODE_DP)
	{
		freeSpacePathDP(dispt, score_, bounds, 100, 32, vt);
	}
	else if (mode == MODE_MIN)
	{
		freeSpacePath(score_, bounds, vt);
	}
	score_ = score_.t();
}