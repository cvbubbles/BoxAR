#pragma once

#include"vxbase.h"
#include"tracker_utils.h"
#include<queue>
#include"v1.DetectorModelData.h"
#include"opencv2/xfeatures2d.hpp"
#include"BFC/thread.h"
#include"opencv2/optflow.hpp"

_VX_BEG(v1)

_IMPL_BEG(impl_base_tracker1)

typedef re3d::RigidPose Pose;

inline Mat1b getRenderMask(const Mat1f& depth, float eps = 1e-6)
{
	Mat1b mask = Mat1b::zeros(depth.size());
	for_each_2(DWHN1(depth), DN1(mask), [eps](float d, uchar& m) {
		m = fabs(1.0f - d) < eps ? 0 : 255;
		});
	return mask;
}

//
//template<typename _Tp>
//inline Rect_<_Tp> _getBoundingBox2D(const std::vector<Point_<_Tp>>& pts)
//{
//	_Tp L = INT_MAX, T = INT_MAX, R = 0, B = 0;
//	for (auto& p : pts)
//	{
//		if (p.x < L)
//			L = p.x;
//		if (p.x > R)
//			R = p.x;
//		if (p.y < T)
//			T = p.y;
//		if (p.y > B)
//			B = p.y;
//	}
//	return Rect_<_Tp>(L, T, R - L, B - T);
//}
//
//inline Rect getBoundingBox2D(const std::vector<Point>& pts)
//{
//	return _getBoundingBox2D(pts);
//}
//inline Rect_<float> getBoundingBox2D(const std::vector<Point2f>& pts)
//{
//	return _getBoundingBox2D(pts);
//}

struct CPoint
{
	Point3f    center;
	Point3f    normal;

	DEFINE_BFS_IO_2(CPoint, center, normal)
};



class EdgeSampler
{
public:
	static void _sampleExternalContours(Rect roiRect, const Mat1f& depth, CVRProjector& prj, std::vector<CPoint>& c3d, int nSamples)
	{
		Mat1b depthMask = getRenderMask(depth);

		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(depthMask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

		const int dstride = stepC(depth);
		auto get3D = [&depthMask, &depth, dstride, &prj, roiRect](int x, int y, Point3f& P, float& _z) {
			float z = 0;
			if (depthMask(y, x) != 0)
				z = depth(y, x);
			else
			{
				CV_Assert(false);
			}
			P = prj.unproject(float(x + roiRect.x), float(y + roiRect.y), z);
			_z = z;

			return true;
		};

		size_t npt = 0;
		for (auto& c : contours)
			npt += c.size();
		int nSkip = (int)npt / nSamples;

		const int smoothWSZ = 15, hwsz = smoothWSZ / 2;
		float maxDiff = 1.f;

		for (auto& c : contours)
		{
			double area = cv::contourArea(c, true);
			if (area < 0) //if is clockwise
				std::reverse(c.begin(), c.end()); //make it counter-clockwise

			Mat2i  cimg(1, c.size(), (Vec2i*)&c[0], int(c.size()) * sizeof(Vec2f));
			Mat2f  smoothed;
			boxFilter(cimg, smoothed, CV_32F, Size(smoothWSZ, 1));
			const Point2f* smoothedPts = smoothed.ptr<Point2f>();

			for (int i = nSkip / 2; i < (int)c.size(); i += nSkip)
			{
				/*Point2f dv = smoothedPts[i] - Point2f(c[i]);
				if (dv.dot(dv) > maxDiff * maxDiff)
					continue;*/

				CPoint P;
				float depth;
				if (get3D(c[i].x, c[i].y, P.center, depth))
				{
					Point2f n(0, 0);
					if (i - hwsz >= 0)
						n += smoothedPts[i] - smoothedPts[i - hwsz];
					if (i + hwsz < smoothed.cols)
						n += smoothedPts[i + hwsz] - smoothedPts[i];
					n = normalize(Vec2f(n));
					n = Point2f(-n.y, n.x);
					Point2f q = Point2f(c[i]) + n + Point2f(roiRect.x, roiRect.y);
					P.normal = prj.unproject(q.x, q.y, depth);
					c3d.push_back(P);
				}
			}
		}
	}

	static void sample(std::vector<CPoint>& c3d, const CVRResult& rr, int nSamples, Rect roiRect = Rect(0, 0, 0, 0), Size imgSize = Size(0, 0))
	{
		if (imgSize.width == 0 || imgSize.height == 0)
			imgSize = rr.img.size();

		if (roiRect.width == 0 || roiRect.height == 0)
			roiRect = Rect(0, 0, imgSize.width, imgSize.height);

		CVRProjector prj(rr.mats, imgSize);

		_sampleExternalContours(roiRect, rr.depth, prj, c3d, nSamples);
	}
	//static void _sampleInnerContours(Rect roiRect, const Mat& img, CVRProjector& prj, std::vector<CPoint>& c3d, int nSamples)
	//{
	//	Mat1b depthMask = getRenderMask(depth);

	//	std::vector<std::vector<cv::Point>> contours;
	//	cv::findContours(depthMask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	//	const int dstride = stepC(depth);
	//	auto get3D = [&depthMask, &depth, dstride, &prj, roiRect](int x, int y, Point3f& P, float& _z) {
	//		float z = 0;
	//		if (depthMask(y, x) != 0)
	//			z = depth(y, x);
	//		else
	//		{
	//			CV_Assert(false);
	//		}
	//		P = prj.unproject(float(x + roiRect.x), float(y + roiRect.y), z);
	//		_z = z;

	//		return true;
	//	};

	//	size_t npt = 0;
	//	for (auto& c : contours)
	//		npt += c.size();
	//	int nSkip = (int)npt / nSamples;

	//	const int smoothWSZ = 15, hwsz = smoothWSZ / 2;
	//	float maxDiff = 1.f;

	//	for (auto& c : contours)
	//	{
	//		double area = cv::contourArea(c, true);
	//		if (area < 0) //if is clockwise
	//			std::reverse(c.begin(), c.end()); //make it counter-clockwise

	//		Mat2i  cimg(1, c.size(), (Vec2i*)&c[0], int(c.size()) * sizeof(Vec2f));
	//		Mat2f  smoothed;
	//		boxFilter(cimg, smoothed, CV_32F, Size(smoothWSZ, 1));
	//		const Point2f* smoothedPts = smoothed.ptr<Point2f>();

	//		for (int i = nSkip / 2; i < (int)c.size(); i += nSkip)
	//		{
	//			/*Point2f dv = smoothedPts[i] - Point2f(c[i]);
	//			if (dv.dot(dv) > maxDiff * maxDiff)
	//				continue;*/

	//			CPoint P;
	//			float depth;
	//			if (get3D(c[i].x, c[i].y, P.center, depth))
	//			{
	//				Point2f n(0, 0);
	//				if (i - hwsz >= 0)
	//					n += smoothedPts[i] - smoothedPts[i - hwsz];
	//				if (i + hwsz < smoothed.cols)
	//					n += smoothedPts[i + hwsz] - smoothedPts[i];
	//				n = normalize(Vec2f(n));
	//				n = Point2f(-n.y, n.x);
	//				Point2f q = Point2f(c[i]) + n + Point2f(roiRect.x, roiRect.y);
	//				P.normal = prj.unproject(q.x, q.y, depth);
	//				c3d.push_back(P);
	//			}
	//		}
	//	}
	//}

	//static void sampleInnerContours(std::vector<CPoint>& c3d, const CVRResult& rr, int nSamples, Rect roiRect = Rect(0, 0, 0, 0), Size imgSize = Size(0, 0))
	//{
	//	if (imgSize.width == 0 || imgSize.height == 0)
	//		imgSize = rr.img.size();

	//	if (roiRect.width == 0 || roiRect.height == 0)
	//		roiRect = Rect(0, 0, imgSize.width, imgSize.height);

	//	CVRProjector prj(rr.mats, imgSize);

	//	_sampleInnerContours(roiRect, rr.depth, prj, c3d, nSamples);
	//}
};


struct Projector
{
	Matx33f _KR;
	Vec3f _Kt;
public:
	Projector(const Matx33f& _K, const Matx33f& _R, const Vec3f& _t)
		:_KR(_K* _R), _Kt(_K* _t)
	{
	}
	Point2f operator()(const Point3f& P) const
	{
		Vec3f p = _KR * Vec3f(P) + _Kt;
		return Point2f(p[0] / p[2], p[1] / p[2]);
	}
	template<typename _ValT, typename _getPointT>
	std::vector<Point2f> operator()(const std::vector<_ValT>& vP, _getPointT getPoint = [](const _ValT& v) {return v; }) const
	{
		std::vector<Point2f> vp(vP.size());
		for (int i = 0; i < (int)vP.size(); ++i)
			vp[i] = (*this)(getPoint(vP[i]));
		return vp;
	}
};


struct DView
{
	Vec3f       viewDir;
	cv::Matx33f R;
	/*std::vector<Point2f>  c2d;
	Point2f     maskCenter;
	Point2f     objCenter;*/
	//std::vector<Point2f>  contour;
	std::vector<CPoint>  contourPoints3d;

	//DEFINE_BFS_IO_6(DView, viewDir, R, c2d, maskCenter, objCenter, contourPoints3d)
	DEFINE_BFS_IO_3(DView, viewDir, R, contourPoints3d)
};

struct ViewIndex
{
	Mat1i  _uvIndex;
	Mat1i  _knnNbrs;
	double _du, _dv;
public:
	static void dir2uv(const Vec3f& dir, double& u, double& v)
	{
		v = acos(dir[2]);
		u = atan2(dir[1], dir[0]);
		if (u < 0)
			u += CV_PI * 2;
	}
	static Vec3f uv2dir(double u, double v)
	{
		return Vec3f(cos(u) * sin(v), sin(u) * sin(v), cos(v));
	}
	const int* getKnn(int vi) const
	{
		return _knnNbrs.ptr<int>(vi) + 1;
	}
	int getK() const
	{
		return _knnNbrs.cols - 1;
	}
	void build(const std::vector<DView>& views, int nU = 200, int nV = 100, int k = 5)
	{
		Mat1f viewDirs(Size(3, views.size()));
		for (size_t i = 0; i < views.size(); ++i)
		{
			memcpy(viewDirs.ptr(i), &views[i].viewDir, sizeof(float) * 3);
		}

		flann::Index  knn;
		knn.build(viewDirs, flann::KDTreeIndexParams(), cvflann::FLANN_DIST_L2);

		Mat1f dists;
		flann::SearchParams param;

		{
			Mat1i indices;
			knn.knnSearch(viewDirs, indices, dists, k + 1);

			CV_Assert(indices.size() == Size(k + 1, views.size()));
			for (int i = 0; i < (int)views.size(); ++i)
			{
				CV_Assert(indices.ptr<int>(i)[0] == i);
			}
			this->_knnNbrs = indices;
		}
		{
			Mat1f uvDir(Size(3, nU * nV));

			double du = 2 * CV_PI / (nU - 1), dv = CV_PI / (nV - 1);
			_du = du;
			_dv = dv;

			for (int vi = 0; vi < nV; ++vi)
			{
				for (int ui = 0; ui < nU; ++ui)
				{
					double u = ui * du, v = vi * dv;
					Vec3f dir = uv2dir(u, v);
					memcpy(uvDir.ptr(vi * nU + ui), &dir, sizeof(float) * 3);
				}
			}

			Mat1i indices;
			knn.knnSearch(uvDir, indices, dists, 1);
			CV_Assert(indices.size() == Size(1, nU * nV));

#if 0
			const int* idxData = indices.ptr<int>();
			Mat1i uvi(nV, nU);
			for (int y = 0; y < nV; ++y)
				for (int x = 0; x < nU; ++x)
				{
					uvi(y, x) = idxData[y * nU + x];
				}
			this->_uvIndex = indices;
#else
			this->_uvIndex = indices.reshape(1, nV);
			//imshow("uvindex", visIndex(_uvIndex));
#endif
		}
	}
	int getViewInDir(const Vec3f& dir) const
	{
		CV_Assert(fabs(dir.dot(dir) - 1) < 1e-2f);

		double u, v;
		dir2uv(dir, u, v);
		int ui = int(u / _du + 0.5), vi = int(v / _dv + 0.5);
		//return _views[_uvIndex(vi, ui)].c3d;
		return _uvIndex(vi, ui);
	}
};


class DFRHandler
{
public:
	virtual int test(const Matx33f& R, const Vec3f& t) = 0;

	virtual ~DFRHandler() {}
};

struct  PointMatch
{
	Point3f   modelPoint;
	Point2f   imPoint;
	float     weight;
};

struct Optimizer
{
public:	
	std::vector<PointMatch>  _pointMatches;


public:
	struct ContourPoint
	{
		float   w; //weight
		float   x; //position on the scan-line
	};

	enum { MAX_POINTS_PER_LINE = 3 };
	struct ScanLine
	{
		float     y;
		Point2f   xdir;
		Point2f   xstart;
		ContourPoint  vPoints[MAX_POINTS_PER_LINE];
		int       nPoints;
		short* cpIndex; //index of the closest contour point for each x position
	public:
		void setCoordinates(const Point2f& start, const Point2f& end, float y)
		{
			this->xstart = start;
			xdir = (Point2f)normalize(Vec2f(end - start));
			this->y = y;
		}
		int getClosestContourPoint(const Point2f& pt, int xsize)
		{
			int x = int((pt - xstart).dot(xdir) + 0.5f);
			if (uint(x) < uint(xsize))
				return cpIndex[x];
			return -1;
		}
	};

	struct DirData
	{
		Vec2f      dir;
		Point2f    ystart;
		Point2f    ydir;
		std::vector<ScanLine>  lines;
		Mat1s         _cpIndexBuf;
	public:
		void setCoordinates(const Point2f& ystart, const Point2f& ypt)
		{
			this->ystart = ystart;
			ydir = (Point2f)normalize(Vec2f(ypt - ystart));
		}
		void resize(int rows, int cols)
		{
			lines.clear();
			lines.resize(rows);
			_cpIndexBuf.create(rows, cols);
			for (int y = 0; y < rows; ++y)
			{
				lines[y].cpIndex = _cpIndexBuf.ptr<short>(y);
			}
		}
		const ScanLine* getScanLine(const Point2f& pt, int& matchedContourPoint)
		{
			int y = int((pt - ystart).dot(ydir) + 0.5f);
			if (uint(y) >= lines.size())
				return nullptr;
			matchedContourPoint = lines[y].getClosestContourPoint(pt, int(_cpIndexBuf.cols));
			return &lines[y];
		}
	};

	std::vector<DirData>  _dirs;
	Rect  _roi;
public:
	static void _gaussianFitting(const float* data, int size, ContourPoint& cp)
	{
		float w = 0.f, wsum = 0.f;
		for (int i = 0; i < size; ++i)
		{
			wsum += data[i] * float(i);
			w += data[i];
		}

		cp.x = wsum / w;
	}
	struct _LineBuilder
	{
		struct LocalMaxima
		{
			int x;
			float val;
		};
		std::vector<LocalMaxima>  _lmBuf;
	public:
		_LineBuilder(int size)
		{
			_lmBuf.resize(size);
		}

		void operator()(ScanLine& line, const float* data, int size, int gaussWindowSizeHalf)
		{
			LocalMaxima* vlm = &_lmBuf[0];
			int nlm = 0;
			for (int i = 1; i < size - 1; ++i)
			{
				if (data[i] > data[i - 1] && data[i] > data[i + 1])
				{
					auto& lm = vlm[nlm++];
					lm.x = i;
					lm.val = data[i];
				}
			}
			if (nlm > MAX_POINTS_PER_LINE)
			{
				std::sort(vlm, vlm + nlm, [](const LocalMaxima& a, const LocalMaxima& b) {
					return a.val > b.val;
					});
				nlm = MAX_POINTS_PER_LINE;

				std::sort(vlm, vlm + nlm, [](const LocalMaxima& a, const LocalMaxima& b) {
					return a.x < b.x;
					});
			}
			for (int i = 0; i < nlm; ++i)
			{
				auto& lm = vlm[i];
				auto& cp = line.vPoints[i];

				const int start = __max(0, lm.x - gaussWindowSizeHalf), end = __min(size, lm.x + gaussWindowSizeHalf);
				_gaussianFitting(data + start, end - start, cp);

				cp.x += (float)start;
				cp.w = lm.val;
			}
			line.nPoints = nlm;

			if (nlm <= 1)
				memset(line.cpIndex, nlm == 0 ? 0xFF : 0, sizeof(short) * size);
			else
			{
				int start = 0;
				for (int pi = 0; pi < nlm - 1; ++pi)
				{
					int end = int(int(line.vPoints[pi].x + line.vPoints[pi + 1].x) / 2 + 0.5f) + 1;
					for (int i = start; i < end; ++i)
						line.cpIndex[i] = pi;
					start = end;
				}
				for (int i = start; i < size; ++i)
					line.cpIndex[i] = nlm - 1;

				//for (int i = 0; i < size; ++i)
				//{
				//	float dmin = FLT_MAX;
				//	float mj = 0;
				//	for (int j = 0; j < nlm; ++j)
				//	{
				//		float d = line.vPoints[j].x - float(i);
				//		d = d * d;// / line.vPoints[j].variance;
				//		if (d < dmin)
				//		{
				//			dmin = d;
				//			mj = j;
				//		}
				//	}
				//	line.cpIndex[i] = mj;
				//}
			}
		}
	};
	static void _calcScanLinesForRows(const Mat1f& prob, DirData& dirPositive, DirData& dirNegative, const Matx23f& invA)
	{
		const int gaussWindowSizeHalf = 3;

		Mat1f edgeProb;
		cv::Sobel(prob, edgeProb, CV_32F, 1, 0, 7);


		{
			Point2f O = transA(Point2f(0.f, 0.f), invA.val), P = transA(Point2f(0.f, float(prob.rows - 1)), invA.val);
			dirPositive.setCoordinates(O, P);
			dirNegative.setCoordinates(P, O);
		}
		dirPositive.resize(prob.rows, prob.cols);
		dirNegative.resize(prob.rows, prob.cols);

		std::unique_ptr<float[]> _rdata(new float[prob.cols * 2]);
		float* posData = _rdata.get(), * negData = posData + prob.cols;
		_LineBuilder buildLine(prob.cols);

		const int xend = int(prob.cols - 1);
		for (int y = 0; y < prob.rows; ++y)
		{
			auto& positiveLine = dirPositive.lines[y];
			auto& negativeLine = dirNegative.lines[prob.rows - 1 - y];

			Point2f O = transA(Point2f(0.f, float(y)), invA.val), P = transA(Point2f(float(prob.cols - 1), float(y)), invA.val);
			positiveLine.setCoordinates(O, P, float(y));
			negativeLine.setCoordinates(P, O, float(prob.rows - 1 - y));

			const float* ep = edgeProb.ptr<float>(y);

			for (int x = 0; x < prob.cols; ++x)
			{
				if (ep[x] > 0)
				{
					posData[x] = ep[x]; negData[xend - x] = 0.f;
				}
				else
				{
					posData[x] = 0.f; negData[xend - x] = -ep[x];
				}
			}

			buildLine(positiveLine, posData, prob.cols, gaussWindowSizeHalf);
			buildLine(negativeLine, negData, prob.cols, gaussWindowSizeHalf);
		}
	}

	//Mat  _prob;
	std::vector<int>  _dirIndex;

	void computeScanLines(const Mat1f& prob_, Rect roi)
	{
		//_prob = prob_.clone(); //save for visualization

		const int N = 4;

		Point2f center(float(roi.x + roi.width / 2), float(roi.y + roi.height / 2));
		Rect_<float> roif(roi);
		std::vector<Point2f>  corners = {
			Point2f(roif.x, roif.y), Point2f(roif.x + roif.width,roif.y), Point2f(roif.x + roif.width,roif.y + roif.height),Point2f(roif.x,roif.y + roif.height)
		};

		struct _DDir
		{
			Vec2f   dir;
			Matx23f A;
		};

		_dirs.clear();
		_dirs.resize(N * 2);

		//for (int i = 0; i < N; ++i)
		cv::parallel_for_(cv::Range(0, N), [&](const cv::Range& r) {
			for (int i = r.start; i < r.end; ++i)
			{
				double theta = 180.0 / N * i;
				Matx23f A = getRotationMatrix2D(center, theta, 1.0);
				std::vector<Point2f> Acorners;
				cv::transform(corners, Acorners, A);
				cv::Rect droi = getBoundingBox2D(Acorners);
				A = Matx23f(1, 0, -droi.x,
					0, 1, -droi.y) * A;

				Mat1f dirProb;
				cv::warpAffine(prob_, dirProb, A, droi.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar(0));

				theta = theta / 180.0 * CV_PI;
				auto dir = Vec2f(cos(theta), sin(theta));

				/*imshow("dirProb", dirProb);
				cv::waitKey();*/

				auto& positiveDir = _dirs[i];
				auto& negativeDir = _dirs[i + N];

				auto invA = invertAffine(A);
				_calcScanLinesForRows(dirProb, positiveDir, negativeDir, invA);

				positiveDir.dir = dir;
				negativeDir.dir = -dir;
			}
			});

		//normalize weight of contour points
		{
			float wMax = 0;
			for (auto& dir : _dirs)
			{
				for (auto& dirLine : dir.lines)
				{
					for (int i = 0; i < dirLine.nPoints; ++i)
						if (dirLine.vPoints[i].w > wMax)
							wMax = dirLine.vPoints[i].w;
				}
			}

			for (auto& dir : _dirs)
			{
				for (auto& dirLine : dir.lines)
				{
					for (int i = 0; i < dirLine.nPoints; ++i)
						dirLine.vPoints[i].w /= wMax;
				}
			}
		}

		//build index of dirs
		if (_dirIndex.empty())
		{
			_dirIndex.resize(361);
			for (int i = 0; i < (int)_dirIndex.size(); ++i)
			{
				float theta = i * CV_PI / 180.f - CV_PI;
				Vec2f dir(cos(theta), sin(theta));

				float cosMax = -1;
				int jm = -1;
				for (int j = 0; j < (int)_dirs.size(); ++j)
				{
					float vcos = _dirs[j].dir.dot(dir);
					if (vcos > cosMax)
					{
						cosMax = vcos;
						jm = j;
					}
				}
				_dirIndex[i] = jm;
			}
		}

		_roi = roi;

#if 0
		{
			Mat dimg;

			int di = 1;
			for (auto& dir : _dirs)
			{
				cvtColor(prob_, dimg, CV_GRAY2BGR);

				for (auto& ln : dir.lines)
				{
					for (int i = 0; i < ln.nPoints; ++i)
					{
						auto& g = ln.vPoints[i];
						auto pt = Point(ln.xstart + ln.xdir * g.mean);
						cv::circle(dimg, pt, 3, Scalar(0, 255, 255));
					}
				}
				//break;
				imshow(ff::StrFormat("gaussCenters%d", di++), dimg);
			}
		}
#endif
	}
	DirData* getDirData(const Vec2f& ptNormal)
	{
#if 0
		float cosMax = -1;
		DirData* dd = nullptr;
		for (auto& dir : _dirs)
		{
			float vcos = dir.dir.dot(ptNormal);
			if (vcos > cosMax)
			{
				cosMax = vcos;
				dd = &dir;
			}
		}
		return dd;
#else
		float theta = atan2(ptNormal[1], ptNormal[0]);
		int i = int((theta + CV_PI) * 180 / CV_PI);
		auto* ddx = uint(i) < _dirIndex.size() ? &_dirs[_dirIndex[i]] : nullptr;
		return ddx;
#endif
	}

	struct PoseData
		:public Pose
	{
		int itr = 0;
		DFRHandler* hdl = nullptr;
	};

	float calcError(const PoseData& pose, const Matx33f& K, const std::vector<CPoint>& cpoints, float alpha)
	{
		const Matx33f R = pose.R;
		const Point3f t = pose.t;

		const float fx = K(0, 0), fy = K(1, 1), cx = K(0, 2), cy = K(1, 2);

		auto* vcp = &cpoints[0];
		int npt = (int)cpoints.size();
		float err = 0.f;
		float nerr = 0.f;

		for (int i = 0; i < npt; ++i)
		{
			Point3f Q = R * vcp[i].center + t;
			Point3f q = K * Q;
			{
				const int x = int(q.x / q.z + 0.5), y = int(q.y / q.z + 0.5);
				if (uint(x - _roi.x) >= uint(_roi.width) || uint(y - _roi.y) >= uint(_roi.height))
					continue;

				Point3f qn = K * (R * vcp[i].normal + t);
				Vec2f n(qn.x / qn.z - q.x / q.z, qn.y / qn.z - q.y / q.z);
				n = normalize(n);

				Point2f pt(q.x / q.z, q.y / q.z);

				auto* dd = this->getDirData(n);
				if (!dd)
					continue;
				int cpi;
				auto* dirLine = dd->getScanLine(pt, cpi);
				if (!dirLine || cpi < 0)
					continue;

				auto& cp = dirLine->vPoints[cpi];

				Vec2f nx = dirLine->xdir;

				float du = (pt - dirLine->xstart).dot(nx);

				float w = cp.w * cp.w;
				err += pow(fabs(du - cp.x), alpha) * w;

				nerr += w;
			}
		}

		int npm = int(_pointMatches.size());
		float ws = __min(float(npm) / 50.f, 3.f);
		float wpm = float(npt) / (float(npm) + 1e-3f) * ws;

		for (int i = 0; i < npm; ++i)
		{
			auto& pm = _pointMatches[i];
			Point3f Q = R * pm.modelPoint + t;
			Point3f q = K * Q;

			Point2f pt(q.x / q.z, q.y / q.z);

			Vec2f nx = pt - pm.imPoint;
			err += sqrt(nx.dot(nx)) + 1e-6f;

			nerr += wpm;
		}

		return err / nerr;
	}

	int _update(PoseData& pose, const Matx33f& K, const std::vector<CPoint>& cpoints, float alpha, float eps)
	{
		const Matx33f R = pose.R;
		const Point3f t = pose.t;

		if (pose.hdl && pose.hdl->test(R, t) <= 0)
			return 0;

		const float fx = K(0, 0), fy = K(1, 1);

		auto* vcp = &cpoints[0];
		int npt =  (int)cpoints.size();

		Matx66f JJ = Matx66f::zeros();
		Vec6f J(0, 0, 0, 0, 0, 0);

		for (int i = 0; i < npt; ++i)
		{
			Point3f Q = R * vcp[i].center + t;
			Point3f q = K * Q;
			/*if (q.z == 0.f)
				continue;
			else*/
			{
				const int x = int(q.x / q.z + 0.5), y = int(q.y / q.z + 0.5);
				if (uint(x - _roi.x) >= uint(_roi.width) || uint(y - _roi.y) >= uint(_roi.height))
					continue;

				Point3f qn = K * (R * vcp[i].normal + t);
				Vec2f n(qn.x / qn.z - q.x / q.z, qn.y / qn.z - q.y / q.z);
				n = normalize(n);

				const float X = Q.x, Y = Q.y, Z = Q.z;
				/*      |fx/Z   0   -fx*X/Z^2 |   |a  0   b|
				dq/dQ = |                     | = |        |
						|0    fy/Z  -fy*Y/Z^2 |   |0  c   d|
				*/
				const float a = fx / Z, b = -fx * X / (Z * Z), c = fy / Z, d = -fy * Y / (Z * Z);

				Point2f pt(q.x / q.z, q.y / q.z);

				auto* dd = this->getDirData(n);
				if (!dd)
					continue;
				int cpi;
				auto* dirLine = dd->getScanLine(pt, cpi);
				if (!dirLine || cpi < 0)
					continue;

				auto& cp = dirLine->vPoints[cpi];

				Vec2f nx = dirLine->xdir;
				float du = (pt - dirLine->xstart).dot(nx);

				Vec3f n_dq_dQ(nx[0] * a, nx[1] * c, nx[0] * b + nx[1] * d);

				auto dt = n_dq_dQ.t() * R;
				auto dR = vcp[i].center.cross(Vec3f(dt.val[0], dt.val[1], dt.val[2]));
				//auto dt = n_dq_dQ;
				//auto dR = Q.cross(dt);

				Vec6f j(dt.val[0], dt.val[1], dt.val[2], dR.x, dR.y, dR.z);

				float w = pow(1.f / (fabs(du - cp.x) + 1.f), 2.f - alpha) * cp.w * cp.w;

				J += w * (du - cp.x) * j;

				JJ += w * j * j.t();
			}
		}

		int npm = (int)_pointMatches.size();
		float ws = __min(float(npm) / 50.f, 3.f);
		float wpm =  float(npt) / (float(npm) + 1e-3f) * ws;
		//wpm = 1.f;

		for (int i = 0; i < npm; ++i)
		{
			auto& pm = _pointMatches[i];
			Point3f Q = R * pm.modelPoint + t;
			Point3f q = K * Q;

			/*const int x = int(q.x / q.z + 0.5), y = int(q.y / q.z + 0.5);
			if (uint(x - _roi.x) >= uint(_roi.width) || uint(y - _roi.y) >= uint(_roi.height))
				continue;

			Point3f qn = K * (R * vcp[i].normal + t);
			Vec2f n(qn.x / qn.z - q.x / q.z, qn.y / qn.z - q.y / q.z);
			n = normalize(n);*/

			const float X = Q.x, Y = Q.y, Z = Q.z;
			/*      |fx/Z   0   -fx*X/Z^2 |   |a  0   b|
			dq/dQ = |                     | = |        |
					|0    fy/Z  -fy*Y/Z^2 |   |0  c   d|
			*/
			const float a = fx / Z, b = -fx * X / (Z * Z), c = fy / Z, d = -fy * Y / (Z * Z);

			Point2f pt(q.x / q.z, q.y / q.z);

			Vec2f nx = pt - pm.imPoint;
			float dist = sqrt(nx.dot(nx)) + 1e-6f;
			nx *= 1.f / dist;

			Vec3f n_dq_dQ(nx[0] * a, nx[1] * c, nx[0] * b + nx[1] * d);

			auto dt = n_dq_dQ.t() * R;
			auto dR = pm.modelPoint.cross(Vec3f(dt.val[0], dt.val[1], dt.val[2]));

			Vec6f j(dt.val[0], dt.val[1], dt.val[2], dR.x, dR.y, dR.z);

			float w = pow(1.f / (dist + 1.f), 2.f - alpha) * wpm /** pm.weight*/;

			J += w * dist * j;

			JJ += w * j * j.t();
		}

		const float lambda = 5000.f * (npt + npm*ws) / 200.f;

		for (int i = 0; i < 3; ++i)
			JJ(i, i) += lambda * 100.f;

		for (int i = 3; i < 6; ++i)
			JJ(i, i) += lambda;

		int ec = 0;

		Vec6f p;// = -JJ.inv() * J;
		if (solve(JJ, -J, p))
		{
			cv::Vec3f dt(p[0], p[1], p[2]);
			cv::Vec3f rvec(p[3], p[4], p[5]);
			Matx33f dR;
			cv::Rodrigues(rvec, dR);

			pose.t = pose.R * dt + pose.t;
			pose.R = pose.R * dR;

			//pose.t = dR * pose.t + dt;
			//pose.R = dR * pose.R;

			float diff = p.dot(p);
			//printf("diff=%f\n", sqrt(diff));

			return diff < eps* eps ? 0 : 1;
		}

		return 0;
	}
	bool update(PoseData& pose, const Matx33f& K, const std::vector<CPoint>& cpoints, int maxItrs, float alpha, float eps)
	{
		for (int itr = 0; itr < maxItrs; ++itr)
			if (this->_update(pose, K, cpoints, alpha, eps) <= 0)
				return false;
		return true;
	}
};

class Refiner
{
public:
	struct ModelPoint
	{
		Point3f pt;
		float   w;
	};
	std::vector<ModelPoint> getModelPoints(CVRender& render, Pose& pose, const Matx33f& K, Size imgSize, Rect roi, int maxPoints=5000)
	{
		CVRMats mats;
		mats.mModel = cvrm::fromR33T(pose.R, pose.t);
		mats.mProjection = cvrm::fromK(K, imgSize, 0.1, 10);

		auto rr = render.exec(mats, imgSize, CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr, roi);

		Mat rgray = cv::convertBGRChannels(rr.img, 1);
		Mat1s dx, dy;
		cv::Sobel(rgray, dx, CV_16S, 1, 0);
		cv::Sobel(rgray, dy, CV_16S, 0, 1);
		Mat1b edges;
		cv::Canny(dx, dy, edges, 20, 20);
		
		int nedgePoint = 0;
		for_each_1(DWHN1(edges), [&nedgePoint](uchar m) {
			if (m != 0)
				++nedgePoint;
			});

		std::vector<ModelPoint> points;
		points.reserve(maxPoints);

		CVRProjector prj(rr, imgSize);

		int nstep = __max(1, nedgePoint / maxPoints);
		nedgePoint = 0;
		float wsum = 1e-6f;
		for_each_1c(DWHN1(edges), [&nedgePoint, nstep, &rr, &points, prj, roi, &dx, &dy, &wsum](uchar m, int x, int y) {
			if (m != 0)
			{
				if (++nedgePoint % nstep == 0)
				{
					float z = rr.depth(y, x);
					if (fabs(z - 1.0f) > 1e-6f)
					{
						ModelPoint t;
						t.pt = prj.unproject(float(x + roi.x), float(y + roi.y), z);
						t.w = abs(dx(y, x)) + abs(dy(y, x));
						points.push_back(t);
						wsum += t.w;
					}
				}
			}});
		for (auto& p : points)
			p.w /= wsum;
		return points;
	}
	Mat1f getImageField(Mat img, int nLayers=2)
	{
		img = cv::convertBGRChannels(img, 1);

		auto getF = [](const Mat1b& gray, Size dsize) {
			Mat1f dx, dy;
			cv::Sobel(gray, dx, CV_32F, 1, 0);
			cv::Sobel(gray, dy, CV_32F, 0, 1);
			for_each_2(DWHN1(dx), DN1(dy), [](float& dx, float dy) {
				dx = fabs(dx) + fabs(dy);
				});
			if (dx.size() != dsize)
				dx = imscale(dx, dsize, INTER_LINEAR);
			return dx;
		};

		Mat1f f=getF(img,img.size());
		for (int i = 1; i < nLayers; ++i)
		{
			img = imscale(img, 0.5);
			f += getF(img, f.size());
		}
		float vmax = cv::maxElem(f);
		f *= 1.f / vmax;
		return 1.f-f;
	}
	void operator()(CVRender& render, Pose& pose, const Matx33f& K, Mat img, Rect roi, int nItrs = 10, float eps=1e-6f)
	{
		//return;

		roi = rectOverlapped(roi, Rect(0, 0, img.cols, img.rows));
		if (roi.empty())
			return;

		std::vector<ModelPoint> modelPoints;
		std::thread t1([&modelPoints,this,&render,&pose,&K,&img,&roi]() {
			modelPoints = this->getModelPoints(render, pose, K, img.size(), roi);
			});

		Mat1f f = getImageField(img(roi));
		Mat1f dfx, dfy;
		cv::Sobel(f, dfx, CV_32F, 1, 0);
		cv::Sobel(f, dfy, CV_32F, 0, 1);

		t1.join();

		int itr = 0;
		for (; itr < nItrs; ++itr)
		{
			const Matx33f R = pose.R;
			const Point3f t = pose.t;

			const float fx = K(0, 0), fy = K(1, 1);

			auto* vcp = &modelPoints[0];
			int npt = (int)modelPoints.size();

			Matx66f JJ = Matx66f::zeros();
			Vec6f J(0, 0, 0, 0, 0, 0);
			float nptw = 0;

			for (int i = 0; i < npt; ++i)
			{
				Point3f Q = R * vcp[i].pt + t;
				Point3f q = K * Q;
				{
					const int x = int(q.x / q.z + 0.5), y = int(q.y / q.z + 0.5);
					if (uint(x - roi.x) >= uint(roi.width) || uint(y - roi.y) >= uint(roi.height))
						continue;

					const float X = Q.x, Y = Q.y, Z = Q.z;
					/*      |fx/Z   0   -fx*X/Z^2 |   |a  0   b|
					dq/dQ = |                     | = |        |
							|0    fy/Z  -fy*Y/Z^2 |   |0  c   d|
					*/
					const float a = fx / Z, b = -fx * X / (Z * Z), c = fy / Z, d = -fy * Y / (Z * Z);

					Point2f pt(q.x / q.z, q.y / q.z);

					int dy = y - roi.y, dx = x - roi.x;

					Vec2f nx(dfx(dy, dx), dfy(dy, dx));
					nx = normalize(nx);

					Vec3f n_dq_dQ(nx[0] * a, nx[1] * c, nx[0] * b + nx[1] * d);

					auto dt = n_dq_dQ.t() * R;
					auto dR = vcp[i].pt.cross(Vec3f(dt.val[0], dt.val[1], dt.val[2]));

					Vec6f j(dt.val[0], dt.val[1], dt.val[2], dR.x, dR.y, dR.z);

					float w = vcp[i].w;

					J += w * f(dy, dx) * j;

					JJ += w * j * j.t();
					nptw += w;
				}
			}

			const float lambda = 5000.f * nptw / 200.f;

			for (int i = 0; i < 3; ++i)
				JJ(i, i) += lambda * 100.f;

			for (int i = 3; i < 6; ++i)
				JJ(i, i) += lambda;

			int ec = 0;

			Vec6f p;// = -JJ.inv() * J;
			if (solve(JJ, -J, p))
			{
				cv::Vec3f dt(p[0], p[1], p[2]);
				cv::Vec3f rvec(p[3], p[4], p[5]);
				Matx33f dR;
				cv::Rodrigues(rvec, dR);

				pose.t = pose.R * dt + pose.t;
				pose.R = pose.R * dR;

				float diff = p.dot(p);
				if (diff < eps * eps)
					break;
			}
		}
		//printf("\nnItr=%d    ", itr);
	}
};




//struct Projector
//{
//	Matx33f _KR;
//	Vec3f _Kt;
//public:
//	Projector(const Matx33f& _K, const Matx33f& _R, const Vec3f& _t)
//		:_KR(_K* _R), _Kt(_K* _t)
//	{
//	}
//	Point2f operator()(const Point3f& P) const
//	{
//		Vec3f p = _KR * Vec3f(P) + _Kt;
//		return Point2f(p[0] / p[2], p[1] / p[2]);
//	}
//	template<typename _ValT, typename _getPointT>
//	std::vector<Point2f> operator()(const std::vector<_ValT>& vP, _getPointT getPoint) const
//	{
//		std::vector<Point2f> vp(vP.size());
//		for (int i = 0; i < (int)vP.size(); ++i)
//			vp[i] = (*this)(getPoint(vP[i]));
//		return vp;
//	}
//	template<typename _ValT>
//	std::vector<Point2f> operator()(const std::vector<_ValT>& vP)
//	{
//		return (*this)(vP, [](const _ValT& v) {return v; });
//	}
//};


struct Templates
{
	Point3f               modelCenter;
	std::vector<DView>   views;
	ViewIndex             viewIndex;
	
	DetectorModelData* _detectorModelData;
	float              _modelScale;
	CVRender*		   _render;

	DEFINE_BFS_IO_2(Templates, modelCenter, views)

public:
	void build(CVRModel& model);

	
	void loadExt(re3d::Model& model, CVRender *render, float modelScale)
	{
		_detectorModelData = model.getManaged<DetectorModelData>();
		_modelScale = modelScale;
		_render = render;
	}
	void showInfo()
	{
		int minSamplePoints = INT_MAX;
		for (auto& v : views)
			if (v.contourPoints3d.size() < minSamplePoints)
				minSamplePoints = v.contourPoints3d.size();
		printf("minSamplePoints=%d\n", minSamplePoints);
	}
public:
	Vec3f _getViewDir(const Matx33f& R, const Vec3f& t)
	{
		return normalize(-R.inv() * t - Vec3f(this->modelCenter));
	}
	int  _getNearestView(const Vec3f& viewDir)
	{
		//CV_Assert(fabs(viewDir.dot(viewDir) - 1.f) < 1e-3f);
		if (!(fabs(viewDir.dot(viewDir) - 1.f) < 1e-3f))
		{
			printf("error: invalid view dir=(%.2f,%.2f,%.2f)\n", viewDir[0], viewDir[1], viewDir[2]);
			return 0;
		}
#if 0
		int im = -1;
		float vcosMax = -1;
		for (int i = 0; i < (int)views.size(); ++i)
		{
			float vcos = viewDir.dot(views[i].viewDir);
			if (vcos > vcosMax)
			{
				vcosMax = vcos;
				im = i;
			}
		}
		return im;
#else
		return viewIndex.getViewInDir(viewDir);
#endif
	}
	int  _getNearestView(const Matx33f& R, const Vec3f& t)
	{
		return _getNearestView(this->_getViewDir(R, t));
	}

	Rect getCurROI(const Matx33f &K, const Pose &pose, int *_curView=nullptr)
	{
		Rect curROI;
		int curView = this->_getNearestView(pose.R, pose.t);
		{
			Projector prj(K, pose.R, pose.t);
			std::vector<Point2f>  c2d = prj(views[curView].contourPoints3d, [](const CPoint& p) {return p.center; });
			Rect_<float> rectf = getBoundingBox2D(c2d);
			curROI = Rect(rectf);
		}
		if (_curView)
			*_curView = curView;
		return curROI;
	}

	float pro(const Mat &img, const Matx33f& K, Pose& pose, const Mat1f& curProb, float thetaT, float errT);

	void getProjectedContours(const Matx33f& K, const Pose& pose, std::vector<Point2f>& points, std::vector<Point2f>* normals)
	{
		int curView = this->_getNearestView(pose.R, pose.t);

		Projector prj(K, pose.R, pose.t);
		points = prj(views[curView].contourPoints3d, [](const CPoint& p) {return p.center; });
		if (normals)
		{
			*normals = prj(views[curView].contourPoints3d, [](const CPoint& p) {return p.normal; });
			for (int i = 0; i < (int)points.size(); ++i)
			{
				auto n = (*normals)[i] - points[i];
				(*normals)[i] = normalize(Vec2f(n));
			}
		}
	}
#if 0
	void _getModelPoints(const Matx33f& K, Pose pose, const Mat &img, Rect roi, std::vector<Point3f>& points, Mat& desc, Ptr<ORB> &fd)
	{
		CVRMats mats;
		mats.mModel = cvrm::fromR33T(pose.R, pose.t*_modelScale);
		mats.mProjection = cvrm::fromK(K, img.size(), 0.1, 30);

		time_t beg = clock();
		auto rr = _render->exec(mats, img.size(), CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr, roi);
		printf("\nfd-time=%d   \n", int(clock() - beg));

		//imshow("rr.img", rr.img);
		//cv::waitKey();

		std::vector<KeyPoint> kp;

		
		//auto fdx = cv::AKAZE::create(5, 0, 3, 0.001, 4, 2);
		fd->detectAndCompute(rr.img, Mat(), kp, desc);
		

		//fd->detectAndCompute(rr.img, Mat(), kp, desc);

		
		
		points.clear();
		points.reserve(kp.size());
		CVRProjector prj(rr, img.size());
		for (auto& p : kp)
		{
			points.push_back(prj.unproject(p.pt+Point2f(roi.x,roi.y))/_modelScale);
		}
	}

	std::vector<PointMatch> calcPointMatches(const Matx33f &K, Pose pose, const Mat &img, Rect roi, Size objSize, float spaceDistT=20.f)
	{
		pose.t *= 1.f / _modelScale;

		auto mT = cvrm::fromR33T(pose.R, pose.t);

		auto fd = cv::ORB::create(200, 1.25, 2);
		//auto fd = cv::AKAZE::create(5, 0, 3, 0.001, 4, 2);

		std::vector<Point3f>  selPoints;
		Mat selDesc;
		//_detectorModelData->modelPoints.getSubsetWithView(mT, true, objSize, selPoints, selDesc, 1);
		this->_getModelPoints(K, pose, img, roi, selPoints, selDesc, fd);
		printf("nsel=%d\n", selPoints.size());

		roi = rectOverlapped(roi, Rect(0, 0, img.cols, img.rows));
		if (roi.empty()||selPoints.empty())
			return std::vector<PointMatch>();

		Mat gray = cv::convertBGRChannels(img(roi), 1);
		std::vector<KeyPoint> imKp;
		Mat imDesc;
		fd->detectAndCompute(gray, Mat(), imKp, imDesc);

		if (imKp.empty())
			return std::vector<PointMatch>();

		Mat dimg;
		cv::drawKeypoints(img(roi), imKp, dimg);
		imshow("kpimg", dimg);
		//cv::waitKey();

		std::vector<DMatch> matches;

		cv::BFMatcher matcher(NORM_HAMMING, false);
		matcher.match(imDesc, selDesc, matches);
		//matcher.knnMatch

		/*FMIndex knn;
		knn.build(selDesc, FMIndex::LSH);
		knn.match(imDesc, matches);*/

		//std::sort(matches.begin(), matches.end());
		//matches.resize(matches.size() / 2);
		auto minDistance=std::min_element(matches.begin(), matches.end())->distance;
		auto distT = minDistance * 2.f;

		Projector prj(K, pose.R, pose.t);
		Point2f roiOffset(float(roi.x), float(roi.y));
		spaceDistT *= spaceDistT;

		std::vector<PointMatch> pm;
		std::vector<Point2f>    dv;
		pm.reserve(matches.size());
		dv.reserve(matches.size());

		Point2f dmean(0.f, 0.f);

		for (auto& m : matches)
		{
			if (m.distance > distT || uint(m.trainIdx)>selPoints.size())
				continue;

			Point2f q = roiOffset + imKp[m.queryIdx].pt;
			Point2f d = prj(selPoints[m.trainIdx]) - q;
			if (d.dot(d) > spaceDistT)
				continue;

			PointMatch tm;
			tm.modelPoint = selPoints[m.trainIdx] * _modelScale;
			tm.imPoint = q;

			pm.push_back(tm);
			dv.push_back(d);
			dmean += d;
		}
		if (!pm.empty())
		{
			dmean *= 1.f / float(dv.size());
			float dsum = 0;
			for (size_t i = 0; i < pm.size(); ++i)
			{
				Point2f ddv = dv[i] - dmean;
				float d = ddv.dot(ddv);
				pm[i].weight = exp(-d / 4.f);
				dsum += sqrt(ddv.dot(ddv));
			}
			dsum /= pm.size();

			//printf("\ndsum=%.2f np=%d\n", dsum, pm.size());
		}

		return pm;
	}
#endif
	void _detectAndCompute(const Mat& img, std::vector<KeyPoint>& kp, Mat& desc)
	{
		Mat gray = cv::convertBGRChannels(img, 1);
		std::vector<Point2f> corners;
		cv::goodFeaturesToTrack(gray, corners, 200, 0.01, 5);

		kp.resize(corners.size());
		for (size_t i = 0; i < kp.size(); ++i)
			kp[i].pt = corners[i];
		
		auto fdx = cv::xfeatures2d::DAISY::create();
		fdx->compute(img, kp, desc);
	}

	void _detectAndCompute2(const Mat& img, std::vector<KeyPoint>& kp, Mat& desc)
	{
		Mat gray = cv::convertBGRChannels(img, 1);
		
		bool hasLargeRotation = false;
		auto fd=cv::AKAZE::create(hasLargeRotation ? AKAZE::DESCRIPTOR_MLDB : AKAZE::DESCRIPTOR_MLDB_UPRIGHT,
			0, 3, 0.001, 4, 4, KAZE::DIFF_PM_G2
		);

		//std::vector<Point2f> corners;
		//cv::goodFeaturesToTrack(gray, corners, 200, 0.01, 5);

		//kp.resize(corners.size());
		//for (size_t i = 0; i < kp.size(); ++i)
		//	kp[i].pt = corners[i];

		//auto fdx = cv::xfeatures2d::DAISY::create();
		//fdx->compute(img, kp, desc);
		fd->detectAndCompute(gray, noArray(), kp, desc);
	}

	struct AuxData
	{
	public:
		Mat  src, tar;
		std::vector<Point2f>  srcPoints, tarPoints;
	};
	void showMatches(const AuxData& data)
	{
		if (data.src.empty() || data.tar.empty())
			return;

		Mat dimg=visMatch(cvtPoint(data.srcPoints), cvtPoint(data.tarPoints), data.src, data.tar);
		imshow("matches", dimg);
	}

	void _getModelPoints(const Matx33f& K, Pose pose, const Mat& img, Rect roi, std::vector<Point3f>& points, Mat& desc, AuxData &auxData)
	{
		CVRMats mats;
		mats.mModel = cvrm::fromR33T(pose.R, pose.t * _modelScale);
		mats.mProjection = cvrm::fromK(K, img.size(), 0.1, 10);

		auto rr = _render->exec(mats, img.size(), CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr, roi);
		
		std::vector<KeyPoint> kp;

		this->_detectAndCompute(rr.img, kp, desc);

		points.clear();
		points.reserve(kp.size());
		CVRProjector prj(rr, img.size());
		for (auto& p : kp)
		{
			points.push_back(prj.unproject(p.pt + Point2f(roi.x, roi.y)) / _modelScale);
			auxData.srcPoints.push_back(p.pt);
		}
		auxData.src = rr.img;
	}
	std::vector<PointMatch> calcPointMatches(const Matx33f& K, Pose pose, const Mat& img, Rect roi, Size objSize, AuxData &auxData, float spaceDistT = 20.f)
	{
		roi = rectOverlapped(roi, Rect(0, 0, img.cols, img.rows));
		if (roi.empty())
			return std::vector<PointMatch>();


		pose.t *= 1.f / _modelScale;

		std::vector<Point3f>  selPoints;
		Mat selDesc;
		//AuxData auxData;

		std::thread t1(
			[this, &K,&pose,&img,&roi,&selPoints,&selDesc, &auxData]() {
				this->_getModelPoints(K, pose, img, roi, selPoints, selDesc, auxData);
			});

		std::vector<KeyPoint> imKp;
		Mat imDesc;
		this->_detectAndCompute(img(roi), imKp, imDesc);

		t1.join();

		if (selPoints.size()<10 || imKp.size()<10||selPoints.size()<10)
			return std::vector<PointMatch>();

		auto matcher = cv::DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
		//cv::BFMatcher matcher(imDesc.depth() == CV_8UC1 ? NORM_HAMMING : NORM_L2);

		std::vector<DMatch> matches;
#if 1
		std::vector<std::vector<DMatch>> knnMatches;
		matcher->knnMatch(imDesc, selDesc, knnMatches, 2);

		for (auto& km : knnMatches)
		{
			if (km[0].distance * 1.1 < km[1].distance)
				matches.push_back(km[0]);
		}
#else
		std::vector<DMatch> knnMatches, knnMatchesB;
		matcher.match(imDesc, selDesc, knnMatches);
		/*matcher->add(selDesc);
		matcher->knnMatch(imDesc, knnMatches, 2);*/
		matcher.match(selDesc, imDesc, knnMatchesB);
		for (auto& km : knnMatches)
		{
			if (knnMatchesB[km.trainIdx].trainIdx==km.queryIdx)
				matches.push_back(km);
		}
#endif
		Projector prj(K, pose.R, pose.t);
		Point2f roiOffset(float(roi.x), float(roi.y));
		spaceDistT *= spaceDistT;

		std::vector<PointMatch> pm;
		pm.reserve(matches.size());

		int n = 0;
		for (auto& m : matches)
		{
			if (/*m.distance > distT ||*/ uint(m.trainIdx) > selPoints.size())
				continue;

			Point2f q = roiOffset + imKp[m.queryIdx].pt;
			Point2f d = prj(selPoints[m.trainIdx]) - q;
			if (d.dot(d) > spaceDistT)
				continue;

			PointMatch tm;
			tm.modelPoint = selPoints[m.trainIdx] * _modelScale;
			tm.imPoint = q;

			pm.push_back(tm);

			auxData.srcPoints[n] = auxData.srcPoints[m.trainIdx];
			auxData.tarPoints.push_back(imKp[m.queryIdx].pt);
			++n;
		}
		auxData.tar = img(roi);
		auxData.srcPoints.resize(n);

		//showMatches(auxData);

		//printf("\ndsum=%.2f np=%d\n", 0.f, pm.size());

		return pm;
	}

	std::vector<PointMatch> calcPointMatches2(const Matx33f& K, Pose pose, const Mat& img, Rect roi, Size objSize, AuxData& auxData, float spaceDistT = 20.f)
	{
		roi = rectOverlapped(roi, Rect(0, 0, img.cols, img.rows));
		if (roi.empty()||__max(roi.width,roi.height)<100)
			return std::vector<PointMatch>();

		CVRMats mats;
		mats.mModel = cvrm::fromR33T(pose.R, pose.t);
		mats.mProjection = cvrm::fromK(K, img.size(), 0.1, 10);

		auto rr = _render->exec(mats, img.size(), CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr, roi);

		Mat src, tar;
		cvtColor(rr.img, src, CV_BGR2GRAY);
		cvtColor(img(roi), tar, CV_BGR2GRAY);

		//auto flowPtr = cv::FarnebackOpticalFlow::create();
		auto flowPtr = cv::optflow::createOptFlow_DIS(cv::optflow::DISOpticalFlow::PRESET_MEDIUM);
		Mat2f flowF, flowR;
		flowPtr->calc(src, tar, flowF);
		flowPtr->calc(tar, src, flowR);

		std::vector<Point2f> corners;
		cv::goodFeaturesToTrack(src, corners, 200, 0.01, 5);
		int n = 0;
		for (auto& p : corners)
		{
			int x = int(p.x + 0.5), y = int(p.y + 0.5);
			const float* f = flowF.ptr<float>(y, x);
			int tx = int(x + f[0] + 0.5f), ty = int(y + f[1] + 0.5f);
			if (uint(tx) < uint(flowR.cols) && uint(ty) < uint(flowR.rows))
			{
				const float* tf = flowR.ptr<float>(ty, tx);
				const float dx = f[0] + tf[0], dy = f[1] + tf[1];
				float err = dx * dx + dy * dy;
				if (err < 1.5f)
				{
					corners[n++] = p;
				}
			}
		}
		corners.resize(n);

		std::vector<PointMatch> matches;
		
		CVRProjector prj(rr, img.size());
		for (auto& p : corners)
		{
			PointMatch pm;
			pm.modelPoint=prj.unproject(p + Point2f(roi.x, roi.y));
			Point2f q = p + flowF.at<Point2f>(int(p.y + 0.5f), int(p.x + 0.5f));
			pm.imPoint = Point2f(roi.x, roi.y) + q;
			pm.weight = 1.f;
			matches.push_back(pm);
			auxData.tarPoints.push_back(q);
		}
		
		auxData.src = rr.img;
		auxData.tar = img(roi);
		auxData.srcPoints = corners;
		
		return matches;
	}
	//std::vector<PointMatch> calcPointMatches1(const Matx33f& K, Pose pose, const Mat& img, Rect roi, Size objSize, float spaceDistT = 20.f)
	//{
	//	pose.t *= 1.f / _modelScale;

	//	auto mT = cvrm::fromR33T(pose.R, pose.t);

	//	//auto fd = cv::ORB::create(200, 1.25, 2);
	//	//auto fd = cv::AKAZE::create(5, 0, 3, 0.001, 4, 2);

	//	std::vector<Point3f>  selPoints;
	//	Mat selDesc;
	//	//_detectorModelData->modelPoints.getSubsetWithView(mT, true, objSize, selPoints, selDesc, 1);
	//	this->_getModelPoints(K, pose, img, roi, selPoints, selDesc);
	//	printf("nsel=%d\n", selPoints.size());

	//	roi = rectOverlapped(roi, Rect(0, 0, img.cols, img.rows));
	//	if (roi.empty() || selPoints.empty())
	//		return std::vector<PointMatch>();

	//	Mat gray = cv::convertBGRChannels(img(roi), 1);
	//	std::vector<KeyPoint> imKp;
	//	Mat imDesc;
	//	//fd->detectAndCompute(gray, Mat(), imKp, imDesc);
	//	this->_detectAndCompute(img(roi), imKp, imDesc);

	//	if (imKp.empty())
	//		return std::vector<PointMatch>();

	//	/*Mat dimg;
	//	cv::drawKeypoints(img(roi), imKp, dimg);
	//	imshow("kpimg", dimg);
	//	cv::waitKey();*/

	//	std::vector<DMatch> matches;

	//	//cv::BFMatcher matcher(NORM_HAMMING, false);
	//	//cv::BFMatcher matcher(NORM_L2, false);
	//	//matcher.match(imDesc, selDesc, matches);
	//	//matcher.knnMatch

	//	/*FMIndex knn;
	//	knn.build(selDesc, FMIndex::LINEAR);
	//	knn.match(imDesc, matches);*/
	//	auto matcher = cv::DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
	//	matcher->match(imDesc, selDesc, matches);

	//	//std::sort(matches.begin(), matches.end());
	//	//matches.resize(matches.size() / 2);
	//	auto minDistance = std::min_element(matches.begin(), matches.end())->distance;
	//	auto distT = minDistance * 4.f;

	//	Projector prj(K, pose.R, pose.t);
	//	Point2f roiOffset(float(roi.x), float(roi.y));
	//	spaceDistT *= spaceDistT;

	//	std::vector<PointMatch> pm;
	//	std::vector<Point2f>    dv;
	//	pm.reserve(matches.size());
	//	dv.reserve(matches.size());

	//	Point2f dmean(0.f, 0.f);

	//	for (auto& m : matches)
	//	{
	//		if (m.distance > distT || uint(m.trainIdx) > selPoints.size())
	//			continue;

	//		Point2f q = roiOffset + imKp[m.queryIdx].pt;
	//		Point2f d = prj(selPoints[m.trainIdx]) - q;
	//		if (d.dot(d) > spaceDistT)
	//			continue;

	//		PointMatch tm;
	//		tm.modelPoint = selPoints[m.trainIdx] * _modelScale;
	//		tm.imPoint = q;

	//		pm.push_back(tm);
	//		dv.push_back(d);
	//		dmean += d;
	//	}
	//	if (!pm.empty())
	//	{
	//		dmean *= 1.f / float(dv.size());
	//		float dsum = 0;
	//		for (size_t i = 0; i < pm.size(); ++i)
	//		{
	//			Point2f ddv = dv[i] - dmean;
	//			float d = ddv.dot(ddv);
	//			pm[i].weight = exp(-d / 4.f);
	//			dsum += sqrt(ddv.dot(ddv));
	//		}
	//		dsum /= pm.size();

	//		printf("\ndsum=%.2f np=%d\n", dsum, pm.size());
	//	}

	//	return pm;
	//}
};


class RegionTrajectory
{
	Mat1b  _pathMask;
	float  _delta;

	Point2f _uv2Pt(const Point2f& uv)
	{
		return Point2f(uv.x / _delta + float(_pathMask.cols) / 2.f, uv.y / _delta + float(_pathMask.rows) / 2.f);
	}
public:
	RegionTrajectory(Size regionSize, float delta)
	{
		_pathMask = Mat1b::zeros(regionSize);
		_delta = delta;
	}
	bool  addStep(Point2f start, Point2f end)
	{
		start = _uv2Pt(start);
		end = _uv2Pt(end);

		auto dv = end - start;
		float len = sqrt(dv.dot(dv)) + 1e-6f;
		float dx = dv.x / len, dy = dv.y / len;
		const int  N = int(len) + 1;
		Point2f p = start;
		for (int i = 0; i < N; ++i)
		{
			int x = int(p.x + 0.5), y = int(p.y + 0.5);
			if (uint(x) < uint(_pathMask.cols) && uint(y) < uint(_pathMask.rows))
			{
				if (_pathMask(y, x) != 0 && len > 1.f)
					return true;
				_pathMask(y, x) = 1;
			}
			else
				return false;

			p.x += dx; p.y += dy;
		}

		//imshow("pathMask", imscale(_pathMask, 3.f));
		//cv::waitKey();

		return false;
	}
};

std::thread::id  theMainThreadID = std::this_thread::get_id();

inline bool isMainThread()
{
	return theMainThreadID == std::this_thread::get_id();
}


inline float Templates::pro(const Mat &img, const Matx33f& K, Pose& pose, const Mat1f& curProb, float thetaT, float errT)
{
	int curView;
	Rect curROI = this->getCurROI(K, pose, &curView);

	Optimizer dfr;
	AuxData auxData;

	std::thread t1(
		[this, &K, &pose, &img, curROI, &dfr, &auxData]() {
			dfr._pointMatches = this->calcPointMatches(K, pose, img, curROI, curROI.size(), auxData);
		});

	Rect roi = curROI;
	const int dW = 100;
	rectAppend(roi, dW, dW, dW, dW);
	roi = rectOverlapped(roi, Rect(0, 0, curProb.cols, curProb.rows));
	dfr.computeScanLines(curProb, roi);

	//time_t beg = clock();
	t1.join();
	//printf("\rspeed=%.1ffps       ", 1000.0f / int(clock() - beg));

	//if(isMainThread())
	//	this->showMatches(auxData);

	Optimizer::PoseData dpose;
	static_cast<Pose&>(dpose) = pose;

	const float alpha = 0.125f, alphaNonLocal = 0.75f, eps = 1e-4f;
	const int outerItrs = 10, innerItrs = 3;

	for (int itr = 0; itr < outerItrs; ++itr)
	{
		curView = this->_getNearestView(dpose.R, dpose.t);
		if (!dfr.update(dpose, K, this->views[curView].contourPoints3d, innerItrs, alpha, eps))
			break;
	}

	float errMin = dfr.calcError(dpose, K, this->views[curView].contourPoints3d, alpha);
	if (errMin > errT)
	{
		const auto R0 = dpose.R;

		const int N = int(thetaT / (CV_PI / 12) + 0.5f) | 1;

		const float dc = thetaT / N;

		const int subDiv = 3;
		const int subRegionSize = (N * 2 * 2 + 1) * subDiv;
		RegionTrajectory traj(Size(subRegionSize, subRegionSize), dc / subDiv);

		Mat1b label = Mat1b::zeros(2 * N + 1, 2 * N + 1);
		struct DSeed
		{
			Point coord;
			//bool  isLocalMinima;
		};
		std::deque<DSeed>  seeds;
		seeds.push_back({ Point(N,N)/*,true*/ });
		label(N, N) = 1;


		auto checkAdd = [&seeds, &label](const DSeed& curSeed, int dx, int dy) {
			int x = curSeed.coord.x + dx, y = curSeed.coord.y + dy;
			if (uint(x) < uint(label.cols) && uint(y) < uint(label.rows))
			{
				if (label(y, x) == 0)
				{
					label(y, x) = 1;
					seeds.push_back({ Point(x, y)/*, false*/ });
				}
			}
		};

		while (!seeds.empty())
		{
			auto curSeed = seeds.front();
			seeds.pop_front();

			checkAdd(curSeed, 0, -1);
			checkAdd(curSeed, -1, 0);
			checkAdd(curSeed, 1, 0);
			checkAdd(curSeed, 0, 1);

			auto dR = theta2OutofplaneRotation(float(curSeed.coord.x - N) * dc, float(curSeed.coord.y - N) * dc);

			auto dposex = dpose;
			dposex.R = dR * R0;

			Point2f start = dir2Theta(viewDirFromR(dposex.R * R0.t()));
			for (int itr = 0; itr < outerItrs * innerItrs; ++itr)
			{
				if (itr % innerItrs == 0)
					curView = this->_getNearestView(dposex.R, dposex.t);

				if (!dfr.update(dposex, K, this->views[curView].contourPoints3d, 1, alphaNonLocal, eps))
					break;

				Point2f end = dir2Theta(viewDirFromR(dposex.R * R0.t()));
				if (traj.addStep(start, end))
					break;
				start = end;
			}
			{
				curView = this->_getNearestView(dposex.R, dposex.t);
				float err = dfr.calcError(dposex, K, this->views[curView].contourPoints3d, alpha);
				if (err < errMin)
				{
					errMin = err;
					dpose = dposex;
				}
				if (errMin < errT)
					break;
			}
		}
	}

	for (int itr = 0; itr < outerItrs; ++itr)
	{
		curView = this->_getNearestView(dpose.R, dpose.t);
		if (!dfr.update(dpose, K, this->views[curView].contourPoints3d, innerItrs, alpha, eps))
			break;
	}

	pose = dpose;

	Refiner refine;
	curROI = this->getCurROI(K, pose);
	refine(*_render, pose, K, img, curROI);

	return errMin;
}


inline void Templates::build(CVRModel& model)
{
	std::vector<Vec3f>  viewDirs;
	cvrm::sampleSphere(viewDirs, 3000);

	auto center = model.getCenter();
	auto sizeBB = model.getSizeBB();
	float maxBBSize = __max(sizeBB[0], __max(sizeBB[1], sizeBB[2]));
	float eyeDist = 0.8f;
	float fscale = 2.5f;
	Size  viewSize(2000, 2000);

	this->modelCenter = center;

	std::vector<DView> dviews;
	dviews.reserve(viewDirs.size());

	CVRender render(model);

	auto vertices = model.getVertices();

	int vi = 1;
	for (auto& viewDir : viewDirs)
	{
		printf("build templates %d/%d    \r", vi++, (int)viewDirs.size());
		{
			auto eyePos = center + viewDir * eyeDist;

			CVRMats mats;
			mats.mModel = cvrm::lookat(eyePos[0], eyePos[1], eyePos[2], center[0], center[1], center[2], 0.1f, 1.1f, 0.1f);
			mats.mProjection = cvrm::perspective(viewSize.height * fscale, viewSize, __max(0.01, eyeDist - maxBBSize), eyeDist + maxBBSize);

			auto rr = render.exec(mats, viewSize);
			Mat1b fgMask = getRenderMask(rr.depth);

			{
				auto roi = get_mask_roi(DWHS(fgMask), 127);
				int bwx = __min(roi.x, fgMask.cols - roi.x - roi.width);
				int bwy = __min(roi.y, fgMask.rows - roi.y - roi.height);
				if (__min(bwx, bwy) < 5/* || __max(roi.width,roi.height)<fgMask.cols/4*/)
				{
					imshow("mask", fgMask);
					cv::waitKey();
				}
			}

			dviews.push_back(DView());
			DView& dv = dviews.back();

			dv.viewDir = viewDir;

			Vec3f t;
			cvrm::decomposeRT(mats.mModel, dv.R, t);

			EdgeSampler::sample(dv.contourPoints3d, rr, 200);
		}
	}
	this->views.swap(dviews);
	this->viewIndex.build(this->views);
}


struct Frame
{
	Mat3b    img;
	Mat1b    objMask;
	Mat1f    colorProb;
	Pose	 pose;
	float    err;
	bool     tracked = false;
};

class Object
{
public:
	std::string modelFile;
	float       modelScale;
	Templates   templ;
private:
	CVRModel  model;
	CVRender  render;
public:
	void loadModel(StreamPtr streamPtr, re3d::Model &model, float modelScale, bool forceRebuild = false)
	{
		this->modelFile = model.get3DModel().getFile();
		this->modelScale = modelScale;

		std::string versionCode = model.getInfos().versionCode;

		if (!forceRebuild && streamPtr->HeadMatched(versionCode))
		{
			(*streamPtr) >> versionCode >> templ;
			templ.viewIndex.build(templ.views);
		}
		else
		{
			templ.build(this->get3DModel());
			(*streamPtr) << versionCode << templ;
		}
		templ.loadExt(model, &this->getRender(), modelScale);
	}
	CVRModel& get3DModel()
	{
		if (model.empty())
		{
			model.load(modelFile, 0);
			model.transform(cvrm::scale(modelScale, modelScale, modelScale));
		}
		return model;
	}
	CVRender& getRender()
	{
		if (render.empty())
			render = CVRender(this->get3DModel());
		return render;
	}
};

inline Mat1b renderObjMask(Object& obj, const Pose& pose, const Matx44f& mProj, Size dsize)
{
	CVRMats mats;
	mats.mModel = cvrm::fromR33T(pose.R, pose.t);
	mats.mProjection = mProj;

	CVRResult rr = obj.getRender().exec(mats, dsize, CVRM_DEPTH | CVRM_IMAGE);
	//imshow("rr", rr.img);

	return getRenderMask(rr.depth);
}

class ColorHistogram
{
	static const int CLR_RSB = 3;
	static const int TAB_WIDTH = (1 << (8 - CLR_RSB)); //+2, border
	static const int TAB_WIDTH_2 = TAB_WIDTH * TAB_WIDTH;
	static const int TAB_SIZE = TAB_WIDTH * TAB_WIDTH_2;

	static int _color_index(const uchar* pix)
	{
		return (int(pix[0]) >> CLR_RSB) * TAB_WIDTH_2 + (int(pix[1]) >> CLR_RSB) * TAB_WIDTH + (int(pix[2]) >> CLR_RSB);
	}

	struct TabItem
	{
		float nbf[2];
	};

	int _consideredLength = 20, _unconsiderLength = 1;
	std::vector<TabItem>  _tab, _dtab;

	static void _do_update(TabItem* tab, const TabItem* dtab, float learningRate, float dtabSum[2])
	{
		float tscale = 1.f - learningRate;
		float dscale[] = { learningRate / dtabSum[0], learningRate / dtabSum[1] };

		for (int i = 0; i < TAB_SIZE; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				tab[i].nbf[j] = tab[i].nbf[j] * tscale + dtab[i].nbf[j] * dscale[j];
			}
		}
	}

public:
	ColorHistogram()
	{
		_tab.resize(TAB_SIZE);
		_dtab.resize(TAB_SIZE);
	}
	void update(Object& obj, const Mat3b& img, const Pose& pose, const Matx33f& K, float learningRate)
	{
		//learningRate = 0.1f;

		auto& templ = obj.templ;
		auto modelCenter = templ.modelCenter;

		Projector prj(K, pose.R, pose.t);

		TabItem* dtab = &_dtab[0];
		memset(dtab, 0, sizeof(dtab[0]) * _dtab.size());
		float dtabSum[2] = { 0.f,0.f };
		auto addPixel = [&img, dtab, &dtabSum](const Point2f& p, const int fg_bg_i) {
			int x = int(p.x + 0.5f), y = int(p.y + 0.5f);
			if (uint(x) < uint(img.cols) && uint(y) < uint(img.rows))
			{
				const uchar* p = img.ptr(y, x);
				dtab[_color_index(p)].nbf[fg_bg_i] += 1.f;
				dtabSum[fg_bg_i] += 1.f;
				return true;
			}
			return false;
		};

		int curView = templ._getNearestView(pose.R, pose.t);
		if (uint(curView) < templ.views.size())
		{
			Point2f objCenter = prj(modelCenter);

			auto& view = templ.views[curView];
			for (auto& cp : view.contourPoints3d)
			{
				Point2f c = prj(cp.center);
				Point2f n = objCenter - c;
				const float fgLength = sqrt(n.dot(n));
				n = n * (1.f / fgLength);

				Point2f pt = c + float(_unconsiderLength) * n;
				int end = __min(_consideredLength, int(fgLength));
				for (int i = _unconsiderLength; i < end; ++i, pt += n)
				{
					if (!addPixel(pt, 1))
						break;
				}
				end = _consideredLength * 4;
				pt = c - float(_unconsiderLength) * n;
				for (int i = _unconsiderLength; i < end; ++i, pt -= n)
				{
					if (!addPixel(pt, 0))
						break;
				}
			}
			_do_update(&_tab[0], dtab, learningRate, dtabSum);
		}
	}
	Mat1f getProb(const Mat3b& img)
	{
		Mat1f prob(img.size());
		const TabItem* tab = &_tab[0];
		for_each_2(DWHNC(img), DN1(prob), [this, tab](const uchar* c, float& p) {
			int ti = _color_index(c);
			auto& nbf = tab[ti].nbf;
			p = (nbf[1] + 1e-6f) / (nbf[0] + nbf[1] + 2e-6f);
			});
		//imshow("prob", prob);
		//cv::waitKey();
		return prob;
	}
};


inline float getRDiff(const cv::Matx33f& R1, const cv::Matx33f& R2)
{
	cv::Matx33f tmp = R1.t() * R2;
	float cmin = __min(__min(tmp(0, 0), tmp(1, 1)), tmp(2, 2));
	float r=acos(cmin);
	return isnan(r) ? 0.f : r;
}

class BaseTracker1
{
	float       _modelScale = 0.001f;
	float       _histogramLearningRate = 0.2f;
	cv::Matx33f _K;
	cv::Matx44f _mProj;
	Frame       _prev;
	Frame       _cur;
	Object  _obj;
	ColorHistogram _colorHistogram;
	bool    _isLocalTracking = false;
	bool    _useInnerSeg = false;
	//DetectorModelData* _detectorModelData;

	struct FrameInfo
	{
		float   theta;
		float   err;
	};

	std::deque<FrameInfo>  _frameInfo;

	Pose _scalePose(const Pose& pose) const
	{
		return { pose.R, pose.t * _modelScale };
	}
	Pose _descalePose(const Pose& pose) const
	{
		return { pose.R, pose.t * (1.f / _modelScale) };
	}
public:
	void loadModel(re3d::Model &model, const std::string& argstr)
	{
		auto streamPtr = model.getData().getStream("v1.BaseTracker",true);
		_obj.loadModel(streamPtr, model, _modelScale);

		ff::CommandArgSet args(argstr);
		_isLocalTracking = false;// args.getd<bool>("local", false);
		_useInnerSeg = true;// args.getd<bool>("useInnserSeg", false);
		_cur.tracked = false;
	}
	void reset(const Mat& img, const Pose& pose, const Matx33f& K)
	{
		_cur.img = img.clone();
		_cur.pose = _scalePose(pose);
		_cur.objMask = Mat1b();
		_mProj = cvrm::fromK(K, img.size(), 0.1, 3);
		_K = K;

		if (_useInnerSeg)
			_colorHistogram.update(_obj, _cur.img, _cur.pose, _K, 1.f);

		_cur.tracked = true;
	}
	void startUpdate(const Mat& img, const Mat1f& segMask, Pose gtPose = Pose())
	{
		if (!_prev.img.empty())
		{
			float	theta = getRDiff(_prev.pose.R, _cur.pose.R);
			_frameInfo.push_back({ theta, _cur.err });
			while (_frameInfo.size() > 30)
				_frameInfo.pop_front();
		}

		_prev = _cur;
		_cur.img = img;

		if (_useInnerSeg)
			_cur.colorProb = _colorHistogram.getProb(_cur.img);
		else
			_cur.colorProb = segMask;

		//imshow("prob", _cur.colorProb);
		//imwrite("f:/vis/tmp/prob.png", vis(_cur.colorProb));
		//this->_mask.convertTo(_cur.colorProb, CV_32F, 1.f / 255);
	}

	template<typename _GetValT>
	static float _getMedianOfLastN(const std::deque<FrameInfo>& frameInfo, int nMax, _GetValT getVal)
	{
		int n = __min((int)frameInfo.size(), nMax);
		std::vector<float> tmp(n);
		int i = 0;
		for (auto itr = frameInfo.rbegin(); i < n; ++itr)
		{
			tmp[i++] = getVal(*itr);
		}
		std::sort(tmp.begin(), tmp.end());
		return tmp[n / 2];
	}

	float update(Pose& pose)
	{
		_cur.pose = _scalePose(pose);

		float thetaT = CV_PI / 8, errT = 1.f; //default values used for only the first 2 frames

		if (!_frameInfo.empty())
		{//estimate from previous frames
			thetaT = _getMedianOfLastN(_frameInfo, 5, [](const FrameInfo& v) {return v.theta; });
			errT = _getMedianOfLastN(_frameInfo, 15, [](const FrameInfo& v) {return v.err; });
		}

		_cur.err = _obj.templ.pro(_cur.img, _K, _cur.pose, _cur.colorProb, thetaT, _isLocalTracking ? FLT_MAX : errT);
		pose = _descalePose(_cur.pose);
		return _cur.err;
	}

	void endUpdate(const Pose& pose)
	{
		_cur.pose = _scalePose(pose);
		if (_useInnerSeg)
			_colorHistogram.update(_obj, _cur.img, _cur.pose, _K, _histogramLearningRate);
	}

	bool track(Mat tar, Pose& tarPose, Matx33f K)
	{
		if (!_cur.tracked)
			return false;

		//_cur.pose.t[0] = sqrt(-1);

		auto roi=_obj.templ.getCurROI(K, _cur.pose);
		double scale = sqrt(200*200 / (double(roi.width) * roi.height) );
		if (scale < 1.0)
		{
			scale = __max(scale, 0.25f);
			tar = imscale(tar, scale, INTER_LINEAR);
			K = cvrm::scaleK(K, scale);
		}

		_K = K;

		tarPose = _descalePose(_cur.pose);
		this->startUpdate(tar, Mat());
		this->update(tarPose);
		this->endUpdate(tarPose);
		return true;
	}

	float track(const Mat& src, const Pose& srcPose, const Mat& tar, Pose& tarPose, const Matx33f& K)
	{
		//reset
		auto pose = _scalePose(srcPose);

		ColorHistogram h;
		h.update(_obj, src, pose, K, 1.f);
		//start update
		auto prob = h.getProb(tar);
		//imshow("prob", prob);

		float thetaT = CV_PI / 8, errT = 1.f;

		float err = _obj.templ.pro(tar, K, pose, prob, thetaT, _isLocalTracking ? FLT_MAX : errT);
		//float err = 1.f;

		tarPose = _descalePose(pose);
		return err;
	}

	float refine(const Mat& img, Pose& pose, const Matx33f& K)
	{
		return this->track(img, pose, img, pose, K);
	}

	void getProjectedContours(const Matx33f& K, const Pose& pose, std::vector<Point2f>& points, std::vector<Point2f>* normals)
	{
		auto _pose = _scalePose(pose);
		_obj.templ.getProjectedContours(K, _pose, points, normals);
	}
};

_IMPL_END()

using impl_base_tracker1::BaseTracker1;

_VX_END()


