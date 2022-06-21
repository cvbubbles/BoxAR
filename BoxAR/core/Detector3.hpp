#pragma once

#include"Detector3.h"

_TRACKER_BEG

struct ContourPoint
{
	Point3f  pt3d;
	Point2f  pt2d;
};

void segContourPoints(const std::vector<Point3f> &contour3d, const std::vector<Point2f> &contour2d, std::vector<ContourPoint> &cpts, float maxLen)
{
	cpts.clear();
	cpts.reserve(1000);

	cpts.push_back({ contour3d[0],contour2d[0] });

	for (size_t i = 0; i < contour3d.size() - 1; ++i)
	{
		Vec2f dv = contour2d[i + 1] - contour2d[i];
		float len = sqrt(dv.dot(dv));
		int nseg = 1;
		float segLen = len;
		if (len > maxLen)
		{
			nseg = int(len / maxLen) + 1;
			segLen = len / nseg;
		}
		Vec2f n = normalize(dv);

		Vec3f dv3d = Vec3f(contour3d[i + 1] - contour3d[i]);
		Vec3f n3d = normalize(dv3d);
		float segLen3d = norm(dv3d) / nseg;

		for (int j = 1; j <= nseg; ++j)
		{
			cpts.push_back({ contour3d[i] + Point3f(float(j)*segLen3d*n3d), contour2d[i] + Point2f(float(j)*segLen*n) });
		}
	}
}
void segContourPoints(const std::vector<Point2f> &contour2d, std::vector<ContourPoint> &cpts, float maxLen)
{
	cpts.clear();
	cpts.reserve(1000);

	cpts.push_back({ Point3f(0,0,0),contour2d[0] });

	for (size_t i = 0; i < contour2d.size() - 1; ++i)
	{
		Vec2f dv = contour2d[i + 1] - contour2d[i];
		float len = sqrt(dv.dot(dv));
		int nseg = 1;
		float segLen = len;
		if (len > maxLen)
		{
			nseg = int(len / maxLen) + 1;
			segLen = len / nseg;
		}
		Vec2f n = normalize(dv);

		for (int j = 1; j <= nseg; ++j)
		{
			cpts.push_back({ Point3f(0,0,0), contour2d[i] + Point2f(float(j)*segLen*n) });
		}
	}
}

struct Sampler
{
	Mat _imgx;
	Mat _img;
	int _BW;
	cv::BL4Resampler _resamp;
public:
	Sampler(const Mat &img, int BW)
	{
		copyMakeBorder(img, _imgx, BW, BW, BW, BW, BORDER_REFLECT);
		_img = _imgx(Rect(BW, BW, img.cols, img.rows));
		_BW = BW;
		_resamp.Init(_img.data, _img.cols, _img.rows, _img.step);
	}
	void sampleLine(float *val, const Vec2f &start, const Vec2f &dv, int n)
	{
		Vec2f p = start;
		for (int i = 0; i < n; ++i)
		{
			_resamp.resample<0, 1, uchar, float>(p[0], p[1], &val[i]);
			p += dv;
		}
	}
	void sampleRegion(Mat &result, const Vec2f &O, const Vec2f &dy, int rows, const Vec2f &dx, int cols)
	{
		result.create(rows, cols, CV_32FC1);

		for (int y = 0; y < rows; ++y)
		{
			this->sampleLine(result.ptr<float>(y), O + dy*y, dx, cols);
		}
	}
};

struct Line
{
	Vec2f start, end;
	float grad;

	float cost;
	Line *prev = nullptr;
};
struct Seg
{
	Vec2f  origin;
	Vec2f  dy; //the line direction
	Vec2f  dx; //the normal direction
	std::vector<Line>  lines;
	Line *lnSel;
public:
	static void _gradSum1(const float *p, int size, float *sum)
	{
		sum[0] += p[1] - p[0];
		for (int i = 1; i < size - 1; ++i)
			sum[i] += p[i + 1] - p[i - 1];
		sum[size - 1] += p[size - 1] - p[size - 2];
	}
	static void _gradSum(const float *p, int size, float *sum)
	{
		sum[0] += abs(p[1] - p[0]);
		for (int i = 1; i < size - 1; ++i)
			sum[i] += abs(p[i + 1] - p[i - 1]);
		sum[size - 1] += abs(p[size - 1] - p[size - 2]);
	}
	void construct(Sampler &sampler, const ContourPoint &start, const ContourPoint &end, int sampleY, int searchX)
	{
		dy = Vec2f(end.pt2d - start.pt2d);
		dy *= 1.0f / sampleY;

		Vec2f n(dy[1], -dy[0]);
		dx = n = normalize(n);

		origin = Vec2f(start.pt2d) - n*float(searchX);

		Mat1f img;
		sampler.sampleRegion(img, origin, dy, sampleY, dx, 2 * searchX + 1);
		GaussianBlur(img, img, Size(3, 1), 0.75);

		std::unique_ptr<float[]> _gsum(new float[img.cols]);
		float *gsum = _gsum.get();
		memset(gsum, 0, sizeof(float)*img.cols);

		for (int i = 0; i < img.rows; ++i)
			_gradSum(img.ptr<float>(i), img.cols, gsum);

		lines.resize(img.cols);
		Vec2f yoffset = end.pt2d - start.pt2d;
		for (int i = 0; i < img.cols; ++i)
		{
			auto &ln = lines[i];

			ln.start = origin + dx*float(i);
			ln.end = ln.start + yoffset;
			lines[i].grad = abs(gsum[i]) / img.rows;
		}

		Line *pl = nullptr;
		float gmax = -1;
		for (auto &ln : lines)
			if (ln.grad > gmax)
			{
				gmax = ln.grad;
				pl = &ln;
			}
		lnSel = pl;
	}
};

void getInliers(std::vector<int> &inliers, const std::vector<Point3f> &p3d, const std::vector<Point2f> &p2d, const CVRMats &gtMats, Size viewSize, float prjErr)
{
	CVRProjector prj(gtMats, viewSize);
	std::vector<Point2f>  p2dx;
	prj.project(p3d, p2dx);

	inliers.clear();
	prjErr *= prjErr;
	for (size_t i = 0; i < p2d.size(); ++i)
	{
		Vec2f dv = p2d[i] - p2dx[i];
		if (dv.dot(dv) < prjErr)
			inliers.push_back(int(i));
	}
}
void dpSolve(std::vector<Seg> &segs)
{
	for (auto &s : segs[0].lines)
	{
		s.prev = nullptr;
		s.cost = s.grad;
	}

	for (size_t i = 1; i < segs.size(); ++i)
	{
		for (auto &cur : segs[i].lines)
		{
			float cmin = FLT_MAX;
			Line *pmin = nullptr;
			for (auto &prev : segs[i - 1].lines)
			{
				float ds = norm(cur.start - prev.end) + 1;
				float c = prev.cost + ds*ds;
				if (c < cmin)
				{
					cmin = c;
					pmin = &prev;
				}
			}
			//cur.cost = cmin + __max(64,(255-cur.grad));
			CV_Assert(cur.grad <= 255.0f);
			cur.cost = cmin + (255 - cur.grad);
			cur.prev = pmin;
		}
	}

	Line *sel = nullptr;
	float cmin = FLT_MAX;
	for (auto &s : segs[segs.size() - 1].lines)
	{
		if (s.cost < cmin)
		{
			cmin = s.cost;
			sel = &s;
		}
	}

	for (int i = (int)segs.size() - 1; i >= 0; --i)
	{
		segs[i].lnSel = sel;
		sel = sel->prev;
	}
}

void alignContour(const Mat1b &img, const std::vector<Point2f> &contour2d, std::vector<Point2f> &segs2d, std::vector<Point2f> &segs2dAligned)
{
	float maxSegLen = 50;

	std::vector<ContourPoint> cpts;
	segContourPoints(contour2d, cpts, maxSegLen);

	Sampler sampler(img, 20);

	std::vector<Seg>  segs(cpts.size() - 1);
	for (size_t i = 0; i < cpts.size() - 1; ++i)
	{
		segs[i].construct(sampler, cpts[i], cpts[i + 1], 20, 25);
	}

	dpSolve(segs);

	//std::vector<Point3f>  p3d(segs.size() * 2);
	//std::vector<Point2f>  p2d(segs.size() * 2);
	segs2d.resize(segs.size() * 2);
	segs2dAligned.resize(segs2d.size());
	for (size_t i = 0; i < segs.size(); ++i)
	{
		segs2d[i * 2] = cpts[i].pt2d;
		segs2d[i * 2 + 1] = cpts[i + 1].pt2d;
		//p3d[i * 2] = cpts[i].pt3d;
		segs2dAligned[i * 2] = segs[i].lnSel->start;
		//p3d[i * 2 + 1] = cpts[i + 1].pt3d;
		segs2dAligned[i * 2 + 1] = segs[i].lnSel->end;
	}
}

void detectImageFeatures(FDPtrT fd, const Mat &img, std::vector<KeyPoint> &kp, Mat &desc, Mat mask, const Param &param)
{
	//static auto fd = cv::ORB::create(param.fdFeatures, param.fdScaleFactor, param.fdLevels);

	if (fabs(param.fdImageScale - 1.0f)<1e-3f)
		fd->detectAndCompute(img, mask, kp, desc);
	else
	{
		Mat1b gray;
		if (img.channels() != 1)
			cvtColor(img, gray, CV_BGR2GRAY);
		else
			gray = img.clone();

		Size dsize(Size(int(gray.cols*param.fdImageScale + 0.5f), int(gray.rows*param.fdImageScale + 0.5f)));
		resize(gray, gray, dsize);
		if (!mask.empty())
			resize(mask, mask, dsize);
		fd->detectAndCompute(gray, mask, kp, desc);
		float fscale = 1.0f / param.fdImageScale;
		for (auto &p : kp)
			p.pt *= fscale;
	}
}



int  solveRANSAC(ResultBase &res, std::vector<int> &inliers, const Matx33f &K, const std::vector<Point2f> &imgPts, const std::vector<Point3f> &objPts, Size viewSize, const Param &param, Rect guessROI)
{
	if (imgPts.size() < 8)
	{
		inliers.clear();
		res.nInliers = 0;
		return 0;
	}
	double reprjError = param.ransacReprojectError;

	cv::Mat distCoeffs;

	Mat rvec, tvec;

	if (guessROI.width > 0 && guessROI.height > 0)
	{
		std::vector<Point2f> roiImgPts;
		std::vector<Point3f> roiObjPts;
		std::vector<int>     roiObjIdx;
		roiImgPts.reserve(imgPts.size());
		roiObjPts.reserve(objPts.size());
		roiObjIdx.reserve(objPts.size());

		const int BW = guessROI.width / 4, BH = guessROI.height / 4;
		Rect roi = guessROI;
		rectAppend(roi, BW, BH, BW, BH);

		for (size_t i = 0; i < imgPts.size(); ++i)
		{
			auto &p = imgPts[i];
			if (int(p.x) > guessROI.x&&int(p.x) < guessROI.x + guessROI.width && int(p.y) > guessROI.y&&int(p.y) < guessROI.y + guessROI.height)
			{
				roiImgPts.push_back(imgPts[i]);
				roiObjPts.push_back(objPts[i]);
				roiObjIdx.push_back((int)i);
			}
		}

		if (roiImgPts.size() > 16)
		{
			cv::solvePnPRansac(roiObjPts, roiImgPts, K, distCoeffs, rvec, tvec, false, param.ransacMaxIterations, reprjError, 0.99, inliers, SOLVEPNP_AP3P);
			for (auto &i : inliers)
				i = roiObjIdx[i];
		}
	}

	if (inliers.size() < 16)
	{
		std::vector<int> inliersx;
		Mat rvecx, tvecx;

		cv::solvePnPRansac(objPts, imgPts, K, distCoeffs, rvecx, tvecx, false, param.ransacMaxIterations, reprjError, 0.99, inliersx, SOLVEPNP_AP3P);
		if (inliersx.size() > inliers.size())
		{
			rvec = rvecx;
			tvec = tvecx;
			inliers.swap(inliersx);
		}
	}

	int ninliers = inliers.size();
	res.setResult(ninliers, rvec, tvec);

	return ninliers;
}

void getPointPairsWithMatches(std::vector<DMatch> &matches, const std::vector<Point3f> &objKP, const std::vector<KeyPoint> &camImgKP, const Mat &camImgDesc,
	std::vector<Point2f> &imgPoints, std::vector<Point3f> &objPoints, std::vector<int> &objPointsEx, const Param &param, int nsel)
{
	std::sort(matches.begin(), matches.end(), [](const DMatch &a, const DMatch &b) {
		return a.distance < b.distance;
	});

	if (nsel > 0)
	{
		nsel = __min(nsel, (int)matches.size() / 2);
		matches.resize(nsel);
	}

	imgPoints.resize(matches.size());
	objPoints.resize(matches.size());
	objPointsEx.resize(matches.size());

	//auto &objKP = model.pts.modelPts;
	size_t nvalid = 0;

	for (size_t i = 0; i < matches.size(); ++i)
	{
		auto &m(matches[i]);
		if (uint(m.trainIdx) < objKP.size())
		{
			imgPoints[nvalid] = camImgKP[m.queryIdx].pt;
			objPoints[nvalid] = objKP[m.trainIdx];
			objPointsEx[nvalid] = m.trainIdx;
			++nvalid;
		}
	}
	if (nvalid != objPoints.size())
	{
		imgPoints.resize(nvalid);
		objPoints.resize(nvalid);
	}
}

void getPointPairs(ModelT &model, const std::vector<KeyPoint> &camImgKP, const Mat &camImgDesc,
	std::vector<Point2f> &imgPoints, std::vector<Point3f> &objPoints, std::vector<int> &objPointsEx, const Param &param, int nsel)
{
	std::vector<DMatch> matches;
	model.pts->findMatch(camImgDesc, matches);
	//model.pts.matcher.match(camImgDesc, matches);

	getPointPairsWithMatches(matches, model.pts->modelPts, camImgKP, camImgDesc, imgPoints, objPoints, objPointsEx, param, nsel);
}
Frame::~Frame()
{
	{
		std::lock_guard<std::mutex> lock(site->_mutex2);
		stateItr->isFree = true;
	}
}

void Frame::detect(std::vector<TrackingObject*> &objs, int nThreads, const Matx33f &K, const Matx44f &mProjection, bool forceDetect)
{
	Mat camImgDesc;
	std::vector<KeyPoint> camImgKP;

	if (camImgKP.empty())
	{
		Mat mask;
		if (param.removeStaticBG)
		{
			std::lock_guard<std::mutex> lock(site->_mutex3);
			mask = site->_movDetector.detectMoving(img);
		}

		detectImageFeatures(img, camImgKP, camImgDesc, mask, param);

		if (param.showKeyPoints)
		{
			Mat dimg;
			cvtColor(img, dimg, CV_GRAY2BGR);
			for (auto &p : camImgKP)
				cv::circle(dimg, Point(p.pt), 2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, CV_AA);
			imshow("KeyPoints", dimg);
			waitKey(1);
		}
	}

	if (camImgKP.empty())
		return;

//#pragma omp parallel num_threads(nThreads)
	for (int i = 0; i < (int)objs.size(); ++i)
	{
		auto &obj = *objs[i];
		auto &r = vResults[obj.objID];

		//if has detected
		if (r.detected && !forceDetect)
			continue;
		else
			r.detected = true;

		std::vector<Point2f> imgPts;
		std::vector<Point3f> objPts;
		std::vector<int>     objPtsEx;
		getPointPairs(*obj.model, camImgKP, camImgDesc, imgPts, objPts, objPtsEx, param, param.fdFeaturesSelected);

		//printf("imgPts=%d     \n", (int)imgPts.size());
		r.detectionIsBetter = false;
		if (imgPts.size() > 20)
		{
			std::vector<int> inliers;

			//r.objROI, use tracker guess
			solveRANSAC(r, inliers, K, imgPts, objPts, img.size(), param, r.objROI);
			//printf("xInliers=%d\n", (int)inliers.size());
			r.updateObjectInfo(img, *obj.model, mProjection, img.size());
		}
	}
}

void Tracker3::_doUpdateActive(FramePtr &frame)
{
	const int maxFramesForActiveDetection = 2;
	const int maxTrackingObjs = frame->param.maxTrackingObjs;

	//time_t beg = clock();
	frame->detect(_modelsAll, /*_modelsAll.size()*/1, _K, _mProjection);
	frame->updateFrameState();
	//printf("update fps=%.1lf \r",CLOCKS_PER_SEC/double(clock()-beg));

	while (_stateForActiveDetection.size() > maxFramesForActiveDetection - 1)
		_stateForActiveDetection.pop_front();

	_stateForActiveDetection.push_back(frame->stateItr);

	struct DSort
	{
		int  objID;
		int  score = 0;
		float maxIOU = 0;
	};
	std::vector<DSort>  vsort(_nModels);
	for (int i = 0; i < _nModels; ++i)
		vsort[i].objID = i;
	_StateItrT _first = _stateForActiveDetection.front();
	for (auto itr : _stateForActiveDetection)
	{
		for (int i = 0; i < (int)itr->models.size(); ++i)
		{
			int nm = itr->models[i].nInliers;
			//if(nm>)
			vsort[i].score += nm;
			if (itr != _first)
			{
				Rect &roi1 = _first->models[i].objROI, &roi2 = itr->models[i].objROI;
				Rect roi = rectOverlapped(roi1, roi2);
				float iou = float(roi.width*roi.height) / (roi1.width*roi1.height + roi2.width*roi2.height - roi.width*roi.height + 1);
				if (iou > vsort[i].maxIOU)
					vsort[i].maxIOU = iou;
			}
		}
	}
	std::vector<DSort>  vsortx;
	for (auto &v : vsort)
		if (v.score / _stateForActiveDetection.size() > _param.updateActiveInlierThreshold && v.maxIOU>0.5f)
		{
			vsortx.push_back(v);
		}
	if (vsortx.size() > maxTrackingObjs)
	{
		std::sort(vsortx.begin(), vsortx.end(), [](DSort &a, DSort &b) {
			return a.score > b.score;
		});
		vsortx.resize(maxTrackingObjs);
		std::sort(vsortx.begin(), vsortx.end(), [](DSort &a, DSort &b) {
			return a.objID < b.objID;
		});
	}
	//if (!vsortx.empty())
	{
		std::vector<TrackingObject*>  mx;
		for (auto &v : vsortx)
		{
			printf("activate obj %d, maxIOU=%.2f\n", v.objID, v.maxIOU);
			mx.push_back(&_models[v.objID]);
		}

		{
			if (!mx.empty())
			{
				std::lock_guard<std::mutex>  lock(_mutex1);
				_modelsActive.swap(mx);
			}
			else
			{
				++_nFramesForMovDetector;
				if (_nFramesForMovDetector > 20 && _param.removeStaticBG)
				{
					this->_updateBG(frame->img);
				}
			}
		}
	}
}

_TRACKER_END


