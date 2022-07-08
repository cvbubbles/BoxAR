#pragma once

#include"index.h"
#include"model2.h"
typedef Model2 ModelT;

#include"BFC/thread.h"

#include<mutex>
#include<atomic>
#include<iostream>
//#include<experimental>

#define printf(...)

#define _TRACKER_BEG namespace _detector3 {
#define _TRACKER_END }   namespace tr=_detector3;

_TRACKER_BEG

struct Param
{
	int fdFeatures = 1000;
	/*float fdScaleFactor = 1.5f;
	int  fdLevels = 4;
	float fdImageScale = 2.0f;*/

	float fdScaleFactor = 1.25f;
	int  fdLevels = 2;
	float fdImageScale = 1.0f;

	float ransacReprojectError = 8.0f;
	float ransacMaxIterations = 2000;

	int fdFeaturesSelected = 500;

	int maxTrackingObjs = 1;

	int trackingObjectSize = 600;

	int nFramesUndetectedForUpdateActive = 3;

	int updateActiveInlierThreshold = 16;

	bool showKeyPoints = false;
	bool removeStaticBG = false;
};

class GradSampler
{
public:
	//!!the image should have been padded
	static Vec2f sample(const Mat1b &img, int cx, int cy, int hwsz)
	{
		const uchar *p = img.ptr(cy, cx);
		p = p - img.step*hwsz - hwsz;

		double sx = 1e-6, sy = 1e-6, sg = 1e-3;
		for (int y = -hwsz; y <= hwsz; ++y, p += img.step)
			for (int x = -hwsz; x <= hwsz; ++x)
			{
				double g = p[x];
				sx += g*x;
				sy += g*y;
				sg += g;
			}
		sg = 1.0 / sg;
		return Vec2f(float(sx*sg), float(sy*sg));
	}
};

void splitContour(const std::vector<Point2f> &contour, std::vector<Point2f> &segs, std::vector<Vec2f> &segDirs, float maxLen)
{
	segs.clear();
	segDirs.clear();
	if (contour.empty())
		return;

	segs.reserve(contour.size() * 10);
	segDirs.reserve(segs.capacity());

	auto dv2dir = [](const Vec2f &dv) {
		return normalize(Vec2f(dv[1], -dv[0]));
	};

	Vec2f prevDir = dv2dir(contour[1] - contour[0]);

	for (int i = 0; i < (int)contour.size() - 1; ++i)
	{
		Vec2f dv(contour[i + 1] - contour[i]);
		Vec2f curDir = dv2dir(dv);

		float dvL = sqrt(dv.dot(dv));
		int nsegs = int(dvL / maxLen) + 1;

		segs.push_back(contour[i]);
		segDirs.push_back(normalize((prevDir + curDir)));

		if (nsegs > 1)
		{
			float segL = dvL / nsegs;
			dv = normalize(dv)*segL;
			Vec2f px = contour[i];
			for (int j = 0; j < nsegs - 1; ++j)
			{
				px += dv;
				segs.push_back(px);
				segDirs.push_back(curDir);
			}
		}
		prevDir = curDir;
	}
	segs.push_back(contour[contour.size() - 1]);
	segDirs.push_back(prevDir);
}

void showContourConfidence(const Mat1b &img, const std::vector<Point2f> &segs, const std::vector<Vec2f> &segDirs, const std::vector<float> &segErr)
{
	Mat3b dimg;
	cvtColor(img, dimg, CV_GRAY2BGR);

	Scalar color0(0, 0, 255), color1(255, 0, 0);
	for (size_t i = 0; i < segs.size() - 1; ++i)
	{
		cv::line(dimg, Point(segs[i]), Point(segs[i + 1]), segErr[i] < 0.5f ? color1 : color0, 2);
		cv::line(dimg, Point(segs[i]), Point(segs[i] + Point2f(segDirs[i] * 10.f)), segErr[i] < 0.5f ? color1 : color0, 2);
	}
	imshow("confidence", dimg);
}

void alignContour(const Mat1b &img, const std::vector<Point2f> &contour2d, std::vector<Point2f> &segs2d, std::vector<Point2f> &segs2dAligned);

float estimateContourConfidence(const Mat1b &img, std::vector<Point2f> &contour)
{
	{
		std::vector<Point2f> segs2d, segs2dAligned;
		alignContour(img, contour, segs2d, segs2dAligned);

		Mat dimg;
		cvtColor(img, dimg, CV_GRAY2BGR);
		drawContour(dimg, segs2d, Scalar(255, 0, 0));
		drawContour(dimg, segs2dAligned, Scalar(0, 0, 255));
		imshow("confidence", dimg);
	}

	std::vector<float> segConfidence;
	std::vector<Point2f> segs;
	std::vector<Vec2f>   segDirs;
	splitContour(contour, segs, segDirs, 5);

	segConfidence.resize(segs.size());
	for (size_t i = 0; i < segs.size(); ++i)
	{
		auto &pt = segs[i];
		auto &ptDir = segDirs[i];

		int x = int(pt.x + 0.5), y = int(pt.y + 0.5);
		if (uint(x) < uint(img.cols) && uint(y) < uint(img.rows))
		{
			Vec2f gv = GradSampler::sample(img, x, y, 1);
			if (gv.dot(gv) < 0.0001)
			{
				segConfidence[i] = 2.0f;
				gv = normalize(gv);
			}
			else
			{
				gv = normalize(gv);

				Vec2f dv1 = gv - ptDir, dv2 = -gv - ptDir;
				float d1 = dv1.dot(dv1), d2 = dv2.dot(dv2);
				segConfidence[i] = /*sqrt*/(__min(d1, d2));
			}

			ptDir = gv;
		}
		else
			segConfidence[i] = 2.0f;

	}
	//GaussianBlur(segConfidence, segConfidence, Size(9, 1), 1.0);
	blur(segConfidence, segConfidence, Size(15, 1));

	//showContourConfidence(img, segs, segDirs, segConfidence);
	return 0;
}


class MovingDetector
{
	struct Sample
	{
		int  p1, p2;
	};
	static void _sampling(Size patchSize, int nSamples, std::vector<Sample> &samples, int imgStep)
	{
		auto genPoint = [&patchSize, imgStep](Point &pt) {
			int x = rand() % patchSize.width, y = rand() % patchSize.height;
			pt = Point(x, y);
			return y*imgStep + x;
		};

		int minDist = 3;
		minDist *= minDist;
		samples.resize(nSamples);
		for (size_t i = 0; i < samples.size(); ++i)
		{
			Point pt1, pt2;
			int d2 = 0;
			do
			{
				samples[i].p1 = genPoint(pt1);
				samples[i].p2 = genPoint(pt2);
				Vec2i dv(pt1 - pt2);
				d2 = dv.dot(dv);
			} while (d2 < minDist);
		}
	}
	static void _computeBRIEF1(const uchar *p, const Sample samples[], int nSamples, uchar desc[])
	{
		for (int i = 0; i < nSamples; ++i)
		{
			auto &s = samples[i];
			//desc[i] = p[s.p1] - p[s.p2] > 3 ? 2 : p[s.p2] - p[s.p1] > 3 ? 1 : 0;
			desc[i] = p[s.p1] > p[s.p2] ? 1 : 0;
		}
	}
	static float _descDiff(const uchar desc1[], const uchar desc2[], int size)
	{
		int n = 0;
		for (int i = 0; i < size; ++i)
			if (desc1[i] != desc2[i])
				++n;
		return float(n) / size;
	}
	Mat _computeBRIEF(Mat1b img)
	{
		//if (img.step != img.cols)
		img = img.clone();

		GaussianBlur(img, img, Size(3, 3), 1.0);

		Mat desc(_grids.width*_grids.height, (int)_samples.size(), CV_8UC1);
		int i = 0;
		for (int y = 0; y<_imgSize.height; y += _gridSize.height)
			for (int x = 0; x < _imgSize.width; x += _gridSize.width)
			{
				_computeBRIEF1(img.ptr(y, x), &_samples[0], (int)_samples.size(), desc.ptr(i));
				++i;
			}
		return desc;
	}


	std::vector<Sample>  _samples;
	Size  _imgSize;
	Size  _gridSize;
	Size  _grids;
	Mat   _bgDesc;
public:
	bool isInited()
	{
		return !_samples.empty();
	}
	void updateBg(Mat1b img, Size gridSize, int nSamplesPerGrid)
	{
		CV_Assert(img.cols%gridSize.width == 0 && img.rows%gridSize.height == 0);

		if (nSamplesPerGrid != (int)_samples.size() || img.size() != _imgSize || gridSize != _gridSize)
		{
			this->_sampling(gridSize, nSamplesPerGrid, _samples, img.cols);
			_imgSize = img.size();
			_gridSize = gridSize;
			_grids = Size(img.cols / gridSize.width, img.rows / gridSize.height);
		}
		_bgDesc = _computeBRIEF(img);
	}
	Mat1b detectMoving(const Mat1b &img)
	{
		Mat1b desc = _computeBRIEF(img);
		Mat1b mask = Mat1b::zeros(img.size());
		int i = 0;
		for (int y = 0; y<_imgSize.height; y += _gridSize.height)
			for (int x = 0; x < _imgSize.width; x += _gridSize.width)
			{
				float diff = _descDiff(_bgDesc.ptr(i), desc.ptr(i), (int)_samples.size());
				if (diff > 0.4f)
				{
					setMem(mask(Rect(x, y, _gridSize.width, _gridSize.height)), 255);
				}
				++i;
			}
		return mask;
	}
};




typedef cv::Ptr<cv::ORB>  FDPtrT;

void detectImageFeatures(FDPtrT fd, const Mat &img, std::vector<KeyPoint> &kp, Mat &desc, Mat mask, const Param &param);

void detectImageFeatures(const Mat &img, std::vector<KeyPoint> &kp, Mat &desc, Mat mask, const Param &param)
{
	FDPtrT fd = cv::ORB::create(param.fdFeatures, param.fdScaleFactor, param.fdLevels);
	detectImageFeatures(fd, img, kp, desc, mask, param);
}

class ResultBase
{
public:
	int    objID = -1;

	int    nInliers = -1;
	Vec3f  rvec = Vec3f(0, 0, 0);
	Vec3f  tvec = Vec3f(0, 0, 0);

	std::vector<Point>  objContour;
	Rect				objROI=Rect(0,0,0,0);

	bool     detectionIsBetter = false;
	bool     detected = false;
public:
	bool isTracked() const
	{
		return nInliers > 20;
	}
	void setResult(int _nInliers, const Mat &_rvec, const Mat &_tvec)
	{
		nInliers = _nInliers;
		rvec = _rvec.empty() ? Vec3f(0, 0, 0) : _rvec;
		tvec = _tvec.empty() ? Vec3f(0, 0, 0) : _tvec;
	}
	bool updateObjectInfo(const Mat1b &img, ModelT &model, const Matx44f &mProjection, Size imgSize)
	{
		if (nInliers > 16)
		{
			CVRMats mats;
			mats.mProjection = mProjection;
			mats.mModel = cvrm::fromRT(rvec, tvec);

			CVRProjector prj(mats, imgSize);

			std::vector<Point2f> contour;
			prj.project(model.cts->getContour(mats.mModel), contour);

			//estimateContourConfidence(img, contour);

			objContour.resize(contour.size());
			for (size_t i = 0; i < contour.size(); ++i)
				objContour[i] = Point(contour[i]);
			objROI = getBoundingBox2D(objContour);
			return true;
		}
		else
		{
			nInliers = 0;
			objContour.clear();
			objROI = Rect(0, 0, 0, 0);
		}
		return false;
	}
};

class Result
	:public ResultBase
{
public:
	//std::vector<int>  inlierModelPoints;
};

int evalResult(ResultBase &res, const Mat1b &img, ModelT &model, const Matx44f &mProjection, float maxPrjErr = 5.0);

int  solveRANSAC(ResultBase &res, std::vector<int> &inliers, const Matx33f &K, const std::vector<Point2f> &imgPts, const std::vector<Point3f> &objPts, Size viewSize, const Param &param, Rect guessROI = Rect(0, 0, 0, 0));

void getPointPairs(ModelT &model, const std::vector<KeyPoint> &camImgKP, const Mat &camImgDesc,
	std::vector<Point2f> &imgPoints, std::vector<Point3f> &objPoints, std::vector<int> &objPointsEx, const Param &param, int nsel = -1);


class Frame;
typedef std::shared_ptr<Frame>  FramePtr;


struct TrackingObject
{
	int      objID;
	ModelT  *model;

	int        nFramesUndetected = 0;

public:
	void initialize(int _objID, ModelT *_model, const Param &param)
	{
		objID = _objID;
		model = _model;
	}
};




struct FrameState
{
public:
	struct ModelState
	{
		int  nInliers;
		Rect objROI;
	};
public:
	bool   isFree;
	std::vector<ModelState>  models;
};

typedef std::list<FrameState>::iterator  _StateItrT;

//std::list<Frame*>  gFrameList, gFrameDel;
//std::mutex    gFrameListMutex;

class Tracker3;

struct Frame
{
public:
	Tracker3 *site;
	_StateItrT  stateItr;

	Mat1b _imgPadded;
	Mat1b img;
	Param param;

	std::vector<ResultBase>  vResults;
public:
	//Frame() {}
	Frame(Tracker3 *_site, const Mat &_img, const Param &_param, int _nModels, _StateItrT _stateItr)
		:site(_site), param(_param), stateItr(_stateItr)
	{
		stateItr->isFree = false;

		if (_img.channels() != 1)
			cvtColor(_img, img, CV_BGR2GRAY);
		else
			img = _img;


#if 0
		int BW = 7;
		copyMakeBorder(img, _imgPadded, BW, BW, BW, BW, BORDER_REFLECT);
		img = _imgPadded(Rect(BW, BW, _img.cols, _img.rows));
#endif
		//GaussianBlur(img, img, Size(3, 3), 0.8);

		vResults.resize(_nModels);

		//std::lock_guard<std::mutex> lock(gFrameListMutex);
		//gFrameList.push_back(this);
	}
	~Frame();

	void updateFrameState()
	{
		int nModels = (int)vResults.size();
		stateItr->models.resize(nModels);
		for (int i = 0; i < nModels; ++i)
		{
			auto &m = stateItr->models[i];
			m.nInliers = vResults[i].nInliers;
			m.objROI = vResults[i].objROI;
		}
	}

	void detect(std::vector<TrackingObject*> &objs, int nThreads, const Matx33f &K, const Matx44f &mProjection, bool forceDetect = false);


};

class Tracker3
{
	friend struct Frame;

	Matx33f    _K;
	Size       _imgSize;
	Matx44f    _mProjection;

	TrackingObject  *_models = nullptr;
	int              _nModels = 0;

	std::vector<TrackingObject*>  _modelsAll;

	std::vector<TrackingObject*>  _modelsActive;
	std::mutex    _mutex1;

	std::list<FrameState>  _stateList;
	std::list<_StateItrT>  _stateForActiveDetection;
	std::mutex    _mutex2;

	Param  _param;

	MovingDetector  _movDetector;
	int             _nFramesForMovDetector = 0;
	std::mutex      _mutex3;

	//place the thread object at the end that it can be destructed normally
	//ff::Thread    _thread0;
	//ff::Thread    _thread1;
	ff::Thread    _thread2;
public:
	Tracker3()
	{
		if (/*!_thread0.setPriority(-1) || !_thread1.setPriority(-1) ||*/ !_thread2.setPriority(-2))
			printf("warning: failed to set thread priority\n");
	}
	~Tracker3()
	{
		//_thread0.waitAll();
		//_thread1.waitAll();
		_thread2.waitAll();

		delete[]_models;
	}
	operator bool() const
	{
		return _models != nullptr;
	}
	void setParam(const Param &param)
	{
		_param = param;
	}
	void set(ModelT *models[], int count)
	{
		//_K = K;
		_imgSize = Size(0, 0);

		if (_models)
			delete[]_models;
		_models = new TrackingObject[count];
		_nModels = count;

		_modelsAll.resize(count);
		for (int i = 0; i < count; ++i)
		{
			_models[i].initialize(i, models[i], _param);
			_modelsAll[i] = &_models[i];
		}
		_modelsActive.push_back(&_models[0]);
	}

	void _cleanStateList()
	{
		if (_stateForActiveDetection.empty())
			return;

		//return;
		_StateItrT end = _stateForActiveDetection.front();
		//_StateItrT end = _stateList.end();
		for (auto itr = _stateList.begin(); itr != end;)
		{
			if (itr->isFree)
			{
				auto ditr = itr;
				++itr;
				_stateList.erase(ditr);
			}
			else
				++itr;
		}
	}

	void _doUpdateActive(FramePtr &frame);

	void _updateActive(FramePtr &frame)
	{
		//return;
		_thread2.post([this, frame](ff::Thread::MsgData &data) mutable {
			if (data.nRemainingMessages == 0)
			{
				printf("do update active\n");
				this->_doUpdateActive(frame);
			}
			{
				std::lock_guard<std::mutex> lock(_mutex2);

				this->_cleanStateList();
			}
		});
	}


	void _doDetect(FramePtr &frame, std::vector<TrackingObject*> &activeObjs)
	{
		if (!activeObjs.empty())
		{
			//if (_thread1.nRemainingMessages() == 0)
			{
				//_thread1.post([this, frame, activeObjs](ff::Thread::MsgData &data) mutable 
				{
				//	if (data.nRemainingMessages == 0)
					{
						frame->detect(activeObjs, (int)activeObjs.size(), _K, _mProjection, true);

						//std::vector<TrackingObject*> betterObjs;
						int nlost = 0;
						for (auto *p : activeObjs)
						{
							auto &r = frame->vResults[p->objID];
							//printf("detecting obj %d, inliers=%d\n", p->objID, r.nInliers);
							if (!r.isTracked())
							{
								if (++p->nFramesUndetected > _param.nFramesUndetectedForUpdateActive)
								{
									++nlost;
									p->nFramesUndetected = 0;
								}
							}
							else
							{
								p->nFramesUndetected = 0;
							}
						}

						if (nlost != 0)
							this->_updateActive(frame);
					}
				}
			}
		}
		else
			this->_updateActive(frame);
	}
	void _updateBG(const Mat1b &img)
	{
		if (_param.removeStaticBG)
		{
			std::lock_guard<std::mutex> lock(_mutex3);
			_movDetector.updateBg(img, Size(20, 20), 100);
		}
	}

	int track(Mat img, const Matx33f &K, ResultBase results[])
	{
		_K = K;

		if(img.channels()==3)
			cvtColor(img, img, CV_BGR2GRAY);

		if (img.size() != _imgSize)
		{
			_imgSize = img.size();
			_mProjection = cvrm::fromK(_K, _imgSize, 0.1, 1000);

			this->_updateBG(img);

			if (_param.showKeyPoints)
			{
				namedWindow("KeyPoints");
				imshow("KeyPoints", img);
				waitKey(1);
			}
		}

		std::vector<TrackingObject*>  activeModels;
		_StateItrT stateItr;

		{
			std::lock_guard<std::mutex>  lock(_mutex1);
			activeModels = _modelsActive;
		}
		{
			std::lock_guard<std::mutex>  lock(_mutex2);
			_stateList.push_back(FrameState());
			stateItr = _stateList.end();
			--stateItr;
		}

		FramePtr frame(new Frame(this, img, _param, _nModels, stateItr));
		//if (!activeModels.empty())
		//{
		//	trackAll(frame, activeModels, (int)activeModels.size(), _K, _mProjection);
		//	//frame->detect(activeModels, 1, _K, _mProjection);
		//	//updateObjectInfoOfResults(frame, activeModels, _mProjection);
		//}
		frame->updateFrameState();

		this->_doDetect(frame, activeModels);


		int nt = 0;
		for (size_t i = 0; i < _modelsAll.size(); ++i)
		{
			if (frame->vResults[i].nInliers > 0)
			{
				results[nt] = frame->vResults[i];
				results[nt].objID = i;
				++nt;
			}
		}

		return nt;
	}
};

_TRACKER_END

using namespace tr;

typedef tr::Param ParamT;
typedef tr::Tracker3 TrackerT;



