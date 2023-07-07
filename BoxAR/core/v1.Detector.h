#pragma once

#include"v1.DetectorModelData.h"
#include"BFC/log.h"

_VX_BEG(v1)

namespace vx = v1;

_IMPL_BEG(impl_detector)

struct Param
{
	int fdFeatures = 1000;
	
	float fdScaleFactor = 1.2f;
	int  fdLevels = 4;
	float fdImageScale = 1.0f;

	/*float fdScaleFactor = 1.25f;
	int  fdLevels = 2;
	float fdImageScale = 1.0f;*/

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

//typedef cv::Ptr<cv::ORB>  FDPtrT;

void detectImageFeatures(const Mat& img, std::vector<KeyPoint>& kp, Mat& desc, Mat mask, const Param& param)
{
	auto fd = cv::ORB::create(param.fdFeatures, param.fdScaleFactor, param.fdLevels);
	//auto fd = cv::AKAZE::create(5, 0, 3, 0.001, 4, 2, 1);

	if (fabs(param.fdImageScale - 1.0f) < 1e-3f)
		fd->detectAndCompute(img, mask, kp, desc);
	else
	{
		Mat1b gray;
		if (img.channels() != 1)
			cvtColor(img, gray, CV_BGR2GRAY);
		else
			gray = img.clone();

		Size dsize(Size(int(gray.cols * param.fdImageScale + 0.5f), int(gray.rows * param.fdImageScale + 0.5f)));
		resize(gray, gray, dsize);
		if (!mask.empty())
			resize(mask, mask, dsize);
		fd->detectAndCompute(gray, mask, kp, desc);
		float fscale = 1.0f / param.fdImageScale;
		for (auto& p : kp)
			p.pt *= fscale;
	}
}

//void detectImageFeatures(const Mat& img, std::vector<KeyPoint>& kp, Mat& desc, Mat mask, const Param& param)
//{
//	//FDPtrT fd = cv::ORB::create(param.fdFeatures, param.fdScaleFactor, param.fdLevels);
//	
//	detectImageFeatures(fd, img, kp, desc, mask, param);
//}


void getPointPairsWithMatches(std::vector<DMatch>& matches, const std::vector<Point3f>& objKP, const std::vector<KeyPoint>& camImgKP, const Mat& camImgDesc,
	std::vector<Point2f>& imgPoints, std::vector<Point3f>& objPoints, std::vector<int>& objPointsEx, const Param& param, int nsel)
{
	std::sort(matches.begin(), matches.end(), [](const DMatch& a, const DMatch& b) {
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
		auto& m(matches[i]);
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

void getPointPairs(vx::DetectorModelData& modelData, const std::vector<KeyPoint>& camImgKP, const Mat& camImgDesc,
	std::vector<Point2f>& imgPoints, std::vector<Point3f>& objPoints, std::vector<int>& objPointsEx, const Param& param, int nsel=-1)
{
	std::vector<DMatch> matches;
	modelData.modelPoints.findMatch(camImgDesc, matches);

	getPointPairsWithMatches(matches, modelData.modelPoints.modelPts, camImgKP, camImgDesc, imgPoints, objPoints, objPointsEx, param, nsel);
}


class Result
{
public:
	int    nInliers = -1;
	Vec3f  rvec = Vec3f(0, 0, 0);
	Vec3f  tvec = Vec3f(0, 0, 0);
public:
	void setResult(int _nInliers, const Mat& _rvec, const Mat& _tvec)
	{
		nInliers = _nInliers;
		rvec = _rvec.empty() ? Vec3f(0, 0, 0) : _rvec;
		tvec = _tvec.empty() ? Vec3f(0, 0, 0) : _tvec;
	}
};

Result  solveRANSAC(std::vector<int>& inliers, const Matx33f& K, const std::vector<Point2f>& imgPts, const std::vector<Point3f>& objPts, Size viewSize, const Param& param, Rect guessROI = Rect(0, 0, 0, 0))
{
	Result res;
	if (imgPts.size() < 8)
	{
		inliers.clear();
		res.nInliers = 0;
		return res;
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
			auto& p = imgPts[i];
			if (int(p.x) > guessROI.x && int(p.x) < guessROI.x + guessROI.width && int(p.y) > guessROI.y && int(p.y) < guessROI.y + guessROI.height)
			{
				roiImgPts.push_back(imgPts[i]);
				roiObjPts.push_back(objPts[i]);
				roiObjIdx.push_back((int)i);
			}
		}

		if (roiImgPts.size() > 16)
		{
			cv::solvePnPRansac(roiObjPts, roiImgPts, K, distCoeffs, rvec, tvec, false, param.ransacMaxIterations, reprjError, 0.99, inliers, SOLVEPNP_AP3P);
			for (auto& i : inliers)
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

	return res;
}

class DModel
{
public:
	CVRModel               model;
	vx::DetectorModelData* detectorModelData;
};

class Detector
	:public re3d::FrameProc
{
	DEFINE_RE3D_TYPE2(Detector, "v1.Detector")
private:

	ModelSet* _modelSet;
	std::vector<DModel>  _models;
	Param  _param;
public:
	virtual void init(ModelSet* modelSet, ArgSet* args)
	{
		//using namespace detector;

		bool preload = false;
		std::string d2dModelFile;

		if (args)
		{
			preload = args->getd<bool>("preload", false);
		}

		_modelSet = modelSet;

		//if (preload)
		{
			std::vector<DModel> models(modelSet->size());

			for (int i = 0; i < modelSet->size(); ++i)
			{
				auto md = _modelSet->getModel(i);
				auto* detectorModelData = verify(md)->getManaged<v1::DetectorModelData>();
				if (!detectorModelData)
					FF_EXCEPTION1(ff::logfmt("failed to load model %d\n", i));
				
				models[i].detectorModelData = detectorModelData;
				models[i].model = md->get3DModel();
			}
			_models.swap(models);
		}
	}

	virtual int pro(const Mat& img, FrameData& fd, ff::ArgSet*)
	{
		//using namespace detector;
		Mat camImgDesc;
		std::vector<KeyPoint> camImgKP;

		if (camImgKP.empty())
		{
			Mat mask;
			/*if (param.removeStaticBG)
			{
				std::lock_guard<std::mutex> lock(site->_mutex3);
				mask = site->_movDetector.detectMoving(img);
			}*/

			detectImageFeatures(img, camImgKP, camImgDesc, mask, _param);

			if (_param.showKeyPoints)
			{
				Mat dimg;
				cvtColor(img, dimg, CV_GRAY2BGR);
				for (auto& p : camImgKP)
					cv::circle(dimg, Point(p.pt), 2, Scalar(rand() % 255, rand() % 255, rand() % 255), 2, CV_AA);
				imshow("KeyPoints", dimg);
				waitKey(1);
			}
		}

		if (camImgKP.empty())
			return 0;

		fd.objs.clear();

		//#pragma omp parallel num_threads(nThreads)
		for (int i = 0; i < (int)_models.size(); ++i)
		{
			auto& obj = _models[i];

			std::vector<Point2f> imgPts;
			std::vector<Point3f> objPts;
			std::vector<int>     objPtsEx;
			getPointPairs(*_models[i].detectorModelData, camImgKP, camImgDesc, imgPts, objPts, objPtsEx, _param, _param.fdFeaturesSelected);

			if (imgPts.size() > 20)
			{
				std::vector<int> inliers;

				auto r=solveRANSAC(inliers, fd.cameraK, imgPts, objPts, img.size(), _param);

				re3d::ImageObject r3d;
				r3d.modelIndex = i;
				r3d.pose.make <  std::vector < RigidPose > >() = { RigidPose(r.rvec, r.tvec) };
				r3d.score = (float)r.nInliers;
				
				fd.objs.push_back(r3d);
			}
		}

		return (int)fd.objs.size();
	}
};

_IMPL_END()

using impl_detector::Detector;

REGISTER_RE3D_TYPE(Detector, 0)

_VX_END()

