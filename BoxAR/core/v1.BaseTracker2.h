#pragma once

#include"vxbase.h"
#include<queue>
#include "opencv2/video/tracking.hpp"

_VX_BEG(v1)

_IMPL_BEG(impl_base_tracker2)


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
	std::vector<Point2f> operator()(const std::vector<_ValT>& vP, _getPointT getPoint) const
	{
		std::vector<Point2f> vp(vP.size());
		for (int i = 0; i < (int)vP.size(); ++i)
			vp[i] = (*this)(getPoint(vP[i]));
		return vp;
	}
	template<typename _ValT>
	std::vector<Point2f> operator()(const std::vector<_ValT>& vP)
	{
		return (*this)(vP, [](const _ValT& v) {return v; });
	}
};

class CImpl
{
public:
	CVRModel  model3d;
	std::vector<Point3f>  bbox3d;
	CVRender   render;
public:
	Rect getRenderROI(const RigidPose& pose, const Matx33f& K)
	{
		Projector prj(K, pose.R, pose.t);
		auto pts=prj(this->bbox3d);
		return getBoundingBox2D(pts);
	}
};

struct PointPair
{
	Point2f src, tar;
	float err;
	Point3f pt3d;
};

std::vector<PointPair> findMatchKLT(Mat src, Mat tar, Rect roi, int maxCount=300)
{
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	src = cv::convertBGRChannels(src, 1);
	tar = cv::convertBGRChannels(tar, 1);

	std::vector<Point2f> srcPoints, tarPoints;

	goodFeaturesToTrack(src, srcPoints, maxCount, 0.01, 10, Mat(), 3, 3, 0, 0.04);
	cornerSubPix(src, srcPoints, subPixWinSize, Size(-1, -1), termcrit);

	vector<uchar> status;
	vector<float> err;

	calcOpticalFlowPyrLK(src, tar, srcPoints, tarPoints, status, err, winSize,
		3, termcrit, 0, 0.001);

	std::vector<PointPair> vpp;
	vpp.reserve(srcPoints.size());
	const Point2f roiOffset(float(roi.x), float(roi.y));
	for (size_t i = 0; i < srcPoints.size(); ++i)
		if (status[i] != 0)
		{
			vpp.push_back({ srcPoints[i] + roiOffset, tarPoints[i] + roiOffset, err[i] });

			/*Mat srcx = cv::convertBGRChannels(src, 3);
			Mat tarx = cv::convertBGRChannels(tar, 3);
			cv::circle(srcx, srcPoints[i], 3, Scalar(0, 255, 255), 1);
			cv::circle(tarx, tarPoints[i], 3, Scalar(0, 255, 255), 1);
			imshow("srcx", srcx);
			imshow("tarx", tarx);
			cv::waitKey();*/
		}
	return vpp;
}
void get3dPoints(CVRResult& rr, Size imgSize, std::vector<PointPair>& vpp)
{
	CVRProjector prj(rr, imgSize);
	for (auto& pp : vpp)
		pp.pt3d = prj.unproject(pp.src);
}
void solvePose(const std::vector<PointPair>& vpp, RigidPose& pose, Rect roi, const Matx33f &K)
{
	std::vector<Point2f>  imgPoints(vpp.size());
	std::vector<Point3f>  objPoints(vpp.size());
	for (size_t i = 0; i < vpp.size(); ++i)
	{
		imgPoints[i] += vpp[i].tar;
		objPoints[i] += vpp[i].pt3d;
	}
	cout << pose.t << endl;
	Mat rvec, tvec(pose.t);
	cv::Rodrigues(pose.R, rvec);
	//cv::solvePnPRefineVVS(objPoints, imgPoints, K, Mat(), rvec, tvec);
	cv::solvePnPRansac(objPoints, imgPoints, K, Mat(), rvec, tvec, true, 100, 3.f);
	pose = RigidPose(rvec, tvec);
	cout << pose.t << endl;
}

class BaseTracker
	:public CImpl
{
public:
	void loadModel(re3d::Model &model, const std::string& argstr)
	{
		model3d = model.get3DModel();
		bbox3d = model3d.getBoundingBoxCorners();
		render = CVRender(model3d);
	}
	void refine(const Mat& img, RigidPose& pose, const Matx33f& K)
	{
		Rect roi = getRenderROI(pose, K);
		const int BW = 50;
		rectAppend(roi, BW, BW, BW, BW);
		roi = rectOverlapped(roi, Rect(0, 0, img.cols, img.rows));

		CVRMats mats;
		mats.mModel = cvrm::fromR33T(pose.R, pose.t);
		mats.mProjection = cvrm::fromK(K, img.size(), 1, 3000);

		auto rr=render.exec(mats, img.size(), CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr, roi);
		imshow("rimg", rr.img);
		imshow("cimg", img(roi));

		auto vpp = findMatchKLT(rr.img, img(roi), roi);
		get3dPoints(rr, img.size(), vpp);

		solvePose(vpp, pose, roi, K);
	}
};

_IMPL_END()

using impl_base_tracker2::BaseTracker;

_VX_END()
