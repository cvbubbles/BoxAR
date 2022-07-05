#pragma once

#include"v1.BaseTracker1.h"
#include"BFC/thread.h"
#include<deque>

_VX_BEG(v1)

_IMPL_BEG(impl_tracker)

typedef BaseTracker1  BaseTracker;

class PoseScore
{
	enum{N_LAYERS=2};

	Mat2f         _grad;
public:
	PoseScore(const Mat& img)
	{
		Mat1b gray = cv::convertBGRChannels(img, 1);

		Mat1f dx, dy;

		for (int l = 0; l < N_LAYERS; ++l)
		{
			Mat1f ldx, ldy;
			cv::Sobel(gray, ldx, CV_32F, 1, 0);
			cv::Sobel(gray, ldy, CV_32F, 0, 1);

			if (l != 0)
			{
				dx += imscale(ldx, img.size(), INTER_LINEAR);
				dy += imscale(ldy, img.size(), INTER_LINEAR);
			}
			else
			{
				dx = ldx; dy = ldy;
			}
			if (l != N_LAYERS - 1)
				gray = imscale(gray, 0.5);
		}
		_grad=cv::mergeChannels(dx, dy);
	}
	float getScore(BaseTracker *baseTracker, const RigidPose& pose, const Matx33f &K, float dotT=0.9f)
	{
		std::vector<Point2f>  points, normals;
		baseTracker->getProjectedContours(K, pose, points, &normals);

		float wsum = 1e-6, score=0;
		int   nmatch = 0;
		for (size_t i = 0; i < points.size(); ++i)
		{
			int x = int(points[i].x + 0.5f), y = int(points[i].y + 0.5f);
			if (uint(x) < uint(_grad.cols) && uint(y) < uint(_grad.rows))
			{
				const float* g = _grad.ptr<float>(y, x);
				float w = sqrt(g[0] * g[0] + g[1] * g[1]) + 1e-6f;
				float dot = (g[0] * normals[i].x + g[1] * normals[i].y) / w;
				dot = fabs(dot);
				if (dot > dotT)
				{
					score += dot * w;
					wsum += w;
					nmatch++;
				}
			}
		}
		return score / wsum * (float(nmatch) / float(points.size()));
	}
};

class DModel
{
public:
	BaseTracker  baseTracker;
	//BaseTracker2 baseTracker2;
public:
	//void refine(const Mat &img, re3d::ImageObject& obj, const Matx33f &K)
	//{
	//	std::vector<re3d::RigidPose> &pose = obj.pose.get<std::vector<re3d::RigidPose>>();
	//	if (pose.empty())
	//		return;
	//	CV_Assert(pose.size() == 1);

	//	baseTracker.refine(img, pose[0], K);

	//	/*PoseScore  poseScore(img, &this->baseTracker);
	//	float score=poseScore.getScore(pose[0], K);
	//	printf("score=%.2f\n", score);
	//	obj.score = score;*/
	//}
};

class DPose
{
public:
	int        imodel;
	RigidPose  pose;
	float      score;
public:
	void get(re3d::ImageObject& o)
	{
		o.modelIndex = imodel;
		o.pose.make < std::vector < RigidPose >>() = {pose};
		o.score = score;
	}
};

class Frame
{
public:
	Mat					 img;
	std::vector<DPose>   poseTracked;
	std::vector<DPose>   poseDetected;
	bool        locked=false;
};

class Tracker
	:public re3d::FrameProc
{
	DEFINE_RE3D_TYPE2(Tracker, "v1.Tracker")
private:
	ModelSet* _modelSet;
	std::shared_ptr<re3d::FrameProc>  _detector;
	std::vector<DModel>               _models;
	std::deque<Frame>                 _frames;
	ff::Thread                        _bgThread;
	Frame* _detectedFrame=nullptr;
public:
	virtual void init(ModelSet* modelSet, ArgSet* args)
	{
		_detector = re3d::FrameProc::create("v1.Detector");
		_detector->init(modelSet, args);
		_modelSet = modelSet;
		{
			std::vector<DModel> models(modelSet->size());

			for (int i = 0; i < modelSet->size(); ++i)
			{
				auto md = _modelSet->getModel(i);

				models[i].baseTracker.loadModel(*md, "");

				//models[i].baseTracker2.loadModel(*md, "");
			}
			_models.swap(models);
		}
		_frames.clear();
	}
	void _detect(Frame &frame, Matx33f K)
	{
		FrameData fd;
		fd.cameraK = K;
		
		_detector->pro(frame.img, fd);

		frame.poseDetected.clear();
		for (auto& v : fd.objs)
		{
			auto pose=v.pose.get<std::vector<RigidPose>>();
			if (!pose.empty())
				frame.poseDetected.push_back({ v.modelIndex, pose.front() });
		}
		_detectedFrame = &frame;
	}
	void _detectedToCurFrame(Frame &cur, const Matx33f &K, PoseScore &poseScore)
	{
		if (_detectedFrame)
		{
			cur.poseDetected.clear();

			for (auto& obj : _detectedFrame->poseDetected)
			{
				int imodel = obj.imodel;
				RigidPose tarPose;
				_models[imodel].baseTracker.track(_detectedFrame->img,obj.pose,cur.img, tarPose, K);
				float score = poseScore.getScore(&_models[imodel].baseTracker, tarPose, K);
				cur.poseDetected.push_back({ imodel, tarPose, score });
			}
			_detectedFrame->locked = false;
			_detectedFrame = nullptr;
		}
	}

	int pro1(const Mat& img, FrameData& fd, ff::ArgSet* args)
	{
		_detector->pro(img, fd, args);

		return fd.objs.size();
	}
	
	virtual int pro(const Mat& img, FrameData& fd, ff::ArgSet* args)
	{
		_frames.push_back(Frame());
		auto& cur = _frames.back();
		cur.img = img.clone();
		while (_frames.size() > 30 && !_frames.front().locked)
			_frames.pop_front();

		bool waitBg = false;
		PoseScore poseScore(img);

		if (_bgThread.isIdle())
		{
			if (!_detectedFrame)
			{
				cur.locked = true;
				_bgThread.post([this, &fd, &cur]() {
					this->_detect(cur, fd.cameraK);
					});
			}
			else
			{
				_bgThread.post([this, &cur, &fd, &poseScore]() {
					this->_detectedToCurFrame(cur, fd.cameraK, poseScore);
					});
				waitBg = true;
			}
		}

		if (_frames.size()>1)
		{
			auto& prev = _frames[_frames.size() - 2];

			for (int i = 0; i < (int)prev.poseTracked.size(); ++i)
			{
				int imodel = prev.poseTracked[i].imodel;
				RigidPose pose;
				if (_models[imodel].baseTracker.track(img, pose, fd.cameraK))
				{
					//_models[imodel].baseTracker2.refine(img, pose, fd.cameraK);

					float score = poseScore.getScore(&_models[imodel].baseTracker, pose, fd.cameraK);

					cur.poseTracked.push_back({ imodel, pose, score});
				}
			}
		}
		if (waitBg)
		{
			_bgThread.waitAll();
			for (auto& d : cur.poseDetected)
			{
				DPose* t = nullptr;
				for (auto& x : cur.poseTracked)
				{
					if (d.imodel == x.imodel)
					{
						t = &x; break;
					}
				}
				//detection is better
				if (t && t->score+0.05 < d.score || !t)
				//if(!t)
				{
					//if (t)
					//	printf("reset...%.2f>%.2f \n", d.score, t->score);

					if (t)
						*t = d;
					else
						cur.poseTracked.push_back(d);

					_models[d.imodel].baseTracker.reset(cur.img, d.pose, fd.cameraK);
				}
			}
		}

		fd.objs.resize(cur.poseTracked.size());
		for (int i = 0; i < fd.objs.size(); ++i)
		{
			cur.poseTracked[i].get(fd.objs[i]);
		}

		return (int)fd.objs.size();
	}
};

_IMPL_END()

using impl_tracker::Tracker;
REGISTER_RE3D_TYPE(Tracker, 0)

_VX_END()

