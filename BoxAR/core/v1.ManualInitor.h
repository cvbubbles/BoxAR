#pragma once

#include"vxbase.h"
#include"BFC/log.h"

_VX_BEG(v1)

namespace vx = v1;

_IMPL_BEG(impl_manual)

class DModel
{
public:
	CVRModel               model;
	CVRender               render;
	RigidPose              initPose;
};

class ManualInitor
	:public re3d::FrameProc
{
	DEFINE_RE3D_TYPE2(ManualInitor, "v1.ManualInitor")
private:

	ModelSet* _modelSet;
	std::vector<DModel>  _models;
	Param  _param;
	
	bool  _isInitMode;
public:
	virtual void init(ModelSet* modelSet, ArgSet* args)
	{
		CV_Assert(args);
		if (!args || !args->query("initPose"))
		{
			printf("error : initPose needs to be specified");
		}

		bool preload = false;
		RigidPose initPose;
		if (args)
		{
			std::vector<float> v;
			args->getv("initPose", v);
			CV_Assert(v.size() == 6);
			initPose = RigidPose(Vec3f(v[0], v[1], v[2]), Vec3f(v[3], v[4], v[5]));

			preload = args->getd<bool>("preload", false);
		}

		_modelSet = modelSet;

		//if (preload)
		{
			std::vector<DModel> models(modelSet->size());

			initPose.score = 1000.f;
			for (int i = 0; i < modelSet->size(); ++i)
			{
				auto md = _modelSet->getModel(i);

				models[i].model = verify(md)->get3DModel();
				models[i].render = CVRender(models[i].model);
				models[i].initPose = initPose;
			}
			_models.swap(models);
		}
		_isInitMode = true;

		cv::namedWindow("initPose");
		
	}

	virtual int pro(const Mat& img, FrameData& fd, ff::ArgSet*)
	{
		fd.objs.clear();

		if (fd.query("waitCode"))
		{
			int waitCode = fd.get<int>("waitCode");
			waitCode = toupper(waitCode);
			if (waitCode == ' ')
			{
				if (_isInitMode)
				{
					for (int i = 0; i < (int)_models.size(); ++i)
					{
						auto& obj = _models[i];

						{
							re3d::ImageObject r3d;
							r3d.modelIndex = i;
							r3d.pose.make <  std::vector < RigidPose > >() = { obj.initPose };
							r3d.score = obj.initPose.score;

							fd.objs.push_back(r3d);
						}
					}
					//cv::destroyWindow("initPose");
				}
				_isInitMode = !_isInitMode;
			}
		}

		if (_isInitMode)
		{
			Mat dimg = cv::convertBGRChannels(img, 3);

			CVRMats mats;
			mats.mProjection = cvrm::fromK(fd.cameraK, dimg.size(), 0.1, 1000);
			Scalar color(0, 255, 255);

			for (int i = 0; i < (int)_models.size(); ++i)
			{
				auto& obj = _models[i];

				mats.mModel = cvrm::fromR33T(obj.initPose.R, obj.initPose.t);
				auto rr=obj.render.exec(mats, dimg.size(), CVRM_DEPTH);
				Mat1b mask=rr.getMaskFromDepth();
				
				std::vector<std::vector<Point>> contours;
				cv::findContours(mask, contours, RETR_LIST, CHAIN_APPROX_NONE);
				cv::drawContours(dimg, contours, -1, color, 2);
			}
			imshow("initPose", dimg);
		}


		return (int)fd.objs.size();
	}
};

_IMPL_END()

using impl_manual::ManualInitor;

REGISTER_RE3D_TYPE(ManualInitor, 0)

_VX_END()

