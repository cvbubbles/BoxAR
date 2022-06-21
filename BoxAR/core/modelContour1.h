#pragma once

#include"_std.h"
#include"Re3D/base.h"

class ModelContours1
	:public re3d::Model::ManagedObject
{
	DEFINE_RE3D_TYPE(ModelContours1)
private:
	struct DView
	{
		Vec3f  dir;
		std::vector<Point3f>  contourPoints;

		DEFINE_BFS_IO_2(DView, dir, contourPoints);
	};

	std::string         _sig= "MC100";
	std::vector<DView>  _views;
	Mat1i               _uvIndex;
	double              _du, _dv;
	Vec3f               _modelCenter;
private:
	static void _dir2uv(const Vec3f &dir, double &u, double &v)
	{
		v = acos(dir[2]);
		u = atan2(dir[1], dir[0]);
		if (u < 0)
			u += CV_PI * 2;
	}
	static Vec3f _uv2dir(double u, double v)
	{
		return Vec3f(cos(u)*sin(v), sin(u)*sin(v), cos(v));
	}
	static Mat1i _buildUVIndex(int nU, int nV, const std::vector<DView> &views, CVRMats &objMats, Size viewSize, int K = 5)
	{
		Mat1f viewDirs(Size(3, views.size()));
		for (size_t i = 0; i < views.size(); ++i)
		{
			memcpy(viewDirs.ptr(i), &views[i].dir, sizeof(float) * 3);
		}

		flann::Index  knn;
		knn.build(viewDirs, flann::KDTreeIndexParams(), cvflann::FLANN_DIST_L2);

		Mat1f uvDir(Size(3, nU*nV));

		double du = 2 * CV_PI / (nU - 1), dv = CV_PI / (nV - 1);

		for (int vi = 0; vi < nV; ++vi)
		{
			for (int ui = 0; ui < nU; ++ui)
			{
				double u = ui*du, v = vi*dv;
				Vec3f dir = _uv2dir(u, v);
				memcpy(uvDir.ptr(vi*nU + ui), &dir, sizeof(float) * 3);
			}
		}

		Mat1i indices;
		Mat1f dists;
		knn.knnSearch(uvDir, indices, dists, K);

		Mat1i uvi(nV, nU);
		for (int vi = 0; vi < nV; ++vi)
		{
			for (int ui = 0; ui < nU; ++ui)
			{
				double u = ui*du, v = vi*dv;
				Vec3f dir = _uv2dir(u, v);
				objMats.mModel = cvrm::rotate(dir, Vec3f(0, 0, 1));

				CVRProjector prj(objMats, viewSize);

				const int *vnn = indices.ptr<int>(vi*nU + ui);
				int mi = -1;
				double maxArea = 0;
				for (int j = 0; j < K; ++j)
				{
					std::vector<Point2f>  ppt;
					prj.project(views[vnn[j]].contourPoints, ppt);
					double area = contourArea(ppt);
					if (area > maxArea)
					{
						maxArea = area;
						mi = vnn[j];
					}
				}
				uvi(vi, ui) = mi;

				printf("build uvIndex %d/%d    \r", vi*nU + ui, nU*nV);
			}
		}
		return uvi;
	}
	
public:
	void build(CVRModel &model, int nViews, int nU = 200, int nV = 100, int uvK = 5)
	{
		Size viewSize(500, 500);

		CVRender render(model);
		CVRMats objMats(model, viewSize, 3, 8);
		//objMats.mProjection = cvrm::ortho(-1, 1, -1, 1, 1, 30);

		std::vector<Vec3f> dirs;
		cvrm::sampleSphere(dirs, nViews);

		std::vector<DView>  views(nViews);

		for (int i = 0; i < nViews; ++i)
		{
			objMats.mModel = cvrm::rotate(dirs[i], Vec3f(0, 0, 1));

			time_t beg = clock();
			CVRResult rr = render.exec(objMats, viewSize, CVRM_DEPTH | CVRM_IMAGE, 0);

			//imshow("img", rr.img);
			//waitKey();

			auto &vi(views[i]);
			getRenderContour(rr, vi.contourPoints);
			vi.dir = dirs[i];
			printf("build %d/%d time=%d    \r", i, nViews, int(clock() - beg));
		}
		printf("\n");

		_views.swap(views);
		_uvIndex = _buildUVIndex(nU, nV, _views, objMats, viewSize, uvK);
		_du = 2 * CV_PI / (nU - 1);
		_dv = CV_PI / (nV - 1);
		_modelCenter = model.getCenter();
	}

	void getContoursUV(Vec3f dir, std::vector<Point3f> &vpt, CVRResult &rr)
	{
		dir = normalize(dir);

		double u, v;
		_dir2uv(dir, u, v);
		int ui = int(u / _du + 0.5), vi = int(v / _dv + 0.5);
		vpt = _views[_uvIndex(vi, ui)].contourPoints;
	}
	const std::vector<Point3f>& getContour(const Vec3f &dir) const
	{
		CV_Assert(fabs(dir.dot(dir) - 1) < 1e-6f);

		double u, v;
		_dir2uv(dir, u, v);
		int ui = int(u / _du + 0.5), vi = int(v / _dv + 0.5);
		return _views[_uvIndex(vi, ui)].contourPoints;
	}

	Vec3f getCenter() const
	{
		return _modelCenter;
	}
	size_t getMaxContourLength() const
	{
		size_t maxL = 0;
		for (auto &v : _views)
			if (v.contourPoints.size() > maxL)
				maxL = v.contourPoints.size();
		return maxL;
	}

	const std::vector<Point3f>& getContour(const cv::Matx44f &mModelView, bool isRigid=true) const
	{
		Vec3f dv=getOpticalAxis(mModelView,this->getCenter(),isRigid);
		dv = normalize(dv);
		return getContour(dv);
	}

	
	DEFINE_BFS_OUTPUT_4(ModelContours1, _sig, _uvIndex, _views, _modelCenter)

	template<typename _IBST>
	void _read(_IBST &ibs)
	{
		ibs >> _sig >> _uvIndex >> _views >> _modelCenter;

		_du = 2 * CV_PI / (_uvIndex.cols - 1);
		_dv = CV_PI / (_uvIndex.rows - 1);
	}
	template<typename _IBST>
	friend void BSRead(_IBST &ibs, ModelContours1 &v)
	{
		v._read(ibs);
	}

	void save(const std::string &file)
	{
		ff::OBFStream os(file);
		os << *this;
	}
	void load(const std::string &file)
	{
		ff::IBFStream is(file);
		is >> *this;
	}
	
	const char* streamName() const
	{
		return "contour1";
	}
	void dump(re3d::ModelData &modelData)
	{
		modelData.getStream(streamName(), true) << *this;
	}
	void load(re3d::ModelData &modelData)
	{
		modelData.getStream(streamName()) >> *this;
	}

	virtual void onCreate(re3d::Model &model)
	{
		auto &modelData = model.getData();

		ff::BMStream *stream = modelData.queryStream(this->streamName());
		if (!stream)
		{
			stream = &modelData.getStream(this->streamName(), true);
			this->build(model.get3D(), 3000);
			(*stream) << *this;
			modelData.save();
		}
		else
			(*stream) >> *this;
	}
};

