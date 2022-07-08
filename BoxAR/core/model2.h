#pragma once

#include"modelContour1.h"

namespace _model1 {

	struct ModelParam
	{
		int  nSampleViews = 300;

		int  imageSize=800; //800
		int  fdNLevels = 12; //12
		int  fdNFeaturesPerImage = 500;
		float fdLevelScale = 1.2f;
	};

	static ModelParam  param;

	//static const float LEVEL_SCALE = 1.2f;

	inline void detectOnModel(const Mat3b &img, std::vector<double> &vScale, std::vector<std::vector<KeyPoint>> &kp, std::vector<Mat> &desc, Mat1b mask = Mat1b())
	{
		int nLevels = param.fdNLevels, nKP = param.fdNFeaturesPerImage;
		double scale = param.fdLevelScale;

		Mat1b gray;
		cvtColor(img, gray, CV_BGR2GRAY);

		kp.clear();
		kp.resize(nLevels);
		desc.clear();
		desc.resize(nLevels);
		vScale.resize(nLevels);

		Mat1b grayL = gray.clone(), maskL = mask.clone();

		for (int i = 0; i < nLevels; ++i)
		{
			auto fd = cv::ORB::create(nKP, 1.2, 1);

			fd->detectAndCompute(grayL,maskL,kp[i], desc[i]);
			vScale[i] = double(grayL.cols) / img.cols;

			if (i < nLevels - 1)
			{
				Size dsize(int(grayL.cols / scale + 0.5), int(grayL.rows / scale + 0.5));
				resize(gray, grayL, dsize, 0, 0, INTER_LINEAR);
				if (!mask.empty())
					resize(mask, maskL, dsize, 0, 0, CV_INTER_NN);
			}
			/*else
			{
				imshow("gray", grayL);
				waitKey();
			}*/
		}
	}

	struct DView
	{
		CVRResult rr;

		std::vector<std::vector<KeyPoint>> kp;
		std::vector<std::vector<Point3f>>  kp3;
		std::vector<Mat> desc;
	public:
		void detect()
		{
			std::vector<double> scales;

			CV_Assert(rr.img.type() == CV_8UC3);
			detectOnModel(rr.img, scales, kp, desc, getRenderMask(rr.depth));

			CVRProjector prj(rr);
			kp3.resize(kp.size());

			for (int level = 0; level < (int)kp.size(); ++level)
			{
				auto &kp3L = kp3[level];
				auto &kpL = kp[level];
				float scale = float(1.0 / scales[level]);

				kp3L.resize(kpL.size());
				for (size_t i = 0; i < kpL.size(); ++i)
				{
					kp3L[i] = prj.unproject(kpL[i].pt.x*scale, kpL[i].pt.y*scale);
				}
			}
		}
	};

	struct DPoint
	{
		Point3f pt;
		float  response;
		float density;
		float  size;
	};

	inline void getPoints(const std::vector<DView> &views, int level, std::vector<DPoint> &pts, Mat &desc)
	{
		size_t npt = 0;
		for (auto &v : views)
			npt += v.kp3[level].size();

		pts.resize(npt);

		desc.create(npt, views.front().desc[level].cols, views.front().desc[level].type());
		int descRowSize = desc.elemSize()*desc.cols;

		size_t k = 0;
		for (auto &v : views)
		{
			auto &kp = v.kp[level];
			auto &kp3 = v.kp3[level];

			for (int i = 0; i < kp3.size(); ++i)
			{
				auto &px(pts[k]);
				px.pt = kp3[i];
				px.response = kp[i].response;
				px.size = kp[i].size;
				memcpy(desc.ptr(k), v.desc[level].ptr(i), descRowSize);
				++k;
			}
		}
	}
	inline void getPoints(const std::vector<DView> &views, std::vector<std::vector<DPoint>> &pts, std::vector<Mat> &desc)
	{
		size_t nLevels = views.front().kp3.size();

		pts.resize(nLevels);
		desc.resize(nLevels);

		for (size_t i = 0; i < nLevels; ++i)
			getPoints(views, (int)i, pts[i], desc[i]);
	}
	inline void selPoints(const std::vector<DPoint> &pts, const Mat &ptsDesc, std::vector<DPoint> &dsel, Mat &selDesc, float dmax, int K)
	{
		//int nSel = 10000;

		printf("build index...\n");
		FMIndex knn;
		knn.build(ptsDesc, FMIndex::LSH);

		printf("knn search...\n");
		Mat1i indices;
		Mat1i dists;
		knn.knnSearch(ptsDesc, indices, dists, K);

		dmax *= dmax;

		struct DPt
		{
			int id;
			float nm;
		};
		std::vector<DPt>  dpt(pts.size());
		for (int i = 0; i < (int)pts.size(); ++i)
		{
			const int *vi = indices.ptr<int>(i);
			int nm = 0;
			for (int j = 0; j < K; ++j)
			{
				if(uint(vi[j])<pts.size())
				{
					Vec3f dv = pts[i].pt - pts[vi[j]].pt;
					if (dv.dot(dv) < dmax)
						++nm;
				}
			}
			dpt[i] = { i,(float)nm };
		}
		std::sort(dpt.begin(), dpt.end(), [](const DPt &a, const DPt &b) {
			return a.nm > b.nm;
		});

		dsel.clear();

#if 0
		for (auto &p : dpt)
			if (p.nm > K / 2)
				dsel.push_back(pts[p.id]);
#else
		std::vector<char>  mask(dpt.size(), 0);

		CV_Assert(ptsDesc.type() == CV_8UC1);
		std::unique_ptr<uchar[]> _vdesc(new uchar[ptsDesc.rows*ptsDesc.cols]);
		uchar *vdesc = _vdesc.get();
		memset(vdesc, 0, ptsDesc.rows*ptsDesc.cols);

		for (int i = 0; i < (int)dpt.size(); ++i)
		{
			if (dpt[i].nm < K*0.5f)
			//if(dsel.size()>=nSel)
				break;

			int pid = dpt[i].id;
			if (mask[pid] == 0)
			{
				const int *vi = indices.ptr<int>(pid);
				for (int j = 0; j < K; ++j)
				{
					if(uint(vi[j])<pts.size())
					{
						Vec3f dv = pts[pid].pt - pts[vi[j]].pt;
						if (dv.dot(dv) < dmax)
							mask[vi[j]] = 1;
					}
				}
				dsel.push_back(pts[pid]);

				memcpy(vdesc, ptsDesc.ptr(pid), sizeof(uchar)*ptsDesc.cols);
				vdesc += ptsDesc.cols;
			}
		}
#endif

		selDesc.create(dsel.size(), ptsDesc.cols, ptsDesc.type());
		memcpy(selDesc.data, _vdesc.get(), selDesc.rows*selDesc.cols * sizeof(uchar));
	}

	inline void _printSizes(const std::vector<DPoint> &vpts)
	{
		struct DX
		{
			float size;
			int   n;
		};
		std::vector<DX>  vsizes;
		for (auto &p : vpts)
		{
			size_t i = 0;
			for (; i<vsizes.size(); ++i)
				if (vsizes[i].size == p.size)
				{
					vsizes[i].n++;
					break;
				}
			if (i == vsizes.size())
			{
				vsizes.push_back({ p.size,1 });
			}
		}
		std::sort(vsizes.begin(), vsizes.end(), [](const DX &a, const DX &b) {
			return a.size < b.size;
		});
		printf("\n");
		for (auto &s : vsizes)
			printf("%.2f(%d) ", s.size, s.n);
		printf("\n");
	}

	inline void proModel(std::vector<DView> &views, CVRModel &model, std::vector<Point3f> &modelPoints, std::vector<int> &vLevel, cv::Mat &desc, int selPtK = 20, int nViews = 300, Size viewSize = Size(800, 800))
	{
		for (int i = 0; i < (int)views.size(); ++i)
		{
			views[i].detect();
			printf("%d/%d \r", i + 1, views.size());
		}

		std::vector<std::vector<DPoint>> pts;
		std::vector<Mat> ptsDesc;
		getPoints(views, pts, ptsDesc);
		//views.clear();

		printf("nLevels =%d\n", (int)pts.size());

		//showModelPoints("model", model, pts);

		Vec3f sizeBB = model.getSizeBB();
		float dmax = sqrt(sizeBB.dot(sizeBB)) / 200;
		//printf("dmax=%f\n", dmax);

		//_printSizes(pts);
		size_t nLevels = pts.size();

		std::vector<std::vector<DPoint>> ptSel(nLevels);
		std::vector<Mat>                 descSel(nLevels);
		size_t nSelPts = 0;

		for (size_t i = 0; i < nLevels; ++i)
		{
			selPoints(pts[i], ptsDesc[i], ptSel[i], descSel[i], dmax, selPtK);
			nSelPts += ptSel[i].size();

			printf("level %d : selected points=%d/%d\n", (int)i, (int)ptSel[i].size(),(int)pts[i].size());
		}
		printf("total sel=%d\n", (int)nSelPts);

		modelPoints.resize(nSelPts);
		vLevel.resize(nSelPts);
		desc.create(nSelPts, descSel[0].cols, descSel[0].type());
		size_t n = 0;
		for (size_t i = 0; i < ptSel.size(); ++i)
		{
			if(descSel[i].rows>0)
			{
				cv::Mat roi = desc(Rect(0, n, desc.cols, descSel[i].rows));
				copyMem(descSel[i], roi);

				for (auto &p : ptSel[i])
				{
					modelPoints[n] = p.pt;
					vLevel[n] = (int)i;
					++n;
				}
			}
		}
	}

	struct Builder
	{
		std::vector<DView> views;
		std::vector<Vec3f> viewVecs;

	public:
		void renderViews(CVRModel &model, int nViews, Size viewSize)
		{
			CVRender render(model);
			CVRMats objMats(model, viewSize);

			cvrm::sampleSphere(viewVecs, nViews);

			views.clear();
			views.resize(nViews);
			for (int i = 0; i < nViews; ++i)
			{
				objMats.mModel = cvrm::rotate(viewVecs[i], Vec3f(0, 0, 1));
				views[i].rr = render.exec(objMats, viewSize);
				//imshow("rr", views[i].rr.img);
				//waitKey();
				printf("render %d/%d   \r", i + 1, nViews);
			}
		}
		void build(CVRModel &model, std::vector<Point3f> &modelPoints, std::vector<int> &vLevel, cv::Mat &desc, int selPtK = 20, int nViews = 300, Size viewSize = Size(800, 800))
		{
			renderViews(model, nViews, viewSize);
			proModel(views, model, modelPoints, vLevel, desc, selPtK, nViews, viewSize);
		}
	};

	struct PointCluster
	{
		Point3f center;
		int     level;
		std::vector<int>   vpts;

	public:
		DEFINE_BFS_IO_3(PointCluster,center,level,vpts)
	};

	class ClusterIndex
	{
	public:
		struct _View
		{
			Vec3f   dir;
			float   maxSize;
			std::vector<std::vector<int> >  levels;
		public:
			DEFINE_BFS_IO_3(_View,dir,maxSize,levels)
		public:
			void estimateLevel(int &startLevel, int &endLevel, float objSize, int hwsz, float levelScale = param.fdLevelScale)
			{
				float _level = objSize > maxSize ? 0 : log(maxSize / objSize) / log(levelScale);
				int level = int(_level + 0.5f);
				int nLevels = (int)levels.size() - 1;
				if (level >= nLevels)
					level = nLevels - 1;

				startLevel = __max(0, level - hwsz);
				endLevel = __min(nLevels-1, level + hwsz);
			}
		};
	private:
		std::vector<_View>  _views;
		Mat1i               _uvIndex;
		double              _du, _dv;
	public:
		DEFINE_BFS_IO_4(ClusterIndex,_views,_uvIndex,_du,_dv)
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
		static Mat1i _buildUVIndex(int nU, int nV, const std::vector<_View> &views)
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
			knn.knnSearch(uvDir, indices, dists, 1);

			Mat1i uvi(nV, nU);
			for (int vi = 0; vi < nV; ++vi)
			{
				for (int ui = 0; ui < nU; ++ui)
				{
					const int *vnn = indices.ptr<int>(vi*nU + ui);
					uvi(vi, ui) = vnn[0];
				}
			}
			return uvi;
		}
	public:
		static float _getMaxBBSize(const Mat1f &depth)
		{
			Mat1b mask=getRenderMask(depth);
			Rect roi=get_mask_roi(DWHS(mask), 127);
			return (float)sqrt(double(roi.width*roi.width + roi.height*roi.height));
		}
		void build(Builder &buildData, int nLevels, const std::vector<PointCluster> &pointClusters, int nU=100, int nV=50)
		{
			int nViews = (int)buildData.views.size();

			_views.clear();
			_views.resize(nViews);

			//for (auto &v : buildData.views)
			for (int vi = 0; vi < nViews; ++vi)
			{
				auto &v(buildData.views[vi]);
				auto &vx(_views[vi]);

				CVRResult &rr(v.rr);
				CVRProjector prj(rr);

				vx.levels.resize(nLevels);
				for (int ci = 0; ci < (int)pointClusters.size(); ++ci)
				{
					auto &c = pointClusters[ci];
					Point3f p=prj.project(c.center);
					if (rr.isVisible(p, 1e-3f))
						vx.levels[c.level].push_back(ci);
				}
				vx.maxSize = _getMaxBBSize(rr.depth);
				vx.dir = buildData.viewVecs[vi];
			}

			_uvIndex = _buildUVIndex(nU, nV, _views);
			_du = 2 * CV_PI / (nU - 1);
			_dv = CV_PI / (nV - 1);
		}
		_View &getView(const Vec3f &dir)
		{
			CV_Assert(fabs(dir.dot(dir) - 1) < 1e-6f);

			double u, v;
			_dir2uv(dir, u, v);
			int ui = int(u / _du + 0.5), vi = int(v / _dv + 0.5);

			int size = sizeof(_views);

			return _views[_uvIndex(vi, ui)];
		}
		void getSubset(std::vector<int> &selClusters, const Vec3f &dir, Size objSize, int levelHWSZ=1)
		{
			auto &view = getView(dir);

			int startLevel, endLevel;

			float objSize1 = sqrt(float(objSize.width*objSize.width + objSize.height*objSize.height));
			view.estimateLevel(startLevel, endLevel, objSize1,levelHWSZ);
			
			size_t nsel = 0;
			for (int i = startLevel; i <= endLevel; ++i)
				nsel += view.levels[i].size();

			selClusters.resize(nsel);
			nsel = 0;

			for (int i = startLevel; i <= endLevel; ++i)
			{
				if (!view.levels[i].empty())
				{
					memcpy(&selClusters[nsel], &view.levels[i][0], sizeof(int)*view.levels[i].size());
					nsel += view.levels[i].size();
				}
			}
		}
	};
}

class ModelPoints2
	:public re3d::Model::ManagedObject
{
	DEFINE_RE3D_TYPE(ModelPoints2)
public:
	std::string           sig = "MP121";
	std::vector<Point3f>  modelPts;
	Mat                   ptsDesc;
	std::vector<int>      vLevels;
	int					  nLevels;

	typedef _model1::PointCluster  PointCluster;

	std::vector<PointCluster>  clusters;
	std::vector<int>           ptsCluster;
	_model1::ClusterIndex      clusterIndex;
	Point3f                    modelCenter;
private:
	FMIndex               matcher;
	std::vector<int>      matcherSubsetIndex;
	Mat                   matcherSelDesc;
public:
	void findMatch(const Mat &queryDesc, std::vector<DMatch> &matches)
	{
		matcher.match(queryDesc, matches);
		if (!matcherSubsetIndex.empty())
		{
			for (auto &m : matches)
			{
				if (uint(m.trainIdx) < matcherSubsetIndex.size())
					m.trainIdx = matcherSubsetIndex[m.trainIdx];
			}
		}
	}
public:
	template<typename _OST>
	void _write(_OST &os) const
	{
		os << sig << modelPts << vLevels << ptsDesc;
		os << clusters << ptsCluster << clusterIndex << modelCenter;
	}
	template<typename _OST>
	friend void BSWrite(_OST &os, const ModelPoints2 &v)
	{
		v._write(os);
	}
	template<typename _IBST>
	void _read(_IBST &is)
	{
		is >> sig >> modelPts >> vLevels >> ptsDesc;

		if (sig > "MP120")
		{
			is >> clusters >> ptsCluster >> clusterIndex >> modelCenter;
		}
	}
	template<typename _IBST>
	friend void BSRead(_IBST &ibs, ModelPoints2 &v)
	{
		v._read(ibs);
	}

	void save(const std::string &file)
	{
		ff::OBFStream os(file);
		os << *this;
	}
	void load(const std::string &file, int startLevel=0, int endLevel=-1)
	{
		ff::IBFStream is(file);
		is >> *this;
		nLevels = *std::max_element(vLevels.begin(), vLevels.end()) + 1;
		_buildMatcher(startLevel, endLevel);
	}
#undef _SIG

	const char* streamName() const
	{
		return "points2";
	}
	void dump(re3d::ModelData &modelData)
	{
		modelData.getStream(streamName(), true) << *this;
	}
	void load(re3d::ModelData &modelData)
	{
		modelData.getStream(streamName()) >> *this;

		nLevels = *std::max_element(vLevels.begin(), vLevels.end()) + 1;
		_buildMatcher(0, -1);
	}

private:

	void _getSubsetWithSeed(const std::vector<int> &seedPoints, std::vector<int> &selPoints)
	{
		std::vector<int>  cmask(clusters.size(), 0);
		for (auto i : seedPoints)
		{
			cmask[ptsCluster[i]]++;
		}
		selPoints.clear();
		for (size_t i = 0; i < cmask.size(); ++i)
		{
			if (cmask[i] != 0)
			{
				selPoints.insert(selPoints.end(),clusters[i].vpts.begin(), clusters[i].vpts.end());
			}
		}
	}
	void _getSubsetOfClusters(const std::vector<int> &clusterIdx, std::vector<int> &selPoints)
	{
		selPoints.clear();
		selPoints.reserve(modelPts.size());
		for (auto i : clusterIdx)
			selPoints.insert(selPoints.end(), clusters[i].vpts.begin(), clusters[i].vpts.end());
	}
	void _getSubsetData(const std::vector<int> &selIdx, std::vector<Point3f> &selPoints, Mat &selDesc)
	{
		selPoints.resize(selIdx.size());
		selDesc.create((int)selIdx.size(), ptsDesc.cols, ptsDesc.type());

		int descRowSize = ptsDesc.elemSize()*ptsDesc.cols;
		for (int i = 0; i < (int)selIdx.size(); ++i)
		{
			int idx = (int)selIdx[i];
			selPoints[i] = modelPts[idx];
			memcpy(selDesc.ptr(i),ptsDesc.ptr(idx),descRowSize);
		}
	}

	struct Point
	{
		Point3f pt;
		int     idx;
	};
	void _clusterLevelPoints(int level, const std::vector<Point> &vpts, std::vector<int> &ptsCluster, std::vector<PointCluster> &clusters, float dMax)
	{
		int ibeg = (int)clusters.size(), iend = ibeg;
		dMax *= dMax;

		for (auto &p : vpts)
		{
			float dmin = FLT_MAX;
			int   imin = -1;
			for (int i = ibeg; i < iend; ++i)
			{
				Vec3f dv = p.pt - clusters[i].center;
				float d = dv.dot(dv);
				if (d < dmin)
				{
					dmin = d; imin = i;
				}
			}
			if (dmin > dMax)
			{
				clusters.push_back(PointCluster());
				auto &vx = clusters.back();
				vx.center = Point3f(0, 0, 0);
				vx.level = level;

				imin = iend;
				iend++;
			}

			auto &cm = clusters[imin];
			cm.center = (cm.center*(float)cm.vpts.size() + p.pt) * (1.0f / (cm.vpts.size() + 1));
			cm.vpts.push_back(p.idx);

			ptsCluster[p.idx] = imin;
		}
	}
	void _clusterPoints()
	{
		float dMax = 10;

		{
			Point3f cmin, cmax;
			getBoundingBox3D(modelPts, cmin, cmax);
			Vec3f dv = cmax - cmin;
			dMax = sqrt(dv.dot(dv)) / 10;
		}

		std::vector<std::vector<Point>> levelPts(nLevels);
		for (auto &v : levelPts)
			v.reserve(modelPts.size());

		for (size_t i = 0; i < modelPts.size(); ++i)
		{
			levelPts[vLevels[i]].push_back({ modelPts[i],(int)i });
		}

		clusters.clear();
		clusters.reserve(2000);
		ptsCluster.clear();
		ptsCluster.resize(modelPts.size(), -1);
		//for (auto &v : levelPts)
		for (int i = 0; i < (int)levelPts.size(); ++i)
		{
			_clusterLevelPoints(i, levelPts[i], ptsCluster, clusters, dMax);
		}
		printf("ncluster=%d\n", (int)clusters.size());
	}

	void _buildMatcher(int startLevel=0, int endLevel=-1)
	{
		if (endLevel < startLevel)
			endLevel = nLevels - 1;

		if(endLevel-startLevel<nLevels-1)
		{
			std::vector<std::vector<int>> vLevelPts(nLevels);
			for (int i = 0; i < (int)vLevels.size(); ++i)
				vLevelPts[vLevels[i]].push_back(i);

			int nSelTotal = 0;
			for (int i=startLevel; i <= endLevel; ++i)
			{
				nSelTotal += vLevelPts[i].size();
			}

			std::vector<Point3f> selPts(nSelTotal);
			std::vector<int>  selLevels(nSelTotal);
			Mat  selDesc(nSelTotal,ptsDesc.cols,ptsDesc.type());
			matcherSubsetIndex.resize(nSelTotal);

			const int descRowSize = ptsDesc.cols*ptsDesc.elemSize();
			int k = 0;
			//for (auto &v : vLevelPts)
			for(int level=startLevel; level<=endLevel; ++level)
			{
				auto &v = vLevelPts[level];
				int nsel = v.size();
				for (int j = 0; j < nsel; ++j)
				{
					int i = v[j];
					selPts[k] = modelPts[i];
					selLevels[k] = level;
					memcpy(selDesc.ptr(k), ptsDesc.ptr(i), descRowSize);
					matcherSubsetIndex[k] = i;
					++k;
				}
			}
			matcher.build(selDesc, FMIndex::LSH, "-probes 1");
			matcherSelDesc=selDesc;
			//modelPts.swap(selPts);
			//vLevels.swap(selLevels);
			//ptsDesc = selDesc;
		}
		else
		{
			matcher.build(ptsDesc, FMIndex::LSH, "-probes 1");
			matcherSubsetIndex.clear();
		}
	}
public:
	void _updateParam(ff::ArgSet &args)
	{
		auto &param = _model1::param;
		param.nSampleViews = args.getd<int>("nViews", param.nSampleViews);
		param.imageSize = args.getd<int>("imageSize", param.imageSize);
		param.fdNLevels = args.getd<int>("fdNLevels", param.fdNLevels);
		param.fdLevelScale = args.getd<float>("fdLevelScale", param.fdLevelScale);
		param.fdNFeaturesPerImage = args.getd<int>("fdNFeaturesPerImage", param.fdNFeaturesPerImage);
	}
	void build(CVRModel &model, ff::ArgSet &args)
	{
		this->_updateParam(args);
		auto &param = _model1::param;

		int selPtK = 20, nViews = param.nSampleViews;
		Size viewSize(param.imageSize, param.imageSize);

		printf("nViews=%d, imageSize=%d, nLevels=%d\n", nViews, param.imageSize, param.fdNLevels);

		//this->load("./temp._");

		_model1::Builder builder;
		if (modelPts.empty())
			builder.build(model, modelPts, vLevels, ptsDesc, selPtK, nViews, viewSize);
		else
			builder.renderViews(model,nViews,viewSize);

		nLevels = *std::max_element(vLevels.begin(), vLevels.end()) + 1;
		
		_buildMatcher();

		//this->save("./temp._");

		_clusterPoints();
		clusterIndex.build(builder, nLevels, clusters);

		modelCenter = model.getCenter();
	}

	void _printSelInfo(const std::vector<int> &vsel)
	{
		std::vector<int> hLevels(nLevels, 0);
		for (auto i : vsel)
			hLevels[vLevels[i]]++;
		printf("\n#levels:");
		for (auto v : hLevels)
			printf("%d ", v);
		printf("\n");
	}

	void getSubsetWithSeed(const std::vector<int> &seedPoints, std::vector<Point3f> &selPoints, Mat &selDesc, std::vector<int> *selIndex = nullptr)
	{
		std::vector<int> selIdx;
		_getSubsetWithSeed(seedPoints, selIdx);

		//_printSelInfo(selIdx);
		_getSubsetData(selIdx, selPoints, selDesc);
		if (selIndex)
			selIndex->swap(selIdx);
	}
	void getSubsetWithInliers(const std::vector<int> &matchPoints, const std::vector<int> &inliers, std::vector<Point3f> &selPoints, Mat &selDesc, std::vector<int> *selIndex = nullptr)
	{
		std::vector<int> seedIdx;
		seedIdx.reserve(inliers.size());
		for (auto i : inliers)
			seedIdx.push_back(matchPoints[i]);
		getSubsetWithSeed(seedIdx, selPoints, selDesc, selIndex);
	}
	_model1::ClusterIndex::_View& getClusterView(const Matx44f &mModelView, bool isRigid)
	{
		Vec3f dv = getOpticalAxis(mModelView, this->modelCenter, isRigid);
		dv = normalize(dv);
		return clusterIndex.getView(dv);
	}
	float getViewMaxSize(const Matx44f &mModelView, bool isRigid)
	{
		return getClusterView(mModelView, isRigid).maxSize;
	}
	void getSubsetWithView(const Matx44f &mModelView, bool isRigid, Size objSize, std::vector<Point3f> &selPoints, Mat &selDesc, int levelHWSZ=1, std::vector<int> *selIndex=nullptr)
	{
		Vec3f dv = getOpticalAxis(mModelView, this->modelCenter, isRigid);
		dv = normalize(dv);

		std::vector<int> selClusters;
		clusterIndex.getSubset(selClusters, dv, objSize, levelHWSZ);

		std::vector<int> selPointsIndex;
		this->_getSubsetOfClusters(selClusters, selPointsIndex);

		//_printSelInfo(selPointsIndex);
		this->_getSubsetData(selPointsIndex, selPoints, selDesc);
		if (selIndex)
			selIndex->swap(selPointsIndex);
	}
	virtual void onCreate(re3d::Model &model)
	{
		auto &modelData = model.getData();

		ff::BMStream &stream = modelData.getStream(this->streamName(),true);
		
		//stream.Clear();

		if (stream.Empty())
		{
			ff::ArgSet args;
			this->build(model.get3D(), args);
			stream << *this;
			modelData.save();
		}
		else
		{
			stream >> *this;

			nLevels = *std::max_element(vLevels.begin(), vLevels.end()) + 1;
			_buildMatcher(0, -1);
		}
	}
};


class Model2_0
{
	class IndexFile
	{
	public:
		std::string  ptsFile;
		std::string  ctsFile;
	public:
		void save(const std::string &file)
		{
			FILE *fp = fopen(file.c_str(), "w");
			if (!fp)
				FF_EXCEPTION(ERR_FILE_OPEN_FAILED, "");
			fprintf(fp, "%s\n%s\n", ptsFile.c_str(), ctsFile.c_str());
			fclose(fp);
		}
		void load(const std::string &file)
		{
			FILE *fp = fopen(file.c_str(), "r");
			if (!fp)
				FF_EXCEPTION(ERR_FILE_OPEN_FAILED, "");

			char str[1024];
			std::vector<std::string> files;

			while (fscanf(fp, "%s", str) == 1)
				files.push_back(str);

			if (files.size() != 2)
				FF_EXCEPTION(ERR_FILE_READ_FAILED, "");

			ptsFile.swap(files[0]);
			ctsFile.swap(files[1]);

			fclose(fp);
		}
	};
public:
	ModelPoints2     pts;
	ModelContours1   cts;
public:
	bool _copyFromOldVersion(const std::string &curFile, const std::string &oldExt)
	{
		if (!ff::pathExist(curFile))
		{
			std::string oldFile = ff::ReplacePathElem(curFile, oldExt, ff::RPE_FILE_EXTENTION);
			if (ff::pathExist(oldFile))
			{
				ff::copyFile(oldFile, curFile);
				return true;
			}
		}
		return false;
	}
	void build(const std::string &modelFile, const std::string &indexFile, bool forceBuild = false)
	{
		ff::CommandArgSet args;
		this->build(modelFile, indexFile, args, forceBuild);
	}
	void build(const std::string &modelFile, const std::string &indexFile, ff::ArgSet &args, bool forceBuild = false)
	{
		CVRModel         model;

		std::string modelName = ff::GetFileName(modelFile, false);
		std::string extDir = ff::GetDirectory(indexFile);

		IndexFile idx;

		if (!forceBuild && ff::pathExist(indexFile))
			idx.load(indexFile);
		else
		{
			idx.ptsFile = modelName + ".mp121";
			idx.ctsFile = modelName + ".mc110";
			idx.save(indexFile);
		}

		std::string file = extDir + "/" + idx.ptsFile;

		bool toUpdate=_copyFromOldVersion(file, "mp120");
		bool loaded = false;

		if (!forceBuild && ff::pathExist(file))
		{
			pts.load(file);
			loaded = true;
		}

		if(!loaded || toUpdate)
		{
			model = CVRModel(modelFile);

			pts.build(model,args);
			pts.save(file);
		}

		file = extDir + "/" + idx.ctsFile;
		if (!forceBuild && ff::pathExist(file))
			cts.load(file);
		else
		{
			if (!model)
				model = CVRModel(modelFile);

			cts.build(model, 3000);
			cts.save(file);
		}
	}
	void load(/*const std::string &modelFile,*/ const std::string &indexFile)
	{
		ff::CommandArgSet args;
		this->load(indexFile, args);
	}
	void load(const std::string &indexFile, ff::ArgSet &args)
	{
#if 1
		std::string extDir = ff::GetDirectory(indexFile);
		if (extDir.empty())
			extDir = ".";

		IndexFile idx;
		idx.load(indexFile);

		int startLevel = args.getd<int>("startLevel", 0), endLevel = args.getd<int>("endLevel", -1);

		pts.load(extDir + "/" + idx.ptsFile, startLevel, endLevel);
		cts.load(extDir + "/" + idx.ctsFile);
#endif
	}
};

class Model2
	:public re3d::Model::ManagedObject
{
	DEFINE_RE3D_TYPE(Model2)

	virtual void onCreate(re3d::Model &model)
	{
		auto &modelData = model.getData();

		pts = model.getObject<ModelPoints2>();
		cts = model.getObject<ModelContours1>();
		meshInfo = *model.getObject<re3d::MeshInfo>();
	}
public:
	ModelPoints2     *pts;
	ModelContours1   *cts;
	re3d::MeshInfo   meshInfo;
public:
	void fromIdxFile(const std::string &meshFile, const std::string &idxFile, const std::string &modelFile)
	{
		re3d::ModelData modelData;
		modelData.load(modelFile.c_str(), true);

		Model2_0 oldModel;
		oldModel.load(idxFile);
		oldModel.pts.dump(modelData);
		oldModel.cts.dump(modelData);

		/*CVRModel mesh(meshFile);
		modelData.setMeshInfo(mesh);*/

		modelData.save();
	}
	
};
