
//#include"FLANN/flann.hpp"
#include<memory>
#include"index.h"
#include"BFC/argv.h"
#include"BFC/err.h"
#include"FLANN/umain.h"

class _FMIndex
{
public:
	virtual void build(const cv::Mat &trainDesc, FMIndex::IndexType indexType, const std::string &params) =0;

	virtual void knnSearch(const cv::Mat &queryDesc, cv::Mat &idx, cv::Mat &dist, int K) =0;

	virtual void match(const cv::Mat &queryDesc, std::vector<cv::DMatch> &matches)
	{
		cv::Mat idx, dist;
		this->knnSearch(queryDesc, idx, dist, 1);

		const int np = queryDesc.rows;
		matches.resize(np);
		for (int i = 0; i < np; ++i)
		{
			auto &m(matches[i]);
			m.queryIdx = i;
			m.trainIdx = *idx.ptr<int>(i);
			m.distance = *dist.ptr<uint>(i);
		}
	}
	virtual ~_FMIndex()
	{}
};

class _FMIndexBinary
	:public _FMIndex
{
	typedef flann::Hamming<uchar> Distance;
	typedef flann::Index<Distance> IndexType; // index(data, flann::LinearIndexParams());

	typedef std::shared_ptr<IndexType> IndexPtrT;

	IndexPtrT  _index;
public:
	void build(const cv::Mat &trainDesc, FMIndex::IndexType indexType, const std::string &params)
	{
		ff::ArgSet args;
		args.setArgs(params);

		IndexPtrT index;
		if (indexType == FMIndex::LSH)
			//index = IndexPtrT(new IndexType(flann::LshIndexParams(args.getd<int>("tables", 12), args.getd<int>("keySize", 20), args.getd<int>("probes", 2))));
			index = IndexPtrT(new IndexType(flann::LshIndexParams(12, 20, args.getd<int>("probes", 2))));
		else if (indexType == FMIndex::LINEAR)
			index = IndexPtrT(new IndexType(flann::LinearIndexParams()));
		else if (indexType == FMIndex::HCLUSTER)
			index = IndexPtrT(new IndexType(flann::HierarchicalClusteringIndexParams()));
		else
		{
			FF_EXCEPTION(ERR_INVALID_ARGUMENT, "unknown index type");
		}

		CV_Assert(trainDesc.type() == CV_8UC1);
		flann::Matrix<uchar>  dm((uchar*)trainDesc.data, trainDesc.rows, trainDesc.cols, trainDesc.step);
		index->buildIndex(dm);
		_index = index; 
	}
	void knnSearch(const cv::Mat &queryDesc, cv::Mat &idx, cv::Mat &dist, int K)
	{
		typedef Distance::ResultType _DistT;

		if (queryDesc.empty())
		{
			idx = cv::Mat();
			dist = cv::Mat();
			return;
		} 

		if (_index)
		{
			CV_Assert(queryDesc.type() == CV_8UC1);
			flann::Matrix<uchar>  qm((uchar*)queryDesc.data, queryDesc.rows, queryDesc.cols, queryDesc.step);

			idx.create(queryDesc.rows, K, CV_32SC1);
			dist.create(queryDesc.rows, K, CV_32SC1);

			flann::Matrix<int> _idx(idx.ptr<int>(), idx.rows, idx.cols, idx.step);
			flann::Matrix<uint> _dist(dist.ptr<uint>(), dist.rows, dist.cols, dist.step);

			auto param = flann::SearchParams();
			param.cores = 0;
			//param.sorted = true;
			_index->knnSearch(qm, _idx, _dist, K, param);
		}
	}
};

class _FMIndexFloat
	:public _FMIndex
{
	typedef flann::L2<float>  Distance;
	typedef flann::Index<Distance> IndexType; // index(data, flann::LinearIndexParams());

	typedef std::shared_ptr<IndexType> IndexPtrT;

	IndexPtrT  _index;
public:
	void build(const cv::Mat &trainDesc, FMIndex::IndexType indexType, const std::string &params)
	{
		ff::ArgSet args;
		args.setArgs(params);

		IndexPtrT index;
		if (indexType == FMIndex::LSH)
			//index = IndexPtrT(new IndexType(flann::LshIndexParams(args.getd<int>("tables", 12), args.getd<int>("keySize", 20), args.getd<int>("probes", 2))));
			//index = IndexPtrT(new IndexType(flann::LshIndexParams(12, 20, args.getd<int>("probes", 2))));
			index = IndexPtrT(new IndexType(flann::KDTreeIndexParams()));
		else if (indexType == FMIndex::LINEAR)
			index = IndexPtrT(new IndexType(flann::LinearIndexParams()));
		else if (indexType == FMIndex::HCLUSTER)
			index = IndexPtrT(new IndexType(flann::HierarchicalClusteringIndexParams()));
		else
		{
			FF_EXCEPTION(ERR_INVALID_ARGUMENT, "unknown index type");
		}

		CV_Assert(trainDesc.type() == CV_32FC1);
		flann::Matrix<float>  dm((float*)trainDesc.data, trainDesc.rows, trainDesc.cols, trainDesc.step);
		index->buildIndex(dm);
		_index = index;
	}
	void knnSearch(const cv::Mat &queryDesc, cv::Mat &idx, cv::Mat &dist, int K)
	{
		typedef Distance::ResultType _DistT;

		if (queryDesc.empty())
		{
			idx = cv::Mat();
			dist = cv::Mat();
			return;
		}
		 
		if (_index)
		{
			CV_Assert(queryDesc.type() == CV_32FC1);
			flann::Matrix<float>  qm((float*)queryDesc.data, queryDesc.rows, queryDesc.cols, queryDesc.step);

			idx.create(queryDesc.rows, K, CV_32SC1);
			dist.create(queryDesc.rows, K, CV_32FC1);

			flann::Matrix<int> _idx(idx.ptr<int>(), idx.rows, idx.cols, idx.step);
			flann::Matrix<float> _dist(dist.ptr<float>(), dist.rows, dist.cols, dist.step);

			auto param = flann::SearchParams();
			param.cores = 0;
			//param.sorted = true;
			_index->knnSearch(qm, _idx, _dist, K, param);
		}
	}
};

FMIndex::FMIndex()
{
	impl = nullptr;
}

FMIndex::~FMIndex()
{
	delete impl;
}

void FMIndex::build(const cv::Mat &trainDesc, IndexType indexType, const std::string &params)
{
	if (impl)
		delete impl;

	if (trainDesc.depth() == CV_8U)
		impl = new _FMIndexBinary;
	else
		impl = new _FMIndexFloat;

	impl->build(trainDesc, indexType, params);
}

void FMIndex::knnSearch(const cv::Mat &queryDesc, cv::Mat &idx, cv::Mat &dist, int K)
{
	impl->knnSearch(queryDesc, idx, dist, K);
}

void FMIndex::match(const cv::Mat &queryDesc, std::vector<cv::DMatch> &matches)
{
	impl->match(queryDesc, matches);
}

#include"FLANN/umain.cpp"




