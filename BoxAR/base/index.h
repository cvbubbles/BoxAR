#pragma once

#include"opencv2/core.hpp"


class _FMIndex;

class FMIndex
{
	_FMIndex *impl;
public:
	enum IndexType {
		LINEAR, LSH, HCLUSTER
	};
public:
	FMIndex();

	~FMIndex();

	void build(const cv::Mat &trainDesc, IndexType indexType=LSH, const std::string &params="");

	void knnSearch(const cv::Mat &queryDesc, cv::Mat &idx, cv::Mat &dist, int K);

	void match(const cv::Mat &queryDesc, std::vector<cv::DMatch> &matches);
};

