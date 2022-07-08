#pragma once

#include"CVRender/cvrender.h"
using namespace cv;

Mat1b getRenderMask(const Mat1f &depth, float eps=1e-6f);

void  getRenderContour(const CVRResult &rr, std::vector<Point3f> &cpt3, double approxEPS=1.0, std::vector<Point> *cpt2=nullptr, cv::Mat1b *mask=nullptr);

Rect getBoundingBox2D(const std::vector<Point> &pts);

Rect_<float> getBoundingBox2D(const std::vector<Point2f> &pts);

Rect getBoundingBox2D(const std::vector<std::vector<Point>> &vpts);

Rect_<float> getBoundingBox2D(const std::vector<std::vector<Point2f>> &vpts);

void getBoundingBox3D(const std::vector<Point3f> &pts, Point3f &cornerMin, Point3f &cornerMax);

//int segmentImage(const cv::Mat &img, Mat1i &seg, int D=32, int minSize=0);

Vec3f getOpticalAxis(const cv::Matx44f &mModelView, const Vec3f &modelCenter, bool isRigid=false);

Mat getMatRows(const Mat &m, const std::vector<int> &rows);

template<typename _SrcT, typename _DestT>
inline void cvtVector(const std::vector<_SrcT> &src, std::vector<_DestT> &dest)
{
	dest.resize(src.size());
	for (size_t i = 0; i < src.size(); ++i)
		dest[i] = _DestT(src[i]);
}
template<typename _SrcT, typename _DestT>
inline void cvtVector(const std::vector<std::vector<_SrcT>> &src, std::vector<std::vector<_DestT>> &dest)
{
	dest.resize(src.size());
	for (size_t i = 0; i < src.size(); ++i)
		cvtVector(src[i],dest[i]);
}

void drawContour(Mat &dimg, const std::vector<std::vector<Point>> &contour, cv::Scalar color, int thickness = 2, int pointSize=0);

void drawContour(Mat &dimg, const std::vector<std::vector<Point2f>> &contour, cv::Scalar color, int thickness = 2, int pointSize = 0);

void drawContour(Mat &dimg, const std::vector<Point2f> &contour, cv::Scalar color, int thickness = 2, int pointSize=0);

void drawContour(Mat &dimg, const std::vector<Point> &contour, cv::Scalar color, int thickness = 2, int pointSize = 0);

template<typename _PtT>
inline void addOffset(std::vector<_PtT> &v, const _PtT &offset)
{
	for (auto &p : v)
		p += offset;
}
template<typename _PtT>
inline void addOffset(std::vector<std::vector<_PtT>> &vv, const _PtT &offset)
{
	for (auto &v : vv)
		addOffset(v, offset);
}

Mat1b getLocalMax(const Mat1f &rmap, int hwsz, float rT, int *nMasked=nullptr);

template<typename _PtT>
inline void getLocalMax(const Mat1f &rmap, int hwsz, float rT, std::vector<_PtT> &pts)
{
	int nmask;
	cv::Mat1b mask = getLocalMax(rmap, hwsz, rT, &nmask);
	pts.clear();
	pts.reserve(nmask);
	for_each_1c(DWHN1(mask), [&pts](uchar m, int x, int y) {
		if(m)
			pts.push_back(_PtT(x, y));
	});
}

Mat getFullAffine(const Mat &m);


Mat1b  getDTMask(Size dsize, const std::vector<std::vector<cv::Point>> &poly);

template<typename _PtT>
Mat1b getDTMask(Size dsize, const std::vector<std::vector<_PtT>> &poly)
{
	std::vector<std::vector<cv::Point>> polyx;
	cvtVector(poly, polyx);
	return getDTMask(dsize, polyx);
}

Mat1b getDTMask(const Mat1i &segRegions);

std::shared_ptr<Point> getDTLabelMap(const Mat1b &mask, int *mapSize=nullptr);
//
//inline Mat imscale(const Mat &img, double scale, int interp = INTER_NEAREST)
//{
//	Mat dimg;
//	resize(img, dimg, Size(int(img.cols*scale + 0.5), int(img.rows*scale + 0.5)), 0, 0, interp);
//	return dimg;
//}
//inline Mat imscale(const Mat &img, Size dsize, int interp = INTER_NEAREST)
//{
//	Mat dimg;
//	resize(img, dimg, dsize, 0, 0, interp);
//	return dimg;
//}




class VideoWriterEx
	:public cv::VideoWriter
{
	std::string _file;
	std::string _curFile;
	bool  _autoFileID;
	int _fourcc;
	double _fps;
	int   _state;
public:
	void set(const std::string &file, double fps, int fourcc = CV_FOURCC('M', 'J', 'P', 'G'));
	
	bool writeEx(const Mat &img, int waitCode);

	const std::string& getCurFile() const
	{
		return _curFile;
	}
};

