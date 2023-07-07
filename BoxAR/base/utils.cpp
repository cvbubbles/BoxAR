
#include"utils.h"
#include"BFC/portable.h"
#include"BFC/stdf.h"
using namespace ff;

Mat1b getRenderMask(const Mat1f &depth, float eps)
{
	Mat1b mask = Mat1b::zeros(depth.size());
	for_each_2(DWHN1(depth), DN1(mask), [eps](float d, uchar &m) {
		m = fabs(1.0f - d)<eps ? 0 : 255;
	});
	return mask;
}

void  getRenderContour(const CVRResult &rr, std::vector<Point3f> &cpt3, double approxEPS, std::vector<cv::Point> *_cpt2, Mat1b *_mask)
{
	Mat1b mask = getRenderMask(rr.depth);
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	if (approxEPS > 0)
	{
		std::vector<cv::Point> temp;
		for (auto &c : contours)
		{
			cv::approxPolyDP(c, temp, approxEPS, false);
			c.swap(temp);
		}
	}

#if 0
	Mat dimg = vis(rr.getNormalizedDepth());
	convertBGRChannels(dimg, dimg, 3);

	cv::drawContours(dimg, contours, -1, Scalar(255, 0, 0));
	for (auto &c : contours)
	{
		for (auto &p : c)
			cv::circle(dimg, p, 3, Scalar(0, 0, 255), 1, CV_AA);
	}

	imshow("img", rr.img);
	imshow("contours", dimg);
#endif
	Mat1b maskx;
	cv::copyMakeBorder(mask, maskx, 1, 1, 1, 1, BORDER_CONSTANT, Scalar(0));
	mask = maskx(Rect(1, 1, mask.cols, mask.rows));

	CV_Assert(contours.size() >= 1);
	std::vector<cv::Point> &cpt2(contours.front());
	//cpt2.swap(contours.front());

	CVRProjector prj(rr);

	cpt3.clear();
	cpt3.reserve(cpt2.size());

	const int wsz = 3, hwsz = wsz / 2, dstride = stepC(rr.depth);
	for (auto &p : cpt2)
	{
		float z = 0;
		const uchar *mp = mask.ptr(p.y, p.x);
		const float *dp = rr.depth.ptr<float>(p.y, p.x);

		if (mp[0] != 0)
		{
			z = dp[0];
		}
		else
		{
			mp = mp - mask.step*hwsz - hwsz;
			dp = dp - dstride*hwsz - hwsz;

			float dn = 0;

			for (int dy = 0; dy < wsz; ++dy, mp += mask.step, dp += dstride)
			{
				for (int dx = 0; dx < wsz; ++dx)
				{
					if (mp[dx] != 0)
					{
						z += dp[dx]; dn += 1;
					}
				}
			}
			CV_Assert(dn != 0);
			z /= dn;
		}
		cpt3.push_back(prj.unproject(p.x, p.y, z));
	}
	if (_cpt2)
	{
		CV_Assert(cpt2.size() == cpt3.size());
		_cpt2->swap(cpt2);
	}
	if (_mask)
		*_mask = mask;
}

//template<typename _Tp>
//Rect_<_Tp> _getBoundingBox2D(const std::vector<Point_<_Tp>> &pts)
//{
//	_Tp L = INT_MAX, T = INT_MAX, R = 0, B = 0;
//	for (auto &p : pts)
//	{
//		if (p.x < L)
//			L = p.x;
//		if (p.x > R)
//			R = p.x;
//		if (p.y < T)
//			T = p.y;
//		if (p.y > B)
//			B = p.y;
//	}
//	return Rect_<_Tp>(L, T, R - L, B - T);
//}

//Rect getBoundingBox2D(const std::vector<Point> &pts)
//{
//	return _getBoundingBox2D(pts);
//}
//Rect_<float> getBoundingBox2D(const std::vector<Point2f> &pts)
//{
//	return _getBoundingBox2D(pts);
//}

template<typename _Tp>
Rect_<_Tp> _getBoundingBox2D(const std::vector<std::vector<Point_<_Tp>>> &vpts)
{
	_Tp L = INT_MAX, T = INT_MAX, R = 0, B = 0;

	for(auto &pts : vpts)
		for (auto &p : pts)
		{
			if (p.x < L)
				L = p.x;
			if (p.x > R)
				R = p.x;
			if (p.y < T)
				T = p.y;
			if (p.y > B)
				B = p.y;
		}
	return Rect_<_Tp>(L, T, R - L, B - T);
}
Rect getBoundingBox2D(const std::vector<std::vector<Point>> &vpts)
{
	return _getBoundingBox2D(vpts);
}
Rect_<float> getBoundingBox2D(const std::vector<std::vector<Point2f>> &vpts)
{
	return _getBoundingBox2D(vpts);
}

void getBoundingBox3D(const std::vector<Point3f> &pts, Point3f &cornerMin, Point3f &cornerMax)
{
	Vec3f vmin(0,0,0), vmax(0,0,0);
	if (!pts.empty())
	{
		vmin = vmax = pts.front();

		for (auto &p : pts)
		{
			const Vec3f &v = reinterpret_cast<const Vec3f&>(p);
			for (int i = 0; i < 3; ++i)
			{
				if (v[i] < vmin[i])
					vmin[i] = v[i];
				else if (v[i] > vmax[i])
					vmax[i] = v[i];
			}
		}
	}
	//return Rect(int(L), int(T), int(R - L), int(B - T));
	cornerMin = vmin;
	cornerMax = vmax;
}

static Vec3f _getDV(const float *r0, const float *r1, const float *r2, float t0, float t1, float t2, const Vec3f &objCenter)
{
	return Vec3f(
		-(r0[0] * t0 + r0[1] * t1 + r0[2] * t2) - objCenter[0],
		-(r1[0] * t0 + r1[1] * t1 + r1[2] * t2) - objCenter[1],
		-(r2[0] * t0 + r2[1] * t1 + r2[2] * t2) - objCenter[2]
		);
};

Vec3f getOpticalAxis(const cv::Matx44f &mModelView, const Vec3f &modelCenter, bool isRigid)
{
	Vec3f dv;
	if (isRigid)
		dv = _getDV(&mModelView(0, 0), &mModelView(1, 0), &mModelView(2, 0), mModelView(3, 0), mModelView(3, 1), mModelView(3, 2), modelCenter);
	else
	{
		cv::Matx44f inv = mModelView.inv();
		dv = Vec3f(inv(3, 0), inv(3, 1), inv(3, 2)) - modelCenter;
	}
	if (dv.dot(dv) < 1e-3f)
		dv = normalize(Vec3f(1, 1, 1));
//	dv = normalize(dv);
//	return getContour(dv);
	return dv;
}

Mat getMatRows(const Mat &m, const std::vector<int> &rows)
{
	Mat r(rows.size(), m.cols, m.type());
	uchar *data = r.data;
	const int rowSize = r.cols*r.elemSize();
	for (auto i : rows)
	{
		memcpy(data, m.ptr(i), rowSize);
		data += rowSize;
	}
	return r;
}

void drawContour(Mat &dimg, const std::vector<std::vector<Point>> &contour, cv::Scalar color, int thickness, int pointSize)
{
	cv::polylines(dimg, contour, false, color, thickness, CV_AA);
	if (pointSize > 0)
	{
		for(auto &ct : contour)
		for (auto &p : ct)
			cv::circle(dimg, p, pointSize, color, 1, CV_AA);
	}
}
void drawContour(Mat &dimg, const std::vector<std::vector<Point2f>> &contour, cv::Scalar color, int thickness, int pointSize)
{
	std::vector<std::vector<Point>> temp;
	cvtVector(contour, temp);
	drawContour(dimg, temp, color, thickness, pointSize);
}
void drawContour(Mat &dimg, const std::vector<Point2f> &contour, cv::Scalar color, int thickness, int pointSize)
{
	std::vector<std::vector<Point>> ctrs(1);
	cvtVector(contour, ctrs[0]);
	drawContour(dimg, ctrs, color, thickness, pointSize);
}

void drawContour(Mat &dimg, const std::vector<Point> &contour, cv::Scalar color, int thickness, int pointSize)
{
	std::vector<std::vector<Point>> ctrs(1);
	ctrs[0] = contour;
	drawContour(dimg, ctrs, color, thickness, pointSize);
}

template<typename _OpT>
int maskLocalMaxMin(const float *res, int width, int height, int rstride, uchar *mask, int mstep, int hwsz, float resT, _OpT compare)
{
	int wsz = hwsz * 2 + 1;
	int *dv = new int[wsz*wsz];
	int n = 0;
	for (int i = -hwsz; i <= hwsz; ++i)
		for (int j = -hwsz; j <= hwsz; ++j)
		{
			if (i != 0 || j != 0)
				dv[n++] = i*rstride + j;

		}

	cv::memset_2d(mask, width, height, mstep, 0);

	res += hwsz*rstride + hwsz;
	mask += hwsz*mstep + hwsz;

	int nmask = 0;
	for (int yi = 0; yi < height - hwsz * 2; ++yi, res += rstride, mask += mstep)
	{
		for (int xi = 0; xi < width - hwsz * 2; ++xi)
		{
			const float *p = res + xi;
			float r = *p;

			if (compare(r, resT))
				continue;

			int i = 0;
			for (; i < n; ++i)
			{
				if (compare(r, p[dv[i]]))
					break;
			}

			if (i == n)
			{
				mask[xi] = 255;
				++nmask;
			}
		}
	}

	delete[]dv;
	return nmask;
}

Mat1b getLocalMax(const Mat1f &rmap, int hwsz, float rT, int *nMasked)
{
	Mat1b mask = Mat1b::zeros(rmap.size());
	int nmask=maskLocalMaxMin(DWHN(rmap), DS(mask), hwsz, rT, std::less<float>());
	if (nMasked)
		*nMasked = nmask;
	return mask;
}
//Mat1b getLocalMin(const Mat1f &rmap, int hwsz, float rT)
//{
//	Mat1b mask = Mat1b::zeros(rmap.size());
//	maskLocalMaxMin(DWHN(rmap), DS(mask), hwsz, rT, std::less<float>());
//	return mask;
//}

Mat getFullAffine(const Mat &m)
{
	CV_Assert(m.cols == m.rows + 1 && (m.type()==CV_32FC1 || m.type()==CV_64FC1));

	Mat dst(m.cols, m.cols, m.type());
	copyMem(m, dst(Rect(0, 0, m.cols, m.rows)));
	
	memset(dst.ptr(dst.rows - 1), 0, dst.elemSize()*dst.cols);

	void *last = dst.ptr(dst.rows - 1, dst.cols - 1);
	if (dst.type() == CV_32FC1)
		*(float*)last = 1.0f;
	else
		*(double*)last = 1.0;
	return dst;
}

Mat1b  getDTMask(Size dsize, const std::vector<std::vector<cv::Point>> &poly)
{
	Mat1b dtMask(dsize);
	setMem(dtMask, 0xFF);

	//std::vector<std::vector<Point>> poly;
	//cvtVector(data.segSilhouette, poly);
	cv::polylines(dtMask, poly, true, Scalar(0), 1, 0);
	return dtMask;
}

template<typename _IndexPtrT>
void _mask_dt(_IndexPtrT cc, int width, int height, int cstride, uchar *mask, int mstep)
{
	const int dstep = width * 3;
	const int vdi[] = { 1, cstride + 1, cstride, cstride - 1, -1, -cstride - 1, -cstride, -cstride + 1 };

	cc += cstride;
	mask += mstep;
	for (int yi = 1; yi<height - 1; ++yi, cc += cstride, mask += mstep)
	{
		for (int xi = 1; xi<width - 1; ++xi)
		{
			int im = -1;
			for (int i = 0; i<8; ++i)
			{
				if (cc[xi]<cc[xi + vdi[i]])
				{
					im = i; break;
				}
			}
			if (im >= 0)
				mask[xi] = 0;
		}
	}
}

Mat1b getDTMask(const Mat1i &segRegions)
{
	Mat1b dtMask(segRegions);
	setMem(dtMask, 0xFF);
	_mask_dt(DWHN(segRegions), DS(dtMask));
	return dtMask;
}

std::shared_ptr<Point> getDTLabelMap(const Mat1b &mask, int *mapSize)
{
	std::shared_ptr<Point> _map(new Point[mask.rows*mask.cols]);
	Point *pmap = _map.get();
	pmap[0] = Point(0, 0);

	int nz = 1; ////label start with 1
	for_each_1c(DWHN1(mask), [&nz, pmap](uchar m, int x, int y) {
		if (m == 0)
		{
			pmap[nz] = Point(x, y);
			++nz;
		}
	});

	Point *dst = new Point[nz];
	memcpy(dst, pmap, sizeof(Point)*nz);

	if (mapSize)
		*mapSize = nz;

	return std::shared_ptr<Point>(dst);
}


void VideoWriterEx::set(const std::string &file, double fps, int fourcc)
{
	_file = file;
	_fourcc = fourcc;
	_fps = fps;
	_state = 0;

	_autoFileID = ff::StrFormat(_file.c_str(), 1) != ff::StrFormat(_file.c_str(), 2);
}

bool VideoWriterEx::writeEx(const Mat &img, int waitCode)
{
	bool written = false;

	if (_state == 0 && waitCode == 'b')
	{
		std::string file = _file;
		if (_autoFileID)
		{
			int i = 1;
			for (;; ++i)
			{
				file = ff::StrFormat(_file.c_str(), i);
				if (!ff::pathExist(file))
				{
					break;
				}
			}
		}

		if (!this->open(file, _fourcc, _fps, img.size()))
		{
			printf("can't open %s\n", file.c_str());
			return false;
		}

		printf("start capture to %s\n", file.c_str());
		_state = 1;
		_curFile = file;
	}
	else if (_state == 1)
	{
		this->write(img);
		written = true;

		if (waitCode == 'e')
		{
			printf("end capture\n");
			this->release();
			_state = 0;
		}
	}
	return written;
}


