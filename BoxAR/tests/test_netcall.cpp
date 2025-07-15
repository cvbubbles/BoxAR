
#include"cmdstd.h"
#include"BFC/netcall.h"

_STATIC_BEG


Mat _renderResults(const cv::Mat& img, const Matx33f &camK, CVRModel& model, const std::vector<cv::Mat> &Rts, bool _drawContour=true, bool _drawBlend=true)
{
	cv::Mat dimg = img.clone();

	for (auto& Rt : Rts)
	{
		RigidPose pose(Rt);

		CVRMats mats;
		mats.mModel = cvrm::fromR33T(pose.R, pose.t);
		mats.mProjection = cvrm::fromK(camK, img.size(), 0.1, 3000);

		if (_drawContour || _drawBlend)
		{
			CVRender render(model);
			//auto rr = render.exec(mats, img.size(), CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr, r.roi);
			auto rr = render.exec(mats, img.size(), CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr);
			Mat1b mask = getRenderMask(rr.depth);
			Rect roi = cv::get_mask_roi(DWHS(mask), 127);

			if (roi.empty())
				continue;

			if (_drawBlend)
			{
				Mat t;
				cv::addWeighted(dimg(roi), 0.5, rr.img(roi), 0.5, 0, t);
				t.copyTo(dimg(roi), mask(roi));
			}
			if (_drawContour)
			{
				std::vector<std::vector<Point> > cont;
				cv::findContours(mask, cont, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
				drawContours(dimg, cont, -1, Scalar(255, 0, 0), 2, CV_AA);
			}
		}
	}
	return dimg;
}

void on_test_netcall_detect()
{
	ff::NetcallServer serv("101.76.200.67", 8002);

	ff::setCurrentDirectory("F:/SDUicloudCache/re3d/test");

	std::string modelFile = "./3d/box1.3ds", videoFile = "./box1.mp4";

	cv::VideoCapture cap;
	cap.open(videoFile);

	CVRModel model(modelFile);

	Mat img;
	while (cap.read(img))
	{
		img = imscale(img, 0.5);
		cv::Matx33f camK = cvrm::defaultK(img.size(), 1.5f);

		ff::NetObjs objs = {
			{ "cmd","estpose" },
			{ "image", ff::nct::Image(img,".jpg") },
			{"camK", camK}
		};

		auto robjs=serv.call(objs);
		if (!robjs.hasError())
		{
			//假设只有一个物体，但可能有多个检测结果
			//服务器用numpy ndarray的列表返回Rt(3*4，最后一列是t)，如果没有检测到，返回空列表
			std::vector<cv::Mat> Rts = robjs["Rts"].getv<cv::Mat>();

			img = _renderResults(img, camK, model, Rts);
		}

		imshow("results", img);
	}
}

CMD_BEG()
CMD0("test_netcall_detect", on_test_netcall_detect)
CMD_END()

_STATIC_END


