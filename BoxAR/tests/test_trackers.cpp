
#include"cmdstd.h"

_STATIC_BEG

void test_detectors()
{
	app()->setTempDir("e:/boxar/");

	ff::setCurrentDirectory("F:/SDUicloudCache/re3d/");

	//std::string modelFile = R"(.\scan\3ds-model\bottle2\bottle2.ply)", videoFile = R"(.\BoxAR\video\bottle2-5.avi)";
	//std::string modelFile = R"(.\scan\3ds-model\bottle3\bottle3.ply)", videoFile = R"(.\test\bottle3.mp4)";
	//std::string modelFile = R"(.\test\3d\box1.3ds)", videoFile = R"(.\test\box1.mp4)";
	//std::string modelFile = R"(.\test\3d\box3.3ds)", videoFile = R"(.\test\box3.mp4)";
	std::string modelFile = R"(.\test\3d\car.3ds)", videoFile = R"(.\test\car.mp4)";
	//std::string  modelFile = R"(.\scan\3ds-model\plane\plane.ply)";

	//std::string modelFile = R"(.\BoxAR\model\test1\mesh.obj)", videoFile = R"(.\BoxAR\video\test1_video\test1_1.mp4)";
	//std::string modelFile = R"(.\BoxAR\model\test2\mesh.obj)", videoFile = R"(.\BoxAR\video\test2_video\test2_1.mp4)";
	//std::string modelFile = R"(.\BoxAR\model\test3\mesh.obj)", videoFile = R"(.\BoxAR\video\test3_video\test3_1.mp4)";

	//config model-set
	ModelSet models;

	{
		std::vector<ModelInfos> modelInfos = modelInfosFromSingleFile(modelFile, "re3d");
		models.set(modelInfos);
	}

	auto detector = FrameProc::create("v1.Tracker");

	//init detector
	ff::CommandArgSet args;
	//args.setArgs("-d2dModelFile f:/sdk/torch_models/model_re3d6_v1.ts -d2dScoreT 0.5");
	detector->init(&models, &args);

	FrameData fd;

	cv::VideoCapture cap;
	cap.open(videoFile);
	//cap.open(0+cv::CAP_DSHOW);

	float dK[] = {
		1.324595302424838110e+03, 0.000000000000000000e+00, 6.460060955956646467e+02,
		0.000000000000000000e+00, 1.330463970754883576e+03, 3.568279021773695945e+02,
		0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
	};

	int fi = 0;
	Mat img;
	while (cap.read(img))
	{
		if(img.rows>1000)
		{
			img = imscale(img, 0.5);
			flip(img, img, 0);
			flip(img, img, 1);
		}

		if (fi == 0)
		{
			//camera intrinsics, here we use a default value
			fd.cameraK = cvrm::defaultK(img.size(), 1.6);
			//memcpy(fd.cameraK.val, dK, sizeof(dK));
		}

		time_t beg = clock();
		detector->pro(img, fd);
		printf("\rspeed=%.1ffps       ", 1000.0f/int(clock() - beg));

		//show results
		Mat dimg = redist::renderResults(img, fd, models, true, true, false, false);
		imshow("result", dimg);

		if (cv::waitKey(1) == 'q')
			break;
		
		/*if (fd.objs.front().score < 0.8)
			cv::waitKey();*/

		++fi;
	}
}

CMD_BEG()
CMD0("test_detectors", test_detectors)
CMD_END()

_STATIC_END

