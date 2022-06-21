
#include"cmdstd.h"

_STATIC_BEG

void test_detectors()
{
	app()->setTempDir("e:/boxar/");

	//config model-set
	ModelSet models;

	{//load model-set as all .ply models in a directory
		std::string modelFile = R"(F:\SDUicloudCache\re3d\scan\3ds-model\bottle2\bottle2.ply)";
		std::vector<ModelInfos> modelInfos = modelInfosFromSingleFile(modelFile, "re3d");
		models.set(modelInfos);
	}

	auto detector = FrameProc::create("v1.Detector");

	//init detector
	ff::CommandArgSet args;
	//args.setArgs("-d2dModelFile f:/sdk/torch_models/model_re3d6_v1.ts -d2dScoreT 0.5");
	detector->init(&models, &args);

	FrameData fd;

	cv::VideoCapture cap;
	cap.open(R"(F:\dev\prj-c1\1100-Re3DX\TestX\bottle2-5.avi)");

	int fi = 0;
	Mat img;
	while (cap.read(img))
	{
		if(fi==0)
			//camera intrinsics, here we use a default value
			fd.cameraK = cvrm::defaultK(img.size(), 1.5);

		detector->pro(img, fd);

		//show results
		Mat dimg = redist::renderResults(img, fd, models, true, true, false, false);
		imshow("result", dimg);

		if (cv::waitKey() == 'q')
			break;
		
		++fi;
	}
}

CMD_BEG()
CMD0("test_detectors", test_detectors)
CMD_END()

_STATIC_END

