
#include"cmdstd.h"

_STATIC_BEG

void test_3d_tracking()
{

	app()->setTempDir(TMPDIR); 
	ff::setCurrentDirectory(INPUTDIR);

	std::string modelFile = R"(E:\ZJR\summer\2025cv\data\Mdou_1\Mdou.obj)", videoFile = "./box1.mp4";
	

	//config model-set
	ModelSet models;

	{
		std::vector<std::string> modelFiles = { modelFile};
		std::vector<ModelInfos> modelInfos = modelInfosFromFiles(modelFiles, "re3d");
		models.set(modelInfos);
	}

	auto tracker = FrameProc::create("v1.Tracker");

	ff::CommandArgSet args;
	/*�������ã�
	* -globalSearch : ʹ��ȫ���������ٶȽ��������ǶԿ����˶����ȶ�
	* -usePointMatches : ����ʱʹ���ڲ�������ƥ�䣬������������Ч������
	* -trackScale : ����ģ�������ͼ���scaleϵ��������ٶ�������ʹ�ý�С��scale
	* + ���� true - ���� flase
	*/
	args.setArgs("-globalSearch - -usePointMatches + -trackScale 1.0");

	tracker->init(&models, &args);

	FrameData fd;

	cv::VideoCapture cap;
	//cap.open(videoFile);
	cap.open(1+cv::CAP_DSHOW);

	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT, 480);

	float dK[] = {
		1.324595302424838110e+03, 0.000000000000000000e+00, 6.460060955956646467e+02,
		0.000000000000000000e+00, 1.330463970754883576e+03, 3.568279021773695945e+02,
		0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
	};

	VideoWriterEx vw;

	int fi = 0;
	Mat img;
	while (cap.read(img))
	{
		if(img.rows>1000)
		{
			img = imscale(img, 0.5);
		}

		if (fi == 0)
		{
			//camera intrinsics, here we use a default value
			fd.cameraK = cvrm::defaultK(img.size(), 1.5);
		}

		time_t beg = clock();
		tracker->pro(img, fd);
		if(fi%12==0)
			printf("speed=%.1ffps       \r", 1000.0f/int(clock() - beg));

		//show results
		Mat dimg = redist::renderResults(img, fd, models, true, true, false, false);
		imshow("result", dimg);

		int waitCode = cv::waitKey(1);
		if (waitCode == 'q')
			break;
		

		++fi;
	}
}

CMD_BEG()
CMD0("test_3d_tracking", test_3d_tracking)
CMD_END()



void on_model_select_init_pose()
{
	ff::setCurrentDirectory("D:/ARsystem/");

	std::string file = R"(.\test\3d\box1.3ds)";

	CVRModel model(file);
	Size viewSize(640, 480);

	auto wptr = mdshow("model", model, viewSize);
	Matx33f R;
	Vec3f t;
	wptr->resultFilter = newResultFilter([&R, &t, &model](CVRResult& r) {
		auto mT = r.mats.modelView();

		cvrm::decomposeRT(mT, R, t);
		auto S = R * R.t();
		float s = 1.f / sqrt(S(0, 0));
		R = R * s;
		t *= s;

		Mat _rvec;
		cv::Rodrigues(R, _rvec);
		Vec3f rvec(_rvec);
		cout << "##\n[";
		for (int i = 0; i < 3; ++i)
			cout << rvec[i] << ' ';
		for (int i = 0; i < 3; ++i)
			cout << t.val[i] << ' ';
		cout << "]\n";
		});


	cvxWaitKey();
}

CMD_BEG()
CMD0("model.select_init_pose", on_model_select_init_pose)
CMD_END()



void test_manual_init()
{
	app()->setTempDir("e:/boxar/");

	ff::setCurrentDirectory("F:/SDUicloudCache/re3d/");

	
	std::string modelFile = R"(.\test\3d\box1.3ds)", videoFile = R"(.\test\box1.mp4)";

	//config model-set
	ModelSet models;

	{
		//std::vector<ModelInfos> modelInfos = modelInfosFromSingleFile(modelFile, "re3d");
		std::vector<std::string> modelFiles = { modelFile };
		std::vector<ModelInfos> modelInfos = modelInfosFromFiles(modelFiles, "re3d");
		models.set(modelInfos);
	}

	auto tracker = FrameProc::create("v1.Tracker");

	//init detector
	ff::CommandArgSet args;
	//initPoseָ����ʼλ�ˣ���ʽΪ[rvec tvec]��6ά����������ͨ��"model.select_init_pose"������
	//args.setArgs("-detector v1.ManualInitor -initPose -0.0125247 -2.20621 -1.21235 -144.611 -324.331 -9.27385");  //car.3ds
	args.setArgs("-detector v1.ManualInitor -initPose -1.91254 0.477099 1.39522 -329.007 -332.011 214.1");          //box1.3ds

	tracker->init(&models, &args);

	FrameData fd;

	cv::VideoCapture cap;
	//cap.open(videoFile);
	cap.open(1+cv::CAP_DSHOW);
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT, 480);

	float dK[] = {
		1.324595302424838110e+03, 0.000000000000000000e+00, 6.460060955956646467e+02,
		0.000000000000000000e+00, 1.330463970754883576e+03, 3.568279021773695945e+02,
		0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
	};

	VideoWriterEx vw;
	//vw.set("f:/out.mp4", 25);

	int fi = 0, waitCode = 0;
	Mat img;
	while (cap.read(img))
	{
		if (img.rows > 1000)
		{
			img = imscale(img, 0.5);
			flip(img, img, 0);
			flip(img, img, 1);
		}

		if (fi == 0)
		{
			//camera intrinsics, here we use a default value
			fd.cameraK = cvrm::defaultK(img.size(), 1.5);
			//memcpy(fd.cameraK.val, dK, sizeof(dK));
		}

		time_t beg = clock();

		//ManualInitor��Ҫ������Ϣ���ƣ����ո����ʼ/������ʼ��
		fd.make<int>("waitCode") = waitCode; 
		tracker->pro(img, fd);

		printf("\rspeed=%.1ffps       ", 1000.0f / int(clock() - beg));

		//show results
		Mat dimg = redist::renderResults(img, fd, models, true, true, false, false);
		imshow("result", dimg);

		waitCode = cv::waitKey(1);
		if (waitCode == 'q')
			break;
		++fi;
	}
}

CMD_BEG()
CMD0("test_manual_init", test_manual_init)
CMD_END()

_STATIC_END

