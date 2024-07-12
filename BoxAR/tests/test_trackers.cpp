
#include"cmdstd.h"

_STATIC_BEG

void test_3d_tracking()
{
	//set path to your output path
	//app()->setTempDir("D:/projects/boxar");   //example
	app()->setTempDir(TMPDIR);
	//set path to your current path
	//ff::setCurrentDirectory("D:/ARsystem/BoxAR/BoxAR"); 
	ff::setCurrentDirectory(INPUTDIR);



	//std::string modelFile = "D:/ARsystem/BoxAR_old/BoxAR/model/test1/mesh.obj", videoFile = "D:/ARsystem/BoxAR_old/BoxAR/video/test1_video/test1_1.mp4";

	//std::string  modelFile = R"(.\scan\3ds-model\plane\plane.ply)";

	std::string modelFile = R"(.\BoxAR\model\test1\mesh.obj)", videoFile = R"(.\BoxAR\video\test1_video\test1_1.mp4)";
	//std::string modelFile = R"(.\BoxAR\model\test2\mesh.obj)", videoFile = R"(.\BoxAR\video\test2_video\test2_1.mp4)";
	//std::string modelFile = R"(.\BoxAR\model\test3\mesh.obj)", videoFile = R"(.\BoxAR\video\test3_video\test3_1.mp4)";


	//config model-set
	ModelSet models;

	{
		//std::vector<ModelInfos> modelInfos = modelInfosFromSingleFile(modelFile, "re3d");
		std::vector<std::string> modelFiles = { modelFile};
		std::vector<ModelInfos> modelInfos = modelInfosFromFiles(modelFiles, "re3d");
		models.set(modelInfos);
	}

	auto tracker = FrameProc::create("v1.Tracker");

	//init detector
	ff::CommandArgSet args;
	args.setArgs("-globalSearch -usePointMatches");

	tracker->init(&models, &args);

	FrameData fd;

	cv::VideoCapture cap;
	cap.open(videoFile);
	//cap.open(0+cv::CAP_DSHOW);
	//cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	//cap.set(CAP_PROP_FRAME_HEIGHT, 720);

	
	float dK[] = {
		1.324595302424838110e+03, 0.000000000000000000e+00, 6.460060955956646467e+02,
		0.000000000000000000e+00, 1.330463970754883576e+03, 3.568279021773695945e+02,
		0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
	};

	VideoWriterEx vw;
	//vw.set("f:/out.mp4", 25);  

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
			fd.cameraK = cvrm::defaultK(img.size(), 1.5);
			//memcpy(fd.cameraK.val, dK, sizeof(dK));
		}

		time_t beg = clock();
		tracker->pro(img, fd);
		printf("\rspeed=%.1ffps       ", 1000.0f/int(clock() - beg));

		//show results
		Mat dimg = redist::renderResults(img, fd, models, true, true, false, false);
		imshow("result", dimg);

		int waitCode = cv::waitKey(1);
		if (waitCode == 'q')
			break;
		
		//vw.writeEx(dimg, waitCode);

		/*if (fd.objs.front().score < 0.8)
			cv::waitKey();*/

		++fi;
	}
}

CMD_BEG()
CMD0("test_3d_tracking", test_3d_tracking)
CMD_END()



void on_model_select_init_pose()
{
	ff::setCurrentDirectory(INPUTDIR);


	std::string file = R"(.\test\3d\car.3ds)";
	//std::string file = R"(.\test\3d\box1.3ds)";

	CVRModel model(file);
	Size viewSize(640, 480);
	//cout << "center=" << model.getCenter() << endl;

	auto wptr = mdshow("model", model, viewSize);
	Matx33f R;
	Vec3f t;
	wptr->resultFilter = newResultFilter([&R, &t, &model](CVRResult& r) {
		auto mT = r.mats.modelView();

		cvrm::decomposeRT(mT, R, t);
		auto S = R * R.t();
		float s = 1.f / sqrt(S(0, 0));
		//cout <<"s:"<< s << endl;
		R = R * s;
		t *= s;

		//cout << "\n######" << endl;
		//cout << R * R.t() << endl;
		//cout << R <<endl<< t<<endl << S<<endl;
		Mat _rvec;
		cv::Rodrigues(R, _rvec);
		Vec3f rvec(_rvec);
		cout << "##\n[";
		for (int i = 0; i < 3; ++i)
			cout << rvec[i] << ' ';
		for (int i = 0; i < 3; ++i)
			cout << t.val[i] << ' ';
		cout << "]\n";

		/*cv::Rodrigues(_rvec, R);
		Size viewSize(640, 480);
		CVRMats mats;
		mats.mModel = cvrm::fromR33T(R, t);
		mats.mProjection = cvrm::perspective(viewSize.height * 1.5, viewSize, 1, 3000);
		CVRender render(model);
		auto rr = render.exec(mats, viewSize);
		imshow("curview", rr.img);*/
		});


	cvxWaitKey();
}

CMD_BEG()
CMD0("model.select_init_pose", on_model_select_init_pose)
CMD_END()



void test_manual_init()
{
	app()->setTempDir(TMPDIR);
	//app()->setTempDir("D:/projects/boxar");
	
	ff::setCurrentDirectory(INPUTDIR);
	//ff::setCurrentDirectory("D:/ARsystem/");


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
		//std::vector<ModelInfos> modelInfos = modelInfosFromSingleFile(modelFile, "re3d");
		std::vector<std::string> modelFiles = { modelFile };
		std::vector<ModelInfos> modelInfos = modelInfosFromFiles(modelFiles, "re3d");
		models.set(modelInfos);
	}

	auto tracker = FrameProc::create("v1.Tracker");

	//init detector
	ff::CommandArgSet args;
	//initPose指定初始位姿，格式为[rvec tvec]的6维向量，可以通过"model.select_init_pose"命令获得
	args.setArgs("-detector v1.ManualInitor -initPose -0.0125247 -2.20621 -1.21235 -144.611 -324.331 -9.27385");  //car.3ds
	//args.setArgs("-detector v1.ManualInitor -initPose -1.91254 0.477099 1.39522 -329.007 -332.011 214.1");          //box1.3ds

	tracker->init(&models, &args);

	FrameData fd;

	cv::VideoCapture cap;
	cap.open(videoFile);
	//cap.open(0+cv::CAP_DSHOW);

	//cap.set(CAP_PROP_FRAME_WIDTH, 1280);
	//cap.set(CAP_PROP_FRAME_HEIGHT, 720);

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

		//ManualInitor需要键盘消息控制，按空格键开始/结束初始化
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

