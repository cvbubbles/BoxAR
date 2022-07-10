#pragma once
#include"cmdstd.h"
#include"mGLRender.h"
_STATIC_BEG

void test_ardetectors()
{
	app()->setTempDir("D:/projects/boxar");
	
	ff::setCurrentDirectory("D:/projects/boxar/BoxAR/");

	//std::string modelFile = R"(.\scan\3ds-model\bottle2\bottle2.ply)", videoFile = R"(.\BoxAR\video\bottle2-5.avi)";
	//std::string modelFile = R"(.\scan\3ds-model\bottle3\bottle3.ply)", videoFile = R"(.\test\bottle3.mp4)";
	//std::string modelFile = R"(.\test\3d\box11.ply)", videoFile = R"(.\test\box1.mp4)";
	//std::string modelFile = R"(.\test\3d\box3.3ds)", videoFile = R"(.\test\box3.mp4)";
	//std::string modelFile = R"(.\test\3d\car.3ds)", videoFile = R"(.\test\car.mp4)";
	//std::string modelFile = R"(.\test\cat.obj)", videoFile = R"(.\test\cat.avi)";

	//std::string modelFile = R"(.\tests\model\test1\mesh.obj)", videoFile = R"(.\tests\video\test1_video\test1_1.mp4)";
	//std::string modelFile = R"(.\BoxAR\model\test2\mesh.obj)", videoFile = R"(.\BoxAR\video\test2_video\test2_1.mp4)";
	//std::string modelFile = R"(.\BoxAR\model\test3\mesh.obj)", videoFile = R"(.\BoxAR\video\test3_video\test3_1.mp4)";
	//std::string modelFile = R"(.\scan\obj-model\doll3\doll3.obj)", videoFile = R"(.\scan\obj-model\doll3\test1.mp4)";
	std::string modelFile = R"(.\scan\vase\obj\qinghuaci.obj)", videoFile = R"(.\scan\vase\test9.mp4)";
	// D:\projects\boxar\BoxAR\scan\vase\obj
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
	//cap.open(0);

	float dK[] = {
		1.324595302424838110e+03, 0.000000000000000000e+00, 6.460060955956646467e+02,
		0.000000000000000000e+00, 1.330463970754883576e+03, 3.568279021773695945e+02,
		0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
	};


	int fi = 0;
	Mat img;
	int fourcc = CV_FOURCC('M', 'P', '4', '2');
	VideoWriter writer("output1000_phong.avi", fourcc, cap.get(CAP_PROP_FPS), Size(1280,
		720) , true);
	VideoWriter writer2("output1000_input.avi", fourcc, cap.get(CAP_PROP_FPS), Size(1280,
		720), true);
	VideoWriter writer3("output1000_tracker.avi", fourcc, cap.get(CAP_PROP_FPS), Size(1280,
		720), true);
	string obj_path = "C:\\Users\\Tien\\Desktop\\flower\\flower\\test.obj";
	

	cv::Matx44f mProjection;
	cv::Matx44f mView;


	// glfw: initialize and configure
   // ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
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
		detector->pro(img, fd);
		printf("\rtime=%d      ", int(clock() - beg));

		//show results
		
		cv::Mat1f returnDepth;
		Mat dimg = redist::renderResults(img, returnDepth, fd, models, true, true, false, false);

		mProjection = cvrm::fromK(fd.cameraK, img.size(), 0.1, 3000);
		
		//suppose there is only one object
		if (1)
		{
			cv::Mat ress;
			for (int i = 0; i < (int)fd.objs.size(); i++)
			{

				re3d::ImageObject obj = fd.objs[i];
				auto pose = obj.pose.get<std::vector<RigidPose>>().front();
				

				mView = cvrm::fromR33T(pose.R, pose.t);
				
				//mView = cvrm::fromR33T(pose.R, pose.t);
				time_t ren_beg = clock();
				glRender(obj_path, mView, mProjection,img.cols,img.rows);
				printf("\r render time=%d      ", int(clock() - ren_beg));
				ress = saveImageUsingCV(img, img.cols, img.rows, returnDepth);
				
			}
			
			if (ress.rows != 0)
			{
				cv::imshow("ar_res", ress);
				waitKey(5);
				writer.write(ress);
				writer2.write(img);
				writer3.write(dimg);
			}
			
		}
		
		++fi;
	}
	writer.release();
	glfwTerminate();
}

CMD_BEG()
CMD0("test_ardetectors", test_ardetectors)
CMD_END()

_STATIC_END
