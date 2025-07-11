#pragma once
#include"cmdstd.h"
#include <glrender/mGLRender.h>
_STATIC_BEG

void test_ardetectors()
{

	app()->setTempDir(TMPDIR);

	ff::setCurrentDirectory(INPUTDIR);

	//这是你的模型文件和视频文件
	std::string modelFile = "./BoxAR/model/test1/mesh.obj", videoFile = "./BoxAR/video/test1_video/test1_1.mp4";

	//！！这是你要渲染的模型的obj文件的相对路径，你也可以直接使用绝对路径，
	//！！注意路径不要使用正斜杠/，否则会发生一些问题

	string obj_path = ".\\flower\\test.obj";

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
	//跟踪时间、渲染时间
	int totalTrackingTime = 0;
	int totalRenderTime = 0;
	int frameCount = 0;
	Mat img;

	//以下是要输出的视频文件
	int fourcc = CV_FOURCC('M', 'P', '4', '2');
	VideoWriter writer(R"(output1000_phong.avi)", fourcc, cap.get(CAP_PROP_FPS), Size(1280,
		720), true);
	/*VideoWriter writer2(R"(output1000_input.avi)", fourcc, cap.get(CAP_PROP_FPS), Size(1280,
		720), true);
	VideoWriter writer3(R"(output1000_tracker.avi)", fourcc, cap.get(CAP_PROP_FPS), Size(1280,
		720), true);*/
		//string obj_path = R"(./flower/flower/test.obj)";


	cv::Matx44f mProjection;
	cv::Matx44f mView;


	// 初始化glfw
   // ------------------------------
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	//读视频流的第一帧来确定窗口大小
	Mat frame1;
	cap.read(frame1);
	GLFWwindow* window = glfwCreateWindow(frame1.cols, frame1.rows, "", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(-1);
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	// 初始化glad
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		exit(-1);
	}
	//stbi_set_flip_vertically_on_load(true);

	// configure global opengl state
	// -----------------------------
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);// add phong

	// build and compile shaders
	// -------------------------
	//Shader testShader("./glrender/vertex.glsl", "./glrender/frag.glsl");
	Shader testShader(".\\vertex.glsl", ".\\frag.glsl");
	MModel mModel(obj_path);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	//重置视频流
	cap.set(cv::CAP_PROP_POS_FRAMES, 0);
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
		int trackingTime = int(clock() - beg);
		totalTrackingTime += trackingTime;
		frameCount++;


		time_t ren_beg = clock();
		//开始渲染

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
				glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

				testShader.use();

				glm::mat4 projection = glm::mat4(1.0f);
				glm::mat4 view = glm::mat4(1.0f);

				for (int i = 0; i < 4; i++)
				{
					for (int j = 0; j < 4; j++)
					{
						view[i][j] = mView(i, j);
						projection[i][j] = mProjection(i, j);
					}
				}
				testShader.setMat4("projection", projection);
				testShader.setMat4("view", view);


				// 对加载的模型渲染
				glm::mat4 model = glm::mat4(1.0f);

				testShader.setMat4("model", model);
				mModel.Draw(testShader);

				glfwSwapBuffers(window);
				glfwPollEvents();


				//glRender(obj_path, mView, mProjection, img.cols, img.rows);
				int renderTime = int(clock() - ren_beg);
				totalRenderTime += renderTime;
				printf("\rrender time=%d      ", renderTime);

				ress = saveImageUsingCV(img, img.cols, img.rows, returnDepth);

			}

			if (ress.rows != 0)
			{
				cv::imshow("ar_res", ress);
				waitKey(5);
				writer.write(ress);
				//writer2.write(img);
				//writer3.write(dimg);
			}

		}

		++fi;
	}
	// 计算并输出平均跟踪时间
	if (frameCount > 0) {
		double avgTrackingTime = static_cast<double>(totalTrackingTime) / frameCount;
		std::cout << "\nAverage tracking time per frame: " << avgTrackingTime << " ms" << std::endl;
		double avgRenderTime = static_cast<double>(totalRenderTime) / frameCount;
		std::cout << "Average Render time per frame: " << avgRenderTime << " ms" << std::endl;
	}
	else {
		std::cout << "No frames processed." << std::endl;
	}
	writer.release();
	glfwTerminate();
}

CMD_BEG()
CMD0("test_ardetectors", test_ardetectors)
CMD_END()

_STATIC_END
