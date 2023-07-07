#pragma once
#include"cmdstd.h"
#include "backdrawer.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <model.h>
#include <camera.h>

#include <iostream>
_STATIC_BEG
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}
void processInput(GLFWwindow* window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);
}
void test_ardetectors()
{
	app()->setTempDir("D:/projects/boxar");

	ff::setCurrentDirectory("D:/projects/boxar/BoxAR/");
	/*********************************模型设置*************************************/
	std::string trackedModelFile = "D:\\projects\\boxar\\BoxAR\\scan\\vase\\obj\\qinghuaci.obj", videoFile = R"(.\scan\vase\test2.mp4)";
	//std::string modelFile = R"(.\scan\obj-model\doll3\doll3.obj)", videoFile = R"(.\scan\obj-model\doll3\test1.mp4)";
	//config model-set
	ModelSet models;
	string obj_path = "D:\\projects\\boxar\\BoxAR\\scan\\flower\\test.obj";
	{
		std::vector<ModelInfos> modelInfos = modelInfosFromSingleFile(trackedModelFile, "re3d");
		models.set(modelInfos);
	}
	auto detector = FrameProc::create("v1.Tracker");
	//init detector
	ff::CommandArgSet args;
	//args.setArgs("-d2dModelFile f:/sdk/torch_models/model_re3d6_v1.ts -d2dScoreT 0.5");
	detector->init(&models, &args);
	
	/*********************************VideoCapture*************************************/
	cv::VideoCapture cap;
	//cap.open(0);
	cap.open(videoFile);
	FrameData fd;
	float dK[] = {
		1.324595302424838110e+03, 0.000000000000000000e+00, 6.460060955956646467e+02,
		0.000000000000000000e+00, 1.330463970754883576e+03, 3.568279021773695945e+02,
		0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
	};
	int fi = 0;
	Mat img;
	int fourcc = CV_FOURCC('M', 'P', '4', '2');
	//VideoWriter writer3("output.avi", fourcc, cap.get(CAP_PROP_FPS), Size(1280,720), true);
	
	

	/*********************************glfw初始设置*************************************/
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(1280, 720, "AR_tracker", NULL, NULL);
	//GLFWwindow* window = glfwCreateWindow(cap.get(CAP_PROP_FRAME_WIDTH), cap.get(CAP_PROP_FRAME_HEIGHT), "", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		exit(-1);
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{

		std::cout << "Failed to initialize GLAD" << std::endl;
		exit(-1);
	}
	stbi_set_flip_vertically_on_load(true);


	/*********************************背景*************************************/
	Shader texShader("texture.vs", "texture.fs");
	vector<float> tex_vertices = {
		1.0f,  1.0f, 0.0f,   1.0f, 0.0f, 0.0f,   1.0f, 1.0f, // top right
		1.0f, -1.0f, 0.0f,   0.0f, 1.0f, 0.0f,   1.0f, 0.0f, // bottom right
		-1.0f, -1.0f, 0.0f,   0.0f, 0.0f, 1.0f,   0.0f, 0.0f, // bottom left
		-1.0f,  1.0f, 0.0f,   1.0f, 1.0f, 0.0f,   0.0f, 1.0f  // top left 
	};
	vector<unsigned int> indices = {
		0, 1, 3, // first triangle
		1, 2, 3  // second triangle
	};
	BackDrawer bgDrawer(tex_vertices, indices, &texShader);

	/*********************************相关声明*************************************/
	Shader testShader("vertex.glsl", "frag.glsl");
	MModel mModel(obj_path);
	MModel mTrackedModel(trackedModelFile);
	cv::Matx44f mProjection;
	cv::Matx44f mView;
	glm::mat4 projection = glm::mat4(1.0f);
	glm::mat4 view = glm::mat4(1.0f);
	glm::mat4 model = glm::mat4(1.0f);

	glEnable(GL_DEPTH_TEST);
	while(!glfwWindowShouldClose(window))
	{
		processInput(window);
		//图像预处理
		if (!cap.read(img))
			break;
		
		if (img.rows > 1000)
		{
			img = imscale(img, 0.5);
			flip(img, img, 0);
			flip(img, img, 1);
		}
		if (fi == 0) {
			fd.cameraK = cvrm::defaultK(img.size(), 1.5);
			mProjection = cvrm::fromK(fd.cameraK, img.size(), 0.1, 3000);
		}
		//跟踪获取位姿
		//time_t beg = clock();
		detector->pro(img, fd);
		//printf("detector time=%dms\n", int(clock() - beg));
		cv::flip(img, img, 0);

		//渲染背景
		bgDrawer.Draw(img);
		glClear(GL_DEPTH_BUFFER_BIT);
		
		//cv::Mat1f returnDepth;
		//Mat dimg = redist::renderResults(img, fd, models, true, true, false, false);
		
		
		//此处只有一个物体
		for (int i = 0; i < (int)fd.objs.size(); i++)
		{

			re3d::ImageObject obj = fd.objs[i];
			auto pose = obj.pose.get<std::vector<RigidPose>>().front();
				
			mView = cvrm::fromR33T(pose.R, pose.t);

			//time_t ren_beg = clock();
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				
			cv::Mat res;
			{
				testShader.use();
				for (int k = 0; k < 4; k++)
				{
					for (int j = 0; j < 4; j++)
					{
						view[k][j] = mView(k, j);
						projection[k][j] = mProjection(k, j);
					}
				}
				testShader.setMat4("projection", projection);
				testShader.setMat4("view", view);
				testShader.setMat4("model", model);
				mTrackedModel.Draw(testShader);		
				mModel.Draw(testShader);
			}
		}
		++fi;
		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwTerminate();
}
	//writer.release();
	

CMD_BEG()
CMD0("test_ardetectors", test_ardetectors)
CMD_END()

_STATIC_END
