#pragma once
#include"cmdstd.h"

#include <iostream>
_STATIC_BEG

void test_ardetectors()
{
	app()->setTempDir("D:/projects/boxar");

	ff::setCurrentDirectory("D:/projects/boxar/BoxAR/");
	/*********************************模型设置************************************
	* trackedModelFile是视频中被跟踪物体的obj模型文件路径
	* 要存在一个真实世界中可供追踪的实体和对应的三维模型文件
	* 由于跟踪算法是基于特征的，所以模型表面应该具有充足且明显的特征
	* 
	* videoFile是包含该物体的视频文件路径，如果选择使用摄像头实时拍摄，则此变量可以注释掉或删除
	*/
	std::string trackedModelFile = "D:\\projects\\boxar\\BoxAR\\scan\\vase\\obj\\qinghuaci.obj", videoFile = R"(.\scan\vase\test2.mp4)";
	//std::string trackedModelFile = R"(.\tests\model\test1\mesh.obj)", videoFile = R"(.\tests\video\test1_video\test_1_5_chest.mp4)";
	/*
	* ModelSet是要被跟踪的模型信息的集合
	* 正常情况下此处无需修改
	*/
	//config model-set
	ModelSet models;
	{
		std::vector<ModelInfos> modelInfos  = modelInfosFromSingleFile(trackedModelFile, "re3d");
		models.set(modelInfos);
	}
	/*
	* virtualModelPath是需要叠加在真实视频上的三维模型文件路径，virtualModel是读取该文件后得到的模型类
	* 这里只需要模型文件即可，视频中无需出现此模型对应的真实物体
	*/
	string virtualModelPath = "D:\\projects\\boxar\\BoxAR\\scan\\flower\\test.obj";
	CVRModel virtualModel(virtualModelPath);
	/*
	* 以下部分声明跟踪器并进行初始化
	*/
	auto detector = FrameProc::create("v1.Tracker");
	//init detector
	ff::CommandArgSet args;
	//args.setArgs("-d2dModelFile f:/sdk/torch_models/model_re3d6_v1.ts -d2dScoreT 0.5");
	detector->init(&models, &args);
	
	/*********************************VideoCapture设置*************************************
	* VideoCapture是OpenCV提供的用于连接摄像头或读取视频文件的类
	* 如果使用预先拍摄好的离线视频，应该使用cap.open(videoFile);的形式
	* 如果要使用摄像头捕获实时视频，应该使用cap.open(0);的形式；
	*	其中参数为摄像头编号，如果电脑自带摄像头的情况下又外接摄像头则应该通过cap.open(0);或cap.open(1);确定摄像头和编号的对应关系
	* p.s. 如果使用摄像头拍摄实时视频，一定要做一个人工初始化的操作：应该将现实世界中的物体的姿态与屏幕上显示的结果保持基本一致，之后才能正确跟踪
	*/
	cv::VideoCapture cap;
	//cap.open(0);
	cap.open(videoFile);
	FrameData fd;
	/*
	* dK[]为相机的内参矩阵，如果已经完成相机标定的步骤且确定标定正确
	* 应该将yml文件中的camera_matrix的data矩阵内容复制到这里
	*/
	float dK[] = {
		1.324595302424838110e+03, 0.000000000000000000e+00, 6.460060955956646467e+02,
		0.000000000000000000e+00, 1.330463970754883576e+03, 3.568279021773695945e+02,
		0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00
	};
	int fi = 0;
	Mat img;
	/*
	* VideoWriter是OpenCV用于将图像转写为视频的类
	*	参数1：指定输出视频名
	*	参数2：指定视频编码格式，正常情况下无需修改
	*	参数3：指定视频帧速率fps，这里是保持与输入视频相同，也可以指定，如30.0
	*	参数4：输出视频的分辨率，该项注意应该与要保存为视频的连续图片大小相同
	*/
	int fourcc = CV_FOURCC('M', 'P', '4', '2');
	//VideoWriter writer3("output.avi", fourcc, cap.get(CAP_PROP_FPS), Size(1280,720), true);

	/*********************************变换矩阵类声明************************************
	* CVRMats是CVF提供的变换矩阵集合类，包含模型矩阵、投影矩阵、视图矩阵
	* 具体三种矩阵的作用：https://zhuanlan.zhihu.com/p/386204250
	*/
	CVRMats mats;
	
	while(cap.read(img))
	{
		//processInput(window);
		//图像预处理
		//if (!cap.read(img))
		//	break;
		
		if (img.rows > 1000)
		{
			img = imscale(img, 0.5);
			flip(img, img, 0);
			flip(img, img, 1);
		}

		if (fi == 0) {
			//fd.cameraK = cvrm::defaultK(img.size(), 1.5);
			fd.cameraK = cv::Matx33f(dK);
			//设置openGL的投影矩阵
			mats.mProjection = cvrm::fromK(fd.cameraK, img.size(), 0.1, 3000);
		}
		//跟踪获取位姿
		detector->pro(img, fd);
		//glClear(GL_DEPTH_BUFFER_BIT);
		/*
		* CVF提供的模型集合类，这里代表需要渲染的模型集合类
		* 并将虚拟模型加入待渲染集合
		* 如果在设计AR场景中，有新的虚拟物体想要渲染，则应该通过类似的形式
		* 声明想要渲染的虚拟物体并加入该集合
		*/
		CVRModelArray modelArray;
		modelArray.push_back(virtualModel);
		//此处只有一个物体，这里的循环适用于多物体跟踪，暂时不建议修改相关内容
		for (int i = 0; i < (int)fd.objs.size(); i++)
		{
			re3d::ImageObject obj = fd.objs[i];
			//pose获取的是跟踪得到的物体位姿
			auto pose = obj.pose.get<std::vector<RigidPose>>().front();
			/*
			* 以下两行将被跟踪物体加入待渲染集合
			* 形式相对固定，暂时不需要更改
			*/
			auto modelPtr = models.getModel(fd.objs[i].modelIndex);
			modelArray.push_back(modelPtr->get3DModel());
			/*
			* CVRender是CVF提供的渲染工具类，这里的声明形式固定，传入一个待渲染集合作为构造参数
			* setBgImage指定背景图像一并渲染，这里应该是读取到的视频的每一帧
			*/
			CVRender render(modelArray);
			render.setBgImage(img);
			/* 
			* mats.mView设置OpenGL形式的视图矩阵，
			* 由于OpenCV中的旋转平移参数和OpenGL的变换矩阵格式不同
			* 需要cvrm::fromR33T进行转换，这里应该无需修改
			*/
			mats.mView = cvrm::fromR33T(pose.R, pose.t);
			/*
			* 调用CVRender的exec函数执行渲染：
			*	参数1：变换矩阵集合类，包含OpenGL格式的旋转、视图、投影矩阵
			*	参数2：渲染得到的图像大小，建议与输入图像保持一致
			*	参数3：指定渲染的输出形式，CVRM_IMAGE表示RGB图像，CVRM_DEPTH表示深度图
			*	参数4、5保持现有形式即可
			*/
			auto rr = render.exec(mats, img.size(), CVRM_IMAGE | CVRM_DEPTH, CVRM_DEFAULT, nullptr);
			// rr.img获取渲染的RGB图像并显示
			cv::imshow("test", rr.img);
			cv::waitKey(1);
			/*
			* 所以要自主设计AR场景，需要做的是：
			* 1.构思一个合理的场景，包括被跟踪物体和需要叠加的虚拟模型
			* 2.如果有复杂的被跟踪物体但没有对应的三维模型文件，可以找助教帮忙扫描模型
			*	p.s. 模型一定是表面特征明显的
			* 3.由于虚拟物体的叠加是在被跟踪物体的基础上，因此需要预先设置好虚拟物体和被跟踪物体在同一坐标系下的相对关系
			*/
		}
		++fi;
	}
}
	
CMD_BEG()
CMD0("test_ardetectors", test_ardetectors)
CMD_END()

_STATIC_END
