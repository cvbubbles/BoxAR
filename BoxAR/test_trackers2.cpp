#pragma once
#include"cmdstd.h"
#include"mGLRender.h"
_STATIC_BEG
vector<cv::Point3f> loadBoxVertices(const std::string& obj_path)
{
	ifstream ifs(obj_path);
	string line;
	vector<cv::Point3f> res;
	while (getline(ifs, line))
	{
		if (line[0] == 'v' && line[1] == ' ')
		{
			stringstream ss(line);
			string flag;
			cv::Point3f tmp;
			ss >> flag >> tmp.x >> tmp.y >> tmp.z;
			res.push_back(tmp);
		}
	}
	return res;
}
void prjBoxVertices(const cv::Matx44f& projection, const cv::Matx44f& modelview, const int* viewport,
	vector<Point3f>& points3d, vector<Point3f>& ret)
{//suppose there is no occlusion in demo video

	CVRProjector prj(modelview, projection, Size(viewport[2], viewport[3]));
	for (int i = 0; i < points3d.size(); i++)
	{
		ret.push_back(prj.project(points3d[i]));
	}
}
struct mRect {
	Point3f lb;
	Point3f lu;
	Point3f ru;
	Point3f rb;
	vector<Point2f> vrx;
	int nIdx;
	mRect(Point3f _lb, Point3f _lu,
		Point3f _ru, Point3f _rb, int _nIdx)
	{
		lb = _lb; lu = _lu;
		rb = _rb; ru = _ru;
		nIdx = _nIdx;
		vrx.push_back(Point2f(lb.x, lb.y));
		vrx.push_back(Point2f(lu.x, lu.y));
		vrx.push_back(Point2f(ru.x, ru.y));
		vrx.push_back(Point2f(rb.x, rb.y));
	}
	bool operator < (const mRect x) const {
		return max(max(max(lb.z, rb.z), lu.z), ru.z) < max(max(max(x.lb.z, x.rb.z), x.lu.z), x.ru.z);      //从小到大排序
	}
	bool operator > (const mRect x) const {
		return max(max(max(lb.z, rb.z), lu.z), ru.z) > max(max(max(x.lb.z, x.rb.z), x.lu.z), x.ru.z);
	}
};
void test_arhomography()
{
	app()->setTempDir("D:/projects/boxar");

	ff::setCurrentDirectory("D:/projects/boxar/BoxAR/");


	std::string modelFile = R"(.\tests\model\test1\mesh.obj)", videoFile = "D:\\projects\\boxar\\BoxAR\\tests\\video\\test1_video\\test_1_5_chest.mp4";
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
	VideoWriter writer("output2_homography.avi", fourcc, cap.get(CAP_PROP_FPS), Size(1280,
		720), true);


	//cv::Matx44f mProjection;
	//cv::Matx44f mView;
	vector<cv::Point3f> boxVertices = loadBoxVertices(modelFile);
	CVRMats mats;
	cv::Mat h;
	cv::Mat tmpImg;
	while (cap.read(img))
	{
		cout << img.size() << endl;
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
		Mat dimg = redist::renderResults(img, fd, models, true, true, false, false);
		const int viewport[4] = { 0,0,dimg.cols,dimg.rows };
		mats.mProjection = cvrm::fromK(fd.cameraK, dimg.size(), 0.1, 3000);
		Mat himg = img.clone();
		cv::Mat sideImage = cv::imread("D:\\projects\\boxar\\BoxAR\\tests\\images\\test1\\side.jpg", cv::IMREAD_COLOR);
		cv::Mat frontImage = cv::imread("D:\\projects\\boxar\\BoxAR\\tests\\images\\test1\\face.jpg", cv::IMREAD_COLOR);
		cv::Mat topImage = cv::imread("D:\\projects\\boxar\\BoxAR\\tests\\images\\test1\\back.png", cv::IMREAD_COLOR);
		vector<cv::Point2f> sideV{ Point2f(0,sideImage.rows),Point2f(0,0) ,Point2f(sideImage.cols,0) ,Point2f(sideImage.cols,sideImage.rows) };
		vector<cv::Point2f> frontV{ Point2f(0,frontImage.rows),Point2f(0,0) ,Point2f(frontImage.cols,0) ,Point2f(frontImage.cols,frontImage.rows) };
		vector<cv::Point2f> topV{ Point2f(0,topImage.rows),Point2f(0,0) ,Point2f(topImage.cols,0) ,Point2f(topImage.cols,topImage.rows) };
		for (int i = 0; i < (int)fd.objs.size(); i++)
		{
			re3d::ImageObject obj = fd.objs[i];
			auto pose = obj.pose.get<std::vector<RigidPose>>().front();
			mats.mModel = cvrm::fromR33T(pose.R, pose.t);
			//side
			vector<Point3f> points2d;
			prjBoxVertices(mats.mProjection, mats.mModel, viewport,
				boxVertices, points2d);
			vector<mRect> faces;
			faces.push_back(mRect(points2d[0], points2d[1], points2d[2], points2d[3], 1));//side1
			faces.push_back(mRect(points2d[4], points2d[5], points2d[6], points2d[7], 2));//side2
			faces.push_back(mRect(points2d[0], points2d[1], points2d[5], points2d[4], 3));//front
			faces.push_back(mRect(points2d[7], points2d[6], points2d[2], points2d[3], 4));//back
			faces.push_back(mRect(points2d[1], points2d[2], points2d[6], points2d[5], 5));//top
			faces.push_back(mRect(points2d[0], points2d[3], points2d[7], points2d[4], 6));//bottom
			sort(faces.begin(), faces.end(), greater<mRect>());
			cv::Mat h;
			for (int j = 0; j < 6; j++)
			{
				if (faces[j].nIdx == 1 || faces[j].nIdx == 2)
				{
					h = findHomography(sideV, faces[j].vrx);
					cv::Mat tmp;
					warpPerspective(sideImage, tmp, h, dimg.size());
					for (int x = 0; x < tmp.rows; x++)
					{
						for (int y = 0; y < tmp.cols; y++)
						{
							if (tmp.at<Vec3b>(x, y)[0] != 0 && tmp.at<Vec3b>(x, y)[1] != 0 && tmp.at<Vec3b>(x, y)[2] != 0)
							{
								himg.at<Vec3b>(x, y) = tmp.at<Vec3b>(x, y);
							}
						}
					}

				}
				else if (faces[j].nIdx == 3)
				{
					h = findHomography(frontV, faces[j].vrx);					cv::Mat tmp;
					warpPerspective(frontImage, tmp, h, dimg.size());
					for (int x = 0; x < tmp.rows; x++)
					{
						for (int y = 0; y < tmp.cols; y++)
						{
							if (tmp.at<Vec3b>(x, y)[0] != 0 && tmp.at<Vec3b>(x, y)[1] != 0 && tmp.at<Vec3b>(x, y)[2] != 0)
							{
								himg.at<Vec3b>(x, y) = tmp.at<Vec3b>(x, y);
							}
						}
					}
				}
				else
				{
					h = findHomography(topV, faces[j].vrx);
					cv::Mat tmp;
					warpPerspective(topImage, tmp, h, dimg.size());
					for (int x = 0; x < tmp.rows; x++)
					{
						for (int y = 0; y < tmp.cols; y++)
						{
							if (tmp.at<Vec3b>(x, y)[0] != 0 && tmp.at<Vec3b>(x, y)[1] != 0 && tmp.at<Vec3b>(x, y)[2] != 0)
							{
								himg.at<Vec3b>(x, y) = tmp.at<Vec3b>(x, y);
							}
						}
					}
				}
			}
		}

		imshow("result", himg);
		writer.write(himg);
		if (cv::waitKey(5) == 'q')
			break;

		++fi;
	}
	writer.release();

}
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
				ress = glRender(obj_path, mView, mProjection, dimg.cols, dimg.rows,img, returnDepth);
			}
			
			if (ress.rows != 0)
			{
				writer.write(ress);
				writer2.write(img);
				writer3.write(dimg);
			}
			
		}
		
		++fi;
	}
	writer.release();

}

CMD_BEG()
CMD0("test_ardetectors", test_ardetectors)
CMD0("test_arhomography", test_arhomography)
CMD_END()

_STATIC_END
