
#include"cmdstd.h"


#include"opencv2/objdetect/aruco_detector.hpp"
#include"opencv2/objdetect/aruco_dictionary.hpp"

_STATIC_BEG

void test_create_aruco_dict()
{
	//cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::generateCustomDictionary(36, 5);
	cv::aruco::Dictionary dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

	//for (int i = 0; i < dict->bytesList.rows; ++i)
	for (int i = 0; i < 5; ++i)
	{
		Mat img;
		cv::aruco::generateImageMarker(dict, i, 512, img);
		imshow("marker", img);

		std::string file = cv::format("./aruco/%03d.png", i + 1);
		imwrite(file, img);

		if ('q' == cv::waitKey(10))
			break;
	}

}

CMD_BEG()
CMD0("test_create_aruco_dict", test_create_aruco_dict)
CMD_END()


void test_aruco_ar()
{
	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
	cv::aruco::ArucoDetector detector;
	detector.setDictionary(dictionary);

	cv::VideoCapture inputVideo;
	inputVideo.open(1+cv::CAP_DSHOW);

	inputVideo.set(CAP_PROP_FRAME_WIDTH, 640);
	inputVideo.set(CAP_PROP_FRAME_HEIGHT, 480);

	cv::Mat image;
	inputVideo.read(image);
	if (image.empty())
		return;

	cv::Matx33f cameraMatrix=cvrm::defaultK(image.size(),1.5f);
	/*cv::Matx33f cameraMatrix = { 5.2093072503417386e+02, 0., 3.2627544281606572e+02, 0.,
	   5.2164480491819393e+02, 2.4303275400142539e+02, 0., 0., 1. };*/

	ff::setCurrentDirectory(INPUTDIR);
	std::string modelDir = R"(./test/3d/)";
	
	float markerSize = 0.1f; //marker的实际尺寸，单位米
	float objectSize = 0.15f; //物体的大小，单位米

	CVRModel model0(modelDir + "cat.obj");
	auto center = model0.getCenter();
	auto bbsize = model0.getSizeBB();
	auto initT0 = model0.getUnitize()*cvrm::scale(objectSize); //getUnitize将获得一个变换：把物体中心平移到原点，并把大小归一化为1

	CVRModel model1(modelDir + "bunny.3ds");
	auto initT1 = model1.getUnitize() * cvrm::rotate(CV_PI/2, Vec3f(1.f,0.f,0.f)) * cvrm::scale(objectSize);

	//获取投影变换，100、0.1分别是OpenGL中的远、近裁剪面
	auto mProjection= cvrm::fromK(cameraMatrix, image.size(), 0.1, 100);

	while (inputVideo.read(image) )
	{
		image = image.clone();

		time_t beg = clock(); 
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		detector.detectMarkers(image, corners, ids); //检测markers

		// if at least one marker detected
		if (ids.size() > 0) 
		{
			cv::aruco::drawDetectedMarkers(image, corners, ids);
			std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());
			
			CVRModelArray modelArray(ids.size()); //包含多个模型的场景
			
			for (int i = 0; i < ids.size(); i++)
			{
				static Vec3d rvec, tvec;
				{
					std::vector<Point3f>  modelPoints = { {0.f,0.f,0.f},{0.f,markerSize,0.f},{markerSize,markerSize,0.f},{markerSize,0.f,0.f} };
					cv::solvePnP(modelPoints, corners[i], cameraMatrix, Mat(), rvec, tvec, true);
				}

				int id = ids[i];
				modelArray[i].model = id%2==0? model0 : model1;   //根据id设置模型
				modelArray[i].mModeli = id%2==0? initT0 : initT1;  //设置对应模型的初始变换
				modelArray[i].mModel = cvrm::fromRT(rvec, tvec); //应用当前R, t

				cout << rvec << tvec << endl;
			}
			
			CVRMats mats;
			mats.mProjection = mProjection;

			CVRender render(modelArray);

			render.setBgImage(image);
			auto rr=render.exec(mats, image.size(), CVRM_IMAGE /*| CVRM_DEPTH*/, CVRM_DEFAULT); //可以选择是否输出深度图
			image = rr.img;
		}

		printf("time=%dms    \r", int(clock() - beg));


		cv::imshow("out", image);
		char key = (char)cv::waitKey(100);
		if (key == 27)
			break;
	}
}

CMD_BEG()
CMD0("test_aruco_ar", test_aruco_ar)
CMD_END()



_STATIC_END
