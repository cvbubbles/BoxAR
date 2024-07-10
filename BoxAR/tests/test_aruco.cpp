
#include"cmdstd.h"
#include"opencv2/aruco.hpp"

_STATIC_BEG

void test_create_aruco_dict()
{
	//cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::generateCustomDictionary(36, 5);
	cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

	//for (int i = 0; i < dict->bytesList.rows; ++i)
	for (int i = 0; i < 5; ++i)
	{
		Mat img;
		dict->drawMarker(i, 512, img);
		imshow("marker", img);

		std::string file = cv::format("../data/aruco/%03d.png", i + 1);
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
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

	cv::VideoCapture inputVideo;
	inputVideo.open(0+cv::CAP_DSHOW);

	//inputVideo.set(CAP_PROP_FRAME_WIDTH, 1280);
	//inputVideo.set(CAP_PROP_FRAME_HEIGHT, 720);

	cv::Mat image;
	inputVideo.read(image);
	if (image.empty())
		return;

	cv::Matx33f cameraMatrix=cvrm::defaultK(image.size(),1.5f);
	/*cv::Matx33f cameraMatrix = { 5.2093072503417386e+02, 0., 3.2627544281606572e+02, 0.,
	   5.2164480491819393e+02, 2.4303275400142539e+02, 0., 0., 1. };*/

	std::string modelDir = R"(F:\SDUicloudCache\re3d\test\3d\)";
	
	float markerSize = 0.1f;
	float objectSize = 0.05f;

	CVRModel model0(modelDir + "cat.obj");
	auto initT0 = model0.getUnitize()*cvrm::scale(objectSize);

	CVRModel model1(modelDir + "bunny.3ds");
	auto initT1 = model1.getUnitize() * cvrm::rotate(CV_PI/2, Vec3f(1.f,0.f,0.f)) * cvrm::scale(objectSize);

	auto mProjection= cvrm::fromK(cameraMatrix, image.size(), 0.1, 100);

	while (inputVideo.read(image) )
	{
		image = image.clone();

		auto param = aruco::DetectorParameters::create();
		//param->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
		

		time_t beg = clock();
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		cv::aruco::detectMarkers(image, dictionary, corners, ids, param);

		// if at least one marker detected
		if (ids.size() > 0) 
		{
			cv::aruco::drawDetectedMarkers(image, corners, ids);
			std::vector<cv::Vec3d> rvecs, tvecs;
			cv::aruco::estimatePoseSingleMarkers(corners, 0.1, cameraMatrix, Mat(), rvecs, tvecs);

			CVRModelArray modelArray(ids.size());
			
			// draw axis for each marker
			for (int i = 0; i < ids.size(); i++)
			{
				int id = ids[i];
				modelArray[i].model = id%2==0? model0 : model1;
				modelArray[i].mModeli = id%2==0? initT0 : initT1;
				modelArray[i].mModel = cvrm::fromRT(rvecs[i], tvecs[i]);
			}
			
			CVRMats mats;
			mats.mProjection = mProjection;

			CVRender render(modelArray);

			render.setBgImage(image);
			auto rr=render.exec(mats, image.size());
			image = rr.img;
		}

		printf("time=%dms    \r", int(clock() - beg));


		cv::imshow("out", image);
		char key = (char)cv::waitKey(5);
		if (key == 27)
			break;
	}
}

CMD_BEG()
CMD0("test_aruco_ar", test_aruco_ar)
CMD_END()



_STATIC_END

