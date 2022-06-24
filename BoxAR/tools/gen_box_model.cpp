
#include"cmdstd.h"
#define _CRT_SECURE_NO_WARNINGS
_STATIC_BEG

struct Face
{
	int x, y, z;
	int  xt, yt, zt;
	Face(int x0, int y0, int z0, int xt0, int yt0, int zt0) :x(x0), y(y0), z(z0) , xt(xt0), yt(yt0), zt(zt0){}
};
struct userdata {
	Mat im;
	vector<Point2f> points;
};

void generate_points(vector<Point3f>&points, double L, double W, double H)
{
	//v0-v3
	points.push_back(Point3f(W, 0, 0));
	points.push_back(Point3f(W, 0, H));
	points.push_back(Point3f(0, 0, H));
	points.push_back(Point3f(0, 0, 0));
	//v4-v7
	points.push_back(Point3f(W, L, 0));
	points.push_back(Point3f(W, L, H));
	points.push_back(Point3f(0, L, H));
	points.push_back(Point3f(0, L, 0));
}
void generate_faces(vector<Face>&faces)
{
	// 12 faces 
	faces.push_back(Face(4, 3, 0,11,5,6));
	faces.push_back(Face(4, 7, 3,11,10,5));
	faces.push_back(Face(2, 6, 7,4,9,10));
	faces.push_back(Face(2, 7, 3,4,10,5));

	faces.push_back(Face(1, 5, 2,3,8,4));
	faces.push_back(Face(5, 6, 2,8,9,4));
	faces.push_back(Face(0, 4, 1,2,7,3));
	faces.push_back(Face(4, 5, 1,7,8,3));

	faces.push_back(Face(4, 7, 5,12,13,8));
	faces.push_back(Face(7, 6, 5,13,9,8));
	faces.push_back(Face(0, 1, 2,0,3,4));
	faces.push_back(Face(0, 2, 3,0,4,1));

}

void generate_texture(Mat& texture,vector<Mat>&img_list,double L,double W,double H)
{

	img_list[0].copyTo(texture(Rect(H, 0, W, H)));
	img_list[1].copyTo(texture(Rect(0, H, H, L)));
	img_list[2].copyTo(texture(Rect(H, H, W, L)));
	img_list[3].copyTo(texture(Rect(H + W, H, H, L)));
	img_list[4].copyTo(texture(Rect(2 * H + W, H, W, L)));
	img_list[5].copyTo(texture(Rect(H, H + L, W, H)));
}
void generate_texture_coordinates(vector<Point2f>&tex_coord, double L, double W, double H)
{
	//14 tex_points
	//row 1:
	tex_coord.push_back(Point2f(H, 0));
	tex_coord.push_back(Point2f(H+W, 0));
	//row 2:
	tex_coord.push_back(Point2f(0, H));
	tex_coord.push_back(Point2f(H, H));
	tex_coord.push_back(Point2f(H+W, H));
	tex_coord.push_back(Point2f(2*H+W, H));
	tex_coord.push_back(Point2f(2*(H+W), H));

	//row 3:
	tex_coord.push_back(Point2f(0, H+L));
	tex_coord.push_back(Point2f(H, H+L));
	tex_coord.push_back(Point2f(H + W, H+L));
	tex_coord.push_back(Point2f(2 * H + W, H+L));
	tex_coord.push_back(Point2f(2 * (H + W), H+L));
	//row 4:
	tex_coord.push_back(Point2f(H, 2*H+L));
	tex_coord.push_back(Point2f(H + W, 2*H+L));
}
bool save_pointcloud_obj(const string name, vector<Point3f>points)
{
	FILE * fpt = fopen(name.c_str(), "w");
	if (!fpt)
	{
		printf("Unable to open %s", name.c_str());
		return false;
	}
	for (int i = 0; i < points.size(); i++)fprintf(fpt, "v %f %f %f\n", points[i].x, points[i].y, points[i].z);
	fclose(fpt);
	return true;
}

bool save_mesh_obj(const string name, const string mtl_name,vector<Point3f> &points, vector<Face>&faces,vector<Point2f>&tex_coord,Size size)
{
	FILE * fpt = fopen(name.c_str(), "w");
	if (!fpt)
	{
		printf("Unable to open %s", name.c_str());
		return false; 
	}
	fprintf(fpt, "mtllib %s\n", mtl_name.c_str());
	for (int i = 0; i < points.size(); i++)fprintf(fpt, "v %f %f %f\n", points[i].x, points[i].y, points[i].z);
	for (int i = 0; i < tex_coord.size(); i++)fprintf(fpt, "vt %f %f\n",tex_coord[i].x/size.width, 1-tex_coord[i].y/size.height);
	for (int i = 0; i < faces.size(); i++)fprintf(fpt, "f %d/%d %d/%d %d/%d\n", faces[i].x + 1, faces[i].xt + 1,faces[i].y + 1, faces[i].yt + 1, faces[i].z + 1, faces[i].zt + 1);
	fclose(fpt);
	return true;
}

void save_mtl(const string name, const string texture_name)
{
	FILE * fpt = fopen(name.c_str(), "w");
	fprintf(fpt, "newmtl Material.001\nNs 96.078431\nKa 0.000000 0.000000 0.000000\nKd 0.640000 0.640000 0.640000\nKs 0.500000 0.500000 0.500000\nNi 1.000000\nd 1.000000\nillum 2\n");
	fprintf(fpt, "map_Kd %s", texture_name.c_str());
	fclose(fpt);
}

void mouseHandler(int event, int x, int y, int flags, void* data_ptr)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		userdata *data = ((userdata *)data_ptr);
		circle(data->im, Point(x, y), 3, Scalar(0, 0, 255), 10);
		imshow("Image", data->im);
		if (data->points.size() < 4)
		{
			data->points.push_back(Point2f(x, y));
		}
	}

}


void gen_box_model()
{

	string img_dir = "./images/";
	string model_dir = "./model/";
	string name = "mesh";


	//generate cuboldobj
	
	vector<Point3f>points;
	vector<Point2f>tex_coord;
	vector<Face>faces;
	double L, W, H;
	cout << "please input length ,width and height :"<<endl;
	cin >> L >> W >> H;


	L *= 10;
	W *= 10;
	H *= 10;
	

	namedWindow("Image",0);
	vector<Size> size_list = { Size(W,H) ,Size(H,L),Size(W,L), Size(H,L) ,Size(W,L),Size(W,H) };
	// Read source image.

	vector<Mat> img_list;
	for (int i = 0; i < 6; i++)
	{
		string img_name = img_dir + to_string(i+1) + ".jpg";
		Mat im_src = imread(img_name);
		cout << "Click on the four corners of the book -- top left first and" << endl
			<< "bottom left last -- and then hit ENTER" << endl;
		vector<Point2f> pts_dst;
		Size size = size_list[i];
		cout <<"number:"<<i<<" "<<size.width << " " << size.height << endl;
		pts_dst.push_back(Point2f(0, 0));
		pts_dst.push_back(Point2f(size.width - 1, 0));
		pts_dst.push_back(Point2f(size.width - 1, size.height - 1));
		pts_dst.push_back(Point2f(0, size.height - 1));
		// Destination image. The aspect ratio of the book is 3/4
		Mat im_dst = Mat::zeros(size, CV_8UC3);

		// Create a vector of destination points.
		// Set data for mouse event
		Mat im_temp = im_src.clone();
		userdata data;
		data.im = im_temp;
		// Show image and wait for 4 clicks. 
		imshow("Image", im_temp);
		// Set the callback function for any mouse event
		setMouseCallback("Image", mouseHandler, &data);
		waitKey(0);

		if (data.points.size() == 0)
		{
			printf("Unable to get useful points");
			return;
		}
		// Calculate the homography
		Mat h = findHomography(data.points, pts_dst);

		// Warp source image to destination
		warpPerspective(im_src, im_dst, h, size);

		// Show image
		imshow("Image", im_dst);
		img_list.push_back(im_dst);
		waitKey(0);
	}
	Size size(2*(H+W), 2*H+L);
	Mat texture = Mat::zeros(size, CV_8UC3);
	generate_texture(texture, img_list,L,W,H);
	generate_points(points,L,W,H);
	generate_texture_coordinates(tex_coord, L, W, H);
	generate_faces(faces);
	save_mesh_obj(model_dir+name+".obj",name + ".mtl", points, faces, tex_coord, size);

	imwrite(model_dir + name + ".jpg", texture);
	save_mtl(model_dir + name + ".mtl", name + ".jpg");
	
}


CMD_BEG()
CMD0("tools.gen_box_model", gen_box_model)
CMD_END()


_STATIC_END

