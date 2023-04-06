#define BACKWARD_HAS_DW 1
#include "backward.hpp"

namespace backward
{
	backward::SignalHandling sh;
}

// STL
#include <iomanip>
#include <iostream>
#include <fstream>
// Boost
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// Local
#include <camera_model/camera_models/CameraFactory.h>
#include <camera_model/calib/CameraCalibration.h>
#include <camera_model/chessboard/Chessboard.h>
#include <camera_model/gpl/gpl.h>
#include <camera_model/code_utils/cv_utils.h>

int instinsicCalib(int argc, char** argv)
{
	cv_utils::fisheye::PreProcess* preprocess;
	cv::ocl::setUseOpenCL(false);
	cv::Size boardSize;
	float squareSize;
	std::string inputDir;
	std::string cameraModel;
	std::string cameraName;
	std::string prefix;
	std::string fileExtension;

	bool useOpenCV;
	bool viewResults;
	bool verbose;
	bool is_save_images;
	std::string point_file;
	std::string result_images_save_folder;

	float resize_scale = 1.0;
	cv::Size cropper_size(0, 0);
	cv::Point cropper_center(100, 100);
	bool is_first_run = true;

	//========= Handling Program options =========

	/* clang-format off */
	using namespace boost::program_options;
	boost::program_options::options_description desc( "Allowed options.\n Ask GAO Wenliang if there is any possible questions.\n");
	desc.add_options()
		("help", "produce help message")
		("width,w", value< int >(&boardSize.width)->default_value(8), "Number of inner corners on the chessboard pattern in x direction")
		("height,h", value< int >(&boardSize.height)->default_value(12), "Number of inner corners on the chessboard pattern in y direction")
		("size,s", value< float >(&squareSize)->default_value(7.f), "Size of one square in mm")
		("input,i", value< std::string >(&inputDir)->default_value("calibrationdata"), "Input directory containing chessboard images")
		("prefix,p", value< std::string >(&prefix)->default_value(""), "Prefix of images")
		("file-extension,e", value< std::string >(&fileExtension)->default_value(".png"), "File extension of images")
		("camera-model", value< std::string >(&cameraModel)->default_value("mei"), "Camera model: kannala-brandt | fov | scaramuzza | mei | pinhole | myfisheye")
		("camera-name", value< std::string >(&cameraName)->default_value("camera"), "Name of camera")
		("opencv", value< bool >(&useOpenCV)->default_value(true), "Use OpenCV to detect corners")
		("view-results", value< bool >(&viewResults)->default_value(true), "View results")
		("verbose,v", value< bool >(&verbose)->default_value(false), "Verbose output")
		("save_result", value< bool >(&is_save_images)->default_value(true), "save calibration result chessboard point.")
		("result_images_save_folder", value< std::string  >(&result_images_save_folder)->default_value("calib_images"), " calibration result images save folder.")
		("point_file", value< std::string  >(&point_file)->default_value("calib_images"), " calibration result images save folder.")
		("resize-scale", value< float >(&resize_scale)->default_value(1.0f), "resize scale")
		("cropper_width", value< int >(&cropper_size.width)->default_value(0), "cropper image width")
		("cropper_height", value< int >(&cropper_size.height)->default_value(0), "cropper image height")
		("center_x", value< int >(&cropper_center.x)->default_value(0), "cropper image center x ")
		("center_y", value< int >(&cropper_center.y)->default_value(0), "cropper image center y ")
		;
	/* clang-format on */
	boost::program_options::positional_options_description pdesc;
	pdesc.add("input", 1);

	boost::program_options::variables_map vm;
	boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
		.options(desc)
		.positional(pdesc)
		.run(),
		vm);
	boost::program_options::notify(vm);

	if (vm.count("help"))
	{
		std::cout << desc << std::endl;
		return 1;
	}

	if (!boost::filesystem::exists(inputDir) && !boost::filesystem::is_directory(inputDir))
	{
		std::cerr << "# ERROR: Cannot find input directory " << inputDir << "." << std::endl;
		return 1;
	}

	camera_model::Camera::ModelType modelType;
	if (boost::iequals(cameraModel, "kannala-brandt"))
	{
		modelType = camera_model::Camera::KANNALA_BRANDT;
	}
	else if (boost::iequals(cameraModel, "mei"))
	{
		modelType = camera_model::Camera::MEI;
	}
	else if (boost::iequals(cameraModel, "pinhole"))
	{
		modelType = camera_model::Camera::PINHOLE;
	}
	else if (boost::iequals(cameraModel, "pinhole2"))
	{
		modelType = camera_model::Camera::PINHOLE_FULL;
	}
	else if (boost::iequals(cameraModel, "scaramuzza"))
	{
		modelType = camera_model::Camera::SCARAMUZZA;
	}
	else if (boost::iequals(cameraModel, "myfisheye"))
	{
		modelType = camera_model::Camera::POLYFISHEYE;
	}
	else if (boost::iequals(cameraModel, "spline"))
	{
		modelType = camera_model::Camera::SPLINE;
	}
	else if (boost::iequals(cameraModel, "fov"))
	{
		modelType = camera_model::Camera::FOV;
	}
	else
	{
		std::cerr << "# ERROR: Unknown camera model: " << cameraModel << std::endl;
		return 1;
	}

	switch (modelType)
	{
	case camera_model::Camera::KANNALA_BRANDT:
		std::cout << "# INFO: Camera model: Kannala-Brandt" << std::endl;
		break;
	case camera_model::Camera::MEI:
		std::cout << "# INFO: Camera model: Mei" << std::endl;
		break;
	case camera_model::Camera::PINHOLE:
		std::cout << "# INFO: Camera model: Pinhole" << std::endl;
		break;
	case camera_model::Camera::PINHOLE_FULL:
		std::cout << "# INFO: Camera model: Full Pinhole Model" << std::endl;
		break;
	case camera_model::Camera::SCARAMUZZA:
		std::cout << "# INFO: Camera model: Scaramuzza-Omnidirect" << std::endl;
		break;
	case camera_model::Camera::SPLINE:
		std::cout << "# INFO: Camera model: spline camera model" << std::endl;
		break;
	case camera_model::Camera::FOV:
		std::cout << "# INFO: Camera model: FOV camera model" << std::endl;
		break;
	case camera_model::Camera::POLYFISHEYE:
		std::cout << "# INFO: Camera model: GaoWenliang's polynomial fisheye model" << std::endl;
		break;
	}

	// look for images in input directory
	std::vector< std::string > imageFilenames;
	boost::filesystem::directory_iterator itr;
	for (boost::filesystem::directory_iterator itr(inputDir);
		itr != boost::filesystem::directory_iterator();
		++itr)
	{
		if (!boost::filesystem::is_regular_file(itr->status()))
		{
			continue;
		}
		std::string filename = itr->path().filename().string();
		// check if prefix matches
		if (!prefix.empty())
		{
			if (filename.compare(0, prefix.length(), prefix) != 0)
			{
				continue;
			}
		}
		// check if file extension matches
		if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension)
			!= 0)
		{
			continue;
		}

		imageFilenames.push_back(itr->path().string());

		if (verbose)
		{
			std::cerr << "# INFO: Adding " << imageFilenames.back() << std::endl;
		}
	}

	if (imageFilenames.empty())
	{
		std::cerr << "# ERROR: No chessboard images found." << std::endl;
		return 1;
	}

	if (verbose)
	{
		std::cerr << "# INFO: # images: " << imageFilenames.size() << std::endl;
	}

	cv::Size frameSize;
	if (is_first_run)
	{
		cv::Mat image_src = cv::imread(imageFilenames.front(), -1);

		preprocess = new cv_utils::fisheye::PreProcess(cv::Size(image_src.cols, //
			image_src.rows),
			cropper_size,
			cropper_center,
			resize_scale);
		cv::Mat image = preprocess->do_preprocess(image_src);
		frameSize = image.size();
		std::cout << "frameSize " << frameSize << "\n";
	}

	//double startTime_0 = camera_model::timeInSeconds();

	// TODO need to change mode type
	camera_model::CameraCalibration calibration(modelType, cameraName, frameSize, boardSize, squareSize);
	calibration.setVerbose(verbose);

	std::vector< bool > chessboardFound(imageFilenames.size(), false);

	int image_index;
#pragma omp parallel for private( image_index )
	for (image_index = 0; image_index < imageFilenames.size(); ++image_index)
	{
		std::string image_name = imageFilenames.at(image_index);

		cv::Mat image = preprocess->do_preprocess(cv::imread(image_name, -1));

		camera_model::Chessboard chessboard(boardSize, image);

		chessboard.findCorners(useOpenCV);
		if (chessboard.cornersFound())
		{
			std::cerr << "# INFO: Detected chessboard in image " << image_index + 1 << ", "
				<< imageFilenames.at(image_index) << std::endl;

			calibration.addChessboardData(chessboard.getCorners());
			calibration.addImage(image, image_name);

			cv::Mat sketch;
			chessboard.getSketch().copyTo(sketch);
		}
		else
		{
			std::cout << "\033[31;47;1m"
				<< "# INFO: Did not detect chessboard in image: "
				<< imageFilenames.at(image_index) << "\033[0m" << std::endl;
		}
		chessboardFound.at(image_index) = chessboard.cornersFound();
	}

	if (calibration.sampleCount() < 1)
	{
		std::cerr << "# ERROR: Insufficient number of detected chessboards." << std::endl;
		return 1;
	}

	std::cerr << "# INFO: Calibrating..." << std::endl;

	//double startTime = camera_model::timeInSeconds();

	std::cout << " Calibrate start." << std::endl;
	calibration.calibrate();

	std::cout << " Calibrate done." << std::endl;

	calibration.writeParams(cameraName + "_camera_calib.yaml");
	//    calibration.writeChessboardData( cameraName + "_chessboard_data.dat" );

	/*std::cout << "# INFO: Calibration took a total time of " << std::fixed
		<< std::setprecision(3) << camera_model::timeInSeconds() - startTime_0
		<< " sec, core calibration cost "
		<< camera_model::timeInSeconds() - startTime << " sec." << std::endl;*/

	std::cerr << "# INFO: Wrote calibration file to " << cameraName + "_camera_calib.yaml" << std::endl;

	if (viewResults && verbose)
	{
		std::cout << "\033[32;40;1m"
			<< "# INFO: Used image num: " << calibration.m_ImagesShow.size()
			<< "\033[0m" << std::endl;
		std::cout << "details shown in the images,"
			<< "\033[32;40;1m"
			<< "green points is observed points, "
			<< "\033[31;47;1m"
			<< "red points is estimated points. "
			<< "\033[0m" << std::endl;

		cv::Mat pointDistributedImage = cv::Mat(calibration.m_ImagesShow[0].rows,
			calibration.m_ImagesShow[0].cols,
			CV_8UC3,
			cv::Scalar(0));
		cv::Mat pointDistributedImageGood = cv::Mat(calibration.m_ImagesShow[0].rows,
			calibration.m_ImagesShow[0].cols,
			CV_8UC3,
			cv::Scalar(0));

		// visualize observed and reprojected points
		calibration.drawResultsInitial(calibration.m_ImagesShow, calibration.m_ImageNames, pointDistributedImage);
		calibration.drawResultsFiltered(calibration.m_ImagesGoodShow,
			calibration.m_ImageNames,
			pointDistributedImageGood);

		calibration.save2D(point_file);
		cv::namedWindow("point Distributed Image", cv::WINDOW_NORMAL);
		cv::imshow("point Distributed Image", pointDistributedImage);
		cv::namedWindow("good point Distributed Image", cv::WINDOW_NORMAL);
		cv::imshow("good point Distributed Image", pointDistributedImageGood);
		for (size_t i = 0; i < calibration.m_ImagesShow.size(); ++i)
		{
			cv::namedWindow("Image", cv::WINDOW_NORMAL);
			cv::imshow("Image", calibration.m_ImagesShow.at(i));
			cv::namedWindow("Image GoodPoints", cv::WINDOW_NORMAL);
			cv::imshow("Image GoodPoints", calibration.m_ImagesGoodShow.at(i));

			if (is_save_images)
			{
				std::ostringstream ss;
				ss << i;
				cv::imwrite(result_images_save_folder + "calib_result_" + ss.str()
					+ ".jpg",
					calibration.m_ImagesGoodShow.at(i));
			}
			cv::waitKey(0);
		}
	}
	return 0;
}

bool loadKITTIData(const std::string& filename, std::vector<cv::Point3d> &cloud)
{
	std::fstream input(filename.c_str(), std::ios::in | std::ios::binary);
	if (!input.good())
	{
		return false;
	}
	for (int i = 0; input.good() && !input.eof(); ++i)
	{
		float temp = 0.f;
		float x = 0.f, y = 0.f, z = 0.f;
		input.read((char *)&x, sizeof(float));
		input.read((char *)&y, sizeof(float));
		input.read((char *)&z, sizeof(float));
		input.read((char *)&temp, sizeof(float));
		cv::Point3d point(x, y, z);
		cloud.emplace_back(point);
	}
	input.close();
	return true;
}

bool saveKITTData(const std::string &filename, std::vector<cv::Point3d> &cloud)
{
	std::ofstream ofs;
	ofs.open(filename, std::ios::out);
	if (!ofs)
	{
		std::cerr << "File failed to be writted. " << filename;
		return false;
	}
	std::cout.precision(12);
	for (int j = 0; j < cloud.size(); ++j)
	{
		ofs << cloud[j].x << "," << cloud[j].y << "," << cloud[j].z << std::endl;
	}
	// flush writing points
	ofs.flush();
	ofs.close();
	return true;
}

struct ColorElement
{
	ColorElement(const double &relativePos_, const int &r_, const int &g_, const int &b_)
	{
		relativePos = relativePos_;
		r = r_;
		g = g_;
		b = b_;
	}
	double relativePos;
	int r;
	int g;
	int b;
};
std::vector<ColorElement> ColorRamps;

//image_02: 0.9995185086 0.0041276589 - 0.0307524527 0.7264036936 - 0.0307926666 0.0100608424 - 0.9994751579 - 0.1499658517 - 0.0038160970 0.9999408692 0.0101830998 - 1.0686400091
//image_03 : -0.9996821702 0.0005703407 - 0.0252038325 0.7016842127 - 0.0252033830 0.0007820814 0.9996820384 0.7463650950 0.0005898709 0.9999995315 - 0.0007674583 - 1.0751978255

int main(int argc, char** argv)
{
	// load fisheye image
	std::string fisheye_img_path = "E:/codefiles/NewVS/PointImageFusion/data/FisheyeCalibration/0000000002_r.png";
	cv::Mat fisheyd_img = cv::imread(fisheye_img_path, 1);
	// load pc
	std::string pc_path = "E:/codefiles/NewVS/PointImageFusion/data/FisheyeCalibration/0000000002.bin";
	std::vector<cv::Point3d> pts_world;
	loadKITTIData(pc_path, pts_world);
	//std::string pc_path_out = "E:/codefiles/NewVS/PointImageFusion/data/FisheyeCalibration/test_results/0000000002.txt";
	//saveKITTData(pc_path_out, pts_world);
	// transform the pts from world coordinate to carema coordinate
	std::string rt_file = "E:/codefiles/NewVS/PointImageFusion/data/FisheyeCalibration/calib_cam_to_pose.txt";
	Eigen::Matrix4d T_matrix;
	// image_02: right
	T_matrix << 0.9995185086, 0.0041276589, -0.0307524527, 0.7264036936, -0.0307926666, 0.0100608424, -0.9994751579, -0.1499658517, -0.0038160970, 0.9999408692, 0.0101830998, -1.0686400091, 0.0, 0.0, 0.0, 1.0;
	// image_03: left
	//T_matrix << -0.9996821702, 0.0005703407, -0.0252038325, 0.7016842127, -0.0252033830, 0.0007820814, 0.9996820384, 0.7463650950, 0.0005898709, 0.9999995315, -0.0007674583, -1.0751978255, 0.0, 0.0, 0.0, 1.0;
	T_matrix.inverse();
	std::vector<cv::Point3d> pts;
	pts.resize(pts_world.size());
	for (int pt_id = 0; pt_id < pts_world.size(); ++pt_id)
	{
		pts[pt_id].x = T_matrix(0, 0) * pts_world[pt_id].x + T_matrix(0, 1) * pts_world[pt_id].y + T_matrix(0, 2) * pts_world[pt_id].z + T_matrix(0, 3);
		pts[pt_id].y = T_matrix(1, 0) * pts_world[pt_id].x + T_matrix(1, 1) * pts_world[pt_id].y + T_matrix(1, 2) * pts_world[pt_id].z + T_matrix(1, 3);
		pts[pt_id].z = T_matrix(2, 0) * pts_world[pt_id].x + T_matrix(2, 1) * pts_world[pt_id].y + T_matrix(2, 2) * pts_world[pt_id].z + T_matrix(2, 3);
	}
	// load config file
	std::string camera_model_file = "E:/codefiles/NewVS/PointImageFusion/data/FisheyeCalibration/0000000002_r.yaml";
	camera_model::CameraPtr cam;
	cam = camera_model::CameraFactory::instance()->generateCameraFromYamlFile(camera_model_file);
	// project the points to image plane
	std::vector<cv::Point3d> uv_pts;
	for (int i = 0; i < pts.size(); ++i)
	{
		Eigen::Vector3d P(pts[i].x, pts[i].y, pts[i].z);
		Eigen::Vector2d uv_pt;
		cam->spaceToPlane(P, uv_pt);
		double tmp = P.norm();
		cv::Point3d uv(uv_pt.x(),uv_pt.y(), tmp);
		uv_pts.emplace_back(uv);
	}
	// unproject 2d to 3d unit sphere
	std::vector<cv::Point3d> pts_unproject;
	for (int row = 0; row < fisheyd_img.rows; ++row)
	{
		for (int col = 0; col < fisheyd_img.cols; ++col)
		{
			Eigen::Vector3d P_dst;
			Eigen::Vector2d p(row, col);
			cam->liftSphere(p, P_dst);
			pts_unproject.emplace_back(cv::Point3d(P_dst.x(), P_dst.y(), P_dst.z()));
		}
	}
	//std::string pc_path_out2 = "E:/codefiles/NewVS/PointImageFusion/data/FisheyeCalibration/test_results/0000000002_unproject.txt";
	//saveKITTData(pc_path_out2, pts_unproject);
	// plot the pts on image plane
	// generate color ramp
	unsigned j = 0; //current interval
	ColorRamps.emplace_back(ColorElement(0.0, 0, 0, 255));
	ColorRamps.emplace_back(ColorElement(0.333333333333, 0, 255, 0));
	ColorRamps.emplace_back(ColorElement(0.666666666667, 255, 255, 0));
	ColorRamps.emplace_back(ColorElement(1.0, 255, 0, 0));
	std::vector<cv::Point3d> rgbaScale;
	rgbaScale.resize(32);
	// get only 1024 color level
	for (unsigned i = 0; i < 32; ++i)
	{
		const double relativePos = static_cast<double>(i) / 31;
		//forward to the right intervale
		while (j + 2 <= 4 && ColorRamps[j + 1].relativePos < relativePos)
			++j;
		// linear interpolation
		const cv::Point3d colBefore(ColorRamps[j].r, ColorRamps[j].g, ColorRamps[j].b);
		const cv::Point3d colNext(ColorRamps[j + 1].r, ColorRamps[j + 1].g, ColorRamps[j + 1].b);
		// interpolation coef
		const double alpha = (relativePos - ColorRamps[j].relativePos) / (ColorRamps[j + 1].relativePos - ColorRamps[j].relativePos);
		const cv::Point3d interpCol = colBefore + (colNext - colBefore) * alpha;
		// get color
		rgbaScale[i].x = interpCol.x;
		rgbaScale[i].y = interpCol.y;
		rgbaScale[i].z = interpCol.z;
	}
	double max_range = 120.0; // same with kitti360 officical scripts
	for (int p_i = 0; p_i < uv_pts.size(); p_i++)
	{
		int r = 0, g = 0, b = 0;
		if (uv_pts[p_i].z > max_range) {
			r = 255;
			g = 0;
			b = 0;
		}
		else {
			int index = static_cast<int>(uv_pts[p_i].z);
			r = rgbaScale[index].x;
			g = rgbaScale[index].y;
			b = rgbaScale[index].z;
		}
		cv::circle(fisheyd_img, cv::Point2i(uv_pts[p_i].x,uv_pts[p_i].y), 1, cv::Scalar(b, g, r), 1);
	}
	std::string fisheye_img_out_path = "E:/codefiles/NewVS/PointImageFusion/data/FisheyeCalibration/test_results/0000000002_r_pts.png";
	cv::imwrite(fisheye_img_out_path, fisheyd_img);
	system("pause");
	return 1;
}