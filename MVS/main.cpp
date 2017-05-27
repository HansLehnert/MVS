#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <Windows.h>

#include <opencv2\opencv.hpp>
#include "mvs\mvs.h"


//Loads an image and a projection matrix from the SampleSet
void loadView(mvs::View* view, std::string scan, int index) {
	//Load image data
	std::stringstream image_path;
	image_path << "D:/Hans/Documents/SampleSet/SampleSet/MVS Data/Rectified/"
		<< scan << "/rect_"
		<< std::setw(3) << std::setfill('0') << index
		<< "_max.png";

	view->img = cv::imread(image_path.str());

	//Load projection matrix data
	std::stringstream calib_path;
	calib_path << "D:/Hans/Documents/SampleSet/SampleSet/MVS Data/Calibration/cal18/pos_"
		<< std::setw(3) << std::setfill('0') << index << ".txt";

	std::ifstream calib_file(calib_path.str());
	if (calib_file.is_open()) {
		view->P = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				calib_file >> view->P.at<double>(i, j);
			}
		}
	}
	else {
		std::cout << "Failed to open " << calib_path.str() << std::endl;
	}
}


int main(int argc, char* argv[]) {
	mvs::View view_1, view_2;
	loadView(&view_1, "scan6", 1);
	loadView(&view_2, "scan6", 2);

	cv::Mat grayscale;
	cv::cvtColor(view_1.img, grayscale, cv::COLOR_BGR2GRAY);
	std::vector<cv::Point2d> features;
	cv::goodFeaturesToTrack(grayscale, features, 50, 0.01, 20, cv::noArray(), 3, true);

	float a = 0;
	for (auto& point : features) {
		mvs::Ray3 cast_ray = mvs::castRay(&view_1, point);
		mvs::Ray2 projected_ray = mvs::projectRay(&view_2, cast_ray);

		cv::Scalar color(128 + 128 * sin(2 * 3.14 * a / features.size()),
			128 + 128 * sin(2 * 3.14 * (a / features.size() + 1.0 / 3)),
			128 + 128 * sin(2 * 3.14 * (a / features.size() + 2.0 / 3)));
		a += 1;

		cv::circle(view_1.img, point, 4, color, 8);
		cv::line(view_2.img, projected_ray.start, projected_ray.start + 100 * projected_ray.direction, color, 8);
	}
	
	//Create window
	const char* window_name = "Harris";
	int cv_window = cvNamedWindow(window_name);
	HWND h_window = (HWND)cvGetWindowHandle(window_name);

	if (!view_2.img.empty()) {
		cv::Mat display_image;
		cv::resize(view_1.img, display_image, cv::Size(0, 0), 0.5, 0.5);
		cv::imshow(window_name, display_image);

		cv::resize(view_2.img, display_image, cv::Size(0, 0), 0.5, 0.5);
		cv::imshow("Epipolar projection", display_image);
	}


	while (IsWindowVisible(h_window)) {
		if (cv::waitKey(10) != -1)
			break;
	}

	return 0;
}