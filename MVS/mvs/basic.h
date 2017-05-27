#pragma once

#include <opencv2\opencv.hpp>

namespace mvs {
	struct Ray2 {
		cv::Point2d start;
		cv::Point2d direction;
	};
	

	struct Ray3 {
		cv::Point3d start;
		cv::Point3d direction;
	};


	struct View {
		cv::Mat img;
		cv::Mat P;
	};

	cv::Point3d getCameraPosition(View*);
	Ray3 castRay(View*, cv::Point2d);
	Ray2 projectRay(View*, Ray3);
}