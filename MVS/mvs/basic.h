#pragma once

#include <vector>
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


	struct Feature {
		View* source;
		cv::Point2d position;
		cv::Point3d normal;
	};


	cv::Point3d getCameraPosition(View*);


	cv::Point3d getCameraOrientation(View*);


	std::vector<View*> findClosest(std::vector<View*>*, View*, int);


	Ray3 castRay(View*, cv::Point2d);


	Ray2 projectRay(View*, Ray3);
}