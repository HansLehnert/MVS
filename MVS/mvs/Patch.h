#pragma once

#include <vector>
#include <opencv2\opencv.hpp>

#include "basic.h"

namespace mvs {
	struct Patch {
		View* source;
		cv::Point3d position;
		cv::Point3d normal;
		cv::Point3d tangent_1;
		cv::Point3d tangent_2;

		std::vector<View*> S;
		std::vector<View*> T;

		std::vector<cv::Point3d> grid_points;

		void optimize();
		double meanNcc();
	};
}