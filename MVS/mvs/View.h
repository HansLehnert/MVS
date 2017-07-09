#pragma once

#include <vector>
#include <opencv2\opencv.hpp>

#include "basic.h"

namespace mvs {
	class View {
	public:
		View(cv::Mat, cv::Mat, int);

		cv::Mat img;
		cv::Mat P;

		//Camera paramters obtained from projection matrix
		cv::Point3d orientation;
		cv::Point3d position;

		//Grid dimensions
		int cell_size;
		int grid_w;
		int grid_h;

		//Ocuppancy grids
		std::vector<std::vector<std::vector<Patch*>>> T;
		std::vector<std::vector<std::vector<Patch*>>> S;

		//Depthmap
		std::vector<std::vector<std::pair<double, Patch*>>> depthmap;

		void computeDepthmap();
		void computeDepthmap(int, int);
	};
}