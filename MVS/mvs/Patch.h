#pragma once

#include <vector>
#include <opencv2\opencv.hpp>

#include "basic.h"

namespace mvs {
	class Patch {
	public:
		View* source;
		cv::Point3d position;
		cv::Vec3d normal;
		cv::Vec3d projection_dir;
		cv::Vec3d tangent_1;
		cv::Vec3d tangent_2;

		std::vector<View*> S;
		std::vector<View*> T;

		void optimize();
		void findVisible(std::vector<View*>*, double);
		void registerViews();
		void remove();

		double meanNcc();
		static double meanNcc(unsigned n, const double* x, double* grad, void* func_data);
	private:
		std::vector<cv::Point3d> grid_points;
	};
}