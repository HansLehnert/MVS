#pragma once

#include <vector>
#include <opencv2\opencv.hpp>

namespace mvs {
	class View;
	class Patch;


	struct Ray2 {
		cv::Point2d start;
		cv::Point2d direction;
	};
	

	struct Ray3 {
		cv::Point3d start;
		cv::Point3d direction;
	};


	struct Feature {
		View* src;
		cv::Point2d pos;
		cv::Point3d normal;
	};
	
	
	cv::Point3d getCameraPosition(View*);


	cv::Point3d getCameraOrientation(View*);


	double getProjectedDistance(View*, double);


	std::vector<View*> findClosest(std::vector<View*>*, View*, int);


	cv::Point3d triangulate(View*, cv::Point2d, View*, cv::Point2d);


	cv::Point2d projectPoint(View*, cv::Point3d);


	cv::Vec3d rotateVector(cv::Vec3d, cv::Vec3d, double);


	Ray3 castRay(View*, cv::Point2d);


	Ray2 projectRay(View*, Ray3);


	cv::Point3d intersect(Ray3, Patch*);
}