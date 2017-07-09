#include "basic.h"

#include <limits>

#include "Patch.h"

using namespace mvs;


cv::Point3d mvs::getCameraPosition(View* view) {
	cv::Mat R = view->P(cv::Range(0, 3), cv::Range(0, 3));
	cv::Mat T = view->P(cv::Range(0, 3), cv::Range(3, 4));

	cv::Mat result = -R.inv() * T;

	cv::Point3d position;
	position.x = result.at<double>(0);
	position.y = result.at<double>(1);
	position.z = result.at<double>(2);

	return position;
}


cv::Point3d mvs::getCameraOrientation(View* view) {
	cv::Point3d result;
	result.x = view->P.at<double>(2, 0);
	result.y = view->P.at<double>(2, 1);
	result.z = view->P.at<double>(2, 2);

	result /= cv::sqrt(result.dot(result));

	return result;
}


std::vector<View*> mvs::findClosest(std::vector<View*>* views, View* target, int count) {
	cv::Point3d c1 = getCameraPosition(target);

	std::vector<float> distance(count, std::numeric_limits<float>::infinity());
	std::vector<View*> result(count);

	for (auto& view : *views) {
		if (view == target)
			continue;

		//...
	}

	return result;
}


cv::Point3d mvs::triangulate(View* src1, cv::Point2d p1, View* src2, cv::Point2d p2) {
	cv::Mat a = cv::Mat(4, 3, CV_64F);
	cv::Mat b = cv::Mat(4, 1, CV_64F);

	a.at<double>(0, 0) = src1->P.at<double>(2, 0) * p1.x - src1->P.at<double>(0, 0);
	a.at<double>(0, 1) = src1->P.at<double>(2, 1) * p1.x - src1->P.at<double>(0, 1);
	a.at<double>(0, 2) = src1->P.at<double>(2, 2) * p1.x - src1->P.at<double>(0, 2);
	a.at<double>(1, 0) = src1->P.at<double>(2, 0) * p1.y - src1->P.at<double>(1, 0);
	a.at<double>(1, 1) = src1->P.at<double>(2, 1) * p1.y - src1->P.at<double>(1, 1);
	a.at<double>(1, 2) = src1->P.at<double>(2, 2) * p1.y - src1->P.at<double>(1, 2);
	a.at<double>(2, 0) = src2->P.at<double>(2, 0) * p2.x - src2->P.at<double>(0, 0);
	a.at<double>(2, 1) = src2->P.at<double>(2, 1) * p2.x - src2->P.at<double>(0, 1);
	a.at<double>(2, 2) = src2->P.at<double>(2, 2) * p2.x - src2->P.at<double>(0, 2);
	a.at<double>(3, 0) = src2->P.at<double>(2, 0) * p2.y - src2->P.at<double>(1, 0);
	a.at<double>(3, 1) = src2->P.at<double>(2, 1) * p2.y - src2->P.at<double>(1, 1);
	a.at<double>(3, 2) = src2->P.at<double>(2, 2) * p2.y - src2->P.at<double>(1, 2);

	b.at<double>(0, 0) = -src1->P.at<double>(2, 3) * p1.x + src1->P.at<double>(0, 3);
	b.at<double>(1, 0) = -src1->P.at<double>(2, 3) * p1.y + src1->P.at<double>(1, 3);
	b.at<double>(2, 0) = -src2->P.at<double>(2, 3) * p2.x + src2->P.at<double>(0, 3);
	b.at<double>(3, 0) = -src2->P.at<double>(2, 3) * p2.y + src2->P.at<double>(1, 3);

	cv::Mat aT;
	cv::transpose(a, aT);

	cv::Mat result = (aT * a).inv() * aT * b;

	cv::Point3d point;
	point.x = result.at<double>(0, 0);
	point.y = result.at<double>(1, 0);
	point.z = result.at<double>(2, 0);

	return point;
}


cv::Vec3d mvs::rotateVector(cv::Vec3d base, cv::Vec3d axis, double angle) {
	//Rodrigues rotation formula
	return base * cos(angle) + axis.cross(base) * sin(angle) + axis * base.dot(axis) * (1 - cos(angle));
}


cv::Point2d mvs::projectPoint(View* view, cv::Point3d point) {
	double x_t = view->P.at<double>(0, 0) * point.x
		+ view->P.at<double>(0, 1) * point.y
		+ view->P.at<double>(0, 2) * point.z
		+ view->P.at<double>(0, 3);

	double y_t = view->P.at<double>(1, 0) * point.x
		+ view->P.at<double>(1, 1) * point.y
		+ view->P.at<double>(1, 2) * point.z
		+ view->P.at<double>(1, 3);

	double t = view->P.at<double>(2, 0) * point.x
		+ view->P.at<double>(2, 1) * point.y
		+ view->P.at<double>(2, 2) * point.z
		+ view->P.at<double>(2, 3);

	return cv::Point2d(x_t / t, y_t / t);
}


Ray3 mvs::castRay(View* view, cv::Point2d point) {
	cv::Point3d camera_center = getCameraPosition(view);

	cv::Point3d n1;
	n1.x = view->P.at<double>(2, 0) * point.x - view->P.at<double>(0, 0);
	n1.y = view->P.at<double>(2, 1) * point.x - view->P.at<double>(0, 1);
	n1.z = view->P.at<double>(2, 2) * point.x - view->P.at<double>(0, 2);

	cv::Point3d n2;
	n2.x = view->P.at<double>(2, 0) * point.y - view->P.at<double>(1, 0);
	n2.y = view->P.at<double>(2, 1) * point.y - view->P.at<double>(1, 1);
	n2.z = view->P.at<double>(2, 2) * point.y - view->P.at<double>(1, 2);

	Ray3 result;
	result.start = camera_center;
	result.direction = n2.cross(n1);
	result.direction /= cv::sqrt(result.direction.dot(result.direction));

	return result;
}


Ray2 mvs::projectRay(View* view, Ray3 ray) {
	cv::Mat points(4, 2, CV_64F);
	points.at<double>(0, 0) = ray.start.x;
	points.at<double>(1, 0) = ray.start.y;
	points.at<double>(2, 0) = ray.start.z;
	points.at<double>(3, 0) = 1;
	
	points.at<double>(0, 1) = ray.start.x + ray.direction.x;
	points.at<double>(1, 1) = ray.start.y + ray.direction.y;
	points.at<double>(2, 1) = ray.start.z + ray.direction.z;
	points.at<double>(3, 1) = 1;

	cv::Mat points_t = view->P * points;

	Ray2 result;

	result.start.x = points_t.at<double>(0, 0) / points_t.at<double>(2, 0);
	result.start.y = points_t.at<double>(1, 0) / points_t.at<double>(2, 0);

	result.direction.x = result.start.x - points_t.at<double>(0, 1) / points_t.at<double>(2, 1);
	result.direction.y = result.start.y - points_t.at<double>(1, 1) / points_t.at<double>(2, 1);
	//result.direction /= cv::sqrt(result.direction.dot(result.direction));

	return result;
}


cv::Point3d mvs::intersect(Ray3 ray, Patch* patch) {
	double k = (patch->position - ray.start).dot(patch->normal) / ray.direction.dot(patch->normal);
	return ray.start + k * ray.direction;
}