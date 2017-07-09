#include "Patch.h"

#include <nlopt.hpp>

#include "photoconsistency.h"


using namespace mvs;


//Optimization constants
const int N_PHI = 7;
const int N_THETA[N_PHI] = {3, 5, 9, 13, 17, 20};
const int N_POS = 5;

void Patch::optimize() {
	//Fixed transformation optimization
	/*cv::Vec3d base_tangent_1(tangent_1);
	cv::Vec3d base_tangent_2(tangent_2);
	cv::Vec3b norm_tangent_2 = base_tangent_2 / cv::sqrt(base_tangent_2.ddot(base_tangent_2));
	cv::Vec3d base_position(position);
	cv::Vec3d base_delta(normal);
	normal = base_tangent_1.cross(base_tangent_2);
	normal /= cv::sqrt(normal.ddot(normal));

	cv::Point3d best_tangent_1 = tangent_1;
	cv::Point3d best_tangent_2 = tangent_2;
	double best_ncc = meanNcc();
	for (int i = 1; i < N_PHI; i++) {
		double phi = i * CV_PI / N_PHI / 3;
		double d_theta = 2 * CV_PI / N_THETA[i];

		tangent_1 = rotateVector(base_tangent_1, norm_tangent_2, phi);

		for (int j = 0; j < N_THETA[i]; j++) {

			tangent_1 = rotateVector(tangent_1, normal, d_theta);
			tangent_2 = rotateVector(tangent_2, normal, d_theta);

			double ncc_value = meanNcc();
			if (ncc_value > best_ncc) {
				best_tangent_1 = tangent_1;
				best_tangent_2 = tangent_2;
				best_ncc = ncc_value;
			}
		}
	}

	tangent_1 = best_tangent_1;
	tangent_2 = best_tangent_2;
	normal = cv::Vec3d(tangent_1).cross(cv::Vec3d(tangent_2));
	normal /= cv::sqrt(normal.ddot(normal));*/


	//Gradient optimization
	/*cv::Point3d displacement_axis(normal);
	normal = tangent_1.cross(tangent_2);
	normal /= cv::sqrt(normal.ddot(normal));
	double ncc_score = meanNcc();
	double step_size = 0.1;
	for (int i = 0; i < 20; i++) {
		//Follow gradient of every variable
		double del_pos;
		double del_t1;
		double del_t2;

		cv::Point3d old_position = position;
		position += displacement_axis * 0.1;
		del_pos = (meanNcc() - ncc_score) / 0.1;
		position = old_position;

		cv::Point3d old_tangent_1 = tangent_1;
		tangent_1 = rotateVector(tangent_1, tangent_2 / cv::sqrt(tangent_2.ddot(tangent_2)), 0.1);
		del_t1 = (meanNcc() - ncc_score) / 0.1;
		tangent_1 = old_tangent_1;

		cv::Point3d old_tangent_2 = tangent_2;
		tangent_2 = rotateVector(tangent_2, tangent_1 / cv::sqrt(tangent_1.ddot(tangent_1)), 0.1);
		del_t2 = (meanNcc() - ncc_score) / 0.1;
		tangent_2 = old_tangent_2;

		position += displacement_axis * del_pos * step_size;
		tangent_1 = rotateVector(tangent_1, tangent_2 / cv::sqrt(tangent_2.ddot(tangent_2)), del_t1 * step_size * 10);
		tangent_2 = rotateVector(tangent_2, tangent_1 / cv::sqrt(tangent_1.ddot(tangent_1)), del_t1 * step_size * 10);

		step_size *= 0.95;

		ncc_score = meanNcc();
	}

	normal = cv::Vec3d(tangent_1).cross(cv::Vec3d(tangent_2));
	normal /= cv::sqrt(normal.ddot(normal));*/

	//External library optimization
	std::vector<double> x(3, 0);
	double max_val;

	std::vector<double> lower_bounds = { 0, -CV_PI, -10.0 };
	std::vector<double> upper_bounds = { CV_PI / 2, CV_PI, 10.0 };
	//std::vector<double> initial_step = { 0.2, 0.2, 10.0 };

	nlopt::opt opt(nlopt::LN_COBYLA, 3);
	opt.set_max_objective(meanNcc, (void*)this);
	opt.set_lower_bounds(lower_bounds);
	opt.set_upper_bounds(upper_bounds);
	opt.set_maxeval(30);
	
	nlopt::result result = opt.optimize(x, max_val);
	
	//Transform vector based on result
	cv::Vec3d norm_tangent_2 = tangent_2 / cv::sqrt(tangent_2.ddot(tangent_2));
	normal = tangent_1.cross(tangent_2);
	normal /= cv::sqrt(normal.ddot(normal));

	tangent_1 = rotateVector(tangent_1, norm_tangent_2, x[0]);
	tangent_1 = rotateVector(tangent_1, normal, x[1]);
	tangent_2 = rotateVector(tangent_2, normal, x[1]);
	normal = tangent_1.cross(tangent_2);
	normal /= cv::sqrt(normal.ddot(normal));

	position += cv::Point3d(x[2] * projection_dir);
}


void Patch::remove() {
	//Remove pointers from view grids
	for (auto& view : T) {
		cv::Point2i cell = mvs::projectPoint(view, position) / view->cell_size;

		view->T[cell.x][cell.y].erase(std::find(view->T[cell.x][cell.y].begin(), view->T[cell.x][cell.y].end(), this));
		view->computeDepthmap(cell.x, cell.y);
	}

	for (auto& view : S) {
		cv::Point2i cell = mvs::projectPoint(view, position) / view->cell_size;
		view->S[cell.x][cell.y].erase(std::find(view->S[cell.x][cell.y].begin(), view->S[cell.x][cell.y].end(), this));
	}
}


void Patch::findVisible(std::vector<View*>* views, double alpha) {
	for (auto& view : *views) {
		cv::Point2i cell = mvs::projectPoint(view, position) / view->cell_size;

		if (cell.x < 0 || cell.x >= view->grid_w || cell.y < 0 || cell.y >= view->grid_h)
			continue;

		if (view->depthmap[cell.x][cell.y].second == nullptr) {
			S.push_back(view);
			continue;
		}

		double depth = (position - view->position).dot(view->orientation);
		double min_depth = view->depthmap[cell.x][cell.y].first;

		//There's most certainly a better way of getting the focal length...
		//cv::Point2d projection = projectPoint(view, view->depthmap[cell.x][cell.y].second->position);
		//Ray3 ray = castRay(view, projection + cv::Point2d(view->cell_size, 0));
		//ray.direction /= view->orientation.dot(ray.direction);
		//cv::Point3d delta_vec = ray.start + ray.direction * min_depth - view->depthmap[cell.x][cell.y].second->position;
		//double delta = cv::sqrt(delta_vec.dot(delta_vec));

		double delta = getProjectedDistance(view, min_depth);

		if (depth < min_depth + delta) {
			S.push_back(view);
		}
	}

	for (auto& view : S) {
		cv::Point2i cell = mvs::projectPoint(view, position) / view->cell_size;

		if (cell.x < 0 || cell.x >= view->grid_w || cell.y < 0 || cell.y >= view->grid_h)
			continue;

		if (ncc<5>(this, source, view) > alpha) {
			T.push_back(view);
		}
	}
}


void Patch::registerViews() {
	for (auto& view : T) {
		cv::Point2i cell = cv::Point2i(mvs::projectPoint(view, position) / view->cell_size);
		view->T[cell.x][cell.y].push_back(this);
		view->computeDepthmap(cell.x, cell.y);
	}

	for (auto& view : S) {
		cv::Point2i cell = cv::Point2i(mvs::projectPoint(view, position) / view->cell_size);
		view->S[cell.x][cell.y].push_back(this);
	}
}


double mvs::Patch::meanNcc() {
	double x[3] = { 0 };

	return meanNcc(3, x, nullptr, this);
}


double mvs::Patch::meanNcc(unsigned n, const double * x, double * grad, void * func_data) {
	Patch* patch = static_cast<Patch*>(func_data);

	double result = 0;

	double phi = x[0];
	double theta = x[1];
	double offset = x[2];

	//Calculate transformed plane tangents
	cv::Vec3d norm_tangent_2 = patch->tangent_2 / cv::sqrt(patch->tangent_2.ddot(patch->tangent_2));
	cv::Vec3d normal = patch->tangent_1.cross(patch->tangent_2);
	normal /= cv::sqrt(normal.ddot(normal));

	cv::Vec3d tangent_1 = rotateVector(patch->tangent_1, norm_tangent_2, phi);
	tangent_1 = rotateVector(tangent_1, normal, theta);
	
	cv::Vec3d tangent_2 = rotateVector(patch->tangent_2, normal, theta);

	//Generate grid for NCC evaluation
	const int N = 5;
	cv::Point3d grid[N * N];
	for (int k = 0; k < 25; k++)
		grid[k] = (patch->position + offset * cv::Point3d(patch->projection_dir)) + cv::Point3d(tangent_1 * (k / N - N / 2) + tangent_2 * (k % N - N / 2));

	for (auto view : patch->T) {
		if (view != patch->source)
			result += ncc<5>(grid, patch->source, view); //Wont work for different values of grid size
	}

	return result / (patch->T.size() - 1);
}