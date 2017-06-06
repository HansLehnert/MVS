#include "Patch.h"

#include "photoconsistency.h"


using namespace mvs;


const int N_PHI = 7;
const int N_THETA[N_PHI] = {3, 5, 9, 13, 17, 20};
const int N_POS = 5;

void Patch::optimize() {
	cv::Vec3d base_tangent_1(tangent_1);
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
	normal /= cv::sqrt(normal.ddot(normal));


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
}


double Patch::meanNcc() {
	double result = 0;

	for (auto view : T) {
		if (view != source)
			result += ncc<5>(this, source, view); //Wont work for different values of grid size
	}

	return result / (T.size() - 1);
}