#include "Patch.h"

#include "photoconsistency.h"


using namespace mvs;


const int N_THETA = 20;
const int N_PHI = 5;

void Patch::optimize() {
	cv::Vec3d base_tangent_1(tangent_1);
	cv::Vec3d base_tangent_2(tangent_2);
	normal = base_tangent_1.cross(base_tangent_2);
	normal /= cv::sqrt(normal.ddot(normal));

	cv::Point3d best_tangent_1 = tangent_1;
	cv::Point3d best_tangent_2 = tangent_2;
	double best_ncc = meanNcc();
	for (int i = 0; i < N_THETA; i++) {
		double theta = i * CV_PI / N_THETA;

		tangent_1 = rotateVector(base_tangent_1, normal, theta);
		cv::Vec3d temp = rotateVector(base_tangent_2, normal, theta);
		for (int j = 0; j < N_PHI; j++) {
			double phi = j * CV_PI / N_PHI / 6;

			tangent_2 = rotateVector(temp, tangent_1 / cv::sqrt(tangent_1.ddot(tangent_1)), phi);

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
}


double Patch::meanNcc() {
	double result = 0;

	for (auto view : T) {
		if (view != source)
			result += ncc<5>(this, source, view); //Wont work for different values of grid size
	}

	return result / (T.size() - 1);
}