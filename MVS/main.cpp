#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <Windows.h>

#include <opencv2\opencv.hpp>
#include "mvs\mvs.h"


const int BETA_1 = 3;  //Grid size for ...
const int BETA_2 = 32; //Grid size for feature extraction
const int ETA = 4;     //Features per grid cell


//Loads an image and a projection matrix from the SampleSet
void loadView(mvs::View* view, std::string scan, int index) {
	//Load image data
	std::stringstream image_path;
	image_path << "D:/Hans/Documents/SampleSet/SampleSet/MVS Data/Rectified/"
		<< scan << "/rect_"
		<< std::setw(3) << std::setfill('0') << index
		<< "_max.png";

	view->img = cv::imread(image_path.str());

	//Load projection matrix data
	std::stringstream calib_path;
	calib_path << "D:/Hans/Documents/SampleSet/SampleSet/MVS Data/Calibration/cal18/pos_"
		<< std::setw(3) << std::setfill('0') << index << ".txt";

	std::ifstream calib_file(calib_path.str());
	if (calib_file.is_open()) {
		view->P = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				calib_file >> view->P.at<double>(i, j);
			}
		}
	}
	else {
		std::cout << "Failed to open " << calib_path.str() << std::endl;
	}
}


int main(int argc, char* argv[]) {
	cv::Mat display_image;

	//Reconstruction algorithm

	//Load images and cameras
	std::vector<mvs::View*> views;
	for (int i = 1; i <= 2; i++) {
		views.push_back(new mvs::View);
		loadView(views.back(), "scan6", i);
	}

	//Find features
	/*std::map<mvs::View*, std::vector<cv::Point>> features;
	std::map<mvs::View*, cv::Mat> feature_map;
	for (auto view : views) {
		cv::Mat grayscale_img;
		cv::cvtColor(view->img, grayscale_img, cv::COLOR_BGR2GRAY);

		cv::Mat harris_img;
		cv::cornerHarris(grayscale_img, harris_img, 3, 3, 0.06);

		//Divide image in cells and extract features
		feature_map[view] = cv::Mat(view->img.rows, view->img.cols, CV_8U);
		feature_map[view] = 0;
		features[view].reserve(view->img.cols * view->img.rows * ETA / BETA_2 / BETA_2);
		for (int i = 0; i < view->img.rows / BETA_2; i++) {
			for (int j = 0; j < view->img.cols / BETA_2; j++) {
				float max_values[ETA] = { 0 };
				cv::Point cell_features[ETA];
				
				for (int m = 0; m < BETA_2; m++) {
					for (int n = 0; n < BETA_2; n++) {
						float harris_value = harris_img.at<float>(i * BETA_2 + m, j * BETA_2 + n);
						if (harris_value > max_values[0]) {
							max_values[0] = harris_value;
							cell_features[0] = cv::Point(j * BETA_2 + n, i * BETA_2 + m);

							for (int k = 1; k < ETA; k++) {
								if (max_values[k - 1] > max_values[k]) {
									float tmp_value = max_values[k];
									max_values[k] = max_values[k - 1];
									max_values[k - 1] = tmp_value;

									cv::Point tmp_point = cell_features[k];
									cell_features[k] = cell_features[k - 1];
									cell_features[k - 1] = tmp_point;
								}
								else {
									break;
								}
							}
						}
					}
				}
				
				for (int k = 0; k < ETA; k++) {
					if (max_values[k] > 1e-7) {
						features[view].push_back(cell_features[k]);
						feature_map[view].at<unsigned char>(cell_features[k]) = 255;
					}
					else {
						break;
					}
				}
			}
		}
		
		display_image = view->img;

		//break;
	}*/

	//Match features
	std::map<mvs::View*, cv::Mat> depth_maps;
	for (auto view : views) {
		int rows = view->img.rows;
		int cols = view->img.cols;

		cv::Mat depth_map(rows, cols, CV_32F);

		float min_match = -1;
		float max_match = 0;

		std::cout << "Computing depth map" << std::endl;

		for (int i = 3; i < rows - 3; i+=2) {
			for (int j = 3; j < cols - 3; j+=2) {
				mvs::Ray3 cast_ray = mvs::castRay(view, cv::Point(j, i));
				cv::Mat patch = view->img(cv::Rect(j - 1, i - 1, 3, 3));

				float best_score = 0;
				float best_match = -1;
				
				for (auto neighbor_view : views) {
					if (neighbor_view == view)
						continue;

					mvs::Ray2 projected_ray = mvs::projectRay(neighbor_view, cast_ray);

					//Cast a line that crosses the image
					cv::Point intersections[5];
					intersections[0] = projected_ray.start;
					intersections[1] = projected_ray.start - projected_ray.direction * (projected_ray.start.x - 3) / projected_ray.direction.x;
					intersections[2] = projected_ray.start - projected_ray.direction * (projected_ray.start.x - neighbor_view->img.cols + 4) / projected_ray.direction.x;
					intersections[3] = projected_ray.start - projected_ray.direction * (projected_ray.start.y - 3) / projected_ray.direction.y;
					intersections[4] = projected_ray.start - projected_ray.direction * (projected_ray.start.y - neighbor_view->img.rows + 4) / projected_ray.direction.y;

					int k = 0;
					cv::Point p[2];
					for (int i = 0; i < 5; i++) {
						if (intersections[i].x > 2 && intersections[i].x < neighbor_view->img.cols - 3 && intersections[i].y > 2 && intersections[i].y < neighbor_view->img.rows - 3) {
							p[k] = intersections[i];
							k++;

							if (k == 2)
								break;
						}
					}
					
					//If the projected line doesn't cross the neighboring image continue
					if (k != 2 || p[0] == p[1])
						continue;

					cv::LineIterator line(neighbor_view->img, p[0], p[1], 4);
					for (int i = 0; i < line.count; i++, ++line) {
						cv::Point pos = line.pos();
						float score = mvs::ncc<7>(&(neighbor_view->img), &patch, pos);
						if (score > best_score) {
							best_score = score;
							best_match = (pos.x - projected_ray.start.x) / projected_ray.direction.x;
						}
					}
				}

				depth_map.at<float>(i, j) = best_match;
				if (best_match != -1 && (best_match < min_match || min_match == -1)) {
					min_match = best_match;
				}
				if (max_match < best_match) {
					max_match = best_match;
				}
			}

			if (i % 100 == 1) {
				std::cout << "Rows finished: " << i << std::endl;
			}

			if (i >= 600)
				break;
		}

		depth_map = (depth_map - min_match) / (max_match - min_match);
		depth_maps[view] = depth_map;

		std::cout << max_match << std::endl;

		display_image = depth_map;

		break;
	}
	
	//Create window
	const char* window_name = "Output";
	int cv_window = cvNamedWindow(window_name);
	HWND h_window = (HWND)cvGetWindowHandle(window_name);

	if (!display_image.empty()) {
		cv::Mat resized_display_image;
		cv::resize(display_image, resized_display_image, cv::Size(0, 0), 1, 1);
		//cv::resize(display_image, resized_display_image, cv::Size(0, 0), 0.5, 0.5);
		cv::imshow(window_name, resized_display_image);

		while (IsWindowVisible(h_window)) {
			if (cv::waitKey(10) != -1)
				break;
		}
	}

	return 0;
}