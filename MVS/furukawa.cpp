#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>

#include <Windows.h>

#include <opencv2\opencv.hpp>
#include "mvs\mvs.h"


//Algorithm parameters

const int BETA_1 = 3;  //Grid size for ...
const int BETA_2 = 32; //Grid size for feature extraction
const int ETA = 4;     //Features per grid cell
const int MU = 5;      //Size of the grid used for photoconsistency calculation
const int GAMMA = 3;   //Amount of photoconsistent patches needed for detection
const double ALPHA_0 = 0.4;
const double ALPHA_1 = 0.7;

//Additional parameters

const int IMAGE_BORDER = 100;

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


void printTime() {
	static time_t start_time = time(0);
	int elapsed_time = difftime(time(0), start_time);
	std::cout << "[" << std::setfill('0') << std::setw(2) << elapsed_time / 3600 << ":"
		<< std::setfill('0') << std::setw(2) << elapsed_time % 3600 / 60 << ":"
		<< std::setfill('0') << std::setw(2) << elapsed_time % 60 << "]\t";
}


int main(int argc, char* argv[]) {
	cv::Mat display_image;

	//Reconstruction algorithm

	//Load images and cameras
	std::vector<mvs::View*> views;
	for (int i = 1; i <= 15; i++) {
		printTime();
		std::cout << "Loading view " << i << "..." << std::endl;

		mvs::View* view = new mvs::View;
		loadView(view, "scan6", i);
		view->S.resize(view->img.cols / BETA_1 * view->img.rows / BETA_1);
		view->C.resize(view->img.cols / BETA_1 * view->img.rows / BETA_1);
		
		views.push_back(view);
	}

	//Display views side by side
	//display_image = cv::Mat(views[0]->img.rows, views[0]->img.cols * 2, CV_8UC3);
	//views[0]->img.copyTo(display_image(cv::Rect(0, 0, 1600, 1200)));
	//views[1]->img.copyTo(display_image(cv::Rect(1600, 0, 1600, 1200)));
	display_image = views[0]->img;


	//Find features
	printTime();
	std::cout << "Extracting features." << std::endl;
	std::map<mvs::View*, std::vector<mvs::Feature>> features;
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
		for (int i = 0; i < (view->img.rows - 2 * IMAGE_BORDER) / BETA_2; i++) {
			for (int j = 0; j < (view->img.cols - 2 * IMAGE_BORDER) / BETA_2; j++) {
				float max_values[ETA] = { 0 };
				cv::Point cell_features[ETA];
				
				for (int m = 0; m < BETA_2; m++) {
					for (int n = 0; n < BETA_2; n++) {
						float harris_value = harris_img.at<float>(i * BETA_2 + m + IMAGE_BORDER, j * BETA_2 + n + IMAGE_BORDER);
						if (harris_value > max_values[0]) {
							max_values[0] = harris_value;
							cell_features[0] = cv::Point(j * BETA_2 + n + IMAGE_BORDER, i * BETA_2 + m + IMAGE_BORDER);

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
						mvs::Feature detected_feature;
						detected_feature.src = view;
						detected_feature.pos = cell_features[k];

						features[view].push_back(detected_feature);
						feature_map[view].at<unsigned char>(cell_features[k]) = 255;
					}
					else {
						break;
					}
				}
			}
		}
	}

	//Match features
	printTime();
	std::cout << "Initial feature match." << std::endl;
	std::map<mvs::View*, cv::Mat> depth_maps;
	std::vector<std::pair<double, mvs::Feature*>> matches;
	std::vector<mvs::Patch> patches;
	for (int i = 1; i <= views.size(); i++) {
		printTime();
		std::cout << "Matching view " << i << "..." << std::endl;

		mvs::View* view = views[i - 1];

		int rows = view->img.rows;
		int cols = view->img.cols;

		//cv::Vec3d camera_front = mvs::castRay(view, cv::Point2d((cols - 1) / 2, (rows - 1) / 2)).direction;
		cv::Vec3d camera_front = mvs::getCameraOrientation(view);

		//features[view] = std::vector<mvs::Feature>(1, features[view][900]);

		//Iterate over ever detected feature
		for (auto& feature : features[view]) {
			int cell = feature.pos.x / BETA_1 + feature.pos.y / BETA_1 * view->img.rows / BETA_1;
			if (!view->C[cell].empty()) {
				continue;
			}
			
			mvs::Ray3 ray = mvs::castRay(view, feature.pos);
			feature.normal = -ray.direction;
			ray.direction /= camera_front.ddot(ray.direction);

			matches.clear();

			//Find features along the epipolar lines
			for (auto neighbor : views) {
				if (view == neighbor)
					continue;

				mvs::Ray2 projected_ray = mvs::projectRay(neighbor, ray);
				double ray_magnitude = sqrt(projected_ray.direction.ddot(projected_ray.direction));


				for (auto& match : features[neighbor]) {
					double dist = (match.pos - projected_ray.start).cross(projected_ray.direction) / ray_magnitude;
					//Check if distance to projected ray is less than 2
					if (dist <= 2 && dist >= -2) {
						double depth = (match.pos.x - projected_ray.start.x) / projected_ray.direction.x; //Not the real depth
						matches.push_back(std::pair<double, mvs::Feature*>(depth, &match));
					}
				}
			}

			if (matches.size() == 0) {
				continue;
			}

			//Sort features based on distance to the camera center
			std::sort(matches.begin(), matches.end(), [](std::pair<double, mvs::Feature*>& a, std::pair<double, mvs::Feature*>& b) {
				return a.first < b.first;
			});

			//Find the projections of the feature ROI
			/*cv::Vec3b source_pixels[MU * MU];
			mvs::Ray3 grid_rays[MU * MU];
			cv::Point3d grid_points[MU * MU];

			for (int i = 0; i < MU * MU; i++) {
				cv::Point pos = cv::Point((int)feature.position.x - 1 + (i % MU), (int)feature.position.y - 1 + (i / 3));
				grid_rays[i] = mvs::castRay(view, cv::Point2f(pos));
				grid_rays[i].direction /= camera_front.ddot(grid_rays[i].direction);
				source_pixels[i] = view->img.at<cv::Vec3b>(pos);
			}

			//Find photoconsistent match
			for (auto& match : matches) {
				//Get global coordinates of pixel grid
				for (int i = 0; i < MU * MU; i++) {
					grid_points[i] = grid_rays[i].start + match.first * grid_rays[i].direction;
				}

				for (auto neighbor : views) {
					if (view == neighbor)
						continue;

					cv::Vec3b projected_pixels[MU * MU];

					for (int i = 0; i < MU * MU; i++) {
						mvs::Ray2 projected_ray = projectRay(neighbor, grid_rays[i]);
						cv::Point projected_point = projected_ray.start + match.first * projected_ray.direction;
					}

					double ncc_score = mvs::ncc<MU>(source_pixels, projected_pixels);

					if (ncc_score > ALPHA_0) {
						mvs::Ray2 projected_ray = projectRay(neighbor, ray);
			
						cv::circle(display_image, projected_ray.start + match.first * projected_ray.direction + cv::Point2d(1600, 0), 4, cv::Scalar(255), 2);
						cv::circle(display_image, feature.position, 4, cv::Scalar(255), 2);
						//cv::line(display_image, feature.position, projected_ray.start + match.first * projected_ray.direction + cv::Point2d(1600, 0), cv::Scalar(255), 2);
						
						break;
					}
				}
			}*/

			//Find photoconsistent match
			mvs::Ray3 ray_h = mvs::castRay(view, feature.pos + cv::Point2d(1, 0));
			mvs::Ray3 ray_v = mvs::castRay(view, feature.pos + cv::Point2d(0, 1));
			ray_h.direction /= camera_front.ddot(ray_h.direction);
			ray_v.direction /= camera_front.ddot(ray_v.direction);

			int n = 0;
			for (auto& match : matches) {
				mvs::Patch patch;
				patch.source = view;
				patch.normal = -camera_front;
				patch.position = mvs::triangulate(feature.src, feature.pos, match.second->src, match.second->pos);

				patch.tangent_1 = ((patch.position - ray.start).ddot(camera_front) * ray_h.direction + ray.start - patch.position) * 1.4;
				patch.tangent_2 = ((patch.position - ray.start).ddot(camera_front) * ray_v.direction + ray.start - patch.position) * 1.4;

				patch.T.push_back(view);
				patch.S.push_back(view);

				for (auto& neighbor : views) {
					if (neighbor == view)
						continue;

					double ncc_score = mvs::ncc<MU>(&patch, view, neighbor);

					if (ncc_score > ALPHA_0) {
						patch.S.push_back(neighbor);
					}
					if (ncc_score > ALPHA_1) {
						patch.T.push_back(neighbor);
					}
				}

				if (patch.T.size() >= GAMMA) {
					//Refine patch normal
					patch.optimize();

					patches.push_back(patch);

					for (auto neighbor : patch.T) {
						cv::Point2d projection = mvs::projectPoint(neighbor, patch.position);
						int cell_index = feature.pos.x / BETA_1 + feature.pos.y / BETA_1 * view->img.rows / BETA_1;
						neighbor->C[cell_index].push_back(&patches.back());
					}
					break;
				}
			}
		}

		std::cout << "\t\tFinished. Total patches: " << patches.size() << std::endl;


		if (i == 2) {
			break;
		}
	}


	//Write output file
	printTime();
	std::cout << "Writing PLY file..." << std::endl;
	std::ofstream output_file("output.ply", std::ios::binary);
	if (output_file.is_open()) {
		output_file << "ply" << std::endl;
		output_file << "format binary_little_endian 1.0" << std::endl;
		output_file << "element vertex " << patches.size() << std::endl;
		output_file << "property float x" << std::endl;
		output_file << "property float y" << std::endl;
		output_file << "property float z" << std::endl;
		output_file << "property uchar red" << std::endl;
		output_file << "property uchar green" << std::endl;
		output_file << "property uchar blue" << std::endl;
		output_file << "element face 0" << std::endl;
		output_file << "property list uchar int vertex_indices" << std::endl;
		output_file << "end_header" << std::endl;

		float mean_x = 0;
		float mean_y = 0;
		float mean_z = 0;
		//float max = 0;
		for (auto& patch : patches) {
			mean_x += patch.position.x;
			mean_y += patch.position.y;
			mean_z += patch.position.z;
		}

		mean_x /= patches.size();
		mean_y /= patches.size();
		mean_z /= patches.size();

		for (auto& patch : patches) {
			cv::Vec3b color = mvs::bilinearSample(&patch.source->img, mvs::projectPoint(patch.source, patch.position));

			float x = patch.position.x - mean_x;
			float y = patch.position.y - mean_y;
			float z = patch.position.z - mean_z;

			output_file.write(reinterpret_cast<const char*>(&x), sizeof(x));
			output_file.write(reinterpret_cast<const char*>(&y), sizeof(y));
			output_file.write(reinterpret_cast<const char*>(&z), sizeof(z));
			output_file.write((const char*)&color[2], 1);
			output_file.write((const char*)&color[1], 1);
			output_file.write((const char*)&color[0], 1);
		}

		output_file.close();
		std::cout << "\t\tFinished." << std::endl;
	}
	else {
		std::cout << "\t\tFailed to write file.";
	}
	

	//Draw patches
	for (auto& patch : patches) {
		cv::circle(display_image, mvs::projectPoint(views[0], patch.position), 4, cv::Scalar(0, 0, 255), 2);
		cv::line(display_image, mvs::projectPoint(views[0], patch.position), mvs::projectPoint(views[0], patch.position + 10 * patch.normal), cv::Scalar(255), 2);
	}


	//Create window
	const char* window_name = "Output";
	int cv_window = cvNamedWindow(window_name);
	HWND h_window = (HWND)cvGetWindowHandle(window_name);

	if (!display_image.empty()) {
		cv::Mat resized_display_image;
		//cv::resize(display_image, resized_display_image, cv::Size(0, 0), 1, 1);
		cv::resize(display_image, resized_display_image, cv::Size(0, 0), 0.5, 0.5);
		cv::imshow(window_name, resized_display_image);

		while (IsWindowVisible(h_window)) {
			if (cv::waitKey(10) != -1)
				break;
		}
	}

	//Delete loaded views
	for (auto view : views) {
		delete view;
	}

	return 0;
}