#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <time.h>

#include <Windows.h>

#include <opencv2\opencv.hpp>
#include "mvs\mvs.h"


//Algorithm parameters

const int BETA_1 = 3;       //Grid size for ...
const int BETA_2 = 32;      //Grid size for feature extraction
const int ETA = 4;          //Features per grid cell
const int MU = 5;           //Size of the grid used for photoconsistency calculation
const int GAMMA = 3;        //Amount of photoconsistent patches needed for detection
const double ALPHA_0 = 0.4; //Weak match threshold
const double ALPHA_1 = 0.7; //Strong match threshold

//Additional parameters

const int IMAGE_BORDER = 20;

//Loads an image and a projection matrix from the SampleSet
mvs::View* loadView(std::string scan, int index) {
	//Load image data
	std::stringstream image_path;
	image_path << "D:/Hans/Documents/SampleSet/SampleSet/MVS Data/Rectified/"
		<< scan << "/rect_"
		<< std::setw(3) << std::setfill('0') << index
		<< "_max.png";

	cv::Mat img = cv::imread(image_path.str());

	if (img.empty()) {
		std::cout << "Failed to open " << image_path.str() << std::endl;
		return nullptr;
	}

	//Load projection matrix data
	std::stringstream calib_path;
	calib_path << "D:/Hans/Documents/SampleSet/SampleSet/MVS Data/Calibration/cal18/pos_"
		<< std::setw(3) << std::setfill('0') << index << ".txt";

	cv::Mat P;

	std::ifstream calib_file(calib_path.str());
	if (calib_file.is_open()) {
		P = cv::Mat(3, 4, CV_64F);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 4; j++) {
				calib_file >> P.at<double>(i, j);
			}
		}
	}
	else {
		std::cout << "Failed to open " << calib_path.str() << std::endl;
		return nullptr;
	}

	mvs::View* view = new mvs::View(img, P, BETA_1);

	return view;
}


void printTime() {
	static time_t start_time = time(0);
	int elapsed_time = (int)difftime(time(0), start_time);
	std::cout << "[" << std::setfill('0') << std::setw(2) << elapsed_time / 3600 << ":"
		<< std::setfill('0') << std::setw(2) << elapsed_time % 3600 / 60 << ":"
		<< std::setfill('0') << std::setw(2) << elapsed_time % 60 << "]\t";
}


int main(int argc, char* argv[]) {
	cv::Mat display_image;

	///////////////////////////////////////////////////////////////////////////
	//Reconstruction algorithm
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////
	//Load images and cameras
	std::vector<mvs::View*> views;
	for (int i = 21; i <= 30; i++) {
		printTime();
		std::cout << "Loading view " << i << "..." << std::endl;

		views.push_back(loadView("scan6", i));
	}

	///////////////////////////////////////////////////////////////////////////
	//Load bundler files

	/*if (argc < 2) {
		std::cout << "Missing input folder";
		return 0;
	}

	std::vector<mvs::View*> views;

	//Load views from bundler output
	std::string input_dir = argv[1];
	if (input_dir.back() != '/' || input_dir.back() != '\\')
		input_dir += '/';

	std::ifstream bundle_file(input_dir + std::string("bundle/bundle.out"));
	std::ifstream list_file(input_dir + std::string("prepare/list.txt"));

	if (!list_file.is_open() || !bundle_file.is_open()) {
		std::cout << "Failed to open bundler files";
		return 0;
	}

	bundle_file.ignore(1024, '\n');

	int n;
	bundle_file >> n;
	bundle_file.ignore(1024, '\n');

	for (int i = 0; i < n; i++) {
		mvs::View* view = new mvs::View;

		std::string img_filename;
		list_file >> img_filename;
		list_file.ignore(1024, '\n');

		printTime();
		std::cout << "Loading view " << img_filename << " ..." << std::endl;

		//Load image
		img_filename = input_dir + img_filename;
		view->img = cv::imread(img_filename);

		bundle_file.ignore(1024, '\n');

		//Load focal length
		cv::Mat perspective_mat(3, 3, CV_64F);
		double focal_length;
		bundle_file >> focal_length;
		perspective_mat = 0;
		perspective_mat.at<double>(0, 0) = focal_length;
		perspective_mat.at<double>(1, 1) = focal_length;
		perspective_mat.at<double>(2, 2) = 1;

		//Load camera location
		cv::Mat pose_mat(3, 4, CV_64F);
		bundle_file >> pose_mat.at<double>(0, 0) >> pose_mat.at<double>(0, 1) >> pose_mat.at<double>(0, 2);
		bundle_file >> pose_mat.at<double>(1, 0) >> pose_mat.at<double>(1, 1) >> pose_mat.at<double>(1, 2);
		bundle_file >> pose_mat.at<double>(2, 0) >> pose_mat.at<double>(2, 1) >> pose_mat.at<double>(2, 2);
		bundle_file >> pose_mat.at<double>(0, 3) >> pose_mat.at<double>(1, 3) >> pose_mat.at<double>(2, 3);

		view->P = perspective_mat * pose_mat;

		view->S.resize(view->img.cols / BETA_1 * view->img.rows / BETA_1);
		view->C.resize(view->img.cols / BETA_1 * view->img.rows / BETA_1);

		views.push_back(view);
	}*/


	///////////////////////////////////////////////////////////////////////////
	//Find features
	printTime();
	std::cout << "Extracting features." << std::endl;
	std::map<mvs::View*, std::vector<mvs::Feature>> features;
	for (auto view : views) {
		cv::Mat grayscale_img;
		cv::cvtColor(view->img, grayscale_img, cv::COLOR_BGR2GRAY);

		cv::Mat harris_img;
		cv::cornerHarris(grayscale_img, harris_img, 3, 3, 0.06);

		//Divide image in cells and extract features
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
					}
					else {
						break;
					}
				}
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////
	//Match features
	printTime();
	std::cout << "Initial feature match." << std::endl;
	std::map<mvs::View*, cv::Mat> depth_maps;
	std::vector<std::pair<double, mvs::Feature*>> matches;
	std::list<mvs::Patch> patches;
	for (unsigned int i = 1; i <= views.size(); i++) {
		printTime();
		std::cout << "Matching view " << i << "..." << std::endl;

		mvs::View* view = views[i - 1];

		int rows = view->img.rows;
		int cols = view->img.cols;

		cv::Vec3d camera_front = mvs::getCameraOrientation(view);

		//Iterate over ever detected feature
		for (auto& feature : features[view]) {
			//Check if a feature was already detected in the region the feature lies
			if (!view->T[feature.pos.x / BETA_1][feature.pos.y / BETA_1].empty()) {
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

			//Find photoconsistent match
			mvs::Ray3 ray_h = mvs::castRay(view, feature.pos + cv::Point2d(1, 0));
			mvs::Ray3 ray_v = mvs::castRay(view, feature.pos + cv::Point2d(0, 1));
			ray_h.direction /= camera_front.ddot(ray_h.direction);
			ray_v.direction /= camera_front.ddot(ray_v.direction);

			int n = 0;
			for (auto& match : matches) {
				mvs::Patch patch;
				patch.source = view;
				patch.projection_dir = -feature.normal;
				patch.position = mvs::triangulate(feature.src, feature.pos, match.second->src, match.second->pos);

				patch.tangent_1 = ((patch.position - ray.start).ddot(camera_front) * ray_h.direction + ray.start - patch.position) * 1.0;
				patch.tangent_2 = ((patch.position - ray.start).ddot(camera_front) * ray_v.direction + ray.start - patch.position) * 1.0;

				patch.T.push_back(view);

				//Find initial matches
				for (auto& neighbor : views) {
					if (neighbor == view)
						continue;

					double ncc_score = mvs::ncc<MU>(&patch, view, neighbor);

					if (ncc_score > ALPHA_0)
						patch.T.push_back(neighbor);
				}

				if (patch.T.size() >= GAMMA)
					patch.optimize();
				else
					continue;

				//Find photoconsistent sets
				patch.T.clear();
				patch.T.push_back(view);
				patch.S.push_back(view);
				for (auto& neighbor : views) {
					if (neighbor == view)
						continue;

					double ncc_score = mvs::ncc<MU>(&patch, view, neighbor);

					if (ncc_score > ALPHA_1)
						patch.T.push_back(neighbor);
					else if (ncc_score > ALPHA_0)
						patch.S.push_back(neighbor);
				}

				if (patch.T.size() >= GAMMA) {
					//Accept patch
					patches.push_back(patch);
					mvs::Patch* p_patch = &patches.back();

					for (auto& neighbor : patch.T) {
						cv::Point2i cell = cv::Point2i(mvs::projectPoint(neighbor, patch.position) / BETA_1);
						neighbor->T[cell.x][cell.y].push_back(p_patch);
						neighbor->S[cell.x][cell.y].push_back(p_patch);
					}

					for (auto& neighbor : patch.S) {
						cv::Point2i cell = cv::Point2i(mvs::projectPoint(neighbor, patch.position) / BETA_1);
						neighbor->S[cell.x][cell.y].push_back(p_patch);
					}

					p_patch->S.insert(p_patch->S.end(), p_patch->T.begin(), p_patch->T.end());

					break;
				}

				/*patch.T.push_back(view);
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
						int cell_index = feature.pos.x / BETA_1 + feature.pos.y / BETA_1 * rows / BETA_1;
						neighbor->C[cell_index].push_back(&patches.back());
					}
					break;
				}*/

			}

			//if (patches.size() >= 10)
			//	break;
		}

		printTime();
		std::cout << "Finished. Total patches: " << patches.size() << std::endl;

		//break;
	}

	//Compute depthmaps
	for (auto& view : views)
		view->computeDepthmap();

	///////////////////////////////////////////////////////////////////////////
	//Expansion
	printTime();
	std::cout << "Expanding patches..." << std::endl;
	auto last_patch = --(patches.end());
	for (auto patch = patches.begin(); patch != last_patch; patch++) {
		for (auto& view : patch->T) {
			cv::Point2i cell_pos = cv::Point2i(mvs::projectPoint(view, patch->position) / BETA_1);

			//Check 8-connected neighboring cells
			for (int i = 0; i < 9; i++) {
				if (i == 4)
					continue; //Skip current cell

				int cell_x = cell_pos.x - 1 + i % 3;
				int cell_y = cell_pos.y - 1 + i / 3;

				if (cell_x < 0 || cell_x >= view->grid_w || cell_y < 0 || cell_y >= view->grid_h)
					continue; //Skip if cell is out of range

				if (view->T[cell_x][cell_y].size() > 0)
					continue; //Skip if cell already has a patch

				mvs::Patch new_patch;
				new_patch.source = patch->source;
				new_patch.tangent_1 = patch->tangent_1;
				new_patch.tangent_2 = patch->tangent_2;
				new_patch.projection_dir = new_patch.projection_dir;
				new_patch.T = patch->T;

				mvs::Ray3 ray = mvs::castRay(view, cv::Point2d((double)cell_x + 0.5, (double)cell_y + 0.5) * BETA_1);
				new_patch.position = mvs::intersect(ray, &(*patch));

				new_patch.optimize();

				//Find views where the patch should be visible
				new_patch.T.clear();
				new_patch.findVisible(&views, ALPHA_1);

				if (new_patch.T.size() >= GAMMA) {
					//Accept patch
					patches.push_back(new_patch);
					patches.back().registerViews();
					/*mvs::Patch* p_patch = &patches.back();

					for (auto& neighbor : new_patch.T) {
						cv::Point2i projection = cv::Point2i(mvs::projectPoint(neighbor, new_patch.position) / BETA_1);
						neighbor->T[projection.x][projection.y].push_back(p_patch);
					}

					for (auto& neighbor : new_patch.S) {
						cv::Point2i projection = cv::Point2i(mvs::projectPoint(neighbor, new_patch.position) / BETA_1);
						neighbor->S[projection.x][projection.y].push_back(p_patch);
					}*/
				}
			}
		}
	}

	printTime();
	std::cout << "Finished. Total patches: " << patches.size() << std::endl;

	///////////////////////////////////////////////////////////////////////////
	//Filtering

	printTime();
	std::cout << "Filtering..." << std::endl;

	int filtered_patches = 0;
	std::list<mvs::Patch>::iterator patch = patches.begin();
	while (patch != patches.end()) {
		//Filter occluding patches
		double score_1 = patch->T.size() * patch->meanNcc();
		double score_2 = 0;

		for (auto& view : patch->T) {
			cv::Point2i cell = mvs::projectPoint(view, patch->position) / view->cell_size;
			if (view->depthmap[cell.x][cell.y].second != &(*patch))
				continue;
			
			for (auto& ocluded_patch : view->T[cell.x][cell.y]) {
				if (ocluded_patch != &(*patch))
					score_2 += ocluded_patch->meanNcc();
			}
		}

		if (score_1 < score_2) {
			patch->remove();
			patches.erase(patch++);
			filtered_patches++;
			continue;
		}

		//Filter occluded patches
		//...

		//Regularization
		int total_patches = 0;
		int nadjacent_patches = 0;
		for (auto& view : patch->S) {
			cv::Point2i cell_pos = cv::Point2i(mvs::projectPoint(view, patch->position) / BETA_1);

			//Check 8-connected neighboring cells
			for (int i = 0; i < 9; i++) {
				int cell_x = cell_pos.x - 1 + i % 3;
				int cell_y = cell_pos.y - 1 + i / 3;

				if (cell_x < 0 || cell_x >= view->grid_w || cell_y < 0 || cell_y >= view->grid_h)
					continue; //Skip if cell is out of range

				for (auto& adjacent : view->S[cell_x][cell_y]) {
					total_patches++;

					cv::Point3d distance_vector = patch->position - adjacent->position;
					double distance = cv::abs(distance_vector.dot(patch->normal)) + cv::abs(distance_vector.dot(adjacent->normal));
					double mid_depth = ((patch->position + adjacent->position) / 2).dot(view->orientation);
					double margin = 2 * mvs::getProjectedDistance(view, mid_depth);

					if (distance < margin)
						nadjacent_patches++;
				}
			}
		}

		if (total_patches > 0 && 4 * nadjacent_patches / total_patches == 0) {
			patch->remove();
			patches.erase(patch++);
			filtered_patches++;
			continue;
		}

		patch++;
	}

	printTime();
	std::cout << "Removed " << filtered_patches << " patches" << std::endl;

	///////////////////////////////////////////////////////////////////////////
	//Write output file
	printTime();
	std::cout << "Writing PLY file..." << std::endl;
	std::ofstream output_file("output.ply", std::ios::binary);
	if (output_file.is_open()) {
		//Write header
		output_file << "ply" << std::endl;
		output_file << "format binary_little_endian 1.0" << std::endl;
		output_file << "element vertex " << patches.size() << std::endl;
		output_file << "property float x" << std::endl;
		output_file << "property float y" << std::endl;
		output_file << "property float z" << std::endl;
		output_file << "property float nx" << std::endl;
		output_file << "property float ny" << std::endl;
		output_file << "property float nz" << std::endl;
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
		/*for (auto& patch : patches) {
			mean_x += patch.position.x;
			mean_y += patch.position.y;
			mean_z += patch.position.z;
		}

		mean_x /= patches.size();
		mean_y /= patches.size();
		mean_z /= patches.size();*/

		//Write data
		for (auto& patch : patches) {
			cv::Vec3b color = mvs::bilinearSample(&patch.source->img, mvs::projectPoint(patch.source, patch.position));

			float x = (float)patch.position.x - mean_x;
			float y = (float)patch.position.y - mean_y;
			float z = (float)patch.position.z - mean_z;
			float nx = (float)patch.normal[0];
			float ny = (float)patch.normal[1];
			float nz = (float)patch.normal[2];

			output_file.write(reinterpret_cast<const char*>(&x), sizeof(x));
			output_file.write(reinterpret_cast<const char*>(&y), sizeof(y));
			output_file.write(reinterpret_cast<const char*>(&z), sizeof(z));
			output_file.write(reinterpret_cast<const char*>(&nx), sizeof(nx));
			output_file.write(reinterpret_cast<const char*>(&ny), sizeof(ny));
			output_file.write(reinterpret_cast<const char*>(&nz), sizeof(nz));
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
	display_image = views[0]->img;
	for (auto& patch : patches) {
		cv::circle(display_image, mvs::projectPoint(views[0], patch.position), 4, cv::Scalar(0, 0, 255), 2);
		cv::line(display_image, mvs::projectPoint(views[0], patch.position), mvs::projectPoint(views[0], patch.position + 10 * cv::Point3d(patch.normal)), cv::Scalar(255), 2);
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

	//std::cin.ignore();

	return 1;
}
