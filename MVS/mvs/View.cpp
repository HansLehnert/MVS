#include "View.h"

#include <limits>

#include "Patch.h"

using namespace mvs;


View::View(cv::Mat image, cv::Mat projection_matrix, int cell_width) : 
	img(image),
	P(projection_matrix),
	cell_size(cell_width)
{
	grid_w = img.cols / cell_size;
	grid_h = img.rows / cell_size;

	S = std::vector<std::vector<std::vector<Patch*>>>(grid_w, std::vector<std::vector<Patch*>>(grid_h));
	T = std::vector<std::vector<std::vector<Patch*>>>(grid_w, std::vector<std::vector<Patch*>>(grid_h));
	depthmap = std::vector<std::vector<std::pair<double, Patch*>>>(grid_w, std::vector<std::pair<double, Patch*>>(grid_h));

	position = getCameraPosition(this);
	orientation = getCameraOrientation(this);
}


void View::computeDepthmap() {
	for (int i = 0; i < grid_w; i++) {
		for (int j = 0; j < grid_h; j++) {
			computeDepthmap(i, j);
		}
	}
}


void View::computeDepthmap(int x, int y) {
	depthmap[x][y] = { std::numeric_limits<float>::max(), nullptr };
	
	for (auto& patch : T[x][y]) {
		double depth = (patch->position - position).dot(orientation);
		if (depth < depthmap[x][y].first)
			depthmap[x][y] = { depth, patch };
	}
}