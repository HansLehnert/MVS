#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <string>
#include <ctime>

//Shaders
const char* vertex_shader_src = R"glsl(
#version 410

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

out vec3 vertex_color;

uniform mat4 model_matrix;

void main() {
	gl_Position = model_matrix * vec4(position, 1);
	//gl_Position.z *= 0.001;
	gl_PointSize = 3;
	vertex_color = color;
}

)glsl";

const char* fragment_shader_src = R"glsl(
#version 410

in vec3 vertex_color;

out vec3 frag_color;

void main() {
	frag_color = vertex_color;
}

)glsl";


struct ModelLayout {
	int vertex_count;
	size_t vertex_size;
	size_t x_offset;
	size_t y_offset;
	size_t z_offset;
	size_t r_offset;
	size_t g_offset;
	size_t b_offset;
};


bool loadPly(std::string filename, std::vector<unsigned char>* model_data, ModelLayout* layout) {
	layout->vertex_size = 0;
	layout->x_offset = -1;
	layout->y_offset = -1;
	layout->z_offset = -1;
	layout->r_offset = -1;
	layout->g_offset = -1;
	layout->b_offset = -1;
	layout->vertex_count = 0;

	std::ifstream ply_file(filename, std::ios::binary);
	if (!ply_file.is_open()) {
		return false;
	}

	std::string magic_number;
	ply_file >> magic_number;
	if (magic_number != "ply") {
		ply_file.close();
		return false;
	}

	bool end_header = false;
	std::string element;
	while (!end_header) {
		std::string keyword;
		ply_file >> keyword;

		if (keyword == "comment") {
			ply_file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
		else if (keyword == "format") {
			std::string format;
			std::string version;
			ply_file >> format;
			ply_file >> version;
			if (format != "binary_little_endian") {
				std::cout << "Unsupported format" << std::endl;
				ply_file.close();
				return false;
			}
			if (version != "1.0") {
				std::cout << "Unsupported version" << std::endl;
				ply_file.close();
				return false;
			}
		}
		else if (keyword == "element") {
			ply_file >> element;
			if (element == "vertex") {
				ply_file >> layout->vertex_count;
			}
			else {
				ply_file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			}
		}
		else if (keyword == "property") {
			std::string property_name;
			std::string property_type;
			size_t property_size = 0;
			ply_file >> property_type;
			ply_file >> property_name;

			if (property_type == "float")
				property_size = sizeof(float);
			else if (property_type == "uchar")
				property_size = sizeof(unsigned char);

			if (element == "vertex") {
				if (property_name == "x")
					layout->x_offset = layout->vertex_size;
				else if (property_name == "y")
					layout->y_offset = layout->vertex_size;
				else if (property_name == "z")
					layout->z_offset = layout->vertex_size;
				else if (property_name == "r" || property_name == "red")
					layout->r_offset = layout->vertex_size;
				else if (property_name == "g" || property_name == "green")
					layout->g_offset = layout->vertex_size;
				else if (property_name == "b" || property_name == "blue")
					layout->b_offset = layout->vertex_size;

				layout->vertex_size += property_size;
			}
		}
		else if (keyword == "end_header") {
			end_header = true;
		}
	}

	ply_file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	model_data->resize(layout->vertex_size * layout->vertex_count);

	int read_bytes = 0;
	while (read_bytes < model_data->size() && !ply_file.eof()) {
		ply_file.read((char*)&(*model_data)[read_bytes], model_data->size() - read_bytes);
		read_bytes += ply_file.gcount();
	}

	if (read_bytes < model_data->size()) {
		return false;
	}

	return true;
}


void printTime() {
	static time_t start_time = time(0);
	int elapsed_time = difftime(time(0), start_time);
	std::cout << "[" << std::setfill('0') << std::setw(2) << elapsed_time / 3600 << ":"
		<< std::setfill('0') << std::setw(2) << elapsed_time % 3600 / 60 << ":"
		<< std::setfill('0') << std::setw(2) << elapsed_time % 60 << "]\t";
}


int main(int argc, char* argv[]) {
	if (argc < 3) {
		std::cout << "MVS reconstruction evaluation" << std::endl << std::endl;
		std::cout << "Usage" << std::endl;
		std::cout << '\t' << argv[0] << "reference_file evaluated_file" << std::endl;
		return 0;
	}

	///////////////////////////////////////////////////////////////////////////
	//Load models

	ModelLayout reference_layout;
	ModelLayout test_layout;
	std::vector<unsigned char> reference_data;
	std::vector<unsigned char> test_data;

	printTime();
	std::cout << "Loading reference model" << std::endl;
	if (!loadPly(argv[1], &reference_data, &reference_layout)) {
		std::cout << "\t\tFailed to load reference model" << std::endl;
		return 0;
	}

	printTime();
	std::cout << "Loading test model" << std::endl;
	if (!loadPly(argv[2], &test_data, &test_layout)) {
		std::cout << "\t\tFailed to load test model" << std::endl;
		return 0;
	}

	///////////////////////////////////////////////////////////////////////////
	//Evaluation code

	float max = 0;

	for (int i = 0; i < test_layout.vertex_count; i++) {
		float test_x = *((float*)&test_data[test_layout.vertex_size * i + test_layout.x_offset]);
		float test_y = *((float*)&test_data[test_layout.vertex_size * i + test_layout.y_offset]);
		float test_z = *((float*)&test_data[test_layout.vertex_size * i + test_layout.z_offset]);

		for (int j = 0; j < reference_layout.vertex_count; j++) {
			float ref_x = *((float*)&reference_data[reference_layout.vertex_size * j + reference_layout.x_offset]);
			float ref_y = *((float*)&reference_data[reference_layout.vertex_size * j + reference_layout.y_offset]);
			float ref_z = *((float*)&reference_data[reference_layout.vertex_size * j + reference_layout.z_offset]);

			float distance = (test_x - ref_x) * (test_x - ref_x)
				+ (test_y - ref_y) * (test_y - ref_y)
				+ (test_z - ref_z) * (test_z - ref_z);
			if (distance > max)
				max = distance;
		}

		if (i % 1000 == 0) {
			printTime();
			std::cout << "Progress: " << i << "/" << test_layout.vertex_count << std::endl;
		}
	}

	printTime();
	std::cout << "Complete" << std::endl;
	std::cout << "\t\tResult: " << max << std::endl;

	std::cin.ignore();

	return 0;
}