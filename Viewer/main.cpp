#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <GL/glew.h>
#include <SDL.h>

#include <glm\glm.hpp>
#include <glm\gtx\transform.hpp>

//Shaders
const char* vertex_shader_src = R"glsl(
#version 410

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec3 color;

out vec3 vertex_color;

uniform mat4 model_matrix;

void main() {
	gl_Position = model_matrix * vec4(position, 1);
	//float difuse = dot(normalize((model_matrix * vec4(normal, 0)).xyz), vec3(1, 1, 1));
	//gl_Position.z *= 0.001;
	gl_PointSize = 3;
	//vertex_color = color * (difuse + 1) / 2;
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
	size_t nx_offset;
	size_t ny_offset;
	size_t nz_offset;
	size_t r_offset;
	size_t g_offset;
	size_t b_offset;
};


bool loadPly(std::string filename, std::vector<unsigned char>* model_data, ModelLayout* layout) {
	layout->vertex_size = 0;
	layout->x_offset = -1;
	layout->y_offset = -1;
	layout->z_offset = -1;
	layout->nx_offset = -1;
	layout->ny_offset = -1;
	layout->nz_offset = -1;
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
				else if (property_name == "nx")
					layout->nx_offset = layout->vertex_size;
				else if (property_name == "ny")
					layout->ny_offset = layout->vertex_size;
				else if (property_name == "nz")
					layout->nz_offset = layout->vertex_size;
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

	unsigned int read_bytes = 0;
	while (read_bytes < model_data->size() && !ply_file.eof()) {
		ply_file.read((char*)&(*model_data)[read_bytes], model_data->size() - read_bytes);
		read_bytes += (unsigned int)ply_file.gcount();
	}

	if (read_bytes < model_data->size()) {
		return false;
	}

	return true;
}


GLuint loadShader(const char* src, GLuint type) {
	GLuint shader = glCreateShader(type);
	glShaderSource(shader, 1, &src, NULL);
	glCompileShader(shader);

	GLint success;
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (success == GL_FALSE) {
		GLint log_length;
		GLchar* log_content;

		glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &log_length);
		log_content = new GLchar[log_length];
		glGetShaderInfoLog(shader, log_length, NULL, log_content);
		std::cout << log_content << std::endl;
		glDeleteShader(shader);
		delete log_content;
		return 0;
	}

	return shader;
}


int main(int argc, char* argv[]) {
	if (argc < 2) {
		std::cout << "Point cloud viewer" << std::endl << std::endl;
		std::cout << "Usage" << std::endl;
		std::cout << '\t' << argv[0] << "ply_file" << std::endl;
		return 0;
	}

	///////////////////////////////////////////////////////////////////////////
	//Load models

	ModelLayout model_layout;
	std::vector<unsigned char> model_data;

	std::cout << "Loading reference model" << std::endl;
	if (!loadPly(argv[1], &model_data, &model_layout)) {
		std::cout << "Failed to load reference model" << std::endl;
		return 0;
	}

	///////////////////////////////////////////////////////////////////////////
	//OpenGL setup

	//SDL initialization
	if (SDL_Init(SDL_INIT_VIDEO) != 0) {
		std::cout << "Failed to initialize SDL" << std::endl;
		return 0;
	}

	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);

	//Window size
	int window_width = 600;
	int window_height = 600;

	//Window and context creation
	SDL_Window* window = SDL_CreateWindow("Model viewer", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, window_width, window_height, SDL_WINDOW_OPENGL);
	SDL_GLContext context;
	if ((context = SDL_GL_CreateContext(window)) == 0) {
		std::cout << "Failed to create context" << std::endl;
		return 0;
	}

	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cout << "GLEW init failed." << std::endl;
		return 0;
	}

	//GL configuration
	glClearColor(0, 0, 0, 0);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glEnable(GL_PROGRAM_POINT_SIZE);

	GLuint null_vao;
	glGenVertexArrays(1, &null_vao);
	glBindVertexArray(null_vao);

	//Load shaders
	GLuint program = glCreateProgram();
	GLuint vertex_shader = loadShader(vertex_shader_src, GL_VERTEX_SHADER);
	GLuint fragment_shader = loadShader(fragment_shader_src, GL_FRAGMENT_SHADER);
	glAttachShader(program, vertex_shader);
	glAttachShader(program, fragment_shader);
	glLinkProgram(program);
	glDetachShader(program, vertex_shader);
	glDetachShader(program, fragment_shader);
	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);
	glUseProgram(program);

	//Load models
	GLuint model_buffer[2];
	glGenBuffers(2, model_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, model_buffer[0]);
	glBufferData(GL_ARRAY_BUFFER, model_data.size(), &(model_data[0]), GL_STATIC_DRAW);

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, model_layout.vertex_size, (void*)model_layout.x_offset);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, model_layout.vertex_size, (void*)model_layout.nx_offset);
	glVertexAttribPointer(2, 3, GL_UNSIGNED_BYTE, GL_TRUE, model_layout.vertex_size, (void*)model_layout.r_offset);

	//Model parameters
	float yaw = 0;
	float pitch = 0;
	float pan = 0;
	float scale = 0;
	bool pan_active = false;


	GLuint model_matrix_loc = glGetUniformLocation(program, "model_matrix");

	///////////////////////////////////////////////////////////////////////////
	//Program loop

	bool should_exit = false;
	while (!should_exit) {
		SDL_GL_SwapWindow(window);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Poll input evenys
		SDL_Event sdl_event;
		while (SDL_PollEvent(&sdl_event)) {
			switch (sdl_event.type) {
			case SDL_QUIT:
				should_exit = true;
				break;

			case SDL_MOUSEWHEEL:
				scale += sdl_event.wheel.y;
				break;

			case SDL_MOUSEBUTTONDOWN:
				if (sdl_event.button.button == SDL_BUTTON_LEFT)
					pan_active = true;
				break;

			case SDL_MOUSEBUTTONUP:
				if (sdl_event.button.button == SDL_BUTTON_LEFT)
					pan_active = false;
				break;

			case SDL_MOUSEMOTION:
				if (pan_active) {
					pan += sdl_event.motion.yrel;
				}
				else {
					yaw = 360 * ((float)sdl_event.motion.x / window_width);
					pitch = 180 * ((float)sdl_event.motion.y / window_height);
				}
				break;

			}
		}

		//Calculate model matrix
		glm::mat4 model_matrix = glm::translate(glm::vec3(0, 0, pan));;
		model_matrix = glm::scale(glm::vec3((float)exp(scale * 0.5))) * model_matrix;
		model_matrix = glm::rotate(yaw, glm::vec3(0, 0, 1)) * model_matrix;
		model_matrix = glm::rotate(pitch, glm::vec3(1, 0, 0)) * model_matrix;

		//Draw model
		//glUseProgram(program);
		//glBindBuffer(GL_ARRAY_BUFFER, model_buffer[0]);
		glUniformMatrix4fv(model_matrix_loc, 1, GL_FALSE, (GLfloat*)&model_matrix);
		glDrawArrays(GL_POINTS, 0, model_data.size() / model_layout.vertex_size);
	}

	return 0;
}