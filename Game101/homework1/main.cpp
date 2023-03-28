#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate <<
		1, 0, 0, -eye_pos[0],
		0, 1, 0, -eye_pos[1],
		0, 0, 1, -eye_pos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the model matrix for rotating the triangle around the Z axis.
	// Then return it.
	float cos = cosf(rotation_angle);
	float sin = sinf(rotation_angle);
	model <<
		cos, -sin, 0, 0,
		sin, cos, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		;

	return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	// Students will implement this function

	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
	float tan = tanf(eye_fov / 2);
	projection <<
		-1 / (aspect_ratio * tan), 0, 0, 0,
		0, -1 / tan, 0, 0,
		0, 0, (zFar + zNear) / (zNear - zFar), 2 * (zFar * zNear) / (zFar - zNear),
		0, 0, 1, 0;

	// TODO: Implement this function
	// Create the projection matrix for the given parameters.
	// Then return it.

	return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
	axis.normalize();
	float cos = cosf(angle);
	float sin = sinf(angle);
	Eigen::Matrix3f m1 = Eigen::Matrix3f::Identity();
	m1 <<
		0, -axis[2], axis[1],
		axis[2], 0, -axis[0],
		-axis[1], axis[0], 0;
	Eigen::Matrix3f m2 = Eigen::Matrix3f::Identity() * cos + (1 - cos) * axis * axis.transpose() + sin * m1;
	Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
	result <<
		m2(0, 0), m2(0, 1), m2(0, 2), 0,
		m2(1, 0), m2(1, 1), m2(1, 2), 0,
		m2(2, 0), m2(2, 1), m2(2, 2), 0,
		0, 0, 0, 1;
	return result;
}

int main(int argc, const char** argv)
{
	float angle = 0;
	bool command_line = false;
	std::string filename = "output.png";

	if (argc >= 3) {
		command_line = true;
		angle = std::stof(argv[2]); // -r by default
		if (argc == 4) {
			filename = std::string(argv[3]);
		}
		else
			return 0;
	}

	rst::rasterizer r(700, 700);

	Eigen::Vector3f eye_pos = { 0, 0, 5 };

	std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

	std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	int key = 0;
	int frame_count = 0;

	if (command_line) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(MY_PI / 8, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		//r.set_model(get_rotation(Eigen::Vector3f(1, 0, 0), angle));
		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';

		if (key == 'a') {
			angle += 0.1;
		}
		else if (key == 'd') {
			angle -= 0.1;
		}
	}

	return 0;
}
