#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// degree to radian
float deg_to_rad(float angle)
{
	return angle * MY_PI / 180.0;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();

	auto angle_rad = deg_to_rad(angle);
	Eigen::Matrix3f r, t;

	t << 0, -axis[2], axis[1],
		axis[2], 0, -axis[0],
		-axis[1], axis[0], 0;

	r = cos(angle_rad) * identity + (1 - cos(angle_rad)) * axis * axis.adjoint() + sin(angle_rad) * t;

	model << r(0, 0), r(0, 1), r(0, 2), 0,
		r(1, 0), r(1, 1), r(1, 2), 0,
		r(2, 0), r(2, 1), r(2, 2), 0,
		0, 0, 0, 1;

	return model;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity(); // get Identity Matrix

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0],
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
	Eigen::Matrix4f translate;
	translate << cos(deg_to_rad(rotation_angle)), -sin(deg_to_rad(rotation_angle)), 0, 0,
		sin(deg_to_rad(rotation_angle)), cos(deg_to_rad(rotation_angle)), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	model = translate * model;

	return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	// Students will implement this function

	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the projection matrix for the given parameters.
	// Then return it.

	// 先做透视投影变换再做正交投影变换，我们要根据fov和aspect求出l,r,t,b

	Eigen::Matrix4f persp_to_ortho;
	persp_to_ortho << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;

	float tan_half_fov = tan(deg_to_rad(eye_fov / 2));
	float top = tan_half_fov * zNear;
	float bottom = -top;
	float right = top * aspect_ratio;
	float left = -right;

	Eigen::Matrix4f ortho_scale, ortho_translate;
	ortho_scale << 2 / (right - left), 0, 0, 0,
		0, 2 / (top - bottom), 0, 0,
		0, 0, 2 / (zNear - zFar), 0,
		0, 0, 0, 1;

	ortho_translate << 1, 0, 0, -(right + left) / 2,
		0, 1, 0, -(top + bottom) / 2,
		0, 0, 1, -(zNear + zFar) / 2,
		0, 0, 0, 1;

	projection = ortho_scale * ortho_translate * persp_to_ortho;

	return projection;
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

	Eigen::Vector3f axis = { 0, 0, 1 }; // default is z axis

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	int key = 0;
	int frame_count = 0;

	if (command_line) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		//r.set_model(get_model_matrix(angle));
		r.set_model(get_rotation(axis, angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		//r.set_model(get_model_matrix(angle));
		r.set_model(get_rotation(axis, angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';

		if (key == 'a') {
			angle += 10;
		}
		else if (key == 'd') {
			angle -= 10;
		}
	}

	return 0;
}
