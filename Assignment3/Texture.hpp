//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
	cv::Mat image_data;

public:
	Texture(const std::string& name)
	{
		image_data = cv::imread(name);
		cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
		width = image_data.cols;
		height = image_data.rows;
	}

	int width, height;

	Eigen::Vector3f getColor(float u, float v)
	{
		auto u_img = u * width;
		auto v_img = (1 - v) * height;

		if (u_img < 0) u_img = 0;
		if (u_img >= width) u_img = width - 1;
		if (v_img < 0) v_img = 0;
		if (v_img >= height) v_img = height - 1;

		auto color = image_data.at<cv::Vec3b>(v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

	Eigen::Vector3f getColorBilinear(float u, float v)
	{
		auto u_img = u * width;
		auto v_img = (1 - v) * height;

		auto rangeCheck = [width = this->width, height = this->height](float x, bool isU) {
			if (x < 0)
				return 0.0f;

			if (isU && x >= width)
				return width - 1.0f;

			if (!isU && x >= height)
				return height - 1.0f;

			return x;
		};

		float u_min = rangeCheck(std::floor(u_img), true);
		float u_max = rangeCheck(std::ceil(u_img), true);
		float v_min = rangeCheck(std::floor(v_img), false);
		float v_max = rangeCheck(std::ceil(v_img), false);

		// get color at u00, u01, u10, u11
		auto u00 = image_data.at<cv::Vec3b>(v_max, u_min);
		auto u01 = image_data.at<cv::Vec3b>(v_min, u_min);
		auto u10 = image_data.at<cv::Vec3b>(v_max, u_max);
		auto u11 = image_data.at<cv::Vec3b>(v_min, u_max);

		float s = (u_img - u_min) / (u_max - u_min); // range [0, 1]
		float t = (v_img - v_min) / (v_max - v_min);

		auto lerp = [](float x, cv::Vec3b v0, cv::Vec3b v1) {
			return v0 + x * (v1 - v0);
		};

		auto u0 = lerp(s, u00, u10);
		auto u1 = lerp(s, u01, u11);

		auto color = lerp(t, u1, u0);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

};
#endif //RASTERIZER_TEXTURE_H
