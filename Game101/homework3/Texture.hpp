//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
	cv::Mat image_data;

public:
	Texture(const std::string& name) {
		image_data = cv::imread(name);
		cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
		width = image_data.cols;
		height = image_data.rows;
	}

	int width, height;

	Eigen::Vector3f getColor(float u, float v) {
		auto u_img = fmax(u, 0) * width;
		auto v_img = (1 - fmax(v, 0)) * height;
		auto color = image_data.at<cv::Vec3b>(v_img, u_img);
		return Eigen::Vector3f(color[0], color[1], color[2]);
	}

#ifdef BILINEAR 
	Eigen::Vector3f getColorBilinear(float u, float v) {
		float w1 = (int)(u * width), h1 = (int)(v * height);
		float w2 = w1 + 1, h2 = h1;
		float w3 = w1, h3 = h1 + 1;
		float w4 = w1 + 1, h4 = h1 + 1;

		Eigen::Vector3f c1, c2, c3, c4, c5, c6;
		c1 = getColor(w1 / width, h1 / height); 
		c2 = getColor(w2 / width, h2 / height);
		c3 = getColor(w3 / width, h3 / height);
		c4 = getColor(w4 / width, h4 / height);
		c5 = c1 + (c2 - c1) * (u * width - w1);
		c6 = c3 + (c4 - c3) * (u * width - w1);
		return c5 + (c6 - c5) * (v * height - h1);
	}
#endif // BILINEAR

};
#endif //RASTERIZER_TEXTURE_H
