// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

#ifdef PERFORMANCE_TEST
#include <windows.h>
#else
#define min(a,b) (((a) < (b)) ? (a) : (b))
#define max(a,b) (((a) < (b)) ? (b) : (a))
#endif //  PERFORMANCE_TEST

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions) {
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices) {
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols) {
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f) {
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v) {
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	float x0 = x;
	float y0 = y;
	Vector3f pa = Vector3f(_v[0].x() - x0, _v[0].y() - y0, 0);
	Vector3f pb = Vector3f(_v[1].x() - x0, _v[1].y() - y0, 0);
	Vector3f pc = Vector3f(_v[2].x() - x0, _v[2].y() - y0, 0);
	float t1 = pa.x() * pb.y() - pa.y() * pb.x();
	float t2 = pb.x() * pc.y() - pb.y() * pc.x();
	float t3 = pc.x() * pa.y() - pc.y() * pa.x();
	return t1 * t2 >= 0 && t2 * t3 >= 0;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v) {
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind) {
		Triangle t;
		Eigen::Vector4f v[] = {
				mvp * to_vec4(buf[i[0]], 1.0f),
				mvp * to_vec4(buf[i[1]], 1.0f),
				mvp * to_vec4(buf[i[2]], 1.0f)
		};
		//Homogeneous division
		for (auto& vec : v) {
			vec /= vec.w();
		}
		//Viewport transformation
		for (auto& vert : v) {
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i) {
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
			t.setVertex(i, v[i].head<3>());
		}

		auto col_x = col[i[0]];
		auto col_y = col[i[1]];
		auto col_z = col[i[2]];

		t.setColor(0, col_x[0], col_x[1], col_x[2]);
		t.setColor(1, col_y[0], col_y[1], col_y[2]);
		t.setColor(2, col_z[0], col_z[1], col_z[2]);

		rasterize_triangle(t);
	}
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	// TODO : Find out the bounding box of current triangle.
	// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

#ifdef PERFORMANCE_TEST
	LARGE_INTEGER  nFreq, t1, t2;
	double dt;
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&t1);
#endif // DEBUG


	auto v = t.toVector4();


	/*
	No AA性能参考：
	Running time : 442.019ms
	Running time : 283.464ms
	*/
#ifdef  __NOAA__
	float xmax = v[0][0], xmin = v[0][0], ymax = v[0][1], ymin = v[0][1];
	for (int i = 1; i < 3; i++) {
		if (v[i][0] > xmax)
			xmax = v[i][0];
		else if (v[i][0] < xmin)
			xmin = v[i][0];

		if (v[i][1] > ymax)
			ymax = v[i][1];
		else if (v[i][1] < ymin)
			ymin = v[i][1];
	}
	int x_max = ceil(xmax), x_min = floor(xmin), y_max = ceil(ymax), y_min = floor(ymin);
	// iterate through the pixel and find if the current pixel is inside the triangle
	for (int i = x_min; i < x_max; i++) {

		for (int j = y_min; j < y_max; j++) {
			if (insideTriangle(i + 0.5, j + 0.5, t.v)) {
				// If so, use the following code to get the interpolated z value.
				// given code
				auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;
				// end given code

				if (z_interpolated < depth_buf[get_index(i, j)]) {
					Vector3f point(i, j, 1.0f);
					set_pixel(point, t.getColor());
					depth_buf[get_index(i, j)] = z_interpolated;
				}
			}
		}
	}
#endif //  __NOAA__


	/*
	2倍SSAA性能参考：
	Running time : 1657.56ms
	Running time : 989.113ms
	*/
#ifdef __SSAA__

	// 将三角形整体缩放SS_SCALE倍
	Vector3f vec3[3] =
	{
		t.v[0] * SS_SCALE,
		t.v[1] * SS_SCALE,
		t.v[2] * SS_SCALE
	};

	float xmax = vec3[0][0], xmin = vec3[0][0], ymax = vec3[0][1], ymin = vec3[0][1];
	for (int i = 1; i < 3; i++) {
		if (vec3[i][0] > xmax)
			xmax = vec3[i][0];
		else if (vec3[i][0] < xmin)
			xmin = vec3[i][0];

		if (vec3[i][1] > ymax)
			ymax = vec3[i][1];
		else if (vec3[i][1] < ymin)
			ymin = vec3[i][1];
	}
	int x_max = ceil(xmax), x_min = floor(xmin), y_max = ceil(ymax), y_min = floor(ymin);

	// 超采样
	for (int i = x_min; i < x_max; i++) {
		for (int j = y_min; j < y_max; j++) {
			if (insideTriangle(i + 0.5, j + 0.5, (const Vector3f*)vec3)) {
				auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;
				// end given code

				if (z_interpolated < ssaa_depth_buf[get_ssaa_index(i, j)]) {
					Vector3f point(i, j, 1.0f);
					ssaa_set_pixel(point, t.getColor());
					ssaa_depth_buf[get_ssaa_index(i, j)] = z_interpolated;
				}
			}
		}
	}

	// 写入frame_buffer
	Vector3f color = Vector3f();
	for (int i = x_min; i < x_max; i += SS_SCALE) {
		for (int j = y_min; j < y_max; j += SS_SCALE) {
			color = ssaa_frame_buf[get_ssaa_index(i, j)];
			color += ssaa_frame_buf[get_ssaa_index(i + 1, j)] + ssaa_frame_buf[get_ssaa_index(i + 1, j + 1)] + ssaa_frame_buf[get_ssaa_index(i, j + 1)];
			color /= 4;
			set_pixel(Vector3f(i / SS_SCALE, j / SS_SCALE, 1.0), color);
		}
	}

#endif //  __SSAA__


	/*
	2倍MSAA性能参考：
	Running time : 898.361ms
	Running time : 556.871ms
	*/
#ifdef __MSAA__
	float xmax = v[0][0], xmin = v[0][0], ymax = v[0][1], ymin = v[0][1];
	for (int i = 1; i < 3; i++) {
		if (v[i][0] > xmax)
			xmax = v[i][0];
		else if (v[i][0] < xmin)
			xmin = v[i][0];

		if (v[i][1] > ymax)
			ymax = v[i][1];
		else if (v[i][1] < ymin)
			ymin = v[i][1];
	}
	int x_max = min(ceil(xmax) + 1, width),
		x_min = max(floor(xmin) - 1, 0),
		y_max = min(ceil(ymax) + 1, width),
		y_min = max(floor(ymin - 1), 0);

	float diff = 1.0 / MS_SCALE;
	float diff_div_2 = diff / 2;
	int sample_num = MS_SCALE * MS_SCALE;
	for (int i = x_min; i < x_max; i++) {
		for (int j = y_min; j < y_max; j++) {
			int color_alp_counter = 0;
			for (float k = 0; k <= 1 - FLT_EPSILON; k += diff) {
				for (float l = 0; l <= 1 - FLT_EPSILON; l += diff) {
					if (insideTriangle(i + k + diff_div_2, j + l + diff_div_2, t.v)) {
						color_alp_counter++;
					}
				}
			}
			if (color_alp_counter == 0) {
				continue;
			}
			// If so, use the following code to get the interpolated z value.
			// given code
			auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
			float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
			float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
			z_interpolated *= w_reciprocal;
			// end given code

			if (z_interpolated < depth_buf[get_index(i, j)]) {
				Vector3f point(i, j, 1.0f);
				float alpha = color_alp_counter / (float)sample_num;
				// 灰线应该用rgba颜色表示法解决
				// Vector3f color = t.getColor() * alpha + get_pixel(point).rgb() + get_pixel(point).a();
				Vector3f color = t.getColor() * alpha + get_pixel(point);
				set_pixel(point, color);
				depth_buf[get_index(i, j)] = z_interpolated;
			}
		}
	}
#endif //  __MSAA__


#ifdef PERFORMANCE_TEST
	QueryPerformanceCounter(&t2);
	dt = (t2.QuadPart - t1.QuadPart) / (double)nFreq.QuadPart;
	std::cout << "Running time : " << dt * 1000 << "ms" << std::endl;
#endif // DEBUG

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) {
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v) {
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p) {
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {

#ifdef  __NOAA__
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
	}
#endif //  __NOAA__


#ifdef  __SSAA__
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
		std::fill(ssaa_depth_buf.begin(), ssaa_depth_buf.end(), std::numeric_limits<float>::infinity());
	}
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
		std::fill(ssaa_frame_buf.begin(), ssaa_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
#endif //  __SSAA__


#ifdef  __MSAA__
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
	}
#endif //  __MSAA__

	if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
	frame_buf.resize(w * h);

#ifdef __NOAA__ 
	depth_buf.resize(w * h);
#endif //  __NOAA__


#ifdef __SSAA__ 
	ssaa_depth_buf.resize(SS_SCALE * SS_SCALE * w * h);
	ssaa_frame_buf.resize(SS_SCALE * SS_SCALE * w * h);
#endif //  __SSAA__


#ifdef __MSAA__ 
	depth_buf.resize(w * h);
#endif //  __MSAA__

}

int rst::rasterizer::get_index(int x, int y) {
	int ret = (height - 1 - y) * width + x;
	return ret;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color) {
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;
}

Vector3f rst::rasterizer::get_pixel(const Eigen::Vector3f& point) {
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	return frame_buf[ind];
}

#ifdef __SSAA__
void rst::rasterizer::ssaa_set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color) {
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height * SS_SCALE - 1 - point.y()) * width * SS_SCALE + point.x();
	ssaa_frame_buf[ind] = color;
	}

int rst::rasterizer::get_ssaa_index(int x, int y) {
	return (height * SS_SCALE - 1 - y) * width * SS_SCALE + x;
}
#endif

// clang-format on