// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

#ifndef MSAA
#define MSAA true
#endif

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
	Vector3f point(x, y, 0);
	Vector3f e0 = _v[1] - _v[0]; // vector _v[0] to _v[1]
	Vector3f e1 = _v[2] - _v[1];
	Vector3f e2 = _v[0] - _v[2];
	Vector3f p0 = point - _v[0]; // vector _v[0] to point
	Vector3f p1 = point - _v[1];
	Vector3f p2 = point - _v[2];

	return e0.cross(p0).z() > 0 && e1.cross(p1).z() > 0 && e2.cross(p2).z() > 00;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
	auto& buf = pos_buf[pos_buffer.pos_id];
	auto& ind = ind_buf[ind_buffer.ind_id];
	auto& col = col_buf[col_buffer.col_id];

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (auto& i : ind)
	{
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
		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = -vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i)
		{
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
	auto v = t.toVector4();

	// TODO : Find out the bounding box of current triangle.
	float min_x = width;
	float max_x = 0;
	float min_y = height;
	float max_y = 0;
	for (const auto& vi : v) {
		min_x = std::min(min_x, vi.x());
		max_x = std::max(max_x, vi.x());
		min_y = std::min(min_y, vi.y());
		max_y = std::max(max_y, vi.y());
	}

#if MSAA
	// store centers of four smaller pixel MSAA 4X
	std::vector<std::pair<float, float>> centers_of_pixel{
		{ 0.25, 0.25 },
		{ 0.25, 0.75 },
		{ 0.75, 0.25 },
		{ 0.75, 0.75 }
	};

	for (int x = min_x; x <= max_x; x++) {
		for (int y = min_y; y <= max_y; y++) {
			// 此处不用count而是改为判断是否通过了深度测试，只要子采样点通过了深度测试就要更新父采样点的像素
			bool depth_test = false;

			for (int k = 0; k < height_times * width_times; k++) {
				auto& [dx, dy] = centers_of_pixel[k];
				if (!insideTriangle(x + dx, y + dy, t.v))
					continue;

				auto [alpha, beta, gamma] = computeBarycentric2D(x + dx, y + dy, t.v);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;

				// z-buffering algorithm
				auto index = get_super_index(x, y, k);
				if (z_interpolated < super_depth_buf[index]) {
					super_depth_buf[index] = z_interpolated;
					super_frame_buf[index] = t.getColor();
					depth_test = true;
				}

				if (depth_test) {
					set_pixel(Vector3f(x, y, 0), get_super_color(x, y));
				}
			}
		}
	}

#else
	// iterate through the pixel and find if the current pixel is inside the triangle
	for (int x = min_x; x <= max_x; x++) {
		for (int y = min_y; y <= max_y; y++) {
			// If so, use the following code to get the interpolated z value.
			if (insideTriangle(x + 0.5, y + 0.5, t.v)) {
				auto [alpha, beta, gamma] = computeBarycentric2D(x + 0.5, y + 0.5, t.v);
				float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal; // 得到当前像素的深度值

				// z-buffering algorithm
				if (z_interpolated < depth_buf[get_index(x, y)]) {
					// TODO : set the current pixel (use the set_pixel function) 
					// to the color of the triangle (use getColor function) 
					// if it should be painted.
					Vector3f point(x, y, z_interpolated);
					set_pixel(point, t.getColor());
					depth_buf[get_index(x, y)] = z_interpolated;
				}
			}
		}
	}
#endif
}


void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
	if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
	{
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
		std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
	super_frame_buf.resize(w * h * msaa_times());
	super_depth_buf.resize(w * h * msaa_times());
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

int rst::rasterizer::get_super_index(int x, int y, int k)
{
	return (height - 1 - y) * width * msaa_times() + (width - 1 - x) * msaa_times() + k;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;
}

Vector3f rst::rasterizer::get_super_color(int x, int y)
{
	auto index = get_super_index(x, y, 0);
	Vector3f sum(0.0, 0.0, 0.0);
	for (int i = 0; i < msaa_times(); i++) {
		sum += super_frame_buf[index + i];
	}

	return sum / 4.0f;
}

// clang-format on