#ifndef GRID_HPP_INCLUDED
#define GRID_HPP_INCLUDED

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Grid
{
public:
	std::array<float, 4> find_min_max(const pcl::PointCloud<pcl::PointXYZRGB> &c)
	{
		// min_x, min_y, max_x, max_y
		std::array<float, 4> ret{ INFINITY, INFINITY, -INFINITY, -INFINITY };
		for (const auto &p : c)
		{
			if (p.x < ret[0])
			{
				ret[0] = p.x;
			}

			if (p.y < ret[1])
			{
				ret[1] = p.y;
			}

			if (p.x > ret[2])
			{
				ret[2] = p.x;
			}

			if (p.y > ret[3])
			{
				ret[3] = p.y;
			}
		}

		return ret;
	}

	std::array<float, 4> find_min_max(const std::vector<pcl::PointXYZRGB> &c)
	{
		//min_x, min_y, max_x, max_y
		std::array<float, 4> ret{ INFINITY, INFINITY, -INFINITY, -INFINITY };
		for (const auto &p : c)
		{
			if (p.x < ret[0])
			{
				ret[0] = p.x;
			}

			if (p.y < ret[1])
			{
				ret[1] = p.y;
			}

			if (p.x > ret[2])
			{
				ret[2] = p.x;
			}

			if (p.y > ret[3])
			{
				ret[3] = p.y;
			}
		}

		return ret;
	}

	std::vector<std::vector<std::vector<pcl::PointXYZRGB>>> grid(const pcl::PointCloud<pcl::PointXYZRGB> &c, const float dx, const float dy)
	{
		auto minmax = find_min_max(c);

		float min_x = minmax[0];
		float max_x = minmax[2];
		const size_t num_rows = floor((max_x - min_x) / dx);

		float min_y = minmax[1];
		float max_y = minmax[3];
		const size_t num_cols = floor((max_y - min_y) / dy);

		std::vector<std::vector<std::vector<pcl::PointXYZRGB>>> table(num_rows, std::vector<std::vector<pcl::PointXYZRGB>>(num_cols));

		for (const auto point : c.points)
		{
			size_t row = floor((point.x - min_x) / dx);
			size_t col = floor((point.y - min_y) / dy);

			if (row == num_rows)
			{
				row--;
			}

			if (col == num_cols)
			{
				col--;
			}

			table[row][col].push_back(point);
		}

		return table;
	}

	std::vector<std::vector<std::vector<pcl::PointXYZRGB>>> grid(const std::vector<pcl::PointXYZRGB> &c, const int SIZE)
	{
		const size_t num_rows = SIZE;
		const size_t num_cols = SIZE;

		auto minmax = find_min_max(c);

		float min_x = minmax[0];
		float max_x = minmax[2];
		float dx = (max_x - min_x) / num_rows;

		float min_y = minmax[1];
		float max_y = minmax[3];
		float dy = (max_y - min_y) / num_cols;

		std::vector<std::vector<std::vector<pcl::PointXYZRGB>>> table(num_rows, std::vector<std::vector<pcl::PointXYZRGB>>(num_cols));

		for (const auto point : c)
		{
			size_t row = floor((point.x - min_x) / dx);
			size_t col = floor((point.y - min_y) / dy);

			if (row == num_rows)
			{
				row--;
			}

			if (col == num_cols)
			{
				col--;
			}

			table[row][col].push_back(point);
		}

		return table;
	}
};

#endif GRID_HPP_INCLUDED