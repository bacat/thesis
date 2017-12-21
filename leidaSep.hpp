#ifndef LEIDASEP_HPP_INCLUDED
#define LEIDASEP_HPP_INCLUDED

#include <vector>

#include <pcl/common/pca.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include "C:\Users\Bacsu Attila\Downloads\Grid.hpp"

using std::vector;

//Kikeresem a globális minimumot, maximumot x és y szerint egy pontfelhõbõl
std::array<float, 4> find_min_max(const pcl::PointCloud<pcl::PointXYZRGB> &c)
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

//Kikeresem a globális minimumot, maximumot x és y szerint egy vectorból
std::array<float, 4> find_min_max(const vector<pcl::PointXYZRGB> &c)
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

//Lokális minimum keresése z szerint
std::vector<float> loc_mins;

float loc_min(const std::vector<pcl::PointXYZRGB> &t)
{
	float ret = INFINITY;
	for (auto point : t)
	{
		if (point.z < ret)
			ret = point.z;
	}

	return ret;
}
/*
float loc_min(const vector<pcl::PointXYZRGB> &t)
{
    if (t.size() == 0) {

        return -1;
    }

	float ret = INFINITY;
	for (auto point : t)
	{
		if (point.z < ret)
			ret = point.z;
	}

	return ret;
}
*/
//Négyzetes kategóriákba osztom a pontokat, a paraméterben megadott sor és oszlop szám szerint
vector<vector<vector<pcl::PointXYZRGB>>> grid(const pcl::PointCloud<pcl::PointXYZRGB> &c, const int SIZE)
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

	vector<vector<vector<pcl::PointXYZRGB>>> table(num_rows, vector<vector<pcl::PointXYZRGB>>(num_cols));

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

//Négyzetes kategóriákba osztom a pontokat, a paraméterben megadott cella méretek alapján
vector<vector<vector<pcl::PointXYZRGB>>> grid(const pcl::PointCloud<pcl::PointXYZRGB> &c, const float dx, const float dy)
{
	auto minmax = find_min_max(c);

	float min_x = minmax[0];
	float max_x = minmax[2];
	const size_t num_rows = floor((max_x - min_x) / dx);

	float min_y = minmax[1];
	float max_y = minmax[3];
	const size_t num_cols = floor((max_y - min_y) / dy);

	if (num_cols == 0 || num_rows == 0) return vector<vector<vector<pcl::PointXYZRGB>>>(0);

	vector<vector<vector<pcl::PointXYZRGB>>> table(num_rows, vector<vector<pcl::PointXYZRGB>>(num_cols));

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

//Négyzetes kategóriákba osztom a pontokat, a paraméterben megadott sor és oszlop szám szerint, vektorból
vector<vector<vector<pcl::PointXYZRGB>>> grid(const vector<pcl::PointXYZRGB> &c, const int SIZE)
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

	vector<vector<vector<pcl::PointXYZRGB>>> table(num_rows, vector<vector<pcl::PointXYZRGB>>(num_cols));

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

std::vector<pcl::PointXYZRGB> Approx_thread(const size_t begin, const size_t end, const std::vector<std::vector<std::vector<pcl::PointXYZRGB>>> &ref)
{
	std::vector<pcl::PointXYZRGB> local;

	for (size_t i = begin; i < end; i++) {
		for (auto col : ref[i]) {
			auto g_min = loc_min(col);
			Grid *g = new Grid();
			auto scale = g->grid(col, 100);
			for (auto row1 : scale) {
				for (auto col1 : row1) {
					auto l_min = loc_min(col1);
					for (auto cell : col1) {
						if (cell.z > g_min + 0.2 && cell.z <= l_min && g_min + 0.25 < l_min) {
							local.push_back(cell);
						}
					}
				}
			}
		}
	}

	return local;
}



std::vector<size_t> partition_lower_boundaries(const size_t size)
{
	const unsigned nCores = std::thread::hardware_concurrency() / 2;

	const size_t end = size;

	if (nCores < 2) {
		//1 thread
		return{
			0,
			end
		};
	}
	else if (nCores < 4) {
		// 2 threads
		return{
			0,
			1 * (size) / 2,
			end
		};
	}
	else if (nCores < 8) {
		// 4 threads
		return{
			0,
			1 * (size) / 4,
			2 * (size) / 4,
			3 * (size) / 4,
			end
		};
	}
	else if (nCores < 16) {
		// 8 threads
		return{
			0,
			1 * (size) / 8,
			2 * (size) / 8,
			3 * (size) / 8,
			4 * (size) / 8,
			5 * (size) / 8,
			6 * (size) / 8,
			7 * (size) / 8,
			end
		};
	}
	else {
		// 16+ threads
		return{
			0,
			1 * (size) / 16,
			2 * (size) / 16,
			3 * (size) / 16,
			4 * (size) / 16,
			5 * (size) / 16,
			6 * (size) / 16,
			7 * (size) / 16,
			8 * (size) / 16,
			9 * (size) / 16,
			10 * (size) / 16,
			11 * (size) / 16,
			12 * (size) / 16,
			13 * (size) / 16,
			14 * (size) / 16,
			15 * (size) / 16,
			end
		};
	}
}

void Approximation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &c, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &o)
{
	Grid *g = new Grid();
	auto ref = g->grid(*c, 1, 1);

	const auto partition_bounds = partition_lower_boundaries(ref.size());
	const unsigned used_threads = partition_bounds.size() - 1;
	std::vector<std::future<std::vector<pcl::PointXYZRGB>>> threads;

	for (size_t i = 0; i < used_threads; i++) {
		auto fut = std::async(
			std::launch::async,
			Approx_thread, partition_bounds[i], partition_bounds[i + 1], ref);
		threads.push_back(std::move(fut));
	}

	for (auto &fut : threads) {
		auto tl = fut.get();
		for (const auto p : tl) {
			o->points.push_back(p);
		}
	}
	ref.clear();
}
/*
pcl::PointCloud<pcl::PointXYZRGB>::Ptr redundancy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
	std::stable_sort(cloud->points.begin(), cloud->points.end(),
		[](pcl::PointXYZRGB first, pcl::PointXYZRGB second) {
		if (first.x < second.x)
			return true;
		if (first.x > second.x)
			return false;
		if (first.y < second.y)
			return true;
		if (first.y > second.y)
			return false;
		return first.z < second.z;
	});

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr retval(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (size_t i = 0; i < cloud->points.size() - 1; i++) {
		if (cloud->points[i].x != cloud->points[i + 1].x || cloud->points[i].y != cloud->points[i + 1].y || cloud->points[i].z != cloud->points[i + 1].z) {
			retval->points.push_back(cloud->points[i]);
		}
	}

	return retval;
}
*/

//Pontosítjuk a szeparálás eredményét
/*
void Approximation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &c, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &o)
{
    if(c->points.size() == 0) return;

	vector<vector<vector<pcl::PointXYZRGB>>> ref = grid(*c, 1, 1);
	if (ref.size() == 0)    return;

	vector<std::future<void>> threads;

	for(size_t i{0}; i < 4; i++)
	{
        std::future<void> result( std::async([&]()
        {
            for (size_t j{i}; j < ref.size(); j+=4) {
                auto row = ref[j];
                for (auto col : row) {
                    auto g_min = loc_min(col);
                    vector<vector<vector<pcl::PointXYZRGB>>> scale = grid(col, 200);
                    for (auto row1 : scale) {
                        for (auto col1 : row1) {
                            auto l_min = loc_min(col1);
                            for (auto cell : col1) {
                                if (cell.z > g_min + 0.25 && cell.z <= l_min) {
                                    o->points.push_back(cell);
                                }
                            }
                        }
                    }
                }
            }

        }));
	
        threads.push_back(std::move(result));

    }

	std::cout << "itt van valami\n";
    std::cout << threads.size() << '\n';
	std::cout << "itt van valami2\n";
    for(auto &thread : threads) thread.get();
	std::cout << "itt van valami3\n";
}
*/
//Szeparálom gridenkénti lokális minimum keresés szerint
pcl::PointCloud<pcl::PointXYZRGB>::Ptr separation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr c)
{
	//Generáljuk le a gridet
	vector<vector<vector<pcl::PointXYZRGB>>> table = grid(*c, 1000);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGB>());

	const float K = 0.2;
	for (auto row : table)
	{
		for (auto col : row)
		{
			auto min = loc_min(col);
			if (col.size() > 2) {
				for (auto cell : col)
				{
					if (cell.z > min + K)
					{
						object->points.push_back(cell);
					}
				}
			}
		}
	}

	return object;
}

//Filtering
pcl::PointCloud<pcl::PointXYZRGB>::Ptr SOR(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(25);
	sor.setStddevMulThresh(0.5);
	sor.setNegative(false);
	sor.filter(*cloud_filtered);

	return cloud_filtered;
}

//Filter által leszedett magasabb dolgok visszarakása (fák, felsõvez, stb...)
void filterApprox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &filtered_cloud) {
	float max = -INFINITY;
	float min = INFINITY;
	for (const auto & p : cloud->points) {
		if (p.z > max)	max = p.z;
		if (p.z < min)	min = p.z;
	}

	auto dz = (max - min) / 3;
	for (const auto point : cloud->points) {
		if (point.z > min + dz)		filtered_cloud->points.push_back(point);
	}
}

//Kiszedjük az approximáció miatti redundaciákat
pcl::PointCloud<pcl::PointXYZRGB>::Ptr redundancy(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    if (cloud->points.size() == 0)  return cloud;

	std::stable_sort(cloud->points.begin(), cloud->points.end(),
		[](pcl::PointXYZRGB first, pcl::PointXYZRGB second) {
		if (first.x < second.x)
			return true;
		if (first.x > second.x)
			return false;
		if (first.y < second.y)
			return true;
		if (first.y > second.y)
			return false;
		return first.z < second.z;
	});

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr retval(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (size_t i = 0; i < cloud->points.size() - 1; i++) {
		if (cloud->points[i].x != cloud->points[i + 1].x || cloud->points[i].y != cloud->points[i + 1].y || cloud->points[i].z != cloud->points[i + 1].z) {
			retval->points.push_back(cloud->points[i]);
		}
	}

	return retval;
}


#endif // LEIDASEP_HPP_INCLUDED
