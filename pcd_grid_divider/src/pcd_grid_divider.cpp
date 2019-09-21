/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: Yuki Kitsukawa (yuki.kitsukawa@tier4.jp)
 *
 * pcd_grid_divider.cpp
 *
 *  Created on: May 15, 2018
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sstream>
#include <string>
#include <vector>

std::string OUT_DIR;
const std::string FILE_NAME = "pcd_info.csv";

struct pcd_xyz_grid
{
  std::string filename;
  std::string name;
  int grid_id;
  int grid_id_x;
  int grid_id_y;
  int lower_bound_x;
  int lower_bound_y;
  int upper_bound_x;
  int upper_bound_y;
  pcl::PointCloud<pcl::PointXYZ> cloud;
};

struct pcd_xyzi_grid
{
  std::string filename;
  int grid_id;
  int grid_id_x;
  int grid_id_y;
  int lower_bound_x;
  int lower_bound_y;
  int upper_bound_x;
  int upper_bound_y;
  pcl::PointCloud<pcl::PointXYZI> cloud;
};

struct pcd_xyzrgb_grid
{
  std::string filename;
  int grid_id;
  int grid_id_x;
  int grid_id_y;
  int lower_bound_x;
  int lower_bound_y;
  int upper_bound_x;
  int upper_bound_y;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
};

void getAllFiles(std::string path, std::vector<std::string> &files)
{
  if (path[path.length() - 1] != '/')
    path = path + "/";
  DIR *dir;
  struct dirent *ptr;
  char base[1000];
  if ((dir = opendir(path.c_str())) == NULL)
  {
    perror("Open dir error...");
    std::cout << "Check: " << path << std::endl;
    exit(1);
  }

  while ((ptr = readdir(dir)) != NULL)
  {
    if (ptr->d_type == 8) // 文件
    {
      std::string name = ptr->d_name;
      int length = name.length();
      if (name.substr(length - 3, length - 1) == "pcd" || name.substr(length - 3, length - 1) == "PCD")
      {
        std::cout << path + name << std::endl;
        files.push_back(path + name);
      }
    }
  }
  closedir(dir);
  std::sort(files.begin(), files.end());
  return;
}

// struct Area
// {
// 	std::string path;
// 	double x_min;
// 	double y_min;
// 	double z_min;
// 	double x_max;
// 	double y_max;
// 	double z_max;
// };
void write_csv(std::vector<pcd_xyz_grid> &grids)
{
  std::string whole_file_name = OUT_DIR + FILE_NAME;
  std::ofstream ofs(whole_file_name.c_str());
  int grid_num = grids.size();
  for (int i = 0; i < grid_num; i++)
  {
    if (grids[i].cloud.points.size() > 0)
    {
      ofs << grids[i].name
          << "," << grids[i].lower_bound_x
          << "," << grids[i].lower_bound_y
          << "," << 0.0
          << "," << grids[i].upper_bound_x
          << "," << grids[i].upper_bound_y
          << "," << 0.0 << std::endl;
    }
  }
}

using Point = pcl::PointXYZ; // default
using PointCloud = pcl::PointCloud<Point>;

int main(int argc, char **argv)
{

  if (argc < 4)
  {
    std::cout << "Usage: rosrun map_tools pcd_grid_divider \"point_type "
                 "[PointXYZ|PointXYZI|PointXYZRGB]\" \"grid_size\" \"input-directory\" \"output-directory\"  "
              << std::endl;
  }

  std::vector<std::string> files;

  std::string point_type = argv[1];
  int grid_size = std::stoi(argv[2]);
  std::string input_dir = argv[3];
  OUT_DIR = argv[4];
  if (OUT_DIR[OUT_DIR.length() - 1] != '/')
    OUT_DIR = OUT_DIR + "/";

  getAllFiles(input_dir, files);

  if (point_type == "PointXYZ")
  {
    using Point = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  }
  else if (point_type == "PointXYZI")
  {
    using Point = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
  }
  else if (point_type == "PointXYZRGB")
  {
    using Point = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
  }

  // Load all PCDs
  PointCloud map;
  PointCloud tmp;
  for (int i = 0; i < files.size(); i++)
  {
    if (pcl::io::loadPCDFile<Point>(files[i], tmp) == -1)
    {
      std::cout << "Failed to load " << files[i] << "." << std::endl;
    }
    map += tmp;
    std::cout << "Finished to load " << files[i] << "." << std::endl;
  }

  std::cout << "Finished to load all PCDs: " << map.size() << " points." << std::endl;

  double min_x = 10000000000.0;
  double max_x = -10000000000.0;
  double min_y = 10000000000.0;
  double max_y = -10000000000.0;

  // Search minimum and maximum points along x and y axis.
  for (PointCloud::const_iterator p = map.begin();
       p != map.end(); p++)
  {
    if (p->x < min_x)
    {
      min_x = p->x;
    }
    if (p->x > max_x)
    {
      max_x = p->x;
    }
    if (p->y < min_y)
    {
      min_y = p->y;
    }
    if (p->y > max_y)
    {
      max_y = p->y;
    }
  }

  // Find minimum and maximum boundary
  int min_x_b = grid_size * static_cast<int>(floor(min_x / grid_size));
  int max_x_b = grid_size * static_cast<int>(floor(max_x / grid_size) + 1);
  int min_y_b = grid_size * static_cast<int>(floor(min_y / grid_size));
  int max_y_b = grid_size * static_cast<int>(floor(max_y / grid_size) + 1);

  // Number of grid along x and y axis
  int div_x = (max_x_b - min_x_b) / grid_size;
  int div_y = (max_y_b - min_y_b) / grid_size;
  int grid_num = div_x * div_y;

  // Define filename, lower/upper bound of every grid
  std::vector<pcd_xyz_grid> grids(grid_num);
  for (int y = 0; y < div_y; y++)
  {
    for (int x = 0; x < div_x; x++)
    {
      int id = div_x * y + x;
      grids[id].grid_id = id;
      grids[id].grid_id_x = x;
      grids[id].grid_id_y = y;
      grids[id].lower_bound_x = min_x_b + grid_size * x;
      grids[id].lower_bound_y = min_y_b + grid_size * y;
      grids[id].upper_bound_x = min_x_b + grid_size * (x + 1);
      grids[id].upper_bound_y = min_y_b + grid_size * (y + 1);
      grids[id].filename = OUT_DIR + std::to_string(grid_size) + "_" +
                           std::to_string(grids[id].lower_bound_x) + "_" +
                           std::to_string(grids[id].lower_bound_y) + ".pcd";
      grids[id].name = std::to_string(grid_size) + "_" +
                       std::to_string(grids[id].lower_bound_x) + "_" +
                       std::to_string(grids[id].lower_bound_y) + ".pcd";
    }
  }

  // Assign all points to appropriate grid according to their x/y value
  for (PointCloud::const_iterator p = map.begin(); p != map.end(); p++)
  {
    int idx = static_cast<int>(floor((p->x - static_cast<float>(min_x_b)) / grid_size));
    int idy = static_cast<int>(floor((p->y - static_cast<float>(min_y_b)) / grid_size));
    int id = idy * div_x + idx;

    const Point &tmp = *p;
    grids[id].cloud.points.push_back(tmp);
  }

  int points_num = 0;
  for (int i = 0; i < grid_num; i++)
  {
    if (grids[i].cloud.points.size() > 0)
    {
      pcl::io::savePCDFileBinary(grids[i].filename, grids[i].cloud);
      std::cout << "Wrote " << grids[i].cloud.points.size() << " points to "
                << grids[i].filename << "." << std::endl;
      points_num += grids[i].cloud.points.size();
    }
  }
  write_csv(grids);
  std::cout << "Total points num: " << points_num << " points." << std::endl;
  return 0;
}
