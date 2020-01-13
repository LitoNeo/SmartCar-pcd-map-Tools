#include "pcd_grid_divider.h"

namespace MAP_TOOLS {
pcd_grid_divider::pcd_grid_divider() {
  this->is_in_folder_set = false;
  this->is_out_folder_set = false;
  this->is_grid_size_set = false;
  this->is_voxel_size_set = false;
}

pcd_grid_divider::~pcd_grid_divider() {}

void pcd_grid_divider::reset() {
  _in_folder = "";
  this->is_in_folder_set = false;

  _out_folder = "";
  this->is_out_folder_set = false;

  grid_size = 30; // default 30
  this->is_grid_size_set = false;

  voxel_size = 0.0;
  this->is_voxel_size_set = false;
}

void pcd_grid_divider::setInFolder(const std::string &folder) {
  _in_folder = folder;
  is_in_folder_set = true;
  if (EndWith(_in_folder, ".pcd") || EndWith(_in_folder, ".PCD"))
    return;
  if (!EndWith(_in_folder, "/"))
    _in_folder = _in_folder + "/";
  return;
}

void pcd_grid_divider::setOutFolder(const std::string &folder) {
  _out_folder = folder;
  is_out_folder_set = true;
  if (!EndWith(_out_folder, "/"))
    _out_folder = _out_folder + "/";
  return;
}

void pcd_grid_divider::setGridSize(double grid_size) {
  if (grid_size < 1.0) {
    std::cout << "grid_size too small, please reset" << std::endl;
    return;
  }
  this->grid_size = grid_size;
  this->is_grid_size_set = true;
  return;
}

void pcd_grid_divider::setVoxelSize(double voxel_size) {
  if (voxel_size < 0.1) {
    this->is_voxel_size_set = false;
    return;
  }
  this->voxel_size = voxel_size;
  this->is_voxel_size_set = true;
  voxel_grid_filter.setLeafSize(this->voxel_size, this->voxel_size,
                                this->voxel_size);
  return;
}

std::vector<std::string> pcd_grid_divider::getAllFiles() {
  // for single pcd file
  if (EndWith(_in_folder, ".pcd") || EndWith(_in_folder, ".PCD")) {
    return std::vector<std::string>{_in_folder};
  }

  // for pcd files folder
  DIR *dir;
  struct dirent *ptr;
  char base[1000];
  if ((dir = opendir(_in_folder.c_str())) == NULL) {
    perror("Open dir error...");
    std::cout << "Check: " << _in_folder << std::endl;
    exit(1);
  }
  std::vector<std::string> files;
  while ((ptr = readdir(dir)) != NULL) {
    if (ptr->d_type == 8) // 文件
    {
      std::string name = ptr->d_name;
      if (EndWith(name, ".pcd") || EndWith(name, ".PCD")) {
        // std::cout << _in_folder + name << std::endl;
        files.push_back(_in_folder + name); // 记录所有pcd文件名
      }
    }
  }
  closedir(dir);
  std::sort(files.begin(), files.end());
  return files;
}

void pcd_grid_divider::check_point_type(const std::string &file) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp3(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  // std::cout << pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file, *tmp3) <<
  // std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(file, *tmp3) == 0) {
    using Point = pcl::PointXYZRGBA;
    using PointCloud = pcl::PointCloud<Point>;
    std::cout << "Point Type is PointXYZRGBA" << std::endl;
    std::cout << tmp3->points.size() << std::endl;
    // delete &tmp3;
    return;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  // std::cout << pcl::io::loadPCDFile<pcl::PointXYZRGB>(file, *tmp2) <<
  // std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(file, *tmp2) == 0) {
    using Point = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<Point>;
    std::cout << "Point Type is PointXYZRGB" << std::endl;
    // delete &tmp2;
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp1(
      new pcl::PointCloud<pcl::PointXYZI>);
  // std::cout << pcl::io::loadPCDFile<pcl::PointXYZI>(file, *tmp1) <<
  // std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(file, *tmp1) == 0) {
    using Point = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<Point>;
    std::cout << "Point Type is PointXYZI" << std::endl;
    // delete &tmp1;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
  ;
  // std::cout << pcl::io::loadPCDFile<pcl::PointXYZ>(file, *tmp) << std::endl;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *tmp) == 0) {
    using Point = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;
    std::cout << "Point Type is PointXYZ" << std::endl;
    // delete &tmp;
    return;
  }

  std::cout << "no type detected, exit" << std::endl;
  exit(1);
}

// template <typename T>
bool pcd_grid_divider::write_csv(const std::vector<pcd_grid> &grids) {
  std::string whole_file_name = _out_folder + CSV_FILE_NAME;
  std::cout << "csv file name = " << whole_file_name << std::endl;
  std::ofstream ofs(whole_file_name.c_str());
  int grid_num = grids.size();
  for (int i = 0; i < grid_num; i++) {
    if (grids[i].cloud.points.size() > 0) {
      ofs << grids[i].name << "," << grids[i].lower_bound_x << ","
          << grids[i].lower_bound_y << "," << 0.0 << ","
          << grids[i].upper_bound_x << "," << grids[i].upper_bound_y << ","
          << 0.0 << std::endl;
    }
  }
  return true;
}

bool pcd_grid_divider::run() {
  if (!is_in_folder_set || !is_out_folder_set || !is_grid_size_set) {
    std::cout << "confirm in_folder/out_folder/grid_size first!" << std::endl;
    return false;
  }
  std::vector<std::string> files = getAllFiles();
  if (files.size() == 0) {
    std::cout << "No pcd file found" << std::endl;
    return false;
  }

  // load all pcds
  // check_point_type(files[0]);
  PointCloud::Ptr tmp(new PointCloud);
  PointCloud::Ptr map(new PointCloud);
  for (size_t i = 0; i < files.size(); i++) {
    tmp->clear();
    if (pcl::io::loadPCDFile<Point>(files[i], *tmp) == -1) {
      std::cout << "Failed to load " << files[i] << "." << std::endl;
    }
    *map += *tmp;
    std::cout << "Finished to load " << files[i] << " " << std::endl;
  }

  std::cout << "Finished to load all PCDs: " << map->size() << " points."
            << std::endl;

  double min_x = 10000000000.0;
  double max_x = -10000000000.0;
  double min_y = 10000000000.0;
  double max_y = -10000000000.0;

  // Search minimum and maximum points along x and y axis.
  for (PointCloud::const_iterator p = map->begin(); p != map->end(); p++) {
    if (p->x < min_x) {
      min_x = p->x;
    }
    if (p->x > max_x) {
      max_x = p->x;
    }
    if (p->y < min_y) {
      min_y = p->y;
    }
    if (p->y > max_y) {
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
  std::vector<pcd_grid> grids(grid_num);
  for (int y = 0; y < div_y; y++) {
    for (int x = 0; x < div_x; x++) {
      int id = div_x * y + x;
      grids[id].grid_id = id;
      grids[id].grid_id_x = x;
      grids[id].grid_id_y = y;
      grids[id].lower_bound_x = min_x_b + grid_size * x;
      grids[id].lower_bound_y = min_y_b + grid_size * y;
      grids[id].upper_bound_x = min_x_b + grid_size * (x + 1);
      grids[id].upper_bound_y = min_y_b + grid_size * (y + 1);
      grids[id].filename = _out_folder + std::to_string(int(grid_size)) + "_" +
                           std::to_string(grids[id].lower_bound_x) + "_" +
                           std::to_string(grids[id].lower_bound_y) + ".pcd";
      grids[id].name = std::to_string(int(grid_size)) + "_" +
                       std::to_string(grids[id].lower_bound_x) + "_" +
                       std::to_string(grids[id].lower_bound_y) + ".pcd";
    }
  }

  // Assign all points to appropriate grid according to their x/y value
  for (PointCloud::const_iterator p = map->points.begin();
       p != map->points.end(); p++) {
    int idx = static_cast<int>(
        floor((p->x - static_cast<float>(min_x_b)) / grid_size));
    int idy = static_cast<int>(
        floor((p->y - static_cast<float>(min_y_b)) / grid_size));
    int id = idy * div_x + idx;

    Point tmp = *p;
    grids[id].cloud.push_back(tmp);
  }

  int points_num = 0;
  for (int i = 0; i < grid_num; i++) {
    if (grids[i].cloud.points.size() > 0) {
      try {
        if (this->is_voxel_size_set) {
          PointCloud::Ptr scan_ptr(new PointCloud);
          PointCloud::Ptr cur(new PointCloud);
          for (auto p : grids[i].cloud) {
            Point t = p;
            cur->push_back(t);
          }
          voxel_grid_filter.setInputCloud(cur);
          voxel_grid_filter.filter(*scan_ptr);
          pcl::io::savePCDFileBinary(grids[i].filename, *scan_ptr);
          std::cout << "Wrote " << scan_ptr->points.size() << " points to "
                    << grids[i].filename << "." << std::endl;
          points_num += scan_ptr->points.size();
        } else {
          pcl::io::savePCDFileBinary(grids[i].filename, grids[i].cloud);
          std::cout << "Wrote " << grids[i].cloud.points.size() << " points to "
                    << grids[i].filename << "." << std::endl;
          points_num += grids[i].cloud.points.size();
        }
      } catch (...) {
        std::cout << "IO exception" << std::endl;
      }
    }
  }
  write_csv(grids);
  std::cout << "Total points num: " << points_num << " points." << std::endl;
  return true;
}
} // namespace MAP_TOOLS