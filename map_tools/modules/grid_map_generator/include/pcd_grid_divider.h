#ifndef MODULES_PCD_GRID_DIVIDER
#define MODULES_PCD_GRID_DIVIDER

#include "../common/macro.h"
#include "../common/util.h"
#include <QMainWindow>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sstream>
#include <string>
#include <vector>

namespace MAP_TOOLS{
// default
using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;

struct pcd_grid
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
    PointCloud cloud;
};

class pcd_grid_divider
{
private:
    std::string _in_folder, _single_pcd_file;
    std::string _out_folder;
    double grid_size, voxel_size;
    pcl::VoxelGrid<Point> voxel_grid_filter;
    const std::string CSV_FILE_NAME = "pcd_info.csv";

    bool is_in_folder_set, is_out_folder_set, is_grid_size_set, is_voxel_size_set;

public:
    void setInFolder(const std::string &folder); // or input file
    void setOutFolder(const std::string &folder);
    void setGridSize(double grid_size);
    void setVoxelSize(double voxel_size);

    void check_point_type(const std::string &file);

    bool run();

private:
    std::vector<std::string> getAllFiles();
    // template <typename T>
    bool write_csv(const std::vector<pcd_grid> &grids);

    void reset();

public:
    pcd_grid_divider();
    ~pcd_grid_divider();

    // public:
    //     DECLARE_SINGLETON(pcd_grid_divider);
};
}
#endif