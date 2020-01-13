#ifndef COMMON_H
#define COMMON_H
#include <string>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io_exception.h>
#include <nav_msgs/Path.h>

namespace smartcar
{
namespace utils
{

enum traj_type
{
    LANE = 0,
    CROSS = 1,
};

struct traj_info
{
    traj_type type;
    int id;
    std::vector<int> pre_ids;
    std::vector<int> next_ids;
    bool reverse;

    //
    double length;
    double weight;
    nav_msgs::Path path;
};

// String function
bool endswith(const std::string &str, const std::string &pattern);

// File function
// 默认参数在cpp文件定义中设置,头文件中只声明,不设置/定义
std::vector<std::string> get_file_list(const std::string folder, bool with_full_path);

// pcl function
pcl::PointCloud<pcl::PointXYZI>::Ptr load_pcd_folder(const std::string folder);

}; // namespace utils
}; // namespace smartcar

#endif // COMMON_H
