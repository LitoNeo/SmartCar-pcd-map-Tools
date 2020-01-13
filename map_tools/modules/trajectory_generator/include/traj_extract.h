#ifndef TRAJ_EXTRACT_H
#define TRAJ_EXTRACT_H
#include <iostream>
#include <string>
#include "../../utils/common.h"
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

namespace smartcar
{
namespace tools
{
using smartcar::utils::load_pcd_folder;
using smartcar::utils::traj_info;
using smartcar::utils::traj_type;

class Traj_Extractor
{
private:
    std::string _input_folder;
    std::string _output_folder;

    ros::Subscriber sub_pose;
    ros::Publisher pub_path;
    ros::Publisher pub_markerArray;
    ros::Publisher pub_texts;
    ros::Publisher pub_map;
    ros::Publisher pub_marker_start;
    visualization_msgs::Marker marker_start;
    int cnt_text;

    nav_msgs::Path current_path;
    std::vector<traj_info> lane_list;
    std::vector<traj_info> cross_list;

    bool is_begin;
    int path_type;

    nav_msgs::Path origin_path; // the copy of current_path, used when weight changed
    double _weight_data;
    double _weight_smooth;
    double _tolerance;

    visualization_msgs::MarkerArray lines, MapPoints;
    visualization_msgs::MarkerArray texts;

public:
    bool launch_thread(const std::string &input_folder, const std::string &output_folder);
    void reload_map();
    bool set_folder(const std::string &input_folder, const std::string &output_folder);
    bool create_trajtory(traj_info &traj_info);
    bool delete_one_point();
    void reset_weight(double weight_data, double weight_smooth, double tolerance);
    bool write_back();
    bool shutdown();

private:
    bool on_shutdown();

    void init_ros();

    bool load_map();

    bool pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

    bool insert_pose(const geometry_msgs::PoseStamped pose);

    bool smooth_path(double weight_data, double weight_smooth, double tolerance);

    bool on_create_trajectory(traj_info &type);

    void on_weight_changed();

    double distance2points(const geometry_msgs::PoseStamped p1,
                           const geometry_msgs::PoseStamped p2);

    bool show();

    // // bool show_tests(std::string label, int id, std::vector<int> &pre_id,
    //                 std::vector<int> &next_id, int length, bool reverse,
    //                 geometry_msgs::Pose pose);

    bool add_text(const traj_info &type, const geometry_msgs::Pose &pose);
    bool add_line(const traj_info &type);

    bool on_write_back();

    bool write_func(const std::string path_type, const int id);

private:
    static Traj_Extractor *instance;

    Traj_Extractor()
    {
        _weight_data = 0.47;
        _weight_smooth = 0.2;
        _tolerance = 0.2;

        lane_list.resize(101); // 暂且认为lane和cross不会超过100条
        cross_list.resize(101);
        is_begin = true;
        cnt_text = 0;
    }

public:
    static Traj_Extractor &getInstance();

    ~Traj_Extractor()
    {
    }
};
}; // namespace tools
}; // namespace smartcar

#endif // TRAJ_EXTRACT_H
