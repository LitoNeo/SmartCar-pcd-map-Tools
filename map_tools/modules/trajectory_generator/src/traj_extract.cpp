#include "traj_extract.h"

#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
// #include <pcl/conversions.h>
// #include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>  // 实现sensor_msgs和pcl::PointCloud的转换

//--------------- public
// !! static 变量需要在cpp中进行初始化声明才能使用,否则报undefined reference错误
smartcar::tools::Traj_Extractor *smartcar::tools::Traj_Extractor::instance = 0;

smartcar::tools::Traj_Extractor &smartcar::tools::Traj_Extractor::getInstance() {
    if (!instance) {
        instance = new Traj_Extractor;
    }
    return *instance;
}

bool smartcar::tools::Traj_Extractor::launch_thread(const std::string &input_folder, const std::string &output_folder) {
    set_folder(input_folder, output_folder);
    init_ros();
    load_map();
    ros::spin();
}

bool smartcar::tools::Traj_Extractor::set_folder(const std::string &input_folder, const std::string &output_folder) {
    _input_folder = input_folder;
    _output_folder = output_folder;
    if (!smartcar::utils::endswith(input_folder, "/")) _input_folder += "/";
    if (!smartcar::utils::endswith(output_folder, "/")) _output_folder += "/";
}

bool smartcar::tools::Traj_Extractor::adjust_orientation() {
    int size = current_path.poses.size();
    if (size < 2) {
        return false;
    }
    // 调整每个轨迹点的角度
    for (size_t i = 0; i < size - 1; i++) {
        geometry_msgs::PoseStamped p_c = current_path.poses[i];
        geometry_msgs::PoseStamped p_n = current_path.poses[i + 1];
        double yaw = std::atan2(p_n.pose.position.y - p_c.pose.position.y, p_n.pose.position.x - p_c.pose.position.x);
        Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q = rollangle * yawangle * pitchangle;
        current_path.poses[i].pose.orientation.x = q.x();
        current_path.poses[i].pose.orientation.y = q.y();
        current_path.poses[i].pose.orientation.z = q.z();
        current_path.poses[i].pose.orientation.w = q.w();
    }
    current_path.poses[size - 1].pose.orientation.x = current_path.poses[size - 2].pose.orientation.x;
    current_path.poses[size - 1].pose.orientation.y = current_path.poses[size - 2].pose.orientation.x;
    current_path.poses[size - 1].pose.orientation.z = current_path.poses[size - 2].pose.orientation.x;
    current_path.poses[size - 1].pose.orientation.w = current_path.poses[size - 2].pose.orientation.x;

    return true;
}

bool smartcar::tools::Traj_Extractor::create_trajtory(traj_info &traj_info) { return on_create_trajectory(traj_info); }

bool smartcar::tools::Traj_Extractor::delete_one_point() {
    if (current_path.poses.empty()) return false;
    current_path.poses.pop_back();
    pub_path.publish(current_path);
    marker_start.pose = current_path.poses[current_path.poses.size() - 1].pose;
    pub_marker_start.publish(marker_start);
    return true;
}

bool smartcar::tools::Traj_Extractor::write_back() { return on_write_back(); }

void smartcar::tools::Traj_Extractor::reload_map() { load_map(); }

// ------------- private`
bool smartcar::tools::Traj_Extractor::load_map() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = load_pcd_folder(this->_input_folder);
    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(*cloud, msg_cloud);
    msg_cloud.header.frame_id = "/map";
    std::cout << "now publish map" << std::endl;
    pub_map.publish(msg_cloud);
    std::cout << "publish map with " << msg_cloud.data.size() << std::endl;
    return true;
}

void smartcar::tools::Traj_Extractor::init_ros() {
    ros::NodeHandle nh;
    current_path.header.frame_id = "map";
    sub_pose =
        nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&smartcar::tools::Traj_Extractor::pose_cb, this, _1));
    pub_path = nh.advertise<nav_msgs::Path>("/current_path", 1);
    pub_markerArray = nh.advertise<visualization_msgs::MarkerArray>("/path/trajectory_defined", 10);
    pub_texts = nh.advertise<visualization_msgs::MarkerArray>("/path/label", 1);
    pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map/full", 1);
    pub_marker_start = nh.advertise<visualization_msgs::Marker>("/marker/start_pose", 1);

    marker_start.header.frame_id = "map";
    marker_start.header.stamp = ros::Time::now();
    marker_start.id = 0;
    marker_start.type = visualization_msgs::Marker::CYLINDER;
    marker_start.action = visualization_msgs::Marker::ADD;
    marker_start.scale.x = 0.5;
    marker_start.scale.y = 0.2;
    marker_start.scale.z = 0.2;
    marker_start.color.r = 1;
    marker_start.color.g = 1;
    marker_start.color.b = 0;
    marker_start.color.a = 1;
}

bool smartcar::tools::Traj_Extractor::pose_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    marker_start.pose = msg->pose.pose;
    pub_marker_start.publish(marker_start);
    if (is_begin) {
        current_path.poses.clear();
        is_begin = false;
    }
    geometry_msgs::PoseStamped temp;
    temp.pose = msg->pose.pose;
    insert_pose(temp);

    // copy current_path to origin_path
    origin_path = current_path;

    smooth_path(_weight_data, _weight_smooth, _tolerance);
    adjust_orientation();
    pub_path.publish(current_path);
    return true;
}

bool smartcar::tools::Traj_Extractor::insert_pose(const geometry_msgs::PoseStamped pose) {
    if (current_path.poses.empty()) {
        current_path.poses.push_back(pose);
        return true;
    }
    int size = current_path.poses.size();
    geometry_msgs::PoseStamped start = current_path.poses[size - 1];
    geometry_msgs::PoseStamped end = pose;
    if (distance2points(start, end) <= 1.0) {
        current_path.poses.push_back(pose);
        return true;
    }
    double lenx = end.pose.position.x - start.pose.position.x;
    double leny = end.pose.position.y - start.pose.position.y;
    int cnt;
    if (std::abs(lenx) > std::abs(leny)) {
        cnt = int(std::abs(lenx)) * 3;
    } else {
        cnt = int(std::abs(leny)) * 3;
    }
    double dx = lenx / cnt;
    double dy = leny / cnt;

    for (int i = 1; i <= cnt; i++) {
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = start.pose.position.x + i * dx;
        temp.pose.position.y = start.pose.position.y + i * dy;
        current_path.poses.push_back(temp);
    }
    std::cout << "path size: " << current_path.poses.size() << std::endl;
    return true;
}

bool smartcar::tools::Traj_Extractor::smooth_path(double weight_data, double weight_smooth, double tolerance) {
    int size = current_path.poses.size();
    int count[size + 1];
    for (size_t i = 0; i <= size; i++) count[i] = 0;

    if (current_path.poses.size() <= 2) {
        // cout << "Can't Smooth Path, origin_path Size=" << path.size() << endl;
        return true;
    }
    std::cout << "Smooth the path." << std::endl;

    double change = _tolerance;
    double xtemp, ytemp;
    int nIterations = 0;

    while (change >= _tolerance) {
        change = 0.0;
        for (int i = std::max(size - 50, 1); i < size - 1; i++)  //
        {
            if (count[i] > 20) continue;
            xtemp = current_path.poses[i].pose.position.x;
            ytemp = current_path.poses[i].pose.position.y;

            current_path.poses[i].pose.position.x += weight_data * (origin_path.poses[i].pose.position.x - current_path.poses[i].pose.position.x);
            current_path.poses[i].pose.position.y += weight_data * (origin_path.poses[i].pose.position.y - current_path.poses[i].pose.position.y);

            current_path.poses[i].pose.position.x +=
                weight_smooth * (current_path.poses[i - 1].pose.position.x + current_path.poses[i + 1].pose.position.x -
                                 (2.0 * current_path.poses[i].pose.position.x));
            current_path.poses[i].pose.position.y +=
                weight_smooth * (current_path.poses[i - 1].pose.position.y + current_path.poses[i + 1].pose.position.y -
                                 (2.0 * current_path.poses[i].pose.position.y));

            change += fabs(xtemp - current_path.poses[i].pose.position.x);
            change += fabs(ytemp - current_path.poses[i].pose.position.y);
            count[i]++;
        }
        nIterations++;
    }
    return true;
}

double smartcar::tools::Traj_Extractor::distance2points(const geometry_msgs::PoseStamped p1, const geometry_msgs::PoseStamped p2) {
    return std::sqrt(std::pow(p2.pose.position.x - p1.pose.position.x, 2) + std::pow(p2.pose.position.y - p1.pose.position.y, 2));
}

bool smartcar::tools::Traj_Extractor::on_create_trajectory(traj_info &type) {
    if (current_path.poses.empty()) {
        ROS_WARN_STREAM("Current path unset! Please draw current path first.");
        return false;
    } else if (type.type == traj_type::LANE) {
        if (lane_list[type.id].path.poses.size() > 0) {
            lane_list[type.id].pre_ids.clear();
            lane_list[type.id].next_ids.clear();
            lane_list[type.id].path.poses.clear();
        }
        path_type = 1;

        // calculate the length of current_path
        double path_length = 0.0;
        for (int i = 0; i < current_path.poses.size() - 1; i++) {
            path_length += distance2points(current_path.poses[i], current_path.poses[i + 1]);
        }
        type.length = path_length;

        lane_list[type.id].id = type.id;
        lane_list[type.id].reverse = type.reverse;
        lane_list[type.id].length = path_length;

        for (int i = 0; i < type.pre_ids.size(); i++) {
            lane_list[type.id].pre_ids.push_back(type.pre_ids[i]);
        }
        for (int i = 0; i < type.next_ids.size(); i++) {
            lane_list[type.id].next_ids.push_back(type.next_ids[i]);
        }

        lane_list[type.id].path = current_path;
        int size = lane_list[type.id].path.poses.size();
        std::cout << "Insert lane id: " << type.id << " with " << size << " points" << std::endl;
    } else if (traj_type::CROSS) {
        if (cross_list[type.id].path.poses.size() > 0) {
            cross_list[type.id].pre_ids.clear();
            cross_list[type.id].next_ids.clear();
            cross_list[type.id].path.poses.clear();
        }
        path_type = 0;

        // calculate the length of current_path
        double path_length = 0.0;
        for (int i = 0; i < current_path.poses.size() - 1; i++) {
            path_length += distance2points(current_path.poses[i], current_path.poses[i + 1]);
        }
        type.length = path_length;

        cross_list[type.id].id = type.id;
        cross_list[type.id].reverse = type.reverse;
        cross_list[type.id].length = path_length;
        for (int i = 0; i < type.pre_ids.size(); i++) {
            cross_list[type.id].pre_ids.push_back(type.pre_ids[i]);
        }
        for (int i = 0; i < type.next_ids.size(); i++) {
            cross_list[type.id].next_ids.push_back(type.next_ids[i]);
        }

        cross_list[type.id].path = current_path;
        int size = cross_list[type.id].path.poses.size();
        std::cout << "Insert cross id: " << type.id << " with " << size << " points" << std::endl;
    }
    add_text(type, current_path.poses[current_path.poses.size() / 2].pose);
    add_line(type);
    is_begin = true;
    show();
    current_path.poses.clear();  // 每次发布完后归位current_path
    return true;
}

void smartcar::tools::Traj_Extractor::reset_weight(double weight_data, double weight_smooth, double tolerance) {
    _weight_data = weight_data;
    _weight_smooth = weight_smooth;
    _tolerance = tolerance;
    on_weight_changed();
}

void smartcar::tools::Traj_Extractor::on_weight_changed() {
    smooth_path(_weight_data, _weight_smooth, _tolerance);
    pub_path.publish(current_path);
}

bool smartcar::tools::Traj_Extractor::show() {
    pub_markerArray.publish(lines);
    pub_texts.publish(texts);
}

bool smartcar::tools::Traj_Extractor::add_text(const traj_info &type, const geometry_msgs::Pose &pose) {
    visualization_msgs::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = ros::Time::now();
    text.action = visualization_msgs::Marker::ADD;
    text.pose = pose;

    text.id = cnt_text++;
    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    std::stringstream str;

    if (type.type == traj_type::LANE)  // lane -> blue // color range: [0,1]
    {
        text.scale.z = 1.0;
        text.color.g = 1;
        text.color.b = 1;
        text.color.r = 0;
        text.color.a = 1;
        str << "lane " << type.id << "\n";
    } else {  // cross -> red
        text.scale.z = 1.0;
        text.color.g = 0;
        text.color.b = 0;
        text.color.r = 1;
        text.color.a = 1;
        str << "cross " << type.id << "\n";
    }

    str << "length: " << type.length << "\n";
    str << "pre_ids: ";
    for (int i = 0; i < type.pre_ids.size(); i++) {
        str << type.pre_ids[i] << " ";
    }
    str << "\n";

    str << "next_ids: ";
    for (int i = 0; i < type.next_ids.size(); i++) {
        str << type.next_ids[i] << " ";
    }
    str << "\n";

    str << "reverse: ";
    if (type.reverse)
        str << "True";
    else
        str << "False";

    text.text = str.str();
    texts.markers.push_back(text);
    return true;
}

bool smartcar::tools::Traj_Extractor::add_line(const traj_info &type) {
    static int cnt = 1;
    for (int j = 0; j < current_path.poses.size() - 1; j++) {
        visualization_msgs::Marker line_strip, points;
        line_strip.header.frame_id = "map";
        // points.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        // points.header.stamp = line_strip.header.stamp;
        // line_strip.ns = "WHOLE_PATH";
        line_strip.action = visualization_msgs::Marker::ADD;

        line_strip.id = cnt;  // the id is unique. or you can use 'lifetime' to flush
                              // out the older id with newer one
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.scale.x = 0.2;  // meters
        if (type.type == traj_type::LANE) {
            line_strip.color.b = 1.0;  // in the range of [0, 1]
            line_strip.color.a = 1.0;  // Don't forget to set the alpha! default:0 means transparent
        } else {
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;
        }

        geometry_msgs::Point p, q;
        p.x = current_path.poses[j].pose.position.x;
        p.y = current_path.poses[j].pose.position.y;

        q.x = current_path.poses[j + 1].pose.position.x;
        q.y = current_path.poses[j + 1].pose.position.y;

        line_strip.points.push_back(p);
        line_strip.points.push_back(q);
        lines.markers.push_back(line_strip);

        cnt++;
    }
}

bool smartcar::tools::Traj_Extractor::on_write_back() {
    // int lane_size,cross_size;
    // for(lane_size = 0;!lane_list[lane_size].path.poses.empty();lane_size++);
    // for(cross_size =
    // 0;!cross_list[cross_size].path.poses.empty();cross_size++);
    std::cout << "********* The Lane list is as followed *************" << std::endl;
    for (int i = 0; i < 101; i++) {
        if (lane_list[i].path.poses.empty()) continue;
        // pre
        std::cout << "lane id: " << i << std::endl;
        int before_size = lane_list[i].pre_ids.size();
        std::cout << "(pre)before size: " << before_size << std::endl;
        for (int j = 0; j < before_size; j++) std::cout << lane_list[i].pre_ids[j] << " ";
        std::cout << std::endl;

        // next
        int before_size1 = lane_list[i].next_ids.size();
        std::cout << "(next)before size: " << before_size1 << std::endl;
        for (int j = 0; j < before_size1; j++) std::cout << lane_list[i].next_ids[j] << " ";
        std::cout << std::endl;

        write_func("lane", i);
        std::cout << "-----------------" << std::endl;
    }

    std::cout << "********* The Cross list is as followed *************" << std::endl;
    for (int i = 0; i < 101; i++) {
        if (cross_list[i].path.poses.empty()) continue;
        // pre
        std::cout << "cross id: " << i << std::endl;
        int before_size = cross_list[i].pre_ids.size();
        std::cout << "(pre)before size: " << before_size << std::endl;
        for (int j = 0; j < before_size; j++) std::cout << cross_list[i].pre_ids[j] << " ";
        std::cout << std::endl;

        // next
        int before_size1 = cross_list[i].next_ids.size();
        std::cout << "(next)before size: " << before_size1 << std::endl;
        for (int j = 0; j < before_size1; j++) std::cout << cross_list[i].next_ids[j] << " ";
        std::cout << std::endl;

        write_func("cross", i);
        std::cout << "-----------------" << std::endl;
    }
    return true;
}

bool smartcar::tools::Traj_Extractor::write_func(const std::string path_type, const int id) {
    std::stringstream file_name;
    file_name << _output_folder << path_type << "_" << id << ".csv";
    std::ofstream f;
    f.open(file_name.str().c_str(), std::ios::out);
    if (!f.is_open()) {
        ROS_WARN_STREAM("Can not open file to save.");
        std::cout << "Check: " << file_name.str() << std::endl;
        return false;
    }

    // 由于Lane和Cross格式可能不一样,因此最好分别单独存储
    if (std::strcmp(path_type.c_str(), "lane") == 0) {
        f << id << std::endl << lane_list[id].length << std::endl;
        if (lane_list[id].reverse) {
            f << 1 << std::endl;
        } else {
            f << 0 << std::endl;
        }

        for (int i = 0; i < lane_list[id].pre_ids.size(); i++) {
            f << lane_list[id].pre_ids[i];
            if (i < lane_list[id].pre_ids.size() - 1) f << ",";
        }
        f << std::endl;

        for (int i = 0; i < lane_list[id].next_ids.size(); i++) {
            f << lane_list[id].next_ids[i];
            if (i < lane_list[id].next_ids.size() - 1) f << ",";
        }
        f << std::endl;

        int len_lane = lane_list[id].path.poses.size();
        for (size_t i = 0; i < len_lane - 1; i++) {
            geometry_msgs::PoseStamped p_c = lane_list[id].path.poses[i];
            geometry_msgs::PoseStamped p_n = lane_list[id].path.poses[i + 1];
            double yaw = std::atan2(p_n.pose.position.y - p_c.pose.position.y, p_n.pose.position.x - p_c.pose.position.x);
            Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
            Eigen::Quaterniond q = rollangle * yawangle * pitchangle;
            lane_list[id].path.poses[i].pose.orientation.x = q.x();
            lane_list[id].path.poses[i].pose.orientation.y = q.y();
            lane_list[id].path.poses[i].pose.orientation.z = q.z();
            lane_list[id].path.poses[i].pose.orientation.w = q.w();
        }
        lane_list[id].path.poses[len_lane - 1].pose.orientation.x = lane_list[id].path.poses[len_lane - 2].pose.orientation.x;
        lane_list[id].path.poses[len_lane - 1].pose.orientation.y = lane_list[id].path.poses[len_lane - 2].pose.orientation.y;
        lane_list[id].path.poses[len_lane - 1].pose.orientation.z = lane_list[id].path.poses[len_lane - 2].pose.orientation.z;
        lane_list[id].path.poses[len_lane - 1].pose.orientation.w = lane_list[id].path.poses[len_lane - 2].pose.orientation.w;

        for (int i = 0; i < lane_list[id].path.poses.size(); i++) {
            f << lane_list[id].path.poses[i].pose.position.x << "," << lane_list[id].path.poses[i].pose.position.y << ","
              << lane_list[id].path.poses[i].pose.position.z << "," << lane_list[id].path.poses[i].pose.orientation.x << ","
              << lane_list[id].path.poses[i].pose.orientation.y << "," << lane_list[id].path.poses[i].pose.orientation.z << ","
              << lane_list[id].path.poses[i].pose.orientation.w << std::endl;
        }
    } else {
        f << id << std::endl << cross_list[id].length << std::endl;
        if (cross_list[id].reverse) {
            f << 1 << std::endl;
        } else {
            f << 0 << std::endl;
        }

        for (int i = 0; i < cross_list[id].pre_ids.size(); i++) {
            f << cross_list[id].pre_ids[i];
            if (i < cross_list[id].pre_ids.size() - 1) f << ",";
        }
        f << std::endl;

        for (int i = 0; i < cross_list[id].next_ids.size(); i++) {
            f << cross_list[id].next_ids[i];
            if (i < cross_list[id].next_ids.size() - 1) f << ",";
        }
        f << std::endl;

        int len_lane = cross_list[id].path.poses.size();
        for (size_t i = 0; i < len_lane - 1; i++) {
            geometry_msgs::PoseStamped p_c = cross_list[id].path.poses[i];
            geometry_msgs::PoseStamped p_n = cross_list[id].path.poses[i + 1];
            double yaw = std::atan2(p_n.pose.position.y - p_c.pose.position.y, p_n.pose.position.x - p_c.pose.position.x);
            Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
            Eigen::Quaterniond q = rollangle * yawangle * pitchangle;
            cross_list[id].path.poses[i].pose.orientation.x = q.x();
            cross_list[id].path.poses[i].pose.orientation.y = q.y();
            cross_list[id].path.poses[i].pose.orientation.z = q.z();
            cross_list[id].path.poses[i].pose.orientation.w = q.w();
        }
        cross_list[id].path.poses[len_lane - 1].pose.orientation.x = cross_list[id].path.poses[len_lane - 2].pose.orientation.x;
        cross_list[id].path.poses[len_lane - 1].pose.orientation.y = cross_list[id].path.poses[len_lane - 2].pose.orientation.y;
        cross_list[id].path.poses[len_lane - 1].pose.orientation.z = cross_list[id].path.poses[len_lane - 2].pose.orientation.z;
        cross_list[id].path.poses[len_lane - 1].pose.orientation.w = cross_list[id].path.poses[len_lane - 2].pose.orientation.w;

        for (int i = 0; i < cross_list[id].path.poses.size(); i++) {
            f << cross_list[id].path.poses[i].pose.position.x << "," << cross_list[id].path.poses[i].pose.position.y << ","
              << cross_list[id].path.poses[i].pose.position.z << "," << cross_list[id].path.poses[i].pose.orientation.x << ","
              << cross_list[id].path.poses[i].pose.orientation.y << "," << cross_list[id].path.poses[i].pose.orientation.z << ","
              << cross_list[id].path.poses[i].pose.orientation.w << std::endl;
        }
    }
    f.close();
    std::cout << "Write back successfully: " << file_name.str() << std::endl;
    return true;
}

bool smartcar::tools::Traj_Extractor::shutdown() { return on_shutdown(); }

bool smartcar::tools::Traj_Extractor::on_shutdown() {
    ros::shutdown();
    return true;
}