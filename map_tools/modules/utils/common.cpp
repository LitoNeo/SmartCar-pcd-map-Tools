#include "common.h"

// String function
bool smartcar::utils::endswith(const std::string &str, const std::string &pattern)
{
    return str.substr(str.length() - pattern.length()) == pattern;
}

// File function
std::vector<std::string> smartcar::utils::get_file_list(const std::string folder, bool with_full_path = false)
{
    std::string pre_fix = (folder[folder.length() - 1] == '/') ? folder : (folder + "/");
    std::vector<std::string> files;
    DIR *dir;
    struct dirent *ptr;
    char base[1000];
    if ((dir = opendir(folder.c_str())) == NULL)
    {
        perror("Open dir error...");
        std::cout << "Check: " << folder << std::endl;
        exit(1);
    }

    if (with_full_path)
    {
        while ((ptr = readdir(dir)) != NULL)
        {
            if (ptr->d_type == 8)
                files.push_back(pre_fix + ptr->d_name);
        }
    }
    else
    {
        while ((ptr = readdir(dir)) != NULL)
        {
            if (ptr->d_type == 8)
                files.push_back(ptr->d_name);
        }
    }

    closedir(dir);
    std::sort(files.begin(), files.end());
    return files;
}

// pcl function
pcl::PointCloud<pcl::PointXYZI>::Ptr smartcar::utils::load_pcd_folder(const std::string folder)
{
    std::vector<std::string> tmp_files = get_file_list(folder, true);
    std::vector<std::string> pcd_file_list;
    for (std::string s : tmp_files)
    {
        if (endswith(s, ".pcd") || endswith(s, ".PCD"))
            pcd_file_list.push_back(s);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    for (std::string file : pcd_file_list)
    {
        pcl::io::loadPCDFile(file, *cloud_tmp);
        *cloud += *cloud_tmp;
    }
    return cloud;
}
