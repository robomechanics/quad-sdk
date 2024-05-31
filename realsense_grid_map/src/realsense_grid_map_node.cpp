#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <filesystem>

//namespace fs = std::filesystem;

class RealSenseGridMap 
{
public:
    RealSenseGridMap()
    {
        depth_sub_ = nh_.subscribe("/camera/depth/image_rect_raw", 1, &RealSenseGridMap::depthCallback, this);
        grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    }

    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        grid_map::GridMap map({"elevation"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(1, 1), 0.05); 
        
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
        {
            grid_map::Position position;
            map.getPosition(*it, position);
            int x = (position.x() + 0.5) * cv_ptr->image.cols; 
            int y = (position.y() + 0.5) * cv_ptr->image.rows;

            if (x >= 0 && x < cv_ptr->image.cols && y >= 0 && y < cv_ptr->image.rows)
            {
                float depth = cv_ptr->image.at<uint16_t>(y, x) * 0.001; // mm to m
                map.at("elevation", *it) = depth;
            }
        }
        
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map, message);
        grid_map_pub_.publish(message); 
        
        // gridMapToSTL(map);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Publisher grid_map_pub_;
    
    // void gridMapToSTL(const grid_map::GridMap& map)
       // {
    //     const float resolution = map.getResolution();
    //     const int width = map.getSize()(0);
    //     const int height = map.getSize()(1);
    //     std::vector<Eigen::Vector3f> vertices;
    //     std::vector<Eigen::Vector3i> faces;S

    //     for (int i = 0; i < width - 1; ++i)
    //     {
    //         for (int j = 0; j < height - 1; ++j)
    //         {
    //             Eigen::Vector3f v1(i * resolution, j * resolution, map.at("elevation", grid_map::Index(i, j)));
    //             Eigen::Vector3f v2((i + 1) * resolution, j * resolution, map.at("elevation", grid_map::Index(i + 1, j)));
    //             Eigen::Vector3f v3(i * resolution, (j + 1) * resolution, map.at("elevation", grid_map::Index(i, j + 1)));
    //             Eigen::Vector3f v4((i + 1) * resolution, (j + 1) * resolution, map.at("elevation", grid_map::Index(i + 1, j + 1)));

    //             vertices.push_back(v1);
    //             vertices.push_back(v2);
    //             vertices.push_back(v3);
    //             vertices.push_back(v4);

    //             int base_idx = vertices.size() - 4;
    //             faces.push_back(Eigen::Vector3i(base_idx, base_idx + 1, base_idx + 2));
    //             faces.push_back(Eigen::Vector3i(base_idx + 1, base_idx + 3, base_idx + 2));
    //         }
    //     }

    //     std::string stl_dir = "/path/to/quad-sdk/quad_simulation/gazebo_scripts/worlds/";
    //     if (!fs::exists(stl_dir))
    //     {
    //         fs::create_directories(stl_dir);
    //     }

    //     std::ofstream file(stl_dir + "terrain_model.stl");
    //     file << "solid terrain\n";
    //     for (const auto& face : faces)
    //     {
    //         Eigen::Vector3f normal = (vertices[face[1]] - vertices[face[0]]).cross(vertices[face[2]] - vertices[face[0]]).normalized();
    //         file << "facet normal " << normal.x() << " " << normal.y() << " " << normal.z() << "\n";
    //         file << "  outer loop\n";
    //         for (int i = 0; i < 3; ++i)
    //         {
    //             file << "    vertex " << vertices[face[i]].x() << " " << vertices[face[i]].y() << " " << vertices[face[i]].z() << "\n";
    //         }
    //         file << "  endloop\n";
    //         file << "endfacet\n";
    //     }
    //     file << "endsolid terrain\n";
    //     file.close();
    // }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "realsense_grid_map_node");
    RealSenseGridMap node;
    ros::spin();
    return 0;
}

// roslaunch realsense_grid_map quad_with_realsense.launch


