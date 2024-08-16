#include <iostream>
#include <Eigen/Core>  

#include <Eigen/Dense>
#include <ros/ros.h>


#include "openlane_bag/Lane.h"
#include <geometry_msgs/PoseStamped.h>
#include "openlane_bag/LaneList.h"
#include "openlane_bag/LanePoint.h"

#include <ros/ros.h>  

#include <nav_msgs/Path.h>  

#include <geometry_msgs/PoseStamped.h>  
#include <visualization_msgs/Marker.h>  
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>    

#include <time.h>  
#include <vector>

struct ViperInput
{
    Eigen::Vector4d m_coe;
    int m_attribute{0};
    int m_category{-1};
    std::vector<Eigen::Vector3d> m_viper_points;
    std::vector<Eigen::Vector3d> m_sample_points;
};

struct EgoOccupyGrid
{
    int m_attribute{0};
    int m_category{-1};
    Eigen::Vector3d m_position_ego;
    Eigen::Vector3d m_position_local;
    // Eigen::Vector2d m_variance;
    bool m_visited{false};
    bool m_grid_used{false};
    int m_dummy_id{-1};
    struct obs_conut
    {
        int m_obs_grid{0};
        int m_obs_split{0};
        int m_obs_merge{0};
        bool m_used{false};
    }m_obs;

    int m_split_merge_attr{0};            // none = 0; split = 1; merge = 2;
    double m_grid_confidence;
    int m_lane_id{-1};
    bool m_lane_split{false};
    bool m_lane_merge{false};
    long m_last_observed_kf_id = -1;

    std::vector<int> window_color_obs{0,0,0,0,0,0,0,0};
    std::vector<int> window_dash_obs{0,0,0,0,0,0,0,0};
};

struct LocalLaneNode
{

    Eigen::Vector3d m_position;
    Eigen::Vector3d m_variance; // describe stability
    // std::vector<LaneNodeOb> m_obs;

    double m_confidences{0};
    struct attribute_obs{
        int m_color_obs_times;
        int m_dash_obs_times;
    }m_attribute_obs;
    int m_lane_id;
    int m_split_merge_attr{0};
};

struct LocalLane
{
    
    std::vector<LocalLaneNode> m_local_lane_nodes;
    std::int8_t m_id = -1; // unique id, not index
    long m_last_observed_kf_id = -1;
    int m_match_times = 0;
    int m_gategory{-1};
    
    int m_split_merge_attr{0};
    std::vector<int>index_splitmerge;
    int m_group_id{0};
    int m_connection_infos{0};
};

struct LaneNodeID
{
    int x;
    int y;
    int dummy_id;
};

struct SameGridID
{
    int x;
    int y;
    int cluster_id;
};

constexpr int grid_y_size=31;
constexpr int grid_x_size=361;
constexpr double resolution = 0.5;
long m_current_kf_id;
static std::vector<std::vector<EgoOccupyGrid>> OccupyGrid(grid_x_size, std::vector<EgoOccupyGrid> (grid_y_size));