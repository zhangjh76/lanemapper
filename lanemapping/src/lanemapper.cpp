#include "utility.hpp"

class LaneMapper{
    
public:
    ros::Subscriber Sublocinfo;
    ros::Subscriber Subviperline;

    ros::Publisher Publaneline;

    LaneMapper()
    {
        init();
        Sublocinfo = nh.subscribe<geometry_msgs::PoseStamped>("/gt_pose_wc",1,&LaneMapper::Loc_process, this);
        Subviperline = nh.subscribe<lane_msgs::LaneList>("/lanes_predict",1, &LaneMapper::LanemapperCallback, this);

        Publaneline = nh.advertise<lane_msgs::LaneList>("/buildedlane",1);
    }

    void init(){
        m_position_local = {0, 0, 0};
        m_R_local = Eigen::Matrix3d::Identity();
    }

    void Loc_process(const geometry_msgs::PoseStamped loc_info){

        Eigen::Matrix3d last_R = m_R_local;

        Eigen::Quaterniond q = {loc_info.pose.orientation.x, 
                                loc_info.pose.orientation.y, 
                                loc_info.pose.orientation.z, 
                                loc_info.pose.orientation.w};
        
        m_R_local = q.matrix();
        

        Eigen::Vector3d last_position = m_position_local;
        m_position_local.x() = loc_info.pose.position.x;
        m_position_local.y() = loc_info.pose.position.y;
        m_position_local.z() = loc_info.pose.position.z;
        
        // ROS_INFO("x : %f , y : %f , z : %f ",m_position_local.x(), m_position_local.y(), m_position_local.z());
        m_t_diff = m_position_local - last_position;
        ROS_INFO("t_diff : %f, %f, %f",m_t_diff.x(), m_t_diff.y(), m_t_diff.z());
        m_R_diff = m_R_local * last_R.transpose();
    }
    void LanemapperCallback(const lane_msgs::LaneListConstPtr& viperIn){
        DataPreprocess(viperIn);
    }

    void DataPreprocess(){

    }

private:

    Eigen::Vector3d m_position_local;
    Eigen::Matrix3d m_R_local;
    Eigen::Vector3d m_t_diff;
    Eigen::Matrix3d m_R_diff;

    ros::NodeHandle nh;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lanemapper");

    // ros::NodeHandle nh;

    LaneMapper LM;

    
    ros::spin();

    return 0;
}