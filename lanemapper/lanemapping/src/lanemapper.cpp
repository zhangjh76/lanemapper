#include "utility.hpp"

class LaneMapper{
    
private:

    struct ViperInput
    {
        Eigen::Vector4d m_coe;
        int m_attribute{0};
        int m_category{-1};
        std::vector<Eigen::Vector3d> m_viper_points;
        std::vector<Eigen::Vector2d> m_sample_points;
    };

    Eigen::Vector3d m_position_local;
    Eigen::Matrix3d m_R_local;
    Eigen::Vector3d m_t_diff;
    Eigen::Matrix3d m_R_diff;

    ros::NodeHandle nh;

public:
    ros::Subscriber Sublocinfo;
    ros::Subscriber Subviperline;
    ros::Publisher path_pub;
    ros::Publisher Publaneline;

    LaneMapper()
    {
        init();
        Sublocinfo = nh.subscribe<geometry_msgs::PoseStamped>("/gt_pose_wc",1,&LaneMapper::Loc_process, this);
        Subviperline = nh.subscribe<openlane_bag::LaneList>("/lanes_predict",1, &LaneMapper::LanemapperCallback, this);
        path_pub = nh.advertise<nav_msgs::Path>("random_paths", 10);  
        Publaneline = nh.advertise<openlane_bag::LaneList>("/buildedlane",1);
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
        // ROS_INFO("t_diff : %f, %f, %f",m_t_diff.x(), m_t_diff.y(), m_t_diff.z());
        m_R_diff = m_R_local * last_R.transpose();
    }
    void LanemapperCallback(const openlane_bag::LaneListConstPtr& viperIn){
        std::vector<ViperInput> viper_lanes; 
        DataPreprocess(viperIn, viper_lanes);


        ViperSampling(viper_lanes);

        
    }

    void ViperSampling(std::vector<ViperInput>& viper_lanes){
        if(viper_lanes.empty()){
            return ;
        }
        for(size_t i_lane = 0; i_lane < viper_lanes.size(); i_lane++){
            auto& cur_viper = viper_lanes[i_lane];
            auto& raw_points = cur_viper.m_viper_points;
            if(raw_points.size() < 4){
                continue;
            }
            for(size_t i_node = 0; i_node < raw_points.size()-3; i_node++){
                Eigen::Vector2d p0 = {raw_points[i_node](0), raw_points[i_node](1)};
                Eigen::Vector2d p1 = {raw_points[i_node+1](0), raw_points[i_node+1](1)};
                Eigen::Vector2d p2 = {raw_points[i_node+2](0), raw_points[i_node+2](1)};
                Eigen::Vector2d p3 = {raw_points[i_node+3](0), raw_points[i_node+3](1)};
                
                Eigen::MatrixXd M(4, 4);

                // Catmull-Rom 样条矩阵
                M << -0.5,  1.5, -1.5,  0.5,
                    1.0, -2.5,  2.0, -0.5,
                    -0.5,  0.0,  0.5,  0.0,
                    0.0,  1.0,  0.0,  0.0;
                Eigen::MatrixXd G(2, 4);
                G << p0(0), p1(0), p2(0), p3(0),
                    p0(1), p1(1), p2(1), p3(1);

                // 生成样条点
                int numPoints = std::floor(p2(0) - p1(0));
                for (int i = 0; i < numPoints; ++i) {
                    double t = static_cast<double>(i) / (numPoints - 1);
            
                    Eigen::MatrixXd T(1, 4);
                    T << t*t*t, t*t, t, 1;

                    Eigen::Vector2d point = (T * M * G.transpose()).transpose();
                    cur_viper.m_sample_points.push_back(point);
                }
            }
            std::cout<<"lane: "<<i_lane<<" size of sample points : "<<cur_viper.m_sample_points.size()<<std::endl;
            for (size_t i = 0; i < cur_viper.m_sample_points.size(); i++) {
                std::cout << "x : " << cur_viper.m_sample_points[i].x() << " y : " << cur_viper.m_sample_points[i].y() << std::endl;
            }
        }
        publishViperLanes(path_pub, viper_lanes);
    }

    void publishViperLanes(const ros::Publisher& pub, std::vector<ViperInput>& viper_lanes) {  
        ros::Rate rate(1); // 1 Hz  
        // while (ros::ok()) {  
        std::vector<nav_msgs::Path> pathMsgs;

        
        for(size_t i_viper = 0; i_viper < viper_lanes.size();i_viper++){
            auto& cur_viper = viper_lanes[i_viper];
            nav_msgs::Path pathMsg;
            pathMsg.header.frame_id = "map";  
            pathMsg.header.stamp = ros::Time::now();  
            for(size_t i_point = 0;i_point < cur_viper.m_sample_points.size();i_point++){
                auto& cur_point = cur_viper.m_sample_points[i_point];
                geometry_msgs::PoseStamped pose;  
                pose.header.frame_id = "map";  
                pose.header.stamp = ros::Time::now();  
    
                // Randomly generate start and end points  
                pose.pose.position.x = cur_point.x();
                pose.pose.position.y = cur_point.y();
                pose.pose.orientation.w = 1.0; // Quaternion for no rotation  
    
                pathMsg.poses.push_back(pose); // Start point  
            }
            pathMsgs.push_back(pathMsg);
            // pub.publish(pathMsg);  
            // rate.sleep();  
        }
        for(size_t i = 0; i < pathMsgs.size();i++){
            pub.publish(pathMsgs[i]);
        }

            // nav_msgs::Path pathMsg;  
            // pathMsg.header.frame_id = "map";  
            // pathMsg.header.stamp = ros::Time::now();  
    
            // srand(time(NULL)); // Seed the random number generator  
            // int numPaths= viper_lanes.size();
            // for (int i = 0; i < numPaths; ++i) {  
            //     geometry_msgs::PoseStamped pose;  
            //     pose.header.frame_id = "map";  
            //     pose.header.stamp = ros::Time::now();  
    
            //     // Randomly generate start and end points  
            //     pose.pose.position.x = viper_lanes[i].m_sample_points.x();
            //     pose.pose.position.y = viper_lanes[i].m_sample_points.y();
            //     pose.pose.orientation.w = 1.0; // Quaternion for no rotation  
    
            //     pathMsg.poses.push_back(pose); // Start point  
    
            //     // Generate end point  
            //     pose.pose.position.x = (rand() % 100) - 50;  
            //     pose.pose.position.y = (rand() % 100) - 50;  
            //     pathMsg.poses.push_back(pose); // End point  
    
            //     // Optionally, you could add more points to make the path smoother  
            // }  
    
            
        // }  
    }  
    // void ViperSampling(std::vector<ViperInput>& viper_lanes){
    //     for (size_t i_lane = 0; i_lane < viper_lanes.size(); i_lane++){
    //         auto& cur_viper = viper_lanes[i_lane];
    //         auto& raw_points = cur_viper.m_viper_points;
    //         cur_viper.m_sample_points.clear();
    //         std::vector<Eigen::Vector2d> sample_points;
            

    //         if (raw_points.size() < 4){
    //             continue;
    //         }

    //         auto it = raw_points.begin();
    //         while ((it + 1) != raw_points.end()){
                
    //             Eigen::Vector2d start_point = {it->x(), it->y()};

    //             Eigen::Vector2d direct = {(it + 1)->x() - it->x(), (it + 1)->y() - it->y()};
    //             double length = direct.norm();

    //             if (length == 0) {
    //                 it++;  
    //                 continue;
    //             }

    //             direct = direct / length;

    //             for (int i_step = 1; i_step <= static_cast<int>(std::floor(length)); i_step++){
    //                 Eigen::Vector2d sample_point = start_point + i_step * direct;
    //                 sample_points.push_back(sample_point);
    //             }
                
    //             it++;
    //         }


    //         cur_viper.m_sample_points = sample_points;
    //         std::cout << "lane : " << i_lane << std::endl;
    //         for (size_t i = 0; i < cur_viper.m_viper_points.size(); i++) {
    //             std::cout << "viper : x : " << cur_viper.m_viper_points[i].x() << " y : " << cur_viper.m_viper_points[i].y() << std::endl;
    //         }
    //         for (size_t i = 0; i < cur_viper.m_sample_points.size(); i++) {
    //             std::cout << "sample : x : " << cur_viper.m_sample_points[i].x() << " y : " << cur_viper.m_sample_points[i].y() << std::endl;
    //         }
    //     }
    // }

    void DataPreprocess(const openlane_bag::LaneListConstPtr& viperIn, std::vector<ViperInput>& viper_lanes){
        // std::cout <<"viper size : " <<viperIn.num_lanes<<std::endl;
        // std::vector<ViperInput> viper_lanes; 
        for(size_t i = 0; i < viperIn->lane_list.size(); i++){
            ViperInput* cur_viper = new ViperInput();
            // std::cout<<"lane : "<<i+1<<std::endl;
            // std::cout<< viperIn->lane_list[i].num_points<<std::endl;
            // std::cout<< viperIn->lane_list[i].attribute<<std::endl;
            // std::cout<< viperIn->lane_list[i].track_id<<std::endl;
            // std::cout<< viperIn->lane_list[i].category<<std::endl;
            Eigen::Vector3d single_point;
            std::vector<Eigen::Vector3d> lane_points;
            for(size_t i_point = 0; i_point < viperIn->lane_list[i].lane.size(); i_point++){
                
                single_point.x() = viperIn->lane_list[i].lane[i_point].x;
                single_point.y() = viperIn->lane_list[i].lane[i_point].y;
                single_point.z() = viperIn->lane_list[i].lane[i_point].z;
                // std::cout<<"lane : "<<i<<" x: "<<single_point.x()<<" y : "<<single_point.y()<<std::endl;
                lane_points.push_back(single_point);
            }

            Eigen::Vector4d coeff;
            // CurveFitting(lane_points, coeff);
            cur_viper->m_viper_points = lane_points;
            cur_viper->m_coe = coeff;
            cur_viper->m_attribute = viperIn->lane_list[i].attribute;
            cur_viper->m_category = viperIn->lane_list[i].category;
            viper_lanes.push_back(*cur_viper);
        }
        // delete cur_viper;
        
    }

    void CurveFitting(const std::vector<Eigen::Vector3d> points, Eigen::Vector4d& coe){
        if(points.size() <  4){
            std::cout<<"too less points"<<std::endl;
            return ;
        }
        std::vector<double>x(points.size());
        std::vector<double>y(points.size());
        for(size_t i_point = 0; i_point < points.size(); i_point++){
            x[i_point] = points[i_point](0);
            y[i_point] = points[i_point](1);
        }
        int n = 3;
        int N = x.size();

        Eigen::MatrixXd A(N, n+1);  

        Eigen::VectorXd b(N);
        for (int i = 0; i < N; ++i) {  
            A(i, 0) = 1;  
            for (int j = 1; j <= n; ++j) {  
                A(i, j) = std::pow(x[i], j);  
            }  
            b(i) = y[i];  
        }  
        
        Eigen::Vector4d result = A.householderQr().solve(b);
        
        coe = result;
        // Eigen::Vector4d result = (A.transpose() * A).transpose() * (A.transpose() * b);

        // std::cout<<"result : "<<result(0)<<"  "<<result(1)<<"  "<<result(2)<<std::endl;
    }
    

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lanemapper");

    // ros::NodeHandle nh;

    LaneMapper LM;

    
    ros::spin();

    return 0;
}