#include "utility.hpp"

class LaneMapper{
    
private:
    
    Eigen::Vector3d m_t_local_ego;
    Eigen::Quaterniond m_q_local_ego;
    Eigen::Matrix3d m_R_local_ego;

    Eigen::Matrix3d m_R_ego_local;
    Eigen::Vector3d m_t_ego_local;

    Eigen::Vector3d m_position_local;
    Eigen::Matrix3d m_R_local;
    Eigen::Vector3d m_t_local;
    Eigen::Vector3d m_t_diff;
    Eigen::Matrix3d m_R_diff;

    bool loc_init_flag{false};
    bool grid_init_flag{false};    
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
        path_pub = nh.advertise<visualization_msgs::MarkerArray>("random_paths", 10);  
        Publaneline = nh.advertise<openlane_bag::LaneList>("/buildedlane",1);
    }

    void init(){
        loc_init_flag = false;
        grid_init_flag = false;
        m_current_kf_id = 0;
        m_position_local = {0, 0, 0};
        m_R_local = Eigen::Matrix3d::Identity();
    }

    void LanemapperCallback(const openlane_bag::LaneListConstPtr& viperIn){
        std::vector<ViperInput> viper_lanes; 
        DataPreprocess(viperIn, viper_lanes);

        ViperSampling(viper_lanes);
        
        if(!grid_init_flag){
            InitializeOccupyGrid(viper_lanes);
        }
        else{
            UpdateTransformation();

            UpdateGridAfterMotion();

            // TransferViper2Grid(viper_lanes);

            AddNewObservation(viper_lanes);

            std::vector<std::vector<SameGridID>> classified_grids = Cluster_nodes();
            
            std::vector<LocalLane> std_local_lanes;

            // RecoverLanes(classified_grids, std_local_lanes);
        }
        
        

        m_current_kf_id++;
    }

    void Loc_process(const geometry_msgs::PoseStamped loc_info){

        // Eigen::Matrix3d last_R = m_R_local;

        Eigen::Quaterniond q = {loc_info.pose.orientation.x, 
                                loc_info.pose.orientation.y, 
                                loc_info.pose.orientation.z, 
                                loc_info.pose.orientation.w};
        
        m_R_local = q.matrix();
        

        // Eigen::Vector3d last_position = m_position_local;
        m_t_local.x() = loc_info.pose.position.x;
        m_t_local.y() = loc_info.pose.position.y;
        m_t_local.z() = loc_info.pose.position.z;
        

    }

    void DataPreprocess(const openlane_bag::LaneListConstPtr& viperIn, std::vector<ViperInput>& viper_lanes){
        // std::cout <<"viper size : " <<viperIn->num_lanes<<std::endl;
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
                // if(viperIn->lane_list[i].lane[i_point].x > 50){
                //     break;
                // }
                single_point.x() = viperIn->lane_list[i].lane[i_point].x;
                single_point.y() = viperIn->lane_list[i].lane[i_point].y;
                single_point.z() = viperIn->lane_list[i].lane[i_point].z;
                // std::cout<<"lane : "<<i<<" x: "<<single_point.x()<<" y : "<<single_point.y()<<std::endl;
                lane_points.push_back(single_point);
            }

            Eigen::Vector4d coeff;
            CurveFitting(lane_points, coeff);
            cur_viper->m_viper_points = lane_points;
            cur_viper->m_coe = coeff;
            cur_viper->m_attribute = viperIn->lane_list[i].attribute;
            cur_viper->m_category = viperIn->lane_list[i].category;
            viper_lanes.push_back(*cur_viper);
        }
        // delete cur_viper;
        
    }
    
    void InitializeOccupyGrid(std::vector<ViperInput>& viper_lanes){        
        for(uint i_lane = 0; i_lane < viper_lanes.size(); i_lane++){
            auto& cur_lane = viper_lanes[i_lane];
            int category = cur_lane.m_category;
            for(size_t i_node = 0; i_node < cur_lane.m_sample_points.size(); i_node++){
                int grid_id_x, grid_id_y;
                CalGridId(cur_lane.m_sample_points[i_node], grid_id_x, grid_id_y);
                OccupyGrid[grid_id_x][grid_id_y].m_grid_used = true;
                OccupyGrid[grid_id_x][grid_id_y].m_position_ego = cur_lane.m_sample_points[i_node];
                OccupyGrid[grid_id_x][grid_id_y].m_position_local = m_R_local_ego * OccupyGrid[grid_id_x][grid_id_y].m_position_ego + m_t_local_ego;
                OccupyGrid[grid_id_x][grid_id_y].m_category = category;
                OccupyGrid[grid_id_x][grid_id_y].m_last_observed_kf_id = m_current_kf_id;
                OccupyGrid[grid_id_x][grid_id_y].m_obs.m_obs_grid = 5;
                OccupyGrid[grid_id_x][grid_id_y].m_grid_confidence = ConfidenceDamping(cur_lane.m_sample_points[i_node].x());

            }
        }
        // for(uint id_x = 0; id_x < grid_x_size; id_x++){
        //     for(uint id_y = 0;id_y < grid_y_size; id_y++){
        //         OccupyGrid[id_x][id_y].window_color_obs = std::vector<int>(8,0);
        //         OccupyGrid[id_x][id_y].window_dash_obs = std::vector<int>(8,0);
        //     }
        // }

    }
    void UpdateGridAfterMotion(){
        std::vector<std::vector<EgoOccupyGrid>> new_grids = OccupyGrid;

        for(uint id_x = 0; id_x < grid_x_size; id_x++){
            for(uint id_y = 0;id_y < grid_y_size; id_y++){
                EgoOccupyGrid& curOccupyGrid = new_grids[id_x][id_y];
                OccupyGrid[id_x][id_y].m_grid_used = false;
                if(!curOccupyGrid.m_grid_used){
                    continue;
                }

                Eigen::Vector3d pos_ego = m_R_ego_local * curOccupyGrid.m_position_local + m_t_ego_local;
                int now_id_x, now_id_y;
                CalGridId(pos_ego, now_id_x, now_id_y);

                if(now_id_x < 0 || now_id_x >360 || now_id_y < 0 || now_id_y >30){
                    // LOGI("Grid_based laneMapper : out of occupygrid!");
                    OccupyGrid[id_x][id_y].m_grid_used = false;
                    continue;
                }
                else{
                    OccupyGrid[now_id_x][now_id_y].m_grid_used       = true;
                    OccupyGrid[now_id_x][now_id_y].m_position_ego    = pos_ego;
                    OccupyGrid[now_id_x][now_id_y].m_position_local  = curOccupyGrid.m_position_local;
                    OccupyGrid[now_id_x][now_id_y].m_category        = curOccupyGrid.m_category;   
                    OccupyGrid[now_id_x][now_id_y].m_obs.m_obs_grid  = curOccupyGrid.m_obs.m_obs_grid;  
                    OccupyGrid[now_id_x][now_id_y].m_grid_confidence = curOccupyGrid.m_grid_confidence;
                    // if(now_id_x < viper_start_x_max){
                    //     OccupyGrid[now_id_x][now_id_y].m_last_observed_kf_id = m_current_kf_id;
                    //     OccupyGrid[now_id_x][now_id_y].m_obs.m_obs_grid  = std::min(curOccupyGrid.m_obs.m_obs_grid+1, 10);
                    // }
                }
            }
        }      
    }


    double ConfidenceDamping(double ego_x){
        // confidence = - 0.01 * ego_x + 1
        if(ego_x > 100 || ego_x < 0){
            return 0;
        }
        else{
            return -0.01*ego_x + 1;
        }
    }

    void AddNewObservation(std::vector<ViperInput>& viper_lanes){
        for(size_t i_lane = 0; i_lane < viper_lanes.size(); i_lane++){
            auto& cur_lane = viper_lanes[i_lane];
            for(size_t i_node = 0; i_node < cur_lane.m_sample_points.size(); i_node++){
                auto& cur_node = cur_lane.m_sample_points[i_node];
                int grid_id_x, grid_id_y;
                CalGridId(cur_node, grid_id_x, grid_id_y);
                if(grid_id_x < 0 || grid_id_x > 360 || grid_id_y < 0 || grid_id_y > 30){
                    continue;
                }
                if(!OccupyGrid[grid_id_x][grid_id_y].m_grid_used){
                    if((grid_id_y>0 && OccupyGrid[grid_id_x][grid_id_y-1].m_grid_used) || (grid_id_y<30 && OccupyGrid[grid_id_x][grid_id_y+1].m_grid_used)){
                        for(int y = grid_id_y - 1;y <= grid_id_y+1; y++){
                            if(!OccupyGrid[grid_id_x][y].m_grid_used){
                                continue;
                            }
                            OccupyGrid[grid_id_x][y].m_grid_used = false;
                            OccupyGrid[grid_id_x][y].m_obs.m_obs_grid = 3;
                            double tmp_node_conf = ConfidenceDamping(cur_node.x());
                            Eigen::Vector3d position_ego = (cur_node - OccupyGrid[grid_id_x][y].m_position_ego) * 
                                                (OccupyGrid[grid_id_x][y].m_grid_confidence/(OccupyGrid[grid_id_x][y].m_grid_confidence +tmp_node_conf)) +
                                                OccupyGrid[grid_id_x][y].m_position_ego;
                            int new_id_x,new_id_y;
                            CalGridId(cur_node, new_id_x, new_id_y);
                            if(new_id_x < 0 || new_id_x > 360 || new_id_y < 0 || new_id_y >30){
                                continue;
                            }
                            OccupyGrid[new_id_x][new_id_y].m_position_ego = position_ego;
                            OccupyGrid[new_id_x][new_id_y].m_position_local = m_R_local_ego * position_ego + m_t_local_ego;
                            OccupyGrid[new_id_x][new_id_y].m_grid_confidence = 0.5 * ConfidenceDamping(cur_node.x())
                                                                            + 0.5 * OccupyGrid[grid_id_x][y].m_grid_confidence;
                            OccupyGrid[new_id_x][new_id_y].m_grid_used = true;
                            OccupyGrid[new_id_x][new_id_y].m_last_observed_kf_id = m_current_kf_id;
                            OccupyGrid[new_id_x][new_id_y].m_category = cur_lane.m_category;
                            if(OccupyGrid[new_id_x][new_id_y].m_obs.m_obs_grid < 10){
                                OccupyGrid[new_id_x][new_id_y].m_obs.m_obs_grid++;
                            }
                        }
                    }
                    else{
                        OccupyGrid[grid_id_x][grid_id_y].m_grid_used = true;
                        OccupyGrid[grid_id_x][grid_id_y].m_position_ego = cur_node;
                        OccupyGrid[grid_id_x][grid_id_y].m_position_local = m_R_local_ego * OccupyGrid[grid_id_x][grid_id_y].m_position_ego + m_t_local_ego;
                        OccupyGrid[grid_id_x][grid_id_y].m_grid_confidence = ConfidenceDamping(cur_node.x());
                        OccupyGrid[grid_id_x][grid_id_y].m_category = cur_lane.m_category;
                        OccupyGrid[grid_id_x][grid_id_y].m_last_observed_kf_id = m_current_kf_id;


                        if(OccupyGrid[grid_id_x][grid_id_y].m_obs.m_obs_grid < 5){
                            OccupyGrid[grid_id_x][grid_id_y].m_obs.m_obs_grid = 5;
                        }
                        else{
                            OccupyGrid[grid_id_x][grid_id_y].m_obs.m_obs_grid = std::min(OccupyGrid[grid_id_x][grid_id_y].m_obs.m_obs_grid + 1, 10);
                        }

                    }
                }
                else{
                    double tmp_node_conf = ConfidenceDamping(cur_node.x());
                    OccupyGrid[grid_id_x][grid_id_y].m_position_ego = (cur_node - OccupyGrid[grid_id_x][grid_id_y].m_position_ego) * 
                                                                        (tmp_node_conf / (OccupyGrid[grid_id_x][grid_id_y].m_grid_confidence +tmp_node_conf)) +
                                                                        OccupyGrid[grid_id_x][grid_id_y].m_position_ego;
                    OccupyGrid[grid_id_x][grid_id_y].m_position_local = m_R_local_ego * OccupyGrid[grid_id_x][grid_id_y].m_position_ego + m_t_local_ego;
                    OccupyGrid[grid_id_x][grid_id_y].m_category = cur_lane.m_category;
                    OccupyGrid[grid_id_x][grid_id_y].m_grid_confidence = 0.75 * ConfidenceDamping(cur_node.x())
                                                                        + 0.25 * OccupyGrid[grid_id_x][grid_id_y].m_grid_confidence;

                    if(OccupyGrid[grid_id_x][grid_id_y].m_obs.m_obs_grid < 10){
                            OccupyGrid[grid_id_x][grid_id_y].m_obs.m_obs_grid++;
                    }
                }
            }
        }
    }

    std::vector<std::vector<SameGridID>> Cluster_nodes(){
        LaneNodeID this_point;
        std::vector<std::vector<SameGridID>> all_nodes;
        for(int id_x = 0; id_x < grid_x_size; ++id_x){
            for(int id_y = 0; id_y < grid_y_size; ++id_y){
                if(!OccupyGrid[id_x][id_y].m_grid_used || OccupyGrid[id_x][id_y].m_visited){
                    continue;
                }
                std::vector<SameGridID> oneCluster;
                this_point.x = id_x;
                this_point.y = id_y;
                // this_point.dummy_id = OccupyGrid[id_x][id_y].dummy_id;
                OccupyGrid[id_x][id_y].m_visited = true;
                oneCluster = SearchCloseNodes(this_point, 5, 3);
                all_nodes.push_back(oneCluster);
            }
        }
        return all_nodes;
    }

    std::vector<SameGridID> SearchCloseNodes(const LaneNodeID& this_point, const int search_round_x, const int search_round_y){
        std::vector<SameGridID> result_cluster;
        SameGridID this_point_;
        this_point_.x = this_point.x;
        this_point_.y = this_point.y;
        this_point_.cluster_id = 0;
        result_cluster.push_back(this_point_);

        for(size_t i = 0; i < result_cluster.size(); i++){
            int cur_x = result_cluster[i].x;
            int cur_y = result_cluster[i].y;
            // if(cur_x <= 180){
            //     int round = search_round_x;
            // }
            // else{
            //     int round = search_round_x -1;
            // }
            for(int x = cur_x - search_round_x; x <= cur_x + search_round_x; x++){
                for(int y = cur_y - search_round_y; y <= cur_y + search_round_y; y++){
                    if(x < 0 || x > 360 || y < 0 || y > 30){
                        continue;
                    }
                    if(!OccupyGrid[x][y].m_grid_used || OccupyGrid[x][y].m_visited){
                        continue;
                    }
                    OccupyGrid[x][y].m_visited = true;
                    SameGridID grid;
                    grid.x = x;
                    grid.y = y;
                    grid.cluster_id = 0;
                    result_cluster.push_back(grid);
                }
            }
        }
        return result_cluster;
    }

    void publishViperLanes(const ros::Publisher& pub, std::vector<ViperInput>& viper_lanes) {  
        
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        int n = viper_lanes.size();
        for(size_t i_lane = 0;i_lane < viper_lanes.size();i_lane++){
            auto& cur_lane = viper_lanes[i_lane];
            std::cout<<"attribute: "<<cur_lane.m_attribute<<" category: "<<cur_lane.m_category<<std::endl;
            // if(cur_lane.m_attribute < 1 || cur_lane.m_attribute > 4){
            //     continue;
            // }
            if(cur_lane.m_category < 1 || cur_lane.m_category > 8){
                n-=1;
                continue;
            }
            visualization_msgs::Marker marker_msg;
            marker_msg.ns = "LaneMapperLane";
            marker_msg.header.stamp = ros::Time::now();
            marker_msg.header.frame_id = "map";  // TODO: configuration
            marker_msg.lifetime = ros::Duration();

            marker_msg.type = visualization_msgs::Marker::LINE_STRIP;

            marker_msg.action = visualization_msgs::Marker::ADD;
            marker_msg.pose.orientation.x = 0.0;
            marker_msg.pose.orientation.y = 0.0;
            marker_msg.pose.orientation.z = 0.0;
            marker_msg.pose.orientation.w = 1.0;
            marker_msg.scale.x = 0.3;
            marker_msg.scale.y = 0.3;
            marker_msg.scale.z = 0.3;
            marker_msg.color.r = 0;
            marker_msg.color.g = 1;
            marker_msg.color.b = 0;
            marker_msg.color.a = 1; // blue for lane boundary
            marker_msg.id = id;

            id++;
            for(size_t i_node = 0;i_node < cur_lane.m_viper_points.size();i_node++){
                geometry_msgs::Point p;
                p.x = cur_lane.m_viper_points[i_node].x();
                p.y = cur_lane.m_viper_points[i_node].y();
                p.z = 0;
                marker_msg.points.push_back(p);
            }
            marker_array.markers.emplace_back(marker_msg);
        }
        std::cout<<"num of viper: "<<n<<std::endl;
        pub.publish(marker_array);

    }  

    void ViperSampling(std::vector<ViperInput>& viper_lanes){
        for (size_t i_lane = 0; i_lane < viper_lanes.size(); i_lane++){
            auto& cur_viper = viper_lanes[i_lane];
            auto& raw_points = cur_viper.m_viper_points;
            cur_viper.m_sample_points.clear();
            std::vector<Eigen::Vector3d> sample_points;
            

            if (raw_points.size() < 2){
                continue;
            }

            auto it = raw_points.begin();
            while ((it + 1) != raw_points.end()){
                
                Eigen::Vector2d start_point = {it->x(), it->y()};

                Eigen::Vector2d direct = {(it + 1)->x() - it->x(), (it + 1)->y() - it->y()};
                double length = direct.norm();

                if (length == 0) {
                    it++;  
                    continue;
                }

                direct = direct / length;

                for (int i_step = 1; i_step <= static_cast<int>(std::floor(length)); i_step++){
                    Eigen::Vector2d sample_point = start_point + i_step * direct;
                    Eigen::Vector3d sample_point_3d = {sample_point.x(), sample_point.y(), 0};
                    sample_points.push_back(sample_point_3d);
                }
                
                it++;
            }

            cur_viper.m_sample_points = sample_points;

        }
        publishViperLanes(path_pub, viper_lanes);
    }
 
 // void ViperSampling(std::vector<ViperInput>& viper_lanes){
    //     if(viper_lanes.empty()){
    //         return ;
    //     }
    //     for(size_t i_lane = 0; i_lane < viper_lanes.size(); i_lane++){
    //         auto& cur_viper = viper_lanes[i_lane];
    //         auto& raw_points = cur_viper.m_viper_points;
    //         if(raw_points.size() < 4){
    //             continue;
    //         }
    //         for(size_t i_node = 0; i_node < raw_points.size()-3; i_node++){
    //             Eigen::Vector2d p0 = {raw_points[i_node](0), raw_points[i_node](1)};
    //             Eigen::Vector2d p1 = {raw_points[i_node+1](0), raw_points[i_node+1](1)};
    //             Eigen::Vector2d p2 = {raw_points[i_node+2](0), raw_points[i_node+2](1)};
    //             Eigen::Vector2d p3 = {raw_points[i_node+3](0), raw_points[i_node+3](1)};              
    //             Eigen::MatrixXd M(4, 4);
    //             // Catmull-Rom 样条矩阵
    //             M << -0.5,  1.5, -1.5,  0.5,
    //                 1.0, -2.5,  2.0, -0.5,
    //                 -0.5,  0.0,  0.5,  0.0,
    //                 0.0,  1.0,  0.0,  0.0;
    //             Eigen::MatrixXd G(2, 4);
    //             G << p0(0), p1(0), p2(0), p3(0),
    //                 p0(1), p1(1), p2(1), p3(1);
    //             // 生成样条点
    //             int numPoints = std::floor(p2(0) - p1(0));
    //             for (int i = 0; i < numPoints; ++i) {
    //                 double t = static_cast<double>(i) / (numPoints - 1);     
    //                 Eigen::MatrixXd T(1, 4);
    //                 T << t*t*t, t*t, t, 1;
    //                 Eigen::Vector2d point = (T * M * G.transpose()).transpose();
    //                 cur_viper.m_sample_points.push_back(point);
    //             }
    //         }
    //         // std::cout<<"lane: "<<i_lane<<" size of sample points : "<<cur_viper.m_sample_points.size()<<std::endl;
    //         // for (size_t i = 0; i < cur_viper.m_sample_points.size(); i++) {
    //         //     std::cout << "x : " << cur_viper.m_sample_points[i].x() << " y : " << cur_viper.m_sample_points[i].y() << std::endl;
    //         // }
    //     }
    //     publishViperLanes(path_pub, viper_lanes);
    // }
    
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

        // std::cout<<"result : "<<result(0)<<"  "<<result(1)<<"  "<<result(2)<<"  "<<result(3)<<std::endl;
    }
            
    void CalGridId(Eigen::Vector2d node,  int& id_x,  int& id_y){
        double id_x_ = std::round(node.x() / resolution) + 160;
        double id_y_ = std::round(node.y() / resolution) + 15;
        id_x = static_cast<int>(id_x_);
        id_y = static_cast<int>(id_y_);
    }

    void CalGridId(Eigen::Vector3d node,  int& id_x,  int& id_y){
        double id_x_ = std::round(node.x() / resolution) + 160;
        double id_y_ = std::round(node.y() / resolution) + 15;
        id_x = static_cast<int>(id_x_);
        id_y = static_cast<int>(id_y_);
    }

    void UpdateTransformation(){
        m_t_local_ego = m_t_local;
        m_R_local_ego = m_R_local;

        m_R_ego_local = m_R_local_ego.transpose();
        m_t_ego_local = - m_R_ego_local * m_t_local_ego;
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