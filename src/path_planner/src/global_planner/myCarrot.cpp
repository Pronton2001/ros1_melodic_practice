#include <pluginlib/class_list_macros.hpp>
#include "myCarrot.h"

PLUGINLIB_EXPORT_CLASS(myCarrot::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace myCarrot
{
    GlobalPlanner::GlobalPlanner(){}
    GlobalPlanner::GlobalPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros): costmap_ros_(NULL){
        initialize(name, costmap_ros);
    }
    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){ 
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);
        double step_size_, min_dist_from_robot_;
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
        world_model_ = new base_local_planner::CostmapModel(*costmap_);

    }

    // Positive if all the points lie outside the footprint, negative otherwise:
    //    *          -1 if footprint covers at least a lethal obstacle cell, or
    //    *          -2 if footprint covers at least a no-information cell, or
    //    *          -3 if footprint is partially or totally outside of the map
    // double GlobalPlanner::footprintCost(double x_i, double y_i, double theta_i){

    //     std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    //     if (footprint.size() < 3) // no footprint -> do nothing
    //         return -1;
        
    //     double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    //     return footprint_cost;
    // }
    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
        plan.clear();


        tf::Stamped<tf::Pose> goal_tf;
        tf::Stamped<tf::Pose> start_tf;

        // convert from StampedMsg to TF. StampedMsg and TF is nearly identical.
        poseStampedMsgToTF(goal,goal_tf);
        poseStampedMsgToTF(start,start_tf);

        double _pitch, _roll, goal_yaw, start_yaw;
        start_tf.getBasis().getEulerYPR(start_yaw, _pitch, _roll);
        goal_tf.getBasis().getEulerYPR(goal_yaw, _pitch, _roll);

        double goal_x= goal.pose.position.x;
        double goal_y= goal.pose.position.y;
        double start_x= goal.pose.position.x;
        double start_y= goal.pose.position.y;
        
        double diff_x= goal_x - start_x;
        double diff_y = goal_y - start_y;

        double diff_yaw= angles::normalize_angle(goal_yaw - start_yaw);
        double target_x = goal_x;
        double target_y = goal_y;
        double target_yaw = goal_yaw;

        bool done;
        double scale = 1;
        double d_scale = .01;
        while(!done){
            target_x = start_x + diff_x * scale;
            target_y = start_y + diff_y * scale;
            target_yaw = start_yaw + diff_yaw * scale;

            // double footprint_cost = footprintCost(target_x, target_y, target_yaw);
            // if (footprint_cost >= 0){
            //     done = true;
            // }
            scale -= d_scale;
        }
        plan.push_back(start);

        // Create new goal with target_x, target_y, target_yaw
        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

        new_goal.pose.position.x = target_x;
        new_goal.pose.position.y = target_y;

        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();
        plan.push_back(new_goal);


        // for (int i=0; i<20; i++){
        //     geometry_msgs::PoseStamped new_goal = goal;
        //     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);

        //     new_goal.pose.position.x = -2.5 + (.05 * i);
        //     new_goal.pose.position.y = -3.5 + (.05 * i);

        //     new_goal.pose.orientation.x = goal_quat.x();
        //     new_goal.pose.orientation.y = goal_quat.y();
        //     new_goal.pose.orientation.z = goal_quat.z();
        //     new_goal.pose.orientation.w = goal_quat.w();

        //     plan.push_back(new_goal);
        // }
        // plan.push_back(goal);
        // return true;
        return (done);
    }
} // namespace myCarrot



