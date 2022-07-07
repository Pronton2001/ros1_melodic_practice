#include<ros/ros.h>
#include<costmap_2d/costmap_2d.h>
#include<costmap_2d/costmap_2d_ros.h>
#include<nav_core/base_global_planner.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/tf.h>
#include<angles/angles.h>
#include<base_local_planner/world_model.h>
#include<base_local_planner/costmap_model.h>

using std::string;

#ifndef MYCARROT_CPP
#define MYCARROT_CPP
namespace myCarrot
{
    class GlobalPlanner : public nav_core::BaseGlobalPlanner{
        public:
        GlobalPlanner();
        GlobalPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped> &plan);

        private:
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;
        base_local_planner::WorldModel* world_model_;
        // double GlobalPlanner::footprintCost(double x_i, double y_i, double theta_i);
    };
} // namespace global_planner
#endif