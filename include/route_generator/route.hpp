#ifndef ROUTE_GENERATOR__ROUTE_HPP_
#define ROUTE_GENERATOR__ROUTE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "std_msgs/msg/int32.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "autoware_planning_msgs/msg/lanelet_segment.hpp"
#include "autoware_planning_msgs/msg/lanelet_primitive.hpp"
#include "autoware_adapi_v1_msgs/srv/set_route_points.hpp"   //SHY
#include "nav_msgs/msg/odometry.hpp"

namespace route_component
{

using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
using LaneletSegment = autoware_planning_msgs::msg::LaneletSegment;
using LaneletPrimitive = autoware_planning_msgs::msg::LaneletPrimitive;
using SetRoutePoints = autoware_adapi_v1_msgs::srv::SetRoutePoints;  //SHY

class RoutePublisher : public rclcpp::Node
{
    private:
        struct Checkpoint
        {
            float x;
            float y;
        };
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_; // HJK

        rclcpp::Publisher<LaneletRoute>::SharedPtr route_pub_;

        rclcpp::Client<SetRoutePoints>::SharedPtr client_;    //SHY
        rclcpp::TimerBase::SharedPtr timer_;    //SHY

        nav_msgs::msg::Odometry pose_msg_;
        std::vector<Checkpoint> checkpoint_vec_;
        bool generate_route_executed_;
        float dist_threshold_;

        void generate_route(const int & closest_checkpoint);
        void pose_Callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
        int calc_closest_checkpoint(const geometry_msgs::msg::Point & pose);
        LaneletRoute create_route1();
        LaneletRoute create_route2();
        LaneletRoute create_route3();
        LaneletRoute create_route4();
        void send_request();  //SHY

    public:
        explicit RoutePublisher(const rclcpp::NodeOptions & node_options);
};
}
#endif  //ROUTE_GENERATOR__ROUTE_HPP_
