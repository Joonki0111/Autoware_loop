#include "route_generator/route.hpp"

namespace route_component
{

RoutePublisher::RoutePublisher(const rclcpp::NodeOptions & node_options)
: Node("route_publisher", node_options)
{
    dist_threshold_ = this->declare_parameter("dist_threshold", 15.0);
    
    client_ = this->create_client<SetRoutePoints>(
        "/api/routing/set_route_points");  

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/kinematic_state", rclcpp::QoS(1),
        std::bind(&RoutePublisher::pose_Callback, this, std::placeholders::_1));

    route_pub_ = this->create_publisher<LaneletRoute>(
        "/planning/mission_planning/route", rclcpp::QoS(1).transient_local());

    checkpoint_vec_.push_back({-95.95, -297.46});
    checkpoint_vec_.push_back({-237.16, -402.70});
    checkpoint_vec_.push_back({-133.35, 108.05});
    checkpoint_vec_.push_back({-13.59, 3.99});
    
    generate_route_executed_ = false;
}

void RoutePublisher::send_request()   //SHY
{
    if (!client_->wait_for_service()) {
        RCLCPP_WARN(this->get_logger(), "Service not available");
        return;
    }

    auto request = std::make_shared<SetRoutePoints::Request>();
    request->header.stamp = this->now();
    request->header.frame_id = "map";
    request->option.allow_goal_modification = false;
    request->goal.position.x = 0.0;
    request->goal.position.y = 0.0;
    request->goal.position.z = 0.0;
    request->goal.orientation.x = 0.0;
    request->goal.orientation.y = 0.0;
    request->goal.orientation.z = 0.0;
    request->goal.orientation.w = 1.0;
    request->waypoints.clear();

    client_->async_send_request(request);
}

void RoutePublisher::generate_route(const int & closest_checkpoint)
{
  LaneletRoute route_msg;

  if(!generate_route_executed_)
  {
    route_msg = create_route1();
    route_pub_->publish(route_msg);
    send_request();
  }

  switch(closest_checkpoint)
  {
    case 1:
    {
      route_msg = create_route2();
      route_pub_->publish(route_msg);
      RCLCPP_INFO(rclcpp::get_logger("route_publisher"),"published 1st route");
      break;
    }
    case 2:
    {
      route_msg = create_route3();
      route_pub_->publish(route_msg);
      RCLCPP_INFO(rclcpp::get_logger("route_publisher"),"published 2nd route");
      break;
    }
    case 3:
    {
      route_msg = create_route4();
      route_pub_->publish(route_msg);
      RCLCPP_INFO(rclcpp::get_logger("route_publisher"),"published 3rd route");
      break;
    }
    case 4:
    {
      route_msg = create_route1();
      route_pub_->publish(route_msg);
      RCLCPP_INFO(rclcpp::get_logger("route_publisher"),"published 4th route");
      break;
    }
    default:
    {
      break;
    }
  }
  generate_route_executed_ = true;
}

void RoutePublisher::pose_Callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
  const geometry_msgs::msg::Point position = pose_msg->pose.pose.position;
  const int closest_checkpoint = calc_closest_checkpoint(position);
  generate_route(closest_checkpoint);
}

int RoutePublisher::calc_closest_checkpoint(const geometry_msgs::msg::Point & position)
{
  for(int i = 0; i < checkpoint_vec_.size(); i++)
  {
    float x_diff = checkpoint_vec_[i].x - position.x;
    float y_diff = checkpoint_vec_[i].y - position.y;

    if(std::abs(x_diff) + std::abs(y_diff) < dist_threshold_)
    {
      return i+1;
    }
  }
  return 0;
}

//[TO-DO]change to single function
LaneletRoute RoutePublisher::create_route1()
{
  LaneletRoute route_msg;
  LaneletSegment segment;
  LaneletPrimitive primitive;

  route_msg.header.stamp = this->get_clock()->now();
  route_msg.header.frame_id = "map";
  
  route_msg.start_pose.position.x = pose_msg_.pose.pose.position.x;
  route_msg.start_pose.position.y = pose_msg_.pose.pose.position.y;
  route_msg.start_pose.position.z = pose_msg_.pose.pose.position.z;

  route_msg.start_pose.orientation.x = pose_msg_.pose.pose.orientation.x;
  route_msg.start_pose.orientation.y = pose_msg_.pose.pose.orientation.y;
  route_msg.start_pose.orientation.z = pose_msg_.pose.pose.orientation.z;
  route_msg.start_pose.orientation.w = pose_msg_.pose.pose.orientation.w;

  route_msg.goal_pose.position.x = -254.95880126953125;
  route_msg.goal_pose.position.y = -370.8807678222656;
  route_msg.goal_pose.position.z = 15.313650152117422;

  route_msg.goal_pose.orientation.x = 0.0;
  route_msg.goal_pose.orientation.y = 0.0;
  route_msg.goal_pose.orientation.z = -0.5042373751665669;
  route_msg.goal_pose.orientation.w = 0.8635650927898434;

  segment.preferred_primitive.id = 524;
  primitive.id = 524;
  primitive.primitive_type = "lane";
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  segment.preferred_primitive.id = 172;
  primitive.id = 172;
  primitive.primitive_type = "lane";
  segment.primitives.clear();
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  return route_msg;
}

LaneletRoute RoutePublisher::create_route2()
{
  LaneletRoute route_msg;
  LaneletSegment segment;
  LaneletPrimitive primitive;

  route_msg.header.stamp = this->get_clock()->now();
  route_msg.header.frame_id = "map";
  
  route_msg.start_pose.position.x = pose_msg_.pose.pose.position.x;
  route_msg.start_pose.position.y = pose_msg_.pose.pose.position.y;
  route_msg.start_pose.position.z = pose_msg_.pose.pose.position.z;

  route_msg.start_pose.orientation.x = pose_msg_.pose.pose.orientation.x;
  route_msg.start_pose.orientation.y = pose_msg_.pose.pose.orientation.y;
  route_msg.start_pose.orientation.z = pose_msg_.pose.pose.orientation.z;
  route_msg.start_pose.orientation.w = pose_msg_.pose.pose.orientation.w;

  route_msg.goal_pose.position.x = -126.45575714111328;
  route_msg.goal_pose.position.y = 45.626609802246094;
  route_msg.goal_pose.position.z = 0.1628499615996558;

  route_msg.goal_pose.orientation.x = 0.0;
  route_msg.goal_pose.orientation.y = 0.0;
  route_msg.goal_pose.orientation.z = 0.5374917981041666;
  route_msg.goal_pose.orientation.w = 0.8432689766443148;

  segment.preferred_primitive.id = 172;
  primitive.id = 172;
  primitive.primitive_type = "lane";
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  segment.preferred_primitive.id = 280;
  primitive.id = 280;
  primitive.primitive_type = "lane";
  segment.primitives.clear();
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  return route_msg;
}

LaneletRoute RoutePublisher::create_route3()
{
  LaneletRoute route_msg;
  LaneletSegment segment;
  LaneletPrimitive primitive;

  route_msg.header.stamp = this->get_clock()->now();
  route_msg.header.frame_id = "map";
  
  route_msg.start_pose.position.x = pose_msg_.pose.pose.position.x;
  route_msg.start_pose.position.y = pose_msg_.pose.pose.position.y;
  route_msg.start_pose.position.z = pose_msg_.pose.pose.position.z;

  route_msg.start_pose.orientation.x = pose_msg_.pose.pose.orientation.x;
  route_msg.start_pose.orientation.y = pose_msg_.pose.pose.orientation.y;
  route_msg.start_pose.orientation.z = pose_msg_.pose.pose.orientation.z;
  route_msg.start_pose.orientation.w = pose_msg_.pose.pose.orientation.w;

  route_msg.goal_pose.position.x = -48.846923828125;
  route_msg.goal_pose.position.y = 58.41415023803711;
  route_msg.goal_pose.position.z = -2.1094233319034466;

  route_msg.goal_pose.orientation.x = 0.0;
  route_msg.goal_pose.orientation.y = 0.0;
  route_msg.goal_pose.orientation.z = -0.528327428173712;
  route_msg.goal_pose.orientation.w = 0.8490407108256652;

  segment.preferred_primitive.id = 280;
  primitive.id = 280;
  primitive.primitive_type = "lane";
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  segment.preferred_primitive.id = 433;
  primitive.id = 433;
  primitive.primitive_type = "lane";
  segment.primitives.clear();
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  return route_msg;
}

LaneletRoute RoutePublisher::create_route4()
{
  LaneletRoute route_msg;
  LaneletSegment segment;
  LaneletPrimitive primitive;

  route_msg.header.stamp = this->get_clock()->now();
  route_msg.header.frame_id = "map";
  
  route_msg.start_pose.position.x = pose_msg_.pose.pose.position.x;
  route_msg.start_pose.position.y = pose_msg_.pose.pose.position.y;
  route_msg.start_pose.position.z = pose_msg_.pose.pose.position.z;

  route_msg.start_pose.orientation.x = pose_msg_.pose.pose.orientation.x;
  route_msg.start_pose.orientation.y = pose_msg_.pose.pose.orientation.y;
  route_msg.start_pose.orientation.z = pose_msg_.pose.pose.orientation.z;
  route_msg.start_pose.orientation.w = pose_msg_.pose.pose.orientation.w;

  route_msg.goal_pose.position.x = -74.1590347290039;
  route_msg.goal_pose.position.y = -253.80279541015625;
  route_msg.goal_pose.position.z = 4.999296961308004;

  route_msg.goal_pose.orientation.x = 0.0;
  route_msg.goal_pose.orientation.y = 0.0;
  route_msg.goal_pose.orientation.z = -0.8228516828727166;
  route_msg.goal_pose.orientation.w = 0.5682561992565838;

  segment.preferred_primitive.id = 433;
  primitive.id = 433;
  primitive.primitive_type = "lane";
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  segment.preferred_primitive.id = 524;
  primitive.id = 524;
  primitive.primitive_type = "lane";
  segment.primitives.clear();
  segment.primitives.push_back(primitive);
  route_msg.segments.push_back(segment);

  return route_msg;
}

} // namespace route_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(route_component::RoutePublisher)