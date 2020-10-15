/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include "performance_test/ros2/node.hpp"
#include "performance_test_msgs/msg/sample.hpp"
#include "performance_test_msgs/srv/sample.hpp"

class TestNode : public ::testing::Test {
   public:
    static void SetUpTestCase() { rclcpp::init(0, nullptr); }
};

TEST_F(TestNode, NodeConstructorTest) {
    std::string ros2_namespace = "node_namespace";

    rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
    node_options.use_intra_process_comms(true);
    node_options.start_parameter_services(true);
    node_options.start_parameter_event_publisher(true);

    auto node = std::make_shared<performance_test::Node>(
        "node_name", ros2_namespace, node_options);

    auto trackers_vector_ptr = node->all_trackers();

    ASSERT_EQ((size_t)0, trackers_vector_ptr->size());
}

TEST_F(TestNode, NodeAddItemsTest) {
    auto sub_topic =
        performance_test::Topic<performance_test_msgs::msg::Sample>(
            "sub_topic");
    auto pub_topic =
        performance_test::Topic<performance_test_msgs::msg::Sample>(
            "pub_topic");
    auto service = performance_test::Topic<performance_test_msgs::srv::Sample>(
        "my_service");

    auto node = std::make_shared<performance_test::Node>("node_name");

    node->add_subscriber(sub_topic, pub_topic, PASS_BY_SHARED_PTR);
    node->add_periodic_publisher(pub_topic, std::chrono::milliseconds(10),
                                 PASS_BY_UNIQUE_PTR);
    node->add_server(service);
    node->add_periodic_client(service, std::chrono::milliseconds(10));

    ASSERT_EQ((size_t)3, node->all_trackers()->size());
}

TEST_F(TestNode, TestNodeNowOverride) {
    auto node = std::make_shared<performance_test::Node>("node_name");

    rclcpp::Time node_current_time = node->now();
    uint64_t current_time = get_timestamp();

    // assume maximum processing delay of 100ns
    ASSERT_NEAR(current_time, node_current_time.nanoseconds(), 1000);
}