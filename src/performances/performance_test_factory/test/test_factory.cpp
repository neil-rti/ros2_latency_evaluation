/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, iRobot ROS
 *  All rights reserved.
 *
 *  This file is part of ros2-performance, which is released under BSD-3-Clause.
 *  You may use, distribute and modify this code under the BSD-3-Clause license.
 */

#include <gtest/gtest.h>

#include "performance_test_factory/factory.hpp"

class TestFactory : public ::testing::Test {
   public:
    static void SetUpTestCase() { rclcpp::init(0, nullptr); }

    std::string get_file_path(const std::string& json_file) {
        std::string this_file_path = __FILE__;
        std::string this_dir_path =
            this_file_path.substr(0, this_file_path.rfind("/"));
        std::string json_path =
            this_dir_path + std::string("/files/") + json_file;
        return json_path;
    }

    performance_test::TemplateFactory factory;
    std::string _path_data_processing_pipeline_nodes_topology =
        get_file_path("test_architecture_data_processing_pipeline_qos.json");
    std::string _path_no_data_processing_pipeline_nodes_topology =
        get_file_path("test_architecture.json");
};

TEST_F(TestFactory, FactoryConstructorTest) {
    performance_test::TemplateFactory factory;
}

TEST_F(TestFactory, FactoryCreateFromStringTest) {
    auto node = std::make_shared<performance_test::Node>("node_name");

    this->factory.add_subscriber_from_strings(
        node, "stamped10b", "sub_topic", "pub_topic",
        performance_test::Tracker::TrackingOptions());
    this->factory.add_periodic_publisher_from_strings(node, "stamped10b",
                                                      "my_topic");
    this->factory.add_server_from_strings(node, "stamped10b", "my_service");
    this->factory.add_periodic_client_from_strings(node, "stamped10b",
                                                   "my_service");

    ASSERT_EQ((size_t)3, node->all_trackers()->size());
}

TEST_F(TestFactory, FactoryCreateFromIndicesTest) {
    int n_subscriber_nodes = 2;
    int n_publisher_nodes = 2;
    std::string msg_type = "stamped10b";
    float frequency = 1;

    int subscriber_start_index = 0;
    int subscriber_end_index = n_subscriber_nodes;
    int publisher_start_index = n_subscriber_nodes;
    int publisher_end_index = n_subscriber_nodes + n_publisher_nodes;

    auto sub_nodes = factory.create_subscriber_nodes(
        subscriber_start_index, subscriber_end_index, n_publisher_nodes,
        msg_type, PASS_BY_SHARED_PTR);

    auto pub_nodes = factory.create_periodic_publisher_nodes(
        publisher_start_index, publisher_end_index, frequency, msg_type,
        PASS_BY_UNIQUE_PTR);

    ASSERT_EQ((size_t)2, sub_nodes.size());
    ASSERT_EQ((size_t)2, pub_nodes.size());

    for (const auto& n : sub_nodes) {
        ASSERT_EQ((size_t)2, n->all_trackers()->size());
    }
}

TEST_F(TestFactory, FactoryCreateFromJsonTest) {
    auto nodes_vec =
        factory.parse_topology_from_json(_path_no_data_processing_pipeline_nodes_topology);

    ASSERT_EQ((size_t)3, nodes_vec.size());

    ASSERT_STREQ("node_0", nodes_vec[0]->get_name());
    ASSERT_STREQ("node_1", nodes_vec[1]->get_name());
    ASSERT_STREQ("node_2", nodes_vec[2]->get_name());

    ASSERT_EQ((size_t)2, nodes_vec[1]->all_trackers()->size());
    ASSERT_EQ((size_t)1, nodes_vec[2]->all_trackers()->size());
}

TEST_F(TestFactory, FactoryCreateFromDataProcessingPipelineNodesJsonTest_CorrectNodeSetup)
{
    auto nodes_vec = factory.parse_topology_from_json(
        _path_data_processing_pipeline_nodes_topology);

    ASSERT_EQ((size_t)7, nodes_vec.size());

    ASSERT_STREQ("start_node", nodes_vec[0]->get_name());
    ASSERT_STREQ("dpp_node0", nodes_vec[1]->get_name());
    ASSERT_STREQ("dpp_node1", nodes_vec[2]->get_name());
    ASSERT_STREQ("dpp_node2", nodes_vec[3]->get_name());
    ASSERT_STREQ("dpp_node3", nodes_vec[4]->get_name());
    ASSERT_STREQ("dpp_node4", nodes_vec[5]->get_name());
    ASSERT_STREQ("end_node", nodes_vec[6]->get_name());
}

TEST_F(TestFactory,
       FactoryCreateFromPassthroughNodesJsonTest_CorrectTopicSetup) {
    auto nodes_vec =
        factory.parse_topology_from_json(_path_data_processing_pipeline_nodes_topology);

    ASSERT_EQ((size_t)1, nodes_vec[0]->count_publishers("topic_0"));

  ASSERT_EQ((size_t)1, nodes_vec[1]->count_subscribers("topic_0"));
  ASSERT_EQ((size_t)1, nodes_vec[1]->count_publishers("dpp_topic_0_1"));

  ASSERT_EQ((size_t)1, nodes_vec[2]->count_subscribers("dpp_topic_0_1"));
  ASSERT_EQ((size_t)1, nodes_vec[2]->count_publishers("dpp_topic_1_2"));

  ASSERT_EQ((size_t)1, nodes_vec[3]->count_subscribers("dpp_topic_1_2"));
  ASSERT_EQ((size_t)1, nodes_vec[3]->count_publishers("dpp_topic_2_3"));

  ASSERT_EQ((size_t)1, nodes_vec[4]->count_subscribers("dpp_topic_2_3"));
  ASSERT_EQ((size_t)1, nodes_vec[4]->count_publishers("dpp_topic_3_4"));

  ASSERT_EQ((size_t)1, nodes_vec[5]->count_subscribers("dpp_topic_3_4"));
  ASSERT_EQ((size_t)1, nodes_vec[5]->count_publishers("end_topic"));

    ASSERT_EQ((size_t)1, nodes_vec[6]->count_subscribers("end_topic"));
}

TEST_F(TestFactory, ChangeNumberOfDataProcessingPipelineNodes_CorrectNumber) {
    const unsigned int no_data_processing_pipeline_nodes = 2;

    nlohmann::json node_system_j = factory.read_json_file(_path_data_processing_pipeline_nodes_topology);
    factory.change_data_processing_pipeline(node_system_j, no_data_processing_pipeline_nodes);

    int actual_number_pipeline_nodes =
        node_system_j["nodes"][1]["data_processing_pipeline_nodes"];
    ASSERT_EQ(actual_number_pipeline_nodes,
              no_data_processing_pipeline_nodes);
}

TEST_F(TestFactory, ChangeNumberNumberOfDataProcessingPipelineNodes_NoDataProcessingPipelineNodesInJson) {
    nlohmann::json node_system_j = factory.read_json_file(_path_no_data_processing_pipeline_nodes_topology);
    factory.change_data_processing_pipeline(node_system_j, 1);

    for (auto &node_j : node_system_j["nodes"]) {
        ASSERT_FALSE(node_j.contains("data_processing_pipeline_nodes"));
    }
}

TEST_F(TestFactory, ChangePayloadSize_NoDataProcessingPipelineNodeArchitecture_CorrectPayload) {
  const std::string MSG_TYPE = "stamped10kb";
  nlohmann::json node_system_j =
      factory.read_json_file(_path_no_data_processing_pipeline_nodes_topology);
  factory.change_payload_size(MSG_TYPE, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    for (auto& publisher_j : node_j["publishers"]) {
      ASSERT_EQ(publisher_j["msg_type"], MSG_TYPE);
    }
    for (auto& subscriber_j : node_j["subscribers"]) {
      ASSERT_EQ(subscriber_j["msg_type"], MSG_TYPE);
    }
  }
}

TEST_F(TestFactory, ChangePayloadSize_NoDataProcessingPipelineNodeArchitecture_EmptyNewPayload) {
  const std::string MSG_TYPE = "";
  nlohmann::json node_system_j =
      factory.read_json_file(_path_no_data_processing_pipeline_nodes_topology);
  factory.change_payload_size(MSG_TYPE, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    for (auto& publisher_j : node_j["publishers"]) {
      ASSERT_EQ(publisher_j["msg_type"], publisher_j["msg_type"]);
      ASSERT_FALSE(publisher_j["msg_type"] == MSG_TYPE);
    }
    for (auto& subscriber_j : node_j["subscribers"]) {
      ASSERT_EQ(subscriber_j["msg_type"], subscriber_j["msg_type"]);
      ASSERT_FALSE(subscriber_j["msg_type"] == MSG_TYPE);
    }
  }
}


TEST_F(TestFactory, ChangePayloadSize_DataProcessingPipelineNodeArchitecture_CorrectPayload) {
  const std::string MSG_TYPE = "stamped10b";
  nlohmann::json node_system_j =
      factory.read_json_file(_path_data_processing_pipeline_nodes_topology);
  factory.change_payload_size(MSG_TYPE, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    if(node_j.contains("data_processing_pipeline_nodes")) {
      std::string sub_start_node_msg_type = node_j["subscribers_start_node"]["msg_type"];
      std::string pub_pt_node_msg_type =
          node_j["publisher_data_processing_pipeline_node"]["msg_type"];
      std::string pub_end_node_msg_type = node_j["publisher_end_node"]["msg_type"];

      ASSERT_EQ(sub_start_node_msg_type, MSG_TYPE);
      ASSERT_EQ(pub_pt_node_msg_type, MSG_TYPE);
      ASSERT_EQ(pub_end_node_msg_type, MSG_TYPE);
    }
  }
}

TEST_F(TestFactory, LoadPayLoadSizeFile_ContentCorrectLoaded) {
    const std::vector<std::string> ACTUAL_PAYLOADS_IN_FILE = {"stamped10mb",
                                                              "stamped10kb"};

    std::vector<std::string> payloads_found =
        factory.read_payloads_from_file(get_file_path("test_payloads.txt"));

    for (int i = 0; i < payloads_found.size(); i++) {
        ASSERT_EQ(ACTUAL_PAYLOADS_IN_FILE[i], payloads_found[i]);
    }
}

TEST_F(TestFactory, LoadPayLoadSizeFile_EmptyString) {
    std::vector<std::string> payloads_found =
        factory.read_payloads_from_file("");

    ASSERT_EQ(payloads_found.size(), 1);
    ASSERT_EQ(payloads_found[0], "");
}

TEST_F(TestFactory, ChangePublishPeriodMs_NoDataProcessingPipelineNodeArchitecture_CorrectPublishPeriodMs) {
  const int PUBLISH_PERIOD_MS = 10;
  nlohmann::json node_system_j =
      factory.read_json_file(_path_no_data_processing_pipeline_nodes_topology);
  factory.change_publish_period_ms(PUBLISH_PERIOD_MS, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    for (auto& publisher_j : node_j["publishers"]) {
      ASSERT_EQ(publisher_j["period_ms"], PUBLISH_PERIOD_MS);
    }
  }
}

TEST_F(TestFactory, ChangePublishPeriodMs_NoDataProcessingPipelineNodeArchitecture_ZeroPublishPeriodMs) {
  const int PUBLISH_PERIOD_MS = 0;
  nlohmann::json node_system_j =
      factory.read_json_file(_path_no_data_processing_pipeline_nodes_topology);
  factory.change_publish_period_ms(PUBLISH_PERIOD_MS, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    for (auto& publisher_j : node_j["publishers"]) {
      ASSERT_NE(publisher_j["period_ms"], PUBLISH_PERIOD_MS);
    }
  }
}


TEST_F(TestFactory, ChangePayloadSize_DataProcessingPipelineNodeArchitecture_CorrectPublishPeriodMs) {
  const int PUBLISH_PERIOD_MS = 10;
  nlohmann::json node_system_j =
      factory.read_json_file(_path_data_processing_pipeline_nodes_topology);
  factory.change_publish_period_ms(PUBLISH_PERIOD_MS, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    if(node_j.contains("data_processing_pipeline_nodes")) {
      int pub_pt_node_period_ms =
          node_j["publisher_data_processing_pipeline_node"]["period_ms"];
      int pub_end_node_period_ms = node_j["publisher_end_node"]["period_ms"];

      ASSERT_EQ(pub_pt_node_period_ms, PUBLISH_PERIOD_MS);
      ASSERT_EQ(pub_end_node_period_ms, PUBLISH_PERIOD_MS);
    }
  }
}


TEST_F(TestFactory, ChangePublishPeriodMs_NoDataProcessingPipelineNodeArchitecture_QosReliabilitySetToReliable) {
  const std::string QOS_POLICY = "qos_reliability";
  const std::string QOS_VALUE  = "reliable";
  nlohmann::json node_system_j =
      factory.read_json_file(_path_no_data_processing_pipeline_nodes_topology);
  factory.change_qos_policy_value(QOS_POLICY, QOS_VALUE, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    for (auto& publisher_j : node_j["publishers"])
      ASSERT_EQ(publisher_j[QOS_POLICY], QOS_VALUE);
    for (auto& subscriber_j : node_j["subscribers"])
      ASSERT_EQ(subscriber_j[QOS_POLICY], QOS_VALUE);
  }
}


TEST_F(TestFactory, ChangePayloadSize_DataProcessingPipelineNodeArchitectureQosReliabilitySetToReliable) {
  const std::string QOS_POLICY = "qos_reliability";
  const std::string QOS_VALUE  = "reliable";
  nlohmann::json node_system_j =
      factory.read_json_file(_path_data_processing_pipeline_nodes_topology);
  factory.change_qos_policy_value(QOS_POLICY, QOS_VALUE, node_system_j);

  for (auto& node_j : node_system_j["nodes"]) {
    if(node_j.contains("data_processing_pipeline_nodes")) {
      ASSERT_EQ(node_j["subscribers_start_node"][QOS_POLICY], QOS_VALUE);
      ASSERT_EQ(node_j["publisher_data_processing_pipeline_node"][QOS_POLICY], QOS_VALUE);
      ASSERT_EQ(node_j["publisher_end_node"][QOS_POLICY], QOS_VALUE);
    }
  }
}