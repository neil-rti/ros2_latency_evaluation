#include <iostream>

#include <gtest/gtest.h>
#include "nlohmann/json.hpp"

#include "performance_test_factory/parse_data_processing_pipeline_section_from_json_node.hpp"

using json = nlohmann::json;


class TestParseDataProcessingPipelineSectionFromJsonNode : public ::testing::Test
{
    public:
        // methods
        TestParseDataProcessingPipelineSectionFromJsonNode() : _p(create_data_processing_pipeline_nodes_section(5, "stamped10b")) {}
        json create_data_processing_pipeline_nodes_section(unsigned int no_nodes, const std::string& msg_type) {
            json section = {
                {"node_name", "data_processing_pipeline"},
                {"data_processing_pipeline_nodes", no_nodes},
                {"subscribers_start_node", {{"topic_name", "start_topic"}, {"msg_type", msg_type}}},
                {"publisher_data_processing_pipeline_node", {{"topic_name", "dpp_topic"}, {"msg_type", msg_type}, {"period_ms", 10}}},
                {"publisher_end_node", {{"topic_name", "end_topic"}, {"msg_type", msg_type}, {"period_ms", 10}}}};
            return section;
        }

        json create_data_processing_pipeline_section_with_qos(unsigned int no_nodes, const std::string& msg_type) {
            json section = {
                {"node_name", "data_processing_pipeline"},
                {"data_processing_pipeline_nodes", no_nodes},
                {"subscribers_start_node", {{"topic_name", "start_topic"}, {"msg_type", msg_type}, {"qos_reliability", "reliable"}}},
                {"publisher_data_processing_pipeline_node", {{"topic_name", "dpp_topic"}, {"msg_type", msg_type}, {"period_ms", 10}, {"qos_reliability", "reliable"}}},
                {"publisher_end_node", {{"topic_name", "end_topic"}, {"msg_type", msg_type}, {"period_ms", 10}, {"qos_reliability", "reliable"}}}};
            return section;
        }
        void assert_correct_node_names(const json &, const int);
        void assert_correct_msg_types(const json &, const std::string &,
                                      const int);
        void assert_correct_topic_names_first_last_node(const json &,
                                                        const std::string,
                                                        const std::string,
                                                        const int);
        void assert_correct_topic_names_in_between(const json &, const int);
        void assert_correct_qos_reliability(const json &, const std::string,
                                            const int);
        // variables
        ParseDataProcessingPipelineSectionFromJsonNode _p;

};

TEST_F(TestParseDataProcessingPipelineSectionFromJsonNode, EndToEnd_FiveDataProcessingPipelineNodes) {
    json parsed_section_j = _p.run();
    assert_correct_msg_types(parsed_section_j, "stamped10b", 5);
    assert_correct_node_names(parsed_section_j, 5);
    assert_correct_topic_names_first_last_node(parsed_section_j, "start_topic", "end_topic", 5);
    assert_correct_topic_names_in_between(parsed_section_j, 5);
}

TEST_F(TestParseDataProcessingPipelineSectionFromJsonNode, EndToEnd_FiveDataProcessingPipelineNodes_WithQoS) {
  json section = create_data_processing_pipeline_section_with_qos(5, "stamped10b");
  ParseDataProcessingPipelineSectionFromJsonNode p(section);

  json parsed_section_j = p.run();
  assert_correct_msg_types(parsed_section_j, "stamped10b", 5);

  assert_correct_node_names(parsed_section_j, 5);

  assert_correct_topic_names_first_last_node(
      parsed_section_j, "start_topic", "end_topic", 5);

  assert_correct_topic_names_in_between(parsed_section_j, 5);

  assert_correct_qos_reliability(parsed_section_j, "reliable", 5);
}


TEST_F(TestParseDataProcessingPipelineSectionFromJsonNode, TopicNameCreation_FirstNode) {
    std::string publisher_topic_name;
    std::string subscriber_topic_name;

    _p.create_topic_names(publisher_topic_name, subscriber_topic_name, 0);

    ASSERT_EQ(subscriber_topic_name, "start_topic");
    ASSERT_EQ(publisher_topic_name, "dpp_topic_0_1");
}

TEST_F(TestParseDataProcessingPipelineSectionFromJsonNode, TopicNameCreation_LastNode) {
    std::string publisher_topic_name;
    std::string subscriber_topic_name;

    _p.create_topic_names(publisher_topic_name, subscriber_topic_name, 4);

    ASSERT_EQ(publisher_topic_name, "end_topic");
}

TEST_F(TestParseDataProcessingPipelineSectionFromJsonNode, TopicNameCreation_InBetween) {
    std::string publisher_topic_name;
    std::string subscriber_topic_name;

    _p.create_topic_names(publisher_topic_name, subscriber_topic_name, 2);

    ASSERT_EQ(publisher_topic_name, "dpp_topic_2_3");
}

TEST_F(TestParseDataProcessingPipelineSectionFromJsonNode, TopicNameCreation_OneDataProcessingPipelineNode){
    json section = create_data_processing_pipeline_nodes_section(1, "stamped10b");
    ParseDataProcessingPipelineSectionFromJsonNode p(section);
    json parsed_section_j = p.run();

    std::string publisher_topic_name;
    std::string subscriber_topic_name;

    p.create_topic_names(publisher_topic_name, subscriber_topic_name, 0);

    ASSERT_EQ(publisher_topic_name, "end_topic");
    ASSERT_EQ(subscriber_topic_name, "start_topic");
}



TEST_F(TestParseDataProcessingPipelineSectionFromJsonNode, NodeSectionCreation) {
    std::string node_name = "MyNodeName";
    std::string sub_topic_name = "subscriber_topic";
    std::string pub_topic_name = "publisher_topic";
    const int idx_data_processing_pipeline_node = 1;
    json node_section_j =
        _p.create_node_section(node_name, sub_topic_name, pub_topic_name, idx_data_processing_pipeline_node);

    ASSERT_EQ(node_section_j["node_name"], node_name);
    ASSERT_EQ(node_section_j["publishers"][0]["topic_name"], pub_topic_name);
    ASSERT_EQ(node_section_j["subscribers"][0]["topic_name"], sub_topic_name);

    ASSERT_EQ(node_section_j["publishers"][0]["msg_type"], node_section_j["subscribers"][0]["msg_type"]);
    ASSERT_EQ(
        node_section_j["executor_id"],
        idx_data_processing_pipeline_node + 2);
}

void TestParseDataProcessingPipelineSectionFromJsonNode::assert_correct_msg_types(
        const json& parsed_section_j, const std::string& msg_type, const int no_nodes){
    for (int i = 0; i < no_nodes; i++)
    {        
        ASSERT_EQ(parsed_section_j[i]["publishers"][0]["msg_type"], msg_type);
        ASSERT_EQ(parsed_section_j[i]["subscribers"][0]["msg_type"], msg_type);
    }
}

void TestParseDataProcessingPipelineSectionFromJsonNode::assert_correct_node_names(
        const json& parsed_section_j, const int no_nodes){
    std::string expected_node_name;
    std::string actual_node_name;
    for (int i = 0; i < no_nodes; i++) {
        expected_node_name = "dpp_node" + std::to_string(i);
        actual_node_name = parsed_section_j[i]["node_name"];
        ASSERT_EQ(actual_node_name, expected_node_name);
    }
}

void TestParseDataProcessingPipelineSectionFromJsonNode::assert_correct_topic_names_first_last_node(const json& parsed_section_j, 
        const std::string topic_sub_first, const std::string topic_pub_last, const int no_nodes) {

    ASSERT_EQ(parsed_section_j[0]["subscribers"][0]["topic_name"].get<std::string>(), topic_sub_first);
    ASSERT_EQ(parsed_section_j[no_nodes - 1]["publishers"][0]["topic_name"].get<std::string>(), topic_pub_last);
}

void TestParseDataProcessingPipelineSectionFromJsonNode::assert_correct_topic_names_in_between(
    const json& parsed_section_j, const int no_nodes) {
    std::string last_sub_topic_name_actual =
        parsed_section_j[no_nodes - 1]["subscribers"][0]["topic_name"];
    std::string last_sub_topic_name_expected = "dpp_topic_" +
                                           std::to_string(no_nodes - 2) + "_" +
                                           std::to_string(no_nodes - 1);

    ASSERT_EQ(last_sub_topic_name_actual, last_sub_topic_name_expected);
    ASSERT_EQ(parsed_section_j[0]["publishers"][0]["topic_name"], "dpp_topic_0_1");

    std::string expected_topic_name;
    std::string actual_topic_name;
    for (int i = 1; i < no_nodes - 1; i++) {
        actual_topic_name = parsed_section_j[i]["publishers"][0]["topic_name"];
        expected_topic_name =
            "dpp_topic_" + std::to_string(i) + "_" + std::to_string(i + 1);
        ASSERT_EQ(actual_topic_name, expected_topic_name);
    }
}

void TestParseDataProcessingPipelineSectionFromJsonNode::assert_correct_qos_reliability(
        const json& parsed_section_j, const std::string reliability, const int no_nodes){
    for (int i = 0; i < no_nodes; i++)
    {
      ASSERT_EQ(
          parsed_section_j[i]["publishers"][0]["qos_reliability"], reliability);
      ASSERT_EQ(
          parsed_section_j[i]["subscribers"][0]["qos_reliability"],
          reliability);
    }
}