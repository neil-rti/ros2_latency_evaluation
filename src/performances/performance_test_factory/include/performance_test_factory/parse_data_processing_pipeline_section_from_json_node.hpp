#include <iostream>

#include "nlohmann/json.hpp"

using json = nlohmann::json;

class ParseDataProcessingPipelineSectionFromJsonNode
{
public:
    ParseDataProcessingPipelineSectionFromJsonNode(
        const json& data_processing_pipeline_section) {
        _data_processing_pipeline_section = data_processing_pipeline_section;
        _pipeline_pub_info_json = _data_processing_pipeline_section
            ["publisher_data_processing_pipeline_node"];  // publisher in
                                                          // between
        _start_sub_info_json = _data_processing_pipeline_section["subscribers_start_node"];
        _end_pub_info_json = _data_processing_pipeline_section["publisher_end_node"];

        _number_of_nodes = _data_processing_pipeline_section["data_processing_pipeline_nodes"];
    }

    json run() {
        json nodes_json = json::array();

        std::string node_name = "";
        std::string publisher_topic_name = "";
        std::string subscriber_topic_name = "";

        for (int idx_node = 0;
            idx_node < _number_of_nodes;
            idx_node++)
        {
            json node_section = json::object();
            node_name = "dpp_node" + std::to_string(idx_node);

            create_topic_names(publisher_topic_name, subscriber_topic_name, idx_node);
            json node_section_j = create_node_section(node_name, subscriber_topic_name, publisher_topic_name, idx_node);

            nodes_json.push_back(node_section_j);
        }
        return nodes_json;
    }

    void create_topic_names(std::string& publisher_topic_name,
                            std::string& subscriber_topic_name, 
                            int idx_node) {
        if (idx_node == 0)
            subscriber_topic_name = 
                _start_sub_info_json["topic_name"].get<std::string>();
        else
            subscriber_topic_name = _pipeline_pub_info_json["topic_name"].get<std::string>() 
                + "_" + std::to_string(idx_node - 1)
                + "_" + std::to_string(idx_node);
        
        if (idx_node == _number_of_nodes - 1)
            publisher_topic_name = _end_pub_info_json["topic_name"].get<std::string>();
        else
            publisher_topic_name = _pipeline_pub_info_json["topic_name"].get<std::string>() + "_" + std::to_string(idx_node) + "_" + std::to_string(idx_node + 1);
    }

    json create_node_section(const std::string& node_name,
                             const std::string& sub_topic_name,
                             const std::string& pub_topic_name,
                             int idx_node) {
        json node_section_j = json::object();
        json publisher_j = _pipeline_pub_info_json;
        json subscriber_j = json::object();

        node_section_j["node_name"] = node_name;
        node_section_j["executor_id"] = idx_node + 2;

        publisher_j = _pipeline_pub_info_json;
        publisher_j["topic_name"] = pub_topic_name; // object, i.e {...}

        subscriber_j               = _pipeline_pub_info_json;
        subscriber_j["topic_name"] = sub_topic_name;
        subscriber_j.erase("period_ms");

        node_section_j["publishers"] = json::array();
        node_section_j["publishers"].push_back(publisher_j);

        node_section_j["subscribers"] = json::array();
        node_section_j["subscribers"].push_back(subscriber_j);

        return node_section_j;
    }

private:
    json _data_processing_pipeline_section;
    json _pipeline_pub_info_json;
    json _start_sub_info_json;
    json _end_pub_info_json;

    int _number_of_nodes;
};
