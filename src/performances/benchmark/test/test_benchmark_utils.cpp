
#include <gtest/gtest.h>
#include <limits.h>
#include <unistd.h>

#include "utils.cpp"

std::string get_current_time_stamp();

class TestBenchmarkUtils : public ::testing::Test {
   public:
    void SetUp() override { _options.experiment_name = "test-experiment"; }
    // variables
    benchmark::Options _options = benchmark::Options();
    nlohmann::json topology_j = R"(
        {
    "nodes": [
        {
            "node_name": "start_node",
            "initial_pub": [
                {
                    "topic_name": "topic_0",
                    "msg_type": "stamped10b",
                    "period_ms": 10
                }
            ]
        },
        {
            "node_name": "data_processing_pipeline",
            "data_processing_pipeline_nodes": 5,
            "subscribers_start_node": {
                "topic_name": "topic_0",
                "msg_type": "stamped10b"
            },
            "publishers_data_processing_pipeline_node": {
                "topic_name": "pt_topic",
                "msg_type": "stamped10b",
                "period_ms": 10
            },
            "publishers_end_node": {
                "topic_name": "end_topic",
                "msg_type": "stamped10b",
                "period_ms": 10
            }
        },
        {
            "node_name": "end_node",
            "subscribers": [
                {
                    "topic_name": "end_topic",
                    "msg_type": "stamped10b"
                }
            ]
        }
    ]
}
    )"_json;
    const std::string _SYSTEM_INFO_FILENAME = "system_info.json";

    std::string get_current_time_stamp() {
        time_t now;
        struct tm* timeinfo;
        char timestamp[100];

        time(&now);
        timeinfo = localtime(&now);

        strftime(timestamp, 100, "%Y-%m-%d_%H-%M_", timeinfo);

        return std::string(timestamp);
    }

    bool contains_string(const std::string& a, const std::string& b) {
        return (a.find(b) != std::string::npos);
    }

    void assert_proper_json_options_conversion(
        const std::string& complete_file_path,
        const benchmark::Options& options);

    std::string get_system_info_file_path(const std::string& filename,
                                          benchmark::Options& opt) {
        opt.topology_json_list.push_back("dummy.json");

        return get_cwd() + "/" + filename;
    }

    std::string get_cwd() {
        char cwd[PATH_MAX];
        if (getcwd(cwd, sizeof(cwd)) == NULL)
            std::cout << "Getting cwd failed..." << std::endl;
        return std::string(cwd);
    }

    std::ifstream load_system_info_file(const std::string& complete_file_path,
                                        nlohmann::json& system_info_j) {
        std::ifstream ifs(complete_file_path);
        ifs >> system_info_j;
        return ifs;
    }
};

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsCorrectTimeStamp) {
    std::string results_dir_name = create_results_dir_name(_options);
    std::string current_time_stamp = get_current_time_stamp();

    ASSERT_TRUE(contains_string(results_dir_name, current_time_stamp));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsExperimentName) {
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, _options.experiment_name));
}

TEST_F(TestBenchmarkUtils,
       ResultsDirName_ContainsNoExperimentNameIfOnlyExperiment) {
    _options.experiment_name = "experiment";
    std::string results_dir_name = create_results_dir_name(_options);
    ASSERT_TRUE(!contains_string(results_dir_name, _options.experiment_name));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_Containsdata_processing_pipelineRangeIfStartEndAreUnequal)
{
  _options.data_processing_pipeline_range_start   = 1;
  _options.data_processing_pipeline_range_end     = 5;
  _options.data_processing_pipeline_range_step_size = 2;
  std::string results_dir_name   = create_results_dir_name(_options);

  ASSERT_TRUE(contains_string(results_dir_name, "_DPPRange1-5-2_"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_DoesNotContaindata_processing_pipelineRangeIfStartEndEqual)
{
  _options.data_processing_pipeline_range_start   = 1;
  _options.data_processing_pipeline_range_end     = 1;
  std::string results_dir_name = create_results_dir_name(_options);

  ASSERT_TRUE(!contains_string(results_dir_name, "_DPPRange1-5_"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsMiddleware) {
    _options.middle_ware = "connext";
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_connext_"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsIpc) {
    _options.ipc = false;
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_IpcOff_"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsPayloadFile) {
    _options.payload_file = "mypayloadfile.txt";
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_mypayloadfile"));
    ASSERT_FALSE(contains_string(results_dir_name, "_mypayloadfile.txt"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsProfiling) {
    _options.tracking_options.enable_profiling = false;
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_ProfilingOff_"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsPublishFrequencyHz) {
    _options.publish_frequency = 15;
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_PublishFrequency15Hz"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsNoPublishFreqencyIfZero) {
    _options.publish_frequency = 0;
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_FALSE(contains_string(results_dir_name, "PublishFrequency"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsTooLateAbsoluteUs) {
    _options.tracking_options.too_late_absolute_us = 100;
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_100us"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsMsgSize) {
    _options.msg_size = "100b";
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_MsgSize100b"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsNoMsgSizeIfEmpty) {
    _options.msg_size = "";
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(!contains_string(results_dir_name, "_MsgSize"));
}

TEST_F(TestBenchmarkUtils, ResultsDirName_ContainsQosReliability) {
    _options.qos_reliability = "reliable";
    std::string results_dir_name = create_results_dir_name(_options);

    ASSERT_TRUE(contains_string(results_dir_name, "_reliable"));
}

TEST_F(TestBenchmarkUtils, GetPayloadBasename_AbsolutePath) {
    const std::string basename = "mypayloads";
    const std::string file_extension = "txt";
    const std::string absolute_path =
        "/home/user/" + basename + "." + file_extension;

    ASSERT_EQ(get_basename(absolute_path, file_extension), basename);
}

TEST_F(TestBenchmarkUtils, GetPayloadBasename_RElativePath) {
    const std::string basename = "mypayloads";
    const std::string file_extension = "txt";
    const std::string relative_path = basename + "." + file_extension;
    ASSERT_EQ(get_basename(relative_path, file_extension), basename);
}

TEST_F(TestBenchmarkUtils, GetWsFromBenchmarkFolder) {
    std::string benchmark_folder = "/ws/install/benchmark/lib/benchmark";
    ASSERT_EQ(get_ws_folder_from_benchmark_folder(benchmark_folder), "/ws");
}

TEST_F(TestBenchmarkUtils, SetConnextAsMiddleware_ProperEnvSet) {
    set_middleware("connext");
    ASSERT_EQ(std::string(getenv("RMW_IMPLEMENTATION")), "rmw_connext_cpp");
}

TEST_F(TestBenchmarkUtils, ChangeMiddleware_DefaultEnvVariableSet) {
    const std::string WS_ROOT = "/home/user/repos/ws";
    set_env_dds_middleware(WS_ROOT, "connext");
    ASSERT_EQ(std::string(getenv("CYCLONEDDS_URI")),
              WS_ROOT + "/config/qos_cyclonedds.xml");
    ASSERT_EQ(std::string(getenv("FASTRTPS_DEFAULT_PROFILES_FILE")),
              WS_ROOT + "/config/qos_fastrtps.xml");
    ASSERT_EQ(std::string(getenv("RMW_FASTRTPS_USE_QOS_FROM_XML")), "1");
    ASSERT_EQ(std::string(getenv("NDDS_QOS_PROFILES")),
              "[" + WS_ROOT + "/config/qos_connext.xml]");
}

TEST_F(TestBenchmarkUtils, LogSystemInfo_FileProperlyCreated) {
    const std::string COMPLETE_FILE_PATH =
        get_system_info_file_path(_SYSTEM_INFO_FILENAME, _options);

    // check if system info file was deleted in last test
    std::ifstream system_info_file_previous_test(COMPLETE_FILE_PATH);
    ASSERT_FALSE(system_info_file_previous_test.good())
        << "File of previous test was not deleted. Please delete it!";

    save_system_info(topology_j, get_cwd(), _SYSTEM_INFO_FILENAME, _options);

    // check if file was created
    std::ifstream system_info_file(COMPLETE_FILE_PATH);
    ASSERT_TRUE(system_info_file.good());

    // now delete file
    system_info_file.close();
    if (!std::remove(COMPLETE_FILE_PATH.c_str()))
        perror("Error occured while deleting file");
}

TEST_F(TestBenchmarkUtils, LogSystemInfo_FileContainsProperJson) {
    // prepare json creation
    const std::string COMPLETE_FILE_PATH =
        get_system_info_file_path(_SYSTEM_INFO_FILENAME, _options);
    nlohmann::json system_info_j_from_file;
    std::ifstream system_info_file;

    save_system_info(topology_j, get_cwd(), _SYSTEM_INFO_FILENAME, _options);
    assert_proper_json_options_conversion(COMPLETE_FILE_PATH, _options);
}

TEST_F(TestBenchmarkUtils,
       LogSystemInfo_FileContainsProperJson_MsgSizeOptionSet) {
    // prepare json creation
    benchmark::Options options = benchmark::Options();
    options.msg_size = "100b";

    const std::string COMPLETE_FILE_PATH =
        get_system_info_file_path(_SYSTEM_INFO_FILENAME, options);
    save_system_info(topology_j, get_cwd(), _SYSTEM_INFO_FILENAME, options);
    assert_proper_json_options_conversion(COMPLETE_FILE_PATH, options);
}

TEST_F(TestBenchmarkUtils,
       LogSystemInfo_FileContainsProperJson_PublishFrequencyOptionSet) {
    // prepare json creation
    benchmark::Options options = benchmark::Options();
    options.publish_frequency = 100;

    const std::string COMPLETE_FILE_PATH =
        get_system_info_file_path(_SYSTEM_INFO_FILENAME, options);
    save_system_info(topology_j, get_cwd(), _SYSTEM_INFO_FILENAME, options);
    assert_proper_json_options_conversion(COMPLETE_FILE_PATH, options);
}

TEST_F(TestBenchmarkUtils,
       LogSystemInfo_FileContainsProperJson_QosReliabilitySet) {
    // prepare json creation
    benchmark::Options options = benchmark::Options();
    options.qos_reliability = "reliable";

    const std::string COMPLETE_FILE_PATH =
        get_system_info_file_path(_SYSTEM_INFO_FILENAME, options);
    save_system_info(topology_j, get_cwd(), _SYSTEM_INFO_FILENAME, options);
    assert_proper_json_options_conversion(COMPLETE_FILE_PATH, options);
}

void TestBenchmarkUtils::assert_proper_json_options_conversion(
    const std::string& complete_file_path, const benchmark::Options& options) {
    nlohmann::json system_info_j_loaded;
    std::ifstream system_info_file =
        load_system_info_file(complete_file_path, system_info_j_loaded);
    // check at first tracking options
    ASSERT_EQ(system_info_j_loaded["tracking_options"]["enabled"],
              options.tracking_options.is_enabled);

    ASSERT_EQ(system_info_j_loaded["tracking_options"]["late_percentage"],
              options.tracking_options.late_percentage);
    ASSERT_EQ(system_info_j_loaded["tracking_options"]["late_absolute_us"],
              options.tracking_options.late_absolute_us);
    ASSERT_EQ(system_info_j_loaded["tracking_options"]["too_late_percentage"],
              options.tracking_options.too_late_percentage);
    ASSERT_EQ(system_info_j_loaded["tracking_options"]["too_late_absolute_us"],
              options.tracking_options.too_late_absolute_us);

    ASSERT_EQ(system_info_j_loaded["tracking_options"]["enable_profiling"],
              options.tracking_options.enable_profiling);

    // check ros2 specific
    ASSERT_EQ(system_info_j_loaded["topology"]["data_processing_pipeline_range_start"],
              options.data_processing_pipeline_range_start);
    ASSERT_EQ(system_info_j_loaded["topology"]["data_processing_pipeline_range_end"],
              options.data_processing_pipeline_range_end);
    ASSERT_EQ(system_info_j_loaded["topology"]["data_processing_pipeline_range_step_size"],
              options.data_processing_pipeline_range_step_size);
    ASSERT_EQ(system_info_j_loaded["topology"]["payload_file"],
              options.payload_file);
    ASSERT_EQ(system_info_j_loaded["topology"]["filename"],
              options.topology_json_list[0]);
    ASSERT_EQ(system_info_j_loaded["topology"]["qos_reliability"],
              options.qos_reliability);

    if (options.msg_size == "")
        ASSERT_FALSE(system_info_j_loaded["topology"]["msg_size"] ==
                     options.msg_size);
    else
        ASSERT_EQ(system_info_j_loaded["topology"]["msg_size"],
                  options.msg_size);

    // scenario
    ASSERT_EQ(system_info_j_loaded["experiment"]["duration_sec"],
              options.duration_sec);
    ASSERT_EQ(system_info_j_loaded["experiment"]["ipc_enabled"], options.ipc);
    ASSERT_EQ(system_info_j_loaded["experiment"]["name"],
              options.experiment_name);
    ASSERT_EQ(system_info_j_loaded["experiment"]["middle_ware"],
              options.middle_ware);
    if (options.publish_frequency <= 0)
        ASSERT_FALSE(system_info_j_loaded["experiment"]["publish_frequency"] ==
                     options.publish_frequency);
    else
        ASSERT_EQ(system_info_j_loaded["experiment"]["publish_frequency"],
                  options.publish_frequency);

    system_info_file.close();
    if (!std::remove(complete_file_path.c_str()))
        perror("Error occured while deleting file");
}