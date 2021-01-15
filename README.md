# Ros2LatencyEvaluation

This repository contains the actual evaluation code for **Latency Overhead of ROS2 for Modular Time-Critical Systems** submitted to the ICRA2021 conference. It is forked from the foxy branch of the [irobot-ros-framework](https://github.com/irobot-ros/ros2-performance/tree/foxy), commit hash  19d484c from June 8.
This is the most recent commit to this branch (last visited: June 10).

**Important Note**: Lots of code chunks are left untouched due to legacy reasons. Some of the previous core functionalities might not be running anymore. The README files in the subdirectories are legacy as well. However, they provide a better insight into the code and how it can be changed. The README files were not changed by BI.

# Install

1. Download this repository.
2. Download the [docker container](https://github.com/Barkhausen-Institut/ros2_latency_evaluation_docker) this evaluation program needs to be run in. Follow the instructions of that repository.
3. `. /opt/ros/foxy/setup.bash && cd <ws-root>`
3. Run `colcon build --symlink-install` in the root directory. 

# Datasets

The generated dataset for the paper can be found [here](https://barkhauseninstitut.sharepoint.com/:f:/s/BIDocTransfer/ErQS-1UxiIRGka0WQ04ql3gBLnIckYFFEyxTtCzN-xLt1w?e=QLCP78).
# Use

Always ensure to source ROS2 by calling `. /opt/ros/foxy/setup.bash` and the workspace by calling `. <ws-root>/install/setup.bash`. Afterwards, run the unit tests with `colcon test` from `<ws-root>`.

At first, we will describe steps necessary to reproduce paper results. Afterwards, we will provide an in-detail use description.

## Reproducing paper results

Hardware parameters are described in the paper. Ensure that kernel parameters for the CPU etc. are set accordingly. This has a major influence on the measured latency.

Paper results were obtained by using the scripts in `src/scripts`. There are two subsequent steps involved:

Firstly, the script `testing_strategy.sh` needs to be called. If you want to change some parameters, change the variables accordingly. By default, all defined parameter values in the paper are set. A total runthrough takes approximately 62 hours. The results can be found in `install/benchmark/lib/benchmark/results`. 

Secondly, the results must be post-processed. For that purpose, the script `prepare_benchmark_output_for_paper.sh <log-files-pth>` is to be used. `<log-files-path>` is the parent diretory of the results obtained from the previous step. This script calls the helper script `create_paper_csv.py`.
In the case of Raspberry Pi, we recommend to create a symbolic link to a virtual directory in order to prevent excessive write access to the SD card. Check the comments in the scripts for further documentation.

Results are saved in `<log-files-path>/paper_csvs`.

## Detailed Use

The main executable is located in `install/benchmark/lib/benchmark`.

Each node system is defined by a [`topology.json`](src/performances/benchmark/README.md). Some examples thereof can be found in the `topology/` folder. You run a topology with

```bash
./benchmark topology/<topology_name>.json
```

### Choosing a topology

Topologies defined by the BI are prepended with a `bi`.

Four our paper results, we used the topology `bi_data_processing_pipeline.json`. The node system will be profiled. If you defined your own topology, make sure it is in the `topology` folder with the other topology files. Afterwards, add the topology to the other topologies in `benchmark/CMakeLists.txt` and rebuild the workspace.

**Note:**
- In order to do some statistics calculations, we make some assumptions. Be aware of these assumptions if you want to use our internal statistics calculations (see below).

### Data-processing pipeline

Call

```bash
./benchmark topology/bi_data_processing_pipeline_nodes.json --data-processing-pipeline-range-start 1 --data-processing-pipeline-range-end 5 --data-processing-pipeline-range-step-size 2
```

- `--data-processing-pipeline-range-start 1`: Start with one node between the starting and end node.
- `--data-processing-pipeline-range-end 5`: End with five nodes between the starting and end node.
- `--data-processing-pipeline-range-step-size 2`: Take a step size of two nodes.

### Payload change via command line parameter

Use `--msg_size <msg-size>` for changing the msg size. The following msg sizes are supported:

- 100b
- 1kb
- 10kb
- 100kb
- 500kb

### Payload change via txt file (not used for paper)

Next to the `topology` folder, you can find a `payload` folder that contains a text file with the payloads. Each payload you want to vary needs to be in a separate line, i.e. `\n` is the separator. Valid payloads can be found in directory `irobot_interfaces_plugin/msg/`. **Attention**: The names of the payloads defined in the payload file must not be the same as the msg files, they must be the ones you would normally use to include this message file as an `.hpp` in your normal ros2-code! Therefore, these names are the ros2-automatically generated ones.

The general rule is: The `.msg` file follows a CamelCase notation, e.g. `CamelCase.msg`.

1. Between each capital letter a underscore is inserted, i.e. `Camel_Case`.
2. The capital letters are replaced by their lower representative, i.e. `camel_case`

Numbers do not count as capital letters.
If you modified your payloads or added a new payloads file, you need to add it to the `irobot_interfaces_plugin/CMakeLists.txt` file as well and rebuild the workspace.

-------

Go to the folder, `irobot_interfaces_plugin`, the script `create_msg_files.py` will be your friend.

Go into the code, fill in the payload sizes you want to add. Execute the script. `.msg` files will be created and can be found in `msg` subdirectory. Further, a `add_cmake.txt` file is created. Copy its content to the `CMakeLists.txt` of `irobot_interfaces_plugin`, to the section

```cmake
set( CUSTOM_MSGS
// ...
)
```


## Activate Profiling

If you want to activate profiling, you must ensure that the intraprocess communication is turned off by setting the parameter to `--ipc off` and `--profiling on`. Further, only messages are profiled that are sent via a topic that contains the substring `_profile_`. **IMPORTANT:** The messages sent must be at least of size 100 bytes! The duration between the last profiling timestamp and the header timestamp must not be larger than 4s.

## Change Middleware

Do so by changing the cli parmeter `--middleware {connext, fastrtps, cyclonedds}`. The folder contains config files for setting the qos parameter.

For QOS profiles:

1. [Cyclone DDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/config.rst) and [available options](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/docs/manual/options.md)
2. [FastRTPS XML profiles](https://fast-rtps.docs.eprosima.com/en/latest/fastdds/xml_configuration/xml_configuration.html)
3. [RTI Connext XML Configuration](https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/RTI_ConnextDDS_CoreLibraries_UsersManual.pdf)

The xml files can be found in `<root_ws>/config`.

## Other Parameters

Additional parameters can be changed as well. Check output of `./benchmark --help`.

# Logging
If statistical quantitites are calculated, there are in general two cases:

- inter-node: Latency entailed between two nodes, namely between the publishing node and the subscription node
- first-to-last-node: The inter-node latencies are added over the whole data-processing pipeline.

For each walkthrough, a directory is created in the `benchmark/results` folder. It contains the following files:

- `events` (legacy): Logs if a message is too late etc.
- `resources` (legacy): Can be used for memory and cpu consumption (not tested yet)
- `profiling_inter_node`: Mean and standard deviation of the delay between two succeeding profiling layers in the inter-node case. For the first layer, difference is calculated to the header. Difference between sending and receiving of message  is calculated as well.
- `profiling_first_to_last_node`: Statistics about the inter-node profiling latencies summed up over the whole data-process pipeline.
- `latency_all` (legacy)
- `tracking_numbers`: Each message is associated with a tracking number which is increased with each publishment at the first node. This file is useful for debugging purposes.
- `summed_up_internode_latencies`: All latencies are summed up from the first node to the last node. No statistics is calculated.
- `header.csv`: Absolute timestamps stored in the header of the received messages.
- `now.csv`: Absolute timestamp used in the callback if a message is received with the timestamp from `header.csv` in its header.
- `absolute_profiling_timestamps*.csv`: Contains the absolute profiling timestamps obtained from the monotonic clock (not UTC referenced!).

Messages that are considered in the original implementation of `ros2-performance` as LATE or TOO LATE are not discarded!

## Remarks on Statistics Calculation

For calculating the statistics, we dump the samples and statistical quantitites (mean, standard deviation) for each random variable (i.e. end-to-end latency, profiling timestamps etc.). The following values are calculated for each random variable:
- absolute timestamps,
- time differences that occur between a publishing a message and receiving it (inter node latencies),
- latencies are summed up over the data-processing pipeline. We assume the pipeline to have one starting node and one end node, i.e.

``` 
a---b---c---d
```

with `a`, `b`, `c` and `d` being nodes. **However, the statistical quantitites only serve as an initial indicator and are not used for the paper. Median values presented in the paper are calculated in the script `create_paper_csv.py` based on the absolute timestamps.**
