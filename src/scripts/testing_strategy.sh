#!/bin/bash

#############################
# change variables here

DDS_BACKENDS='connext fastrtps cyclonedds'

DPP_RANGE_START=1
DPP_RANGE_END=21
DPP_RANGE_STEP_SIZE=2

F_PUBLISHER_SET='1 10 20 30 40 50 60 70 80 90 100'

MSG_SIZE_SET='100b 1kb 10kb 100kb 500kb'

QOS_RELIABILITY_SET='best-effort reliable'

TEST_DURATION=60

cd ../../install/benchmark/lib/benchmark 

for QOS_RELIABILITY in $QOS_RELIABILITY_SET
do
    for MSG_SIZE in $MSG_SIZE_SET
    do
        for F_PUBLISHER in $F_PUBLISHER_SET
        do
            for BACKEND in $DDS_BACKENDS
            do
                timestamp=$(date +%Y-%m-%d_%H-%M-%S)
                echo "[$timestamp]:"
                command="./benchmark topology/bi_data_processing_pipeline_profiled.json --data-processing-pipeline-range-start $DPP_RANGE_START \
                                                            --data-processing-pipeline-range-end $DPP_RANGE_END \
                                                            --data-processing-pipeline-range-step-size $DPP_RANGE_STEP_SIZE \
                                                            --middleware $BACKEND \
                                                            --publish_frequency $F_PUBLISHER \
                                                            --msg_size $MSG_SIZE \
                                                            --too-late-absolute 1000000 \
                                                            --time $TEST_DURATION
                                                            --ipc off
                                                            --profiling on
                                                            --qos_reliability $QOS_RELIABILITY"
                $command
                echo "[$timestamp]: $command" >> log.txt       
            done
        done
    done
done
