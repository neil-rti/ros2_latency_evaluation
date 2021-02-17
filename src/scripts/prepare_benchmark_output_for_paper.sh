# Log files from the script "testing_strategy.sh" are post-processed here for plotting purposes
# The approach is as follows: we assume always that the latency is to be plotted vs a "variable parameter" (on the x-axis).
# Assuming the variable parameter is, say, the publisher frequency, other parameters from the evaluated parameter set
# are fixed, i.e. QoS reliability, payload, etc. These parameters are called "fixed parameters".
# The variable parameter itself is changed below by changing the value "VARIABLE_PARAMETER" to the respective value.
# In this case, this is the publisher frequency. Further, a set of available parameter values for the variable parameter needs to be chosen.
# Usually, this corresponds to the value sets in the fixed parameter section below.

# There are the supported parameter sets (cf. paper) used for the post-processing
# Do not change anything here
PUBLISHER_FREQUENCY="[1, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]"
PAYLOAD="[ \"100b\", \"1kb\", \"10kb\", \"100kb\", \"500kb\"]"
MIDDLEWARE=" [ \"connextdds\", \"fastrtps\", \"cyclonedds\"]"
QOS="[\"reliable\", \"best-effort\"]"

########################################
# DO YOUR CHANGES HERE!

FIXED_PARAMETERS="{\"payload\": \"500kb\", \"middleware\": \"cyclonedds\", \"qos\": \"best-effort\"}"
VARIABLE_PARAMETER="publisher_frequency"
PARAMETER_VAL=$PUBLISHER_FREQUENCY

# STOP DOING CHANGES!
###########################

python_cmd="python3 create_paper_csv.py 
        --results_dir $1
        --parameter_filter '{\"fixed\": $FIXED_PARAMETERS, \"variable\": { \"parameter\": \"$VARIABLE_PARAMETER\", \"values\": $PARAMETER_VAL }}'"
echo "Fixed parameters are: $FIXED_PARAMETERS"
echo "Variable parameters are: $VARIABLE_PARAMETER with value $PARAMETER_VAL"
eval $python_cmd
