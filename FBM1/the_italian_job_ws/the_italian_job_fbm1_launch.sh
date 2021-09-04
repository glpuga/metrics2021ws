#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters. Usage:"
    echo "./example_fbm1_launch.sh </path/to/bag> <result_filename>"
    echo "./example_fbm1_launch.sh dataset/FBM1_flight1.bag example_fbm1_result.txt"
    exit -1
fi
echo "Evaluation bag path: $1"
echo "Results file name: $2"

CWD=$(pwd)

if [[ "$1" = /* ]]; then
    BAGFILE="$1"
else
    # make the path absolute
    BAGFILE="$CWD/$1"
fi

if [[ "$2" = /* ]]; then
    OUTPUTFILE="$2"
else
    # make the path absolute
    OUTPUTFILE="$HOME/FBM1/results/$2"
fi

# In debug mode, the output is the name of the folder where the
# files and the analisys will be stored
if [[ "$DEBUG" -ne "" ]]; then
    COMPETITION_MODE="false"
    # notice that order matters for then next two lines
    GROUNDTRUTHFILE="$OUTPUTFILE/stamped_groundtruth.txt"
    OUTPUTFILE="$OUTPUTFILE/stamped_traj_estimate.txt"
else
    # In this mode the output fill is the name of the output file
    COMPETITION_MODE="true"
fi

# Create any missing directories in the path
mkdir -p $(dirname $OUTPUTFILE)

echo "Input bagfile: $BAGFILE"
echo "Output record: $OUTPUTFILE"

if [[ -e $OUTPUTFILE ]]; then
    echo -e "\nOutput file already exists!\n\n"
    exit 1
fi

cd the_italian_job_ws &&
    source /opt/ros/melodic/setup.bash &&
    catkin_make &&
    source devel/setup.bash &&
    roslaunch tij_challenger process_dataset.launch \
        bag_file:="$BAGFILE" \
        output_file:="$OUTPUTFILE" \
        ground_truth_file:="$GROUNDTRUTHFILE" \
        competition_mode:="$COMPETITION_MODE" \
        enable_rviz:="false"

# If we are on debug mode, run the rpg
if [[ "$DEBUG" -ne "" ]]; then
    cp "eval_cfg.yaml" "$(dirname $OUTPUTFILE)/eval_cfg.yaml"
    source devel/setup.bash &&
        rosrun rpg_trajectory_evaluation analyze_trajectory_single.py --png "$(dirname $OUTPUTFILE)"
fi

echo "Done!"
