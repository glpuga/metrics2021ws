#!/bin/bash

if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters. Usage:"
    echo "./example_fbm2_launch.sh </path/to/images> <result_filename>"
    echo "./example_fbm2_launch.sh dataset/crack_detection_dataset/evaluation example_fbm2_results.txt"
    exit -1
fi
echo "Input image folder: $1"
echo "Result file name: $2"

TEAM_NAME_PREFIX="the_italian_job"
THRESHOLD="0.3"

FBM2_HOME_DIR="$HOME/FBM2"
DARKNET_RUN_DIR="$FBM2_HOME_DIR/the_italian_job_ws/yolo"
RESULTS_FOLDER="$FBM2_HOME_DIR/results"
LABELS_DIR="$FBM2_HOME_DIR/the_italian_job_ws/yolo/data/labels"
TRAINED_NET_DIR="net"

JSON_FILE_LOCATION="/tmp/"$TEAM_NAME_PREFIX"_darknet_results.json"
OUTPUT_RESULT_FILE=$RESULTS_FOLDER"/"$2

FILES_ABSOLUTE_FOLDER=$(cd $1 && pwd)

SUBMISSION_ID_FILE="$HOME/the_italian_job_submission.id"

# During the first run, create a symlink to the labels folder in darknet, which needs to be present
# in the folder where you run darknet.

echo -e "\n"

if [ -e $LABELS_DIR ]; then
    echo -e "Label folder present, will skip symbolic link creation"
    echo -e "$LABELS_DIR"
else
    echo -e "First run, creating symbolic link to labels folder"
    DARKNET_BINARY_PATH=$(which darknet)
    DARKNET_FOLDER_PATH=$(dirname $DARKNET_BINARY_PATH)
    ln -s "$DARKNET_FOLDER_PATH/../data/labels" $LABELS_DIR
fi

echo -e "\n"

if [ -e $SUBMISSION_ID_FILE ]; then
    SUBMISSION_ID=$(cat $SUBMISSION_ID_FILE)
    TRAINED_NET_DIR="$FBM2_HOME_DIR/../data/yolo/net"
    echo "We are in COMPETITION mode!"
    echo "Darknet will use the trained weights at $TRAINED_NET_DIR"
else
    echo "We are in DEVELOPMENT mode!"
    echo "Darknet will use the trained weights at $TRAINED_NET_DIR"
fi

echo -e "\n"

cd $DARKNET_RUN_DIR

rm -f $JSON_FILE_LOCATION

ls $FILES_ABSOLUTE_FOLDER/*.jpg | darknet detector test \
    $TRAINED_NET_DIR/cfg/metrics-rami.data \
    $TRAINED_NET_DIR/cfg/yolov4-metrics-rami.cfg \
    $TRAINED_NET_DIR/backup/yolov4-metrics-rami_best.weights \
    -ext_output \
    -thresh $THRESHOLD \
    -dont_show \
    -out $JSON_FILE_LOCATION

python3 tij_reformatter.py $JSON_FILE_LOCATION $OUTPUT_RESULT_FILE

echo "Done!"
