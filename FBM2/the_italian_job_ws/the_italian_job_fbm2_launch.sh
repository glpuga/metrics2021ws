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

FBM2_HOME_DIR="/home/metrics/FBM2"
DARKNET_RUN_DIR="$FBM2_HOME_DIR/the_italian_job_ws/yolo"
RESULTS_FOLDER="$FBM2_HOME_DIR/results"
LABELS_DIR="$FBM2_HOME_DIR/the_italian_job_ws/yolo/data/labels"

JSON_FILE_LOCATION="/tmp/"$TEAM_NAME_PREFIX"_darknet_results.json"
OUTPUT_RESULT_FILE=$RESULTS_FOLDER"/"$2

FILES_ABSOLUTE_FOLDER=$(cd $1 && pwd)

# During the first run, create a symlink to the labels folder in darknet, which needs to be present
# in the folder where you run darknet.
if [ -e $LABELS_DIR ]; then
    echo -e "Label folder present, will skip symbolic link creation"
    echo -e "$LABELS_DIR"
else
    echo -e "First run, creating symbolic linc to labels folder"
    DARKNET_BINARY_PATH=$(which darknet)
    DARKNET_FOLDER_PATH=$(dirname $DARKNET_BINARY_PATH)
    ln -s "$DARKNET_FOLDER_PATH/../data/labels" $LABELS_DIR
fi

cd $DARKNET_RUN_DIR
ls $FILES_ABSOLUTE_FOLDER/*.jpg | darknet detector test \
    net/cfg/metrics-rami.data \
    net/cfg/yolov4-metrics-rami.cfg \
    net/backup/yolov4-metrics-rami_best.weights \
    -ext_output \
    -thresh $THRESHOLD \
    -dont_show \
    -out $JSON_FILE_LOCATION

python3 tij_reformatter.py $JSON_FILE_LOCATION $OUTPUT_RESULT_FILE

echo "Done!"
