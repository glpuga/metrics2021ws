#!/bin/bash

DATAFILE="net/cfg/metrics-rami.data"
CFGFILE="net/cfg/yolov4-metrics-rami.cfg"
WEIGHTSFILE="/home/metrics/data/yolo/yolov4.conv.137"

LASTWEIGHTS="net/backup/yolov4-metrics-rami_last.weights"

if [[ -e $LASTWEIGHTS ]]; then
    echo "Found a previous session weights file, resuming from there..."
    sleep 3
    WEIGHTSFILE=`echo $LASTWEIGHTS`
fi

darknet detector train $DATAFILE $CFGFILE $WEIGHTSFILE -dont_show -mjpeg_port 8090 -map -iou_thresh 0.4
