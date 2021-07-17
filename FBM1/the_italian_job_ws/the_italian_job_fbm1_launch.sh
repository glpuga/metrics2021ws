#!/bin/bash
if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters. Usage:"
    echo "./example_fbm1_launch.sh </path/to/bag> <result_filename>"
    echo "./example_fbm1_launch.sh dataset/FBM1_flight1.bag example_fbm1_result.txt" 
    exit -1
fi
echo "Evaluation bag path: $1"
echo "Result file name: $2"

python2 /home/metrics/FBM1/example_ws/example_solution/fbm1_example_solution.py $1 $2

echo "Done!"