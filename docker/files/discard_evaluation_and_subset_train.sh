#!/bin/bash -

#
# This script discards the evaluation dataset, and then repartitions the train dataset into a train and
# evaluation subsets. This is because while the train dataset seems to be correctly labeled, the
# evaluation dataset is not and it is making difficult to evaluate training results.
#
#
# Launch this script from the folder containing the train/ and evaluation/ subfolders
#

# remove the evaluation folder completely
rm -rf evaluation
mkdir -p evaluation

# percentage of the train dataset that will be converted to evaluation
evaluation_percentage=15

image_list=`ls train/*.jpg`

for image in $image_list; do
    sample=$(( $RANDOM % 100 ))
    filename_root=`echo $image | cut -f1 -d.`
    if [[ `expr $sample \< $evaluation_percentage` -ne 0 ]]; then
	# move into the evaluation folder
	echo Moving $image into the evaluation dataset
	mv $filename_root.* evaluation/
    else
	echo Leaving $image in the training dataset
    fi
done

evaluation_samples=`ls evaluation/*.jpg | wc -l`
train_samples=`ls train/*.jpg | wc -l`

echo
echo Just partitioned the data into E $evaluation_samples + T $train_samples
echo