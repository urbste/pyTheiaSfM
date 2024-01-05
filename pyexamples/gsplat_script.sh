# to use this script record a dataset using an Android phone and the Pilotguru app
# then copy the dataset to a folder on your computer and run this script
DATASET_PATH=$1
MEDIAN_SCENE_DEPTH_M=$2
FOCAL_LENGTH_PX=$3
DATASET_NAME=$(basename $DATASET_PATH)

if [ -z "$DATASET_PATH" ]
then
    echo "Please provide a path to the dataset"
    exit 1
fi
if [ -z "$MEDIAN_SCENE_DEPTH_M" ]
then
    echo "Please provide a median scene depth in m"
    exit 1
fi
if [ -z "$FOCAL_LENGTH_PX" ]
then
    echo "Please provide a focal length in pixels"
    exit 1
fi

python3 sfm_video_image_extractor.py --path_to_video $DATASET_PATH/video.mp4 --path_to_image_output $DATASET_PATH/images --debug 1 --min_framerate_to_extract 10
python3 sfm_pipeline_generic.py --image_path $DATASET_PATH/images/ --shared_intrinsics --export_nerfstudio_json $DATASET_PATH/transforms.json --average_scene_depth_m $MEDIAN_SCENE_DEPTH_M --export_sfm_depth --reconstruction_type global --focal_length $FOCAL_LENGTH_PX --debug 1