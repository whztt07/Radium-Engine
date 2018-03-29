#!/bin/bash

# arguments : 0 1 2 etc. : run scene
# t : run timings. f : make full video. c : copy to external drive

ROOTDIR="/ssd/canezin/IS_MUSCLES/Radium-Engine"
WORKDIR="/ssd/canezin/IS_MUSCLES/Radium-Engine/Bundle-GNU/Release/bin"
SCRIPTDIR="`date +%Y%m%d-%H%M`"
COPYDIR="/mnt/STORM/wip/muscle-skinning/videos/source_files/"
SCENE_DIR="/home/fcanezin/Downloads/scenefiles"

# returns true if string 1 contains string 2
# than you https://stackoverflow.com/questions/2829613
function contains()
{
    string="$1"
    substring="$2"

    if test "${string#*$substring}" != "$string"
    then
        return 0
    else
        return 1
    fi
}

# run a scene exporting subdivided meshes and anatomy
function run_scene_exportmesh()
{
    local infolder=$1
    local outfolder=$2
    local scenefile=$3

    source "$infolder/$scenefile"

    mkdir -p $outfolder

    DISPLAY=":1" \
    ./main-app --fps 30 --autoplay -e $outfolder -n $numframes \
    -f $infolder/$assetfile \
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile \
    --phyfile $infolder/$phyfile \
    --timestep $timestep \
    --numparticles $numparticles \
    --subdiv --showanat --savemeshes --runanat\
    >& $outfolder/log.txt

    cp $infolder/$camfile $outfolder/camera.cam
}


# run a scene exporting frames with transparent skin for the video
function run_scene_video()
{
    local infolder=$1
    local outfolder=$2
    local scenefile=$3

    source $infolder/$scenefile

    mkdir -p $outfolder

    DISPLAY=":1" \
    ./main-app --fps 30 --autoplay -e $outfolder -n $numframes \
    -f $infolder/$assetfile \
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile \
    --phyfile $infolder/$phyfile \
    --timestep $timestep \
    --numparticles $numparticles \
    --trans --showanat --saveframes --runanat \
    >& $outfolder/log.txt
}

function run_scene_timings()
{

    local infolder=$1
    local outfolder=$2
    local scenefile=$3

    source $infolder/$scenefile

    mkdir -p "$outfolder/on"
    mkdir -p "$outfolder/off"

    # run anatomy
    DISPLAY=":0.0" \
    ./main-app --fps 30 --autoplay -e "$outfolder/on" -n $numframes -t \
    -f $infolder/$assetfile \
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile \
    --phyfile $infolder/$phyfile \
    --timestep $timestep \
    --numparticles $numparticles \
    --runanat \
    >& "$outfolder/on/log.txt"

    #run without anatomy

    DISPLAY=":1" \
    ./main-app --fps 30 --autoplay -e "$outfolder/off" -n $numframes -t \
    -f $infolder/$assetfile \
    -c $infolder/$camfile \
    --anatfile $infolder/$anatfile \
    --kffile $infolder/$kffile \
    --timestep $timestep \
    --numparticles $numparticles\
    --phyfile $infolder/$phyfile \
    >& "$outfolder/off/log.txt"
}


# call ffmpeg on the exported frames
function make_video()
{
    local folder=$1
    local name=$2
    local fps=$3

    # get the number of the first frame
    local n=`ls $folder | grep radiumframe | sort -n | head -n1 | tr -d -c '[0-9]'`
    ffmpeg -framerate $fps -start_number $n -i "$folder/radiumframe_%06d.png" -c:v libx264  $folder/$name.mp4 >& log_ffmpeg.txt

}

function run_scene()
{
    local num=$1
    local in=$2
    local scene=$3

    local out="$SCRIPTDIR/scene$num"

    if $(contains "$args" "m")
    then
        echo "   Export meshes..."
        run_scene_exportmesh "$in" "$out/meshes" "$scene"
        cp $in/desc.txt $out/meshes/scene.txt
    fi

    echo "   Export frames..."
    run_scene_video "$in" "$out/frames" "$scene"

    echo "   Making video from frames..."
    make_video "$out/frames" "scene$num" "30"

}


args="'$*'"

if $(contains "$args" "c")
then
    slackbot.py "msg" "#sa2017paper" "Script started with args $args"
fi

echo "Starting script"
cd $WORKDIR

rm latest
mkdir -p $SCRIPTDIR
ln -s $SCRIPTDIR latest

if $(contains "$args" "0")
then
    echo "Running scene 0 (parameters presentation)"
    run_scene "00" "$SCENE_DIR/arm_nomove" "scene00"
    run_scene "01" "$SCENE_DIR/arm_nomove" "scene01"
fi

if $(contains "$args" "1")
then
    echo "Running scene 1 (arm shake)"
    run_scene 1 "$SCENE_DIR/arm_rise_shake" "scenefile"
fi

if $(contains "$args" "2")
then
    echo "Running scene 2 (biceps curl)"
    run_scene 2 "$SCENE_DIR/arm_biceps_curl" "scenefile"
fi

if $(contains "$args" "3")
then
    echo "Running scene 3 (Pectorals)"
    run_scene 3 "$SCENE_DIR/arm_pecs_biceps" "scenefile"
fi

if $(contains "$args" "4")
then
    echo "Running scene 4 (Boss Jump)"
    run_scene 4 "$SCENE_DIR/boss_jump" "scenefile"
fi

if $(contains "$args" "5")
then
    echo "Running scene 5 (Boss Run)"
    run_scene 5 "$SCENE_DIR/boss_run" "scenefile"
fi


if $(contains "$args" "f")
then
    rm -f list.txt
    touch list.txt
    echo "Making full video"
    find $SCRIPTDIR | grep mp4 | sort | while read file
    do
        echo "file '$file'" >> list.txt
    done
    ffmpeg -f concat -safe 0 -i list.txt -c copy fullvideo.mp4
    mv fullvideo.mp4 $SCRIPTDIR
fi


if $(contains "$args" "t")
then
    echo "Running timings on scene 2"
    run_scene_timings "$SCENE_DIR/arm_biceps_curl" "$SCRIPTDIR/timings" "scenefile"
    # do not cat the first frame because the setup task screw up the stats
    find "$SCRIPTDIR/timings_scene1/on"  | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_on_scene1.txt
    find "$SCRIPTDIR/timings_scene1/off" | grep radiumtimings | grep -v 000000 | xargs cat  |$ROOTDIR/parse_results.py >$SCRIPTDIR/timings_off_scene1.txt

    echo "Running timings on scene 2"
    run_scene_timings "$SCENE_DIR/arm_biceps_curl" "$SCRIPTDIR/timings_scene2" "scenefile"
    # do not cat the first frame because the setup task screw up the stats
    find "$SCRIPTDIR/timings_scene2/on"  | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_on_scene2.txt
    find "$SCRIPTDIR/timings_scene2/off" | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_off_scene2.txt

    echo "Running timings on scene 3"
    run_scene_timings "$SCENE_DIR/arm_pecs_biceps" "$SCRIPTDIR/timings_scene3" "scenefile"
    # do not cat the first frame because the setup task screw up the stats
    find "$SCRIPTDIR/timings_scene3/on"  | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_on_scene3.txt
    find "$SCRIPTDIR/timings_scene3/off" | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_off_scene3.txt

    echo "Running timings on scene 4"
    run_scene_timings "$SCENE_DIR/boss_jump" "$SCRIPTDIR/timings_scene4" "scenefile"
    # do not cat the first frame because the setup task screw up the stats
    find "$SCRIPTDIR/timings_scene4/on"  | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_on_scene4.txt
    find "$SCRIPTDIR/timings_scene4/off" | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_off_scene4.txt

    echo "Running timings on scene 5"
    run_scene_timings "$SCENE_DIR/boss_run" "$SCRIPTDIR/timings_scene5" "scenefile"
    # do not cat the first frame because the setup task screw up the stats
    find "$SCRIPTDIR/timings_scene5/on"  | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_on_scene5.txt
    find "$SCRIPTDIR/timings_scene5/off" | grep radiumtimings | grep -v 000000 | xargs cat | $ROOTDIR/parse_results.py >$SCRIPTDIR/timings_off_scene5.txt
fi

if $(contains "$args" "c")
then
    echo "Copying"
    cp -a $SCRIPTDIR $COPYDIR
    slackbot.py "msg" "#sa2017paper" "Script finished and copied in $COPYDIR"
fi
