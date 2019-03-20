#!/bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

build_dir=$script_dir/Maneuvers/build

# check if build folder exists
if [[ ! -d $build_dir ]]; then
    mkdir -p $build_dir
    cd $build_dir
    cmake ../src &&  make
    cd $SCRIPT_DIR
fi
