#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "You must enter the build directory as a parameter"
    exit
fi

if [ -d "$1" ]; then
	echo "The build directory must not exist"
#	exit
fi

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

echo "creating build directory in $1"

BASE=$(basename $DIR)

mkdir "$1"
mkdir "$1/$BASE"

sudo ln -s "$DIR"/* "$1/$BASE/"

pushd "$1/$BASE"
    if [ "$(uname)" == "Darwin" ]; then
        ./bootstrap_mac && ./configure && make && sudo make install
    else
        ./bootstrap && ./configure && make && sudo make install
    fi
popd

