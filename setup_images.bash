#!/bin/bash

MULTIVERSE_DIR=$(dirname $0)/images/Multiverse-image/Multiverse
if [ ! -d $MULTIVERSE_DIR ]; then
    (git clone https://github.com/Multiverse-Framework/Multiverse $MULTIVERSE_DIR --depth 1)
else
    (cd $MULTIVERSE_DIR; git pull)
fi

(cd $MULTIVERSE_DIR; git submodule update --init --depth 1 multiverse/resources; cd multiverse/resources; git submodule update --init)

MULTIVERSE_TUTORIALS_DIR=$(dirname $0)/images/Multiverse-Tutorials-image/Multiverse-Tutorials
if [ ! -d $MULTIVERSE_TUTORIALS_DIR ]; then
    git clone https://github.com/Multiverse-Framework/Multiverse-Tutorials $MULTIVERSE_TUTORIALS_DIR --depth 1
else
    (cd $MULTIVERSE_TUTORIALS_DIR; git pull)
fi
