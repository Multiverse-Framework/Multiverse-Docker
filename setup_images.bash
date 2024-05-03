#!/bin/bash

if [ ! -d $(dirname $0)/images/Multiverse-image/Multiverse ]; then
    git clone https://github.com/Multiverse-Framework/Multiverse $(dirname $0)/images/Multiverse-image/Multiverse
else
    cd $(dirname $0)/images/Multiverse-image/Multiverse
    git pull
fi
