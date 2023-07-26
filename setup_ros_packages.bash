#!/bin/bash
cd $(dirname $0)/images/Multiverse-image
if [ ! -d "Multiverse" ]; then
    git clone git@github.com:Multiverse-Framework/Multiverse.git
else
    (cd Multiverse && git pull)
fi
wstool update