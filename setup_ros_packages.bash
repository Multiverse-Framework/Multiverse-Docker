#!/bin/bash
cd $(dirname $0)/images/Multiverse-image
if [ ! -d "Multiverse" ]; then
    git clone git@github.com:Multiverse-Framework/Multiverse.git --branch docker
else
    (cd Multiverse && git checkout docker && git pull origin docker)
fi
wstool update