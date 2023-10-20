#!/bin/bash

(cd $(dirname $0)/images/MultiverseCore-image; wstool update; git submodule update --init)
if [ ! -d "$(dirname $0)/images/Multiverse-image/Multiverse" ]; then
    (mkdir $(dirname $0)/images/Multiverse-image/Multiverse)
    (cp -r $(dirname $0)/images/MultiverseCore-image/Multiverse/multiverse_ws $(dirname $0)/images/Multiverse-image/Multiverse/multiverse_ws)
    (cp $(dirname $0)/images/MultiverseCore-image/Multiverse/build_multiverse_ws.sh $(dirname $0)/images/Multiverse-image/Multiverse/build_multiverse_ws.sh)
fi
(cd $(dirname $0)/images/MultiverseDemos-image; wstool update)
(cd $(dirname $0)/images/Giskard-image; wstool update)