#!/bin/bash

(cd $(dirname $0)/images/MultiverseCore-image; wstool update; git submodule update --init)
(cd $(dirname $0)/images/MultiverseDemos-image; wstool update)
(cd $(dirname $0)/images/Giskard-image; wstool update)