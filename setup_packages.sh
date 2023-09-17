#!/bin/bash

cd $(dirname $0)

# Extraction directory
EXTRACT_DIR="UE-Packages"

# Destination filename
DEST_FILE="$EXTRACT_DIR/Demo1.tarz.xz"

if [ ! -d "$EXTRACT_DIR/Demo1" ]; then
    # URL of the .tar.xz file
    URL="https://seafile.zfn.uni-bremen.de/f/bdeeccaf509540d7b6d3/?dl=1"

    # Create the directory
    mkdir -p "$EXTRACT_DIR"

    # Download the file
    wget -O "$DEST_FILE" "$URL"

    # Extract the .tar.xz file
    tar -xf "$DEST_FILE" -C "$EXTRACT_DIR"

    # Remove the .tar.xz file
    rm -f "$DEST_FILE"
fi

# Destination filename
DEST_FILE="$EXTRACT_DIR/Demo2.tarz.xz"

if [ ! -d "$EXTRACT_DIR/Demo2" ]; then
    # URL of the .tar.xz file
    URL="https://seafile.zfn.uni-bremen.de/f/e33bc93082214f7e8efc/?dl=1"

    # Create the directory
    mkdir -p "$EXTRACT_DIR"

    # Download the file
    wget -O "$DEST_FILE" "$URL"

    # Extract the .tar.xz file
    tar -xf "$DEST_FILE" -C "$EXTRACT_DIR"

    # Remove the .tar.xz file
    rm -f "$DEST_FILE"
fi
