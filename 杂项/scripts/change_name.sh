#!/bin/bash

DIRECTORY="imgs"
OLD_PREFIX="circle"
NEW_PREFIX="loop"

cd "$DIRECTORY" || { echo "Directory $DIRECTORY not found"; exit 1; }

for file in "$OLD_PREFIX"*; do
    if [ -e "$file" ]; then
        new_file="${NEW_PREFIX}${file#$OLD_PREFIX}"
        mv "$file" "$new_file"
        echo "Renamed $file to $new_file"
    fi
done