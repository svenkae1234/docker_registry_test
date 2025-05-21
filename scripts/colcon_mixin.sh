#!/bin/bash

# Define the mixin URL as a variable
MIXIN_URL="https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml"

# Check if the 'default' mixin with the specified URL is already added
if ! colcon mixin list | grep -q "$MIXIN_URL"; then
    echo "Adding the 'default' mixin with the URL: $MIXIN_URL"
    colcon mixin add default "$MIXIN_URL"   
else
    echo "'default' mixin with the URL $MIXIN_URL is already added."
fi
