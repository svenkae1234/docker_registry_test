#!/bin/bash

# Start apt-cacher-ng
chmod -R 777 /var/cache/apt-cacher-ng
service apt-cacher-ng start

# Start devpi-server
devpi-init
devpi-server --host 0.0.0.0

# Keep the container running
tail -f /dev/null
