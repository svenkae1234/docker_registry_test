# Use an official Debian as a parent image
FROM ubuntu:24.04

ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ="Europe/Berlin"

# Install necessary packages
RUN apt-get update && apt-get install -y \
    apt-cacher-ng \
    python3-pip \
    python3-dev \    
    wget \
    curl \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install devpi-server for pip caching
RUN pip3 install devpi-server

# Copy the startup script
COPY start.sh /start.sh
RUN sed -i 's/\r$//' /start.sh
RUN chmod +x /start.sh

# Run the startup script
CMD ["/start.sh"]
