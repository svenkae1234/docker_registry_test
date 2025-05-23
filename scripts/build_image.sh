#!/usr/bin/bash

# ===== Global Variables =====

# Container runtime (Docker/Podman)
CONTAINER_COMMAND=""

# Paths
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

CACHE_IMAGE_NAME="ghcr.io/duatic/dev-workspace/dynaarm-cache:latest"
DOCKER_CACHE_FILE="${SCRIPT_DIR}/../docker/Dockerfile.cache"

IMAGE_NAME="ghcr.io/duatic/dev-workspace/dynaarm-jazzy:latest"
DOCKER_JAZZY_FILE="${SCRIPT_DIR}/../docker/Dockerfile.jazzy"

# Flags
CACHE_ENABLED=false

# ===== Functions =====

# Parse script arguments
function parse_arguments() {
    for arg in "$@"; do
        case $arg in        
        --cache | cache)
            CACHE_ENABLED=true
            echo "Package cache support enabled."
            ;;
        *)
            echo "Unknown argument: $arg"
            ;;
        esac
    done
}

# Detect container runtime
function detect_container_runtime() {
    if command -v podman >/dev/null 2>&1; then
        CONTAINER_COMMAND="podman"
    elif command -v docker >/dev/null 2>&1; then
        CONTAINER_COMMAND="docker"
    else
        echo "Neither Podman nor Docker is installed."
        exit 1
    fi
    echo "Using container runtime: ${CONTAINER_COMMAND}"
}

# Generic function to build an image
function build_image() {
    local dockerfile=$1
    local image_name=$2
    local base_image_arg=$3

    echo "Building image: ${image_name} using Dockerfile: ${dockerfile}"

    $CONTAINER_COMMAND build -f "${dockerfile}" -t "${image_name}" ${base_image_arg} .
    if [ $? -ne 0 ]; then
        echo "Failed to build image: ${image_name}."
        exit 1
    fi
}

# ===== Main Execution =====

detect_container_runtime
parse_arguments "$@"

cd ${SCRIPT_DIR}/..

# Build cache image if cache is enabled
if $CACHE_ENABLED; then    
    build_image "${DOCKER_CACHE_FILE}" "${CACHE_IMAGE_NAME}"
    BASE_ARG="--build-arg BASE_IMAGE=${CACHE_IMAGE_NAME}"
else
    BASE_ARG=""
fi

# Build jazzy image
build_image "${DOCKER_JAZZY_FILE}" "${IMAGE_NAME}" "${BASE_ARG}"