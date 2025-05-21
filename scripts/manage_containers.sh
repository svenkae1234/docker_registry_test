#!/usr/bin/bash

# ===== Global Variables =====
CONTAINER_RUNTIME=""
DOCKER_COMPOSE_FILE="../docker/docker-compose.yml"
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)

# ===== GPU support =====
GPU_COMPOSE_FILE="../docker/docker-compose.gpu.yml"
USE_GPU=false

# Services to manage based on OS
WINDOWS_SERVICES=("byts-ros2-workspace-windows" "byts-rosbridge-node")
LINUX_SERVICES=("byts-ros2-workspace-linux")

# ===== Functions =====

# Print script usage
function print_usage() {
    echo "Usage: $(basename "$0") [ACTION]"
    echo
    echo "Actions:"
    echo "  stop        Stop all running containers for the current OS."
    echo "  remove      Stop and remove all containers for the current OS."
    echo "  restart     Restart containers (stop, remove, and start)."
    echo "  attach      Attach to the first container (bash shell)."
    echo "  vscode      Attach VS Code to the first container."
    echo        
    echo "Options:"
    echo "  --gpu       Enable GPU support by using docker-compose.gpu.yml"
    echo
    echo "Examples:"
    echo "  $(basename "$0") stop           # Stop all containers"
    echo "  $(basename "$0") restart        # Restart containers"
    echo "  $(basename "$0") restart --gpu  # Restart with GPU support"
    echo "  $(basename "$0") attach         # Attach to a running container"
    echo "  $(basename "$0") vscode         # Attach VS Code to a container"    
}

# Detect Docker or Podman
function detect_container_runtime() {
    if command -v podman >/dev/null 2>&1; then
        CONTAINER_RUNTIME="podman"
    elif command -v docker >/dev/null 2>&1; then
        CONTAINER_RUNTIME="docker"
    else
        echo "Error: Neither Podman nor Docker is installed."
        exit 1
    fi
    echo "Using container runtime: ${CONTAINER_RUNTIME}"
}

# Determine the OS and services to use
function detect_os() {    
    case "$(uname -rs)" in
        *"WSL2"*)            
            SERVICES=("${WINDOWS_SERVICES[@]}")
            ;;
        *"Linux"*)            
            SERVICES=("${LINUX_SERVICES[@]}")
            ;;
        *)
            echo "Unsupported OS."
            exit 1
            ;;
    esac    
}

# Check if a container is running
function is_container_running() {
    local container_name="$1"
    $CONTAINER_RUNTIME ps -q -f name="${container_name}" | grep -q .
}

# Stop containers
function stop_containers() {
    echo "Stopping containers..."
    for service in "${SERVICES[@]}"; do
        if is_container_running "$service"; then
            echo "Stopping container: $service"
            $CONTAINER_RUNTIME stop "$service"
        else
            echo "Container $service is not running."
        fi
    done
}

# Remove containers
function remove_containers() {
    echo "Removing containers..."
    for service in "${SERVICES[@]}"; do
        if $CONTAINER_RUNTIME ps -a -q -f name="$service" | grep -q .; then
            echo "Removing container: $service"
            $CONTAINER_RUNTIME rm "$service"
        else
            echo "Container $service does not exist."
        fi
    done
}

# Restart containers
function restart_containers() {
    echo "Restarting containers..."
    stop_containers
    start_containers
}

# Start containers with optional GPU support
function start_containers() {
    echo "Starting containers using docker-compose..."
    cd "$(dirname "$DOCKER_COMPOSE_FILE")" || exit 1

    # Use both compose files if --gpu is passed
    if [[ "$USE_GPU" == true ]]; then
        echo "Starting with GPU support..."
        COMPOSE_FILES="-f $(basename "$DOCKER_COMPOSE_FILE") -f $(basename "$GPU_COMPOSE_FILE")"
    else
        echo "Starting without GPU support..."
        COMPOSE_FILES="-f $(basename "$DOCKER_COMPOSE_FILE")"
    fi
    
    for service in "${SERVICES[@]}"; do
        echo "Starting service: $service"
        $CONTAINER_RUNTIME compose $COMPOSE_FILES up -d "$service"
    done
}

# Attach VS Code to a container
function attach_vscode() {
    start_containers    
    local container_name="${SERVICES[0]}"  # Use the first service as the default
    echo "Attaching VS Code to container $container_name..."
    code --folder-uri "vscode-remote://attached-container+$(printf "$container_name" | xxd -p)/ros2_ws"
}

# Attach to the container
function attach_to_container() {
    start_containers
    local container_name="${SERVICES[0]}"  # Use the first service for attachment
    echo "Attaching to container $container_name..."
    xhost + >/dev/null 2>&1
    $CONTAINER_RUNTIME exec -it "$container_name" bash
    xhost - >/dev/null 2>&1
}

# ===== Main Execution =====

# Parse arguments
ACTION=""

for arg in "$@"; do
    case $arg in
        stop|remove|restart|attach|vscode)
            ACTION="$arg"
            ;;
        --help|-h)
            print_usage
            exit 0
            ;;
        --gpu)
            USE_GPU=true
            ;;      
        *)
            echo "Warning: Unknown argument '$arg'. Use --help to see valid options."
            print_usage
            exit 1
            ;;
    esac
done

# If no arguments provided, show usage
if [[ -z "$ACTION" ]]; then
    print_usage
    exit 1
fi

# Execution flow
detect_container_runtime
detect_os

case "$ACTION" in
    stop)
        stop_containers
        ;;
    remove)
        stop_containers
        remove_containers
        ;;
    restart)
        restart_containers
        ;;
    attach)
        attach_to_container
        ;;
    vscode)
        attach_vscode
        ;;
    *)
        print_usage
        exit 1
        ;;
esac
