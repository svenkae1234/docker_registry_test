#!/usr/bin/env bash

set -e

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
MAIN_DIR="${SCRIPT_DIR}/.."
ENV_FILE="${SCRIPT_DIR}/.env"

# If running in CI, set non-interactive defaults
if [[ "$CI" == "true" ]]; then
  export PRIVATE_REPOS="No"
  export GIT_USER="${GIT_USER:-ci}"
  export GIT_EMAIL="${GIT_EMAIL:-ci@example.com}"
fi

# Load .env file if present (for local interactive use)
if [ -f "$ENV_FILE" ]; then
  source "$ENV_FILE"
  echo "Loaded from .env: PRIVATE_REPOS=$PRIVATE_REPOS, PAT=${PAT:+***}, GIT_USER=$GIT_USER"
fi

# Ensure vcs is installed
if ! command -v vcs &> /dev/null; then
  echo "🔧 Installing vcstool..."

  # Add ROS 2 apt repo (for GitHub Actions runners or bare images)
  sudo apt update
  sudo apt install -y curl gnupg lsb-release

  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  sudo apt update
  sudo apt install -y python3-vcstool
fi

# Switch to workspace root
cd "$MAIN_DIR"

# Check repos.list
REPOS_LIST="repos.list"
if [ ! -f "$REPOS_LIST" ]; then
  echo "❌ Error: '$REPOS_LIST' not found in root directory."
  exit 1
fi

# Handle private repos (only in local use)
if [[ "$CI" != "true" && "$PRIVATE_REPOS" != "No" ]]; then
  echo "🔐 Setting up credentials for private repos..."

  if [ -z "$PAT" ]; then
    read -p "Enter your GitHub Personal Access Token: " -s PAT
    echo
    echo "PAT='$PAT'" >> "$ENV_FILE"
    git config --global credential.helper store
    echo "https://$PAT:@github.com" > ~/.git-credentials
  fi

  if [ -z "$GIT_USER" ]; then
    read -p "Enter your Git username: " GIT_USER
    echo "GIT_USER='$GIT_USER'" >> "$ENV_FILE"
  fi

  if [ -z "$GIT_EMAIL" ]; then
    read -p "Enter your Git email: " GIT_EMAIL
    echo "GIT_EMAIL='$GIT_EMAIL'" >> "$ENV_FILE"
  fi
fi

# Set git user if available
if [ -n "$GIT_USER" ] && [ -n "$GIT_EMAIL" ]; then
  git config --global user.name "$GIT_USER"
  git config --global user.email "$GIT_EMAIL"
fi

# Create and enter src/
mkdir -p src
cd src

# Clone or pull repos
if [ -z "$(ls -A .)" ]; then
  echo "📦 Importing repositories from '$REPOS_LIST'..."
  vcs import . < "../$REPOS_LIST"
else
  echo "🔄 Updating existing repositories using vcs pull..."
  vcs pull
fi

if [ $? -ne 0 ]; then
  echo "❌ Error: vcs pull failed. Check access permissions."
  exit 1
fi

echo "✅ Repositories successfully cloned or updated."

# Add COLCON_IGNORE to simulation/testing packages
touch "${MAIN_DIR}/src/cartesian_controllers/cartesian_controller_simulation/COLCON_IGNORE" || true
touch "${MAIN_DIR}/src/cartesian_controllers/cartesian_controller_tests/COLCON_IGNORE" || true
