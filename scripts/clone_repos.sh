#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
MAIN_DIR="${SCRIPT_DIR}/.."
ENV_FILE="${SCRIPT_DIR}/.env"

# Load .env file if it exists
if [ -f "$ENV_FILE" ]; then
  source "$ENV_FILE"
  echo "Loaded from .env: PRIVATE_REPOS=$PRIVATE_REPOS, PAT=${PAT:+***}, GIT_USER=$GIT_USER"
fi

# Ensure vcs command is installed
if ! command -v vcs &> /dev/null; then
  echo "vcs command not found. Installing vcstool..."
  if [ ! -f /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  fi
  
  if ! grep -q "http://packages.ros.org/ros2/ubuntu" /etc/apt/sources.list.d/ros2.list 2>/dev/null; then
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  fi

  sudo apt update
  sudo apt install -y python3-vcstool

  if ! command -v vcs &> /dev/null; then
    echo "Error: Failed to install vcstool. Please check your setup."
    exit 1
  fi
fi

# Switch to the main directory
cd "${MAIN_DIR}"

# Ensure repos.list file exists
REPOS_LIST="repos.list"
if [ ! -f "$REPOS_LIST" ]; then
  echo "Error: '$REPOS_LIST' file not found in the current directory."
  exit 1
fi

# Check if PRIVATE_REPOS is set, otherwise ask
if [ -z "$PRIVATE_REPOS" ]; then
  read -p "Are there any private repositories to be cloned? (Y/N): " PRIVATE_REPOS
  PRIVATE_REPOS=${PRIVATE_REPOS,,}  # lowercase
  [[ "$PRIVATE_REPOS" == "y" || "$PRIVATE_REPOS" == "yes" ]] && PRIVATE_REPOS="Yes" || PRIVATE_REPOS="No"
  echo "PRIVATE_REPOS='$PRIVATE_REPOS'" >> "$ENV_FILE"
fi

if [[ "$PRIVATE_REPOS" == "Yes" ]]; then
  echo "Checking credentials..."

  # Check if PAT or SSH key is already set
  if [ -z "$PAT" ] && [ -z "$SSH_KEY" ]; then
    echo "No existing credentials found. Setting up credential manager..."
    echo "Choose your preferred authentication method:"
    echo "1. Personal Access Token (PAT)"
    echo "2. SSH Key"
    read -p "Enter your choice (1 or 2): " AUTH_CHOICE

    if [ "$AUTH_CHOICE" -eq 1 ]; then
      read -p "Enter your Personal Access Token: " -s PAT
      echo
      echo "PAT='$PAT'" >> "$ENV_FILE"
      git config --global credential.helper store
      echo "https://$PAT:@github.com" > ~/.git-credentials
      echo "Credential manager configured with Personal Access Token."
    elif [ "$AUTH_CHOICE" -eq 2 ]; then
      read -p "Enter the path to your SSH key (default: ~/.ssh/id_rsa): " SSH_KEY
      SSH_KEY=${SSH_KEY:-~/.ssh/id_rsa}
      if [ ! -f "$SSH_KEY" ]; then
        echo "Error: SSH key not found at $SSH_KEY. Please generate one or provide the correct path."
        exit 1
      fi
      eval "$(ssh-agent -s)"
      ssh-add "$SSH_KEY"
      echo "SSH_KEY='$SSH_KEY'" >> "$ENV_FILE"
      echo "SSH key added to the agent. Ensure your public key is uploaded to GitHub."
    else
      echo "Invalid choice. Exiting."
      exit 1
    fi
  else
    echo "Existing authentication method detected. Skipping credential setup."
  fi

  # Set up Git user details if missing
  if [ -z "$GIT_USER" ] || [ -z "$GIT_EMAIL" ]; then
    read -p "Enter your Git username: " GIT_USER
    read -p "Enter your Git email: " GIT_EMAIL
    git config --global user.name "$GIT_USER"
    git config --global user.email "$GIT_EMAIL"
    echo "GIT_USER='$GIT_USER'" >> "$ENV_FILE"
    echo "GIT_EMAIL='$GIT_EMAIL'" >> "$ENV_FILE"
  else
    echo "Git user details already set. Skipping."
  fi
fi

# Ensure 'src' directory exists
if [ ! -d "src" ]; then
  echo "Creating 'src' directory..."
  mkdir src
fi

cd src

# Prüfen, ob Repos vorhanden sind oder ob Import nötig ist
if [ -z "$(ls -A .)" ]; then
  echo "Cloning repositories from '$REPOS_LIST'..."
  vcs import . < "../$REPOS_LIST"
else
  echo "Repositories already exist. Updating repositories using vcs pull..."
  vcs pull
fi

if [ $? -ne 0 ]; then
  echo "Error: Failed to update repositories. Checking for credential issues..."
  echo "Please ensure you have the correct access rights to the repositories in '$REPOS_LIST'."
  exit 1
fi

echo "Repositories successfully cloned or updated."

