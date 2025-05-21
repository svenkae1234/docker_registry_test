# Setup

## Prerequisites

This workspace is primarily designed for Linux systems to connect to real hardware. For Windows, it is currently limited to simulation purposes only.

### Required

* [Docker](https://www.docker.com/) (for Linux and Windows)
* `docker-compose` or `podman-compose` (for Linux and Windows)

### Required for Windows Only

* [WSL 2](https://learn.microsoft.com/de-de/windows/wsl/install) with Ubuntu 24.04:
    * Verify your WSL version:
      ```bash
      wsl --list --verbose
      ```
      Ensure the output indicates `Version 2` and `Ubuntu 24.04`.
    * If WSL 2 or Ubuntu 24.04 is not configured, update WSL:
      ```bash
      wsl --set-default-version 2
      wsl --install -d Ubuntu-24.04
      ```

### Optional Tools

* [VSCode](https://code.visualstudio.com/) with the **Dev Containers Extension** (for Linux and Windows).
* A [Github PAT](https://github.blog/security/application-security/introducing-fine-grained-personal-access-tokens-for-github/) (Personal Access Token) with the following permissions:
    * Commit statuses
    * Contents
    * Metadata

---

## Instructions

1. Clone this repository (`dev-workspace`) to any directory on your system.
2. Open a terminal (depending on your OS):

    * **Linux:** Open your terminal and navigate to the directory:
      ```bash
      # Replace '/path/to/dev-workspace' with your actual directory
      cd /path/to/dev-workspace
      ```
    * **Windows:** Open the directory in WSL and navigate to the path:
      ```bash
      # Replace '<drive_letter>/<path_to_dev_workspace>' with your actual directory
      # e.g. /mnt/c/duatic/dev_workspace
      cd /mnt/<drive_letter>/<path_to_dev_workspace>
      ```
3. Check if [vcstool](https://github.com/dirk-thomas/vcstool) is installed:
    * To check:
      ```bash
      vcs --help
      ```
    * If not installed, follow the installation instructions given in the repository.
      
4. Move to the `scripts` directory and execute `clone_repos.sh` to clone the required repositories:
    ```bash
    cd scripts
    ./clone_repos.sh
    ```
    * This will clone all required repositories into the `src` directory. Or if you already have them it will pull the main branch. So call this script also when you want to have the newest updates.
5. Build the container image:
    ```bash
    ./build_image.sh
    ```
6. Manage and use the container:
    * Use the `manage_containers.sh` script to manage containers. Here’s how:
      * **Start and attach to a container (bash shell):**
        ```bash
        ./manage_containers.sh attach
        ```
      * **Start and attach VSCode to a container:**
        ```bash
        ./manage_containers.sh vscode
        ```
      * **Restart containers:**
        ```bash
        ./manage_containers.sh restart
        ```
      * **Stop all containers:**
        ```bash
        ./manage_containers.sh stop
        ```
      * **Remove all containers:**
        ```bash
        ./manage_containers.sh remove
        ```

    The script will detect your OS and manage the appropriate containers (`dynaarm-jazzy-dev-linux` for Linux or `dynaarm-jazzy-dev-windows` and `rosbridge-node` for Windows).
