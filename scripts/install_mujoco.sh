#!/bin/bash

# ==============================================================================
# MuJoCo Auto-Installer
#
# This script automatically finds the latest release of MuJoCo for Linux,
# downloads it, and extracts it to the ~/.mujoco directory.
#
# It checks for core dependencies (wget, tar, python3) and optional
# dependencies (curl, jq) for fetching the latest version, offering to install
# them if they are missing.
# ==============================================================================

set -e # Exit immediately if a command exits with a non-zero status.

# --- Helper Functions for Colored Output ---
info() {
    echo -e "\033[1;34m[INFO]\033[0m $1"
}

success() {
    echo -e "\033[1;32m[SUCCESS]\033[0m $1"
}

error() {
    echo -e "\033[1;31m[ERROR]\033[0m $1" >&2
    exit 1
}

warning() {
    echo -e "\033[1;33m[WARNING]\033[0m $1"
}

# --- 1. Dependency Management ---

# Generic function to install a list of packages
install_packages() {
    local packages_to_install=("$@")
    if [ ${#packages_to_install[@]} -eq 0 ]; then
        return
    fi

    # Detect package manager
    local pm_install=""
    local is_apt=false
    local is_pacman=false
    if command -v apt-get &> /dev/null; then
        pm_install="sudo apt-get install -y"
        is_apt=true
        info "Updating package lists with 'sudo apt-get update'..."
        sudo apt-get update || warning "Failed to update package lists. Continuing anyway."
    elif command -v dnf &> /dev/null; then
        pm_install="sudo dnf install -y"
    elif command -v yum &> /dev/null; then
        pm_install="sudo yum install -y"
    elif command -v pacman &> /dev/null; then
        pm_install="sudo pacman -S --noconfirm"
        is_pacman=true
    else
        error "Could not detect a supported package manager (apt, dnf, yum, pacman). Please install dependencies manually."
    fi

    # Handle special package names for different distros
    local final_package_list=()
    for pkg in "${packages_to_install[@]}"; do
        if [[ "$pkg" == "python3-venv-package" ]]; then
            if $is_apt; then
                final_package_list+=("python3-venv")
            fi
            # Other distros often bundle venv, so no separate package is added.
        elif [[ "$pkg" == "python3" ]]; then
             if $is_pacman; then
                final_package_list+=("python")
             else
                final_package_list+=("python3")
             fi
        else
            final_package_list+=("$pkg")
        fi
    done

    if [ ${#final_package_list[@]} -gt 0 ]; then
        info "Attempting to install: ${final_package_list[*]}"
        if ! $pm_install "${final_package_list[@]}"; then
            error "Failed to install dependencies: ${final_package_list[*]}. Please install them manually and re-run."
        fi
    fi
}

# Check for core dependencies that are absolutely required
check_core_dependencies() {
    local missing_deps=()
    local deps_to_check=("wget" "tar" "python3")

    info "Checking for core dependencies..."
    for dep in "${deps_to_check[@]}"; do
        if ! command -v "$dep" &> /dev/null; then
            missing_deps+=("$dep")
        fi
    done

    # Check if the python venv module is available by trying to import it
    if command -v python3 &> /dev/null && ! python3 -c "import venv" &> /dev/null; then
        warning "Python 'venv' module is missing."
        missing_deps+=("python3-venv-package")
    fi

    if [ ${#missing_deps[@]} -eq 0 ]; then
        success "Core dependencies are installed."
        return
    fi

    warning "The following essential dependencies are missing: ${missing_deps[*]}"
    read -p "They are required to continue. Do you want to install them now? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        error "User cancelled. Please install the core dependencies and re-run the script."
    fi

    install_packages "${missing_deps[@]}"
    success "Core dependencies installed."
}

# Check for optional dependencies and ask user if they want them
check_optional_dependencies() {
    local missing_deps=()
    local deps_to_check=("curl" "jq")

    info "Checking for optional tools for automatic version fetching..."
    for dep in "${deps_to_check[@]}"; do
        if ! command -v "$dep" &> /dev/null; then
            missing_deps+=("$dep")
        fi
    done

    if [ ${#missing_deps[@]} -eq 0 ]; then
        return 0 # Indicates tools are available
    fi

    warning "Optional tools are missing: ${missing_deps[*]}"
    echo "These are used to automatically find and download the latest MuJoCo version."
    read -p "Do you want to install them? (If not, a default version will be used) (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        info "Skipping installation of optional tools. Will use default MuJoCo version."
        return 1 # Indicates we should use defaults
    fi

    install_packages "${missing_deps[@]}"
    success "Optional dependencies installed."
    return 0 # Indicates tools are now available
}

# --- Run Dependency Checks ---
check_core_dependencies
if check_optional_dependencies; then
    USE_API=true
else
    USE_API=false
fi

# --- 2. Determine MuJoCo Version and Download URL ---
DEFAULT_VERSION="3.3.4"
DEFAULT_URL="https://github.com/google-deepmind/mujoco/releases/download/$DEFAULT_VERSION/mujoco-$DEFAULT_VERSION-linux-x86_64.tar.gz"

if [ -n "$1" ]; then
    VERSION="$1"
    info "Using specified MuJoCo version: $VERSION"
    DOWNLOAD_URL="https://github.com/google-deepmind/mujoco/releases/download/$VERSION/mujoco-$VERSION-linux-x86_64.tar.gz"
    USE_API=false # Override API check if version is specified on the command line
elif $USE_API; then
    info "Attempting to fetch the latest MuJoCo version using the GitHub API."
    API_URL="https://api.github.com/repos/google-deepmind/mujoco/releases/latest"
    
    LATEST_RELEASE_JSON=$(curl --connect-timeout 5 --retry 3 -s "$API_URL")
    
    if [ -n "$LATEST_RELEASE_JSON" ]; then
        VERSION=$(echo "$LATEST_RELEASE_JSON" | jq -r '.tag_name')
        DOWNLOAD_URL=$(echo "$LATEST_RELEASE_JSON" | jq -r '.assets[] | select(.name | endswith("linux-x86_64.tar.gz")) | .browser_download_url')

        if [ -z "$VERSION" ] || [ "$VERSION" == "null" ] || [ -z "$DOWNLOAD_URL" ] || [ "$DOWNLOAD_URL" == "null" ]; then
            warning "Could not determine the latest version from GitHub API. Falling back to default."
            VERSION="$DEFAULT_VERSION"
            DOWNLOAD_URL="$DEFAULT_URL"
        else
            info "Successfully found latest version: $VERSION"
        fi
    else
        warning "Could not fetch from GitHub API. Falling back to default."
        VERSION="$DEFAULT_VERSION"
        DOWNLOAD_URL="$DEFAULT_URL"
    fi
else
    info "Using default MuJoCo version: $DEFAULT_VERSION"
    VERSION="$DEFAULT_VERSION"
    DOWNLOAD_URL="$DEFAULT_URL"
fi

info "Selected MuJoCo version: \033[1;33m$VERSION\033[0m"
echo "Download URL: $DOWNLOAD_URL"
echo ""

# --- 3. Install if Necessary ---
MUJOCO_DIR="$HOME/.mujoco"
EXTRACTED_FOLDER_NAME="mujoco-$VERSION"
INSTALLED_DIR="$MUJOCO_DIR/$EXTRACTED_FOLDER_NAME"

if [ -d "$INSTALLED_DIR" ]; then
    success "MuJoCo $VERSION is already installed."
    info "Skipping download and extraction. Proceeding to check environment configuration."
else
    # --- 4. User Confirmation ---
    read -p "Do you want to proceed with downloading and installing MuJoCo $VERSION? (y/n) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        info "Installation cancelled by user."
        exit 0
    fi

    # --- 5. Download and Extract ---
    TEMP_ARCHIVE="/tmp/mujoco_latest.tar.gz"

    info "Creating installation directory at $MUJOCO_DIR"
    mkdir -p "$MUJOCO_DIR"

    info "Downloading MuJoCo $VERSION..."
    if ! wget -q --show-progress -O "$TEMP_ARCHIVE" "$DOWNLOAD_URL"; then
        # If download failed, provide a helpful suggestion if using defaults
        if ! $USE_API && [ -z "$1" ]; then
            error "Download failed. The default version '$DEFAULT_VERSION' might be outdated or its URL invalid."
            echo ""
            warning "SUGGESTION: Re-run this script and choose 'y' to install 'curl' and 'jq'."
            warning "This will allow the script to find the actual latest version of MuJoCo."
            exit 1
        else
            error "Download failed from URL: $DOWNLOAD_URL"
        fi
    fi

    info "Extracting files to $MUJOCO_DIR..."
    ACTUAL_FOLDER_NAME=$(tar -tf "$TEMP_ARCHIVE" | head -1 | cut -f1 -d"/")
    tar -xf "$TEMP_ARCHIVE" -C "$MUJOCO_DIR"
    EXTRACTED_FOLDER_NAME="$ACTUAL_FOLDER_NAME"

    # --- 6. Cleanup ---
    info "Cleaning up temporary files..."
    rm "$TEMP_ARCHIVE"
    
    success "MuJoCo $VERSION has been successfully installed in $MUJOCO_DIR/$EXTRACTED_FOLDER_NAME"
fi

# --- 7. Configure Environment ---
info "Configuring shell environment..."

SHELL_CONFIG_FILE="$HOME/.bashrc"
if [ -f "$HOME/.bashrc" ] || [ -f "$HOME/.zshrc" ]; then
    if [ -f "$HOME/.bashrc" ] && [ -f "$HOME/.zshrc" ]; then
        warning "Both .bashrc and .zshrc detected. Defaulting to .bashrc. Please ensure to add the variables to your preferred shell config if needed."
    elif [ -f "$HOME/.bashrc" ]; then
        info "Detected .bashrc, will append to .bashrc for configuration."
    elif [ -f "$HOME/.zshrc" ]; then
        SHELL_CONFIG_FILE="$HOME/.zshrc"
        info "Detected .zshrc, will append to .zshrc for configuration."
    fi
else
    error "No suitable shell configuration file found. Please create either .bashrc or .zshrc in your home directory."
fi

MUJOCO_VARS_BLOCK="\n# MuJoCo environment variables\nexport MUJOCO_PATH=\"\$HOME/.mujoco/$EXTRACTED_FOLDER_NAME\"\nexport LD_LIBRARY_PATH=\"\$LD_LIBRARY_PATH:\$MUJOCO_PATH/lib\"\nexport PATH=\"\$PATH:\$MUJOCO_PATH/bin\"\n"

if grep -q "export MUJOCO_PATH" "$SHELL_CONFIG_FILE" 2>/dev/null; then
    warning "MuJoCo environment variables seem to be already set in $SHELL_CONFIG_FILE."
    echo "Please check the file manually to ensure the path is correct for this version."
else
    info "Adding MuJoCo environment variables to $SHELL_CONFIG_FILE..."
    printf "$MUJOCO_VARS_BLOCK" >> "$SHELL_CONFIG_FILE"
    success "Environment variables added successfully."
fi

# --- 8. Final Instructions ---
echo ""
success "MuJoCo $VERSION setup complete. Environment variables have been configured."
echo ""
warning "ACTION REQUIRED: To complete the installation, run the following command or open a new terminal:"
echo "  source $SHELL_CONFIG_FILE"

