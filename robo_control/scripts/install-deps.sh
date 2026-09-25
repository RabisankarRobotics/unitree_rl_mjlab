#!/usr/bin/env bash
#
# install-deps.sh - Download and install project dependencies from GitHub
#
# Dependencies:
#   - ROS 2 (ros-<distro>-ros-base via apt, distro auto-detected from OS)
#   - CycloneDDS (Eclipse DDS implementation in C) - optional
#   - CycloneDDS C++ (C++ bindings for CycloneDDS) - optional
#   - MNN (Alibaba's lightweight deep learning framework)
#   - yaml-cpp (YAML parser and emitter for C++)
#   - spdlog (Fast C++ logging library)
#   - SOEM (Simple Open EtherCAT Master)
#   - GStreamer (Multimedia framework, installed via apt, 1.24.x)
#   - gst-plugins-rs (Rust GStreamer plugins: webrtc plugin)
#   - peel (Modern C++ bindings for GObject/GStreamer)
#
# Usage:
#   ./install-deps.sh                  # Install all dependencies
#   ./install-deps.sh --ros2                        # Install only ROS 2 (auto-detect distro)
#   ./install-deps.sh --ros2 --ros2-distro jazzy    # Install ROS 2 Jazzy explicitly
#   ./install-deps.sh --mnn --yaml-cpp # Install MNN and yaml-cpp
#   ./install-deps.sh --gstreamer --gst-plugins-rs --peel # Install GStreamer stack
#   ./install-deps.sh --prefix /opt    # Install to custom prefix
#
# Requirements:
#   - git, cmake (>= 3.16), C/C++ compiler (gcc/clang)
#   - For ROS 2: Ubuntu 22.04 (Jammy)
#   - For MNN: flatbuffers (optional, for schema generation)
#   - For gst-plugins-rs: Rust toolchain (installed automatically)
#   - For peel: meson, ninja (build system)

set -e
set -o pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default settings
PREFIX="/usr/local"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
WORKDIR=""
CLEANUP=true
SUDO_CMD=""

# Installation flags (0 = don't install, 1 = install)
INSTALL_ROS2=0
ROS2_DISTRO=""          # ROS 2 distro name (auto-detected from OS if empty)
INSTALL_CYCLONEDDS=0
INSTALL_CYCLONEDDS_CXX=0
INSTALL_MNN=0
INSTALL_YAML_CPP=0
INSTALL_SPDLOG=0
INSTALL_EIGEN=0
INSTALL_SOEM=0
INSTALL_GSTREAMER=0
INSTALL_GST_PLUGINS_RS=0
INSTALL_PEEL=0
INSTALL_ALL=0
NO_SUDO=0
REINSTALL=0

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

show_help() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Download and install project dependencies from GitHub.

By default (no flags), all dependencies are installed.

OPTIONS:
    --ros2              Install ROS 2 (distro auto-detected from Ubuntu codename)
    --ros2-distro NAME  Override ROS 2 distro (e.g. humble, jazzy, kilted)
    --cyclonedds        Install CycloneDDS (C core library) - not in default all
    --cyclonedds-cxx    Install CycloneDDS C++ bindings - not in default all
    --mnn               Install MNN (Alibaba ML framework)
    --yaml-cpp          Install yaml-cpp library
    --spdlog            Install spdlog (fast C++ logging library)
    --eigen             Install Eigen3 (header-only linear algebra, apt)
    --soem              Install SOEM (Simple Open EtherCAT Master)
    --gstreamer         Install GStreamer 1.24 (via apt, for media_service)
    --gst-plugins-rs    Install gst-plugins-rs (Rust plugins: webrtc)
    --peel              Install peel C++ bindings library (for media_service)
    --all               Install all dependencies (default if no flags)
    
    --prefix PATH       Installation prefix (default: /usr/local)
    --jobs N            Number of parallel build jobs (default: auto-detect)
    --no-cleanup        Keep the build directory after installation
    --no-sudo           Don't use sudo for installation (use if prefix is writable)
    --reinstall         Force reinstall even if the component is already installed

    -h, --help          Show this help message

EXAMPLES:
    # Install all dependencies to /usr/local
    $(basename "$0")
    
    # Install only ROS 2 (distro auto-detected)
    $(basename "$0") --ros2

    # Install ROS 2 Jazzy explicitly
    $(basename "$0") --ros2 --ros2-distro jazzy
    
    # Install MNN to a custom prefix
    $(basename "$0") --mnn --prefix \$HOME/.local
    
    # Install yaml-cpp with 8 parallel jobs
    $(basename "$0") --yaml-cpp --jobs 8

NOTES:
    - ROS 2 distro is auto-detected from Ubuntu codename (focal→foxy, jammy→humble, noble→jazzy, plucky→kilted)
    - Use --ros2-distro NAME to override the auto-detected distro
    - CycloneDDS/CycloneDDS C++ are optional (ROS 2 includes DDS)
    - CycloneDDS C++ requires CycloneDDS core to be installed first
    - sudo may be required if installing to system directories
    - Ensure git, cmake (>= 3.16), and a C/C++ compiler are installed

EOF
    exit 0
}

check_prerequisites() {
    log_info "Checking prerequisites..."
    
    local missing=()
    
    if ! command -v git &> /dev/null; then
        missing+=("git")
    fi
    
    if ! command -v cmake &> /dev/null; then
        missing+=("cmake")
    fi
    
    if ! command -v make &> /dev/null && ! command -v ninja &> /dev/null; then
        missing+=("make or ninja")
    fi
    
    # Check for C/C++ compiler
    if ! command -v gcc &> /dev/null && ! command -v clang &> /dev/null; then
        missing+=("gcc or clang")
    fi
    
    if [ ${#missing[@]} -ne 0 ]; then
        log_error "Missing required tools: ${missing[*]}"
        log_info "Please install them using your package manager:"
        log_info "  Ubuntu/Debian: sudo apt-get install git cmake build-essential"
        log_info "  Fedora/RHEL:   sudo dnf install git cmake gcc-c++ make"
        log_info "  Arch:          sudo pacman -S git cmake base-devel"
        exit 1
    fi
    
    # Check cmake version
    local cmake_version
    cmake_version=$(cmake --version | head -n1 | grep -oE '[0-9]+\.[0-9]+' | head -n1)
    local cmake_major cmake_minor
    cmake_major=$(echo "$cmake_version" | cut -d. -f1)
    cmake_minor=$(echo "$cmake_version" | cut -d. -f2)
    
    if [ "$cmake_major" -lt 3 ] || { [ "$cmake_major" -eq 3 ] && [ "$cmake_minor" -lt 16 ]; }; then
        log_warn "CMake version $cmake_version detected. Version 3.16+ recommended."
    fi
    
    log_success "All prerequisites satisfied"
}

setup_workdir() {
    WORKDIR=$(mktemp -d -t robo-deps-XXXXXX)
    log_info "Working directory: $WORKDIR"
    cd "$WORKDIR"
}

ensure_meson() {
    local min_version="1.1"

    if command -v meson &> /dev/null; then
        local cur_version
        cur_version="$(meson --version)"
        if printf '%s\n' "$min_version" "$cur_version" | sort -V | head -n1 | grep -qx "$min_version"; then
            log_info "Meson already available: $cur_version"
            return
        fi
        log_info "Meson $cur_version is too old (need >= $min_version), upgrading..."
    else
        log_info "Installing Meson build system..."
    fi

    # pipx installs into an isolated venv (PEP 668-safe) and gives us the latest version
    if ! command -v pipx &> /dev/null; then
        sudo apt-get install -y pipx
    fi
    pipx install --force meson
    # pipx installs to ~/.local/bin — make sure it is on PATH
    export PATH="$HOME/.local/bin:$PATH"
    log_info "Meson installed via pipx: $(meson --version)"
}

add_ros2_to_shell_rc() {
    local distro="$1"
    local setup_dir="/opt/ros/${distro}"

    # When the whole script is run under sudo, $HOME points at /root.
    # Fall back to the invoking user's home in that case so we update
    # the right rc file.
    local user_home="$HOME"
    if [ -n "${SUDO_USER:-}" ] && [ "$(id -u)" -eq 0 ]; then
        local sudo_home
        sudo_home=$(getent passwd "$SUDO_USER" | cut -d: -f6)
        [ -n "$sudo_home" ] && user_home="$sudo_home"
    fi

    # Pick which rc files to update: any that already exist, plus the
    # one matching $SHELL if neither is present yet.
    local -a rc_entries=()
    [ -f "$user_home/.bashrc" ] && rc_entries+=("$user_home/.bashrc:bash")
    [ -f "$user_home/.zshrc" ]  && rc_entries+=("$user_home/.zshrc:zsh")
    if [ ${#rc_entries[@]} -eq 0 ]; then
        case "${SHELL##*/}" in
            zsh) rc_entries+=("$user_home/.zshrc:zsh") ;;
            *)   rc_entries+=("$user_home/.bashrc:bash") ;;
        esac
    fi

    local entry rc_file shell_kind setup_script
    for entry in "${rc_entries[@]}"; do
        rc_file="${entry%:*}"
        shell_kind="${entry##*:}"
        setup_script="${setup_dir}/setup.${shell_kind}"

        if [ -f "$rc_file" ] && grep -Fq "$setup_script" "$rc_file"; then
            log_info "ROS 2 sourcing already present in $rc_file, skipping"
            continue
        fi

        log_info "Adding ROS 2 sourcing to $rc_file"
        {
            echo ""
            echo "# >>> ROS 2 ${distro} setup (added by install-deps.sh) >>>"
            echo "[ -f ${setup_script} ] && source ${setup_script}"
            echo "# <<< ROS 2 ${distro} setup <<<"
        } >> "$rc_file"
    done
}

cleanup_workdir() {
    if [ "$CLEANUP" = true ] && [ -n "$WORKDIR" ] && [ -d "$WORKDIR" ]; then
        log_info "Cleaning up working directory..."
        # Some install steps run under sudo and may leave root-owned files;
        # use sudo for cleanup when available.
        if [ -n "$SUDO_CMD" ]; then
            sudo rm -rf "$WORKDIR"
        else
            rm -rf "$WORKDIR"
        fi
    elif [ -n "$WORKDIR" ]; then
        log_info "Build directory preserved at: $WORKDIR"
    fi
}

# Trap to ensure cleanup on exit
trap cleanup_workdir EXIT

# Returns 0 (skip) if the component is already installed and --reinstall
# was not passed; returns 1 (proceed) otherwise.  Each remaining arg is a
# filesystem path or `cmd:<name>` token — if any of them resolves, the
# component is considered installed.
check_installed() {
    local name="$1"
    shift
    if [ "$REINSTALL" -eq 1 ]; then
        return 1
    fi
    local probe
    for probe in "$@"; do
        case "$probe" in
            cmd:*)
                if command -v "${probe#cmd:}" &> /dev/null; then
                    log_info "$name already installed (found command: ${probe#cmd:}), skipping. Use --reinstall to force."
                    return 0
                fi
                ;;
            pkg:*)
                if dpkg -s "${probe#pkg:}" &> /dev/null; then
                    log_info "$name already installed (dpkg: ${probe#pkg:}), skipping. Use --reinstall to force."
                    return 0
                fi
                ;;
            *)
                if [ -e "$probe" ]; then
                    log_info "$name already installed (found: $probe), skipping. Use --reinstall to force."
                    return 0
                fi
                ;;
        esac
    done
    return 1
}

# -----------------------------------------------------------------------------
# Installation functions
# -----------------------------------------------------------------------------

detect_ros2_distro() {
    if [ -n "$ROS2_DISTRO" ]; then
        log_info "Using specified ROS 2 distro: $ROS2_DISTRO"
        return
    fi

    if [ ! -f /etc/os-release ]; then
        log_warn "Cannot detect OS version, defaulting to ROS 2 Humble"
        ROS2_DISTRO="humble"
        return
    fi

    local codename
    # shellcheck source=/dev/null
    codename=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")

    case "$codename" in
        focal)   ROS2_DISTRO="foxy" ;;
        jammy)   ROS2_DISTRO="humble" ;;
        noble)   ROS2_DISTRO="jazzy" ;;
        plucky)  ROS2_DISTRO="kilted" ;;
        *)
            log_warn "Unknown Ubuntu codename '$codename', defaulting to ROS 2 Humble"
            ROS2_DISTRO="humble"
            ;;
    esac

    log_info "Auto-detected ROS 2 distro: $ROS2_DISTRO (Ubuntu $codename)"
}

install_ros2() {
    if check_installed "ROS 2 ${ROS2_DISTRO}" \
        "/opt/ros/${ROS2_DISTRO}/setup.bash" \
        "pkg:ros-${ROS2_DISTRO}-ros-base"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing ROS 2 ${ROS2_DISTRO}..."
    log_info "=========================================="

    # Check if running on Ubuntu
    if [ ! -f /etc/os-release ]; then
        log_error "Cannot detect OS. ROS 2 requires Ubuntu"
        return 1
    fi

    # shellcheck source=/dev/null
    . /etc/os-release

    if [ "$ID" != "ubuntu" ]; then
        log_error "ROS 2 deb packages are only available for Ubuntu"
        log_info "Detected OS: $ID"
        return 1
    fi

    local codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"

    # Warn if the detected codename doesn't match the expected one for this distro
    local expected_codename
    case "$ROS2_DISTRO" in
        foxy|galactic) expected_codename="focal" ;;
        humble|iron)   expected_codename="jammy" ;;
        jazzy)         expected_codename="noble" ;;
        kilted)        expected_codename="plucky" ;;
        *)             expected_codename="" ;;
    esac

    if [ -n "$expected_codename" ] && [ "$codename" != "$expected_codename" ]; then
        log_warn "ROS 2 $ROS2_DISTRO is designed for Ubuntu $expected_codename"
        log_warn "Detected: $codename. Installation may not work correctly."
    fi

    # Step 1: Set locale
    log_info "Setting up locale..."
    sudo apt-get update
    sudo apt-get install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Step 2: Setup sources
    log_info "Setting up ROS 2 apt sources..."
    sudo apt-get install -y software-properties-common curl
    sudo add-apt-repository -y universe

    # Get the latest ros-apt-source package version
    local ros_apt_version
    ros_apt_version=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')

    if [ -z "$ros_apt_version" ]; then
        log_error "Failed to fetch ROS apt source version"
        return 1
    fi

    log_info "Downloading ros2-apt-source version $ros_apt_version..."
    curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${codename}_all.deb"

    sudo dpkg -i /tmp/ros2-apt-source.deb
    rm -f /tmp/ros2-apt-source.deb

    # Step 3: Install ROS 2 packages
    log_info "Updating package index..."
    sudo apt-get update

    log_info "Upgrading system packages..."
    sudo apt-get upgrade -y

    log_info "Installing ros-${ROS2_DISTRO}-ros-base..."
    sudo apt-get install -y "ros-${ROS2_DISTRO}-ros-base"

    log_info "Installing ROS 2 development tools..."
    sudo apt-get install -y ros-dev-tools

    log_success "ROS 2 $ROS2_DISTRO installed successfully"
    log_info ""
    log_info "To use ROS 2 in this shell, source the setup script:"
    log_info "  source /opt/ros/${ROS2_DISTRO}/setup.bash"
}

install_cyclonedds() {
    if check_installed "CycloneDDS" \
        "$PREFIX/lib/cmake/CycloneDDS/CycloneDDSConfig.cmake" \
        "$PREFIX/lib64/cmake/CycloneDDS/CycloneDDSConfig.cmake"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing CycloneDDS..."
    log_info "=========================================="
    
    local repo_url="https://github.com/eclipse-cyclonedds/cyclonedds.git"
    local src_dir="$WORKDIR/cyclonedds"
    
    if [ -d "$src_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$src_dir"
        git pull
    else
        log_info "Cloning CycloneDDS repository..."
        git clone --depth 1 "$repo_url" "$src_dir"
        cd "$src_dir"
    fi
    
    log_info "Configuring CycloneDDS..."
    mkdir -p build && cd build
    
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF \
        -DENABLE_SSL=OFF
    
    log_info "Building CycloneDDS with $JOBS jobs..."
    cmake --build . --parallel "$JOBS"
    
    log_info "Installing CycloneDDS..."
    $SUDO_CMD cmake --build . --target install
    
    cd "$WORKDIR"
    log_success "CycloneDDS installed successfully"
}

install_cyclonedds_cxx() {
    if check_installed "CycloneDDS C++" \
        "$PREFIX/lib/cmake/CycloneDDS-CXX/CycloneDDS-CXXConfig.cmake" \
        "$PREFIX/lib64/cmake/CycloneDDS-CXX/CycloneDDS-CXXConfig.cmake"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing CycloneDDS C++..."
    log_info "=========================================="
    
    # Check if CycloneDDS core is available
    if [ ! -f "$PREFIX/lib/cmake/CycloneDDS/CycloneDDSConfig.cmake" ] && \
       [ ! -f "$PREFIX/lib64/cmake/CycloneDDS/CycloneDDSConfig.cmake" ]; then
        log_warn "CycloneDDS core not found at $PREFIX"
        log_info "CycloneDDS C++ requires CycloneDDS core. Installing it first..."
        install_cyclonedds
    fi
    
    local repo_url="https://github.com/eclipse-cyclonedds/cyclonedds-cxx.git"
    local src_dir="$WORKDIR/cyclonedds-cxx"
    
    if [ -d "$src_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$src_dir"
        git pull
    else
        log_info "Cloning CycloneDDS C++ repository..."
        git clone --depth 1 "$repo_url" "$src_dir"
        cd "$src_dir"
    fi
    
    log_info "Configuring CycloneDDS C++..."
    mkdir -p build && cd build
    
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_PREFIX_PATH="$PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_TESTING=OFF
    
    log_info "Building CycloneDDS C++ with $JOBS jobs..."
    cmake --build . --parallel "$JOBS"
    
    log_info "Installing CycloneDDS C++..."
    $SUDO_CMD cmake --build . --target install
    
    cd "$WORKDIR"
    log_success "CycloneDDS C++ installed successfully"
}

install_mnn() {
    if check_installed "MNN" \
        "$PREFIX/lib/libMNN.so" \
        "$PREFIX/include/MNN/MNNDefine.h"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing MNN..."
    log_info "=========================================="
    
    local repo_url="https://github.com/alibaba/MNN.git"
    local src_dir="$WORKDIR/MNN"
    
    if [ -d "$src_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$src_dir"
        git pull
    else
        log_info "Cloning MNN repository..."
        git clone --depth 1 "$repo_url" "$src_dir"
        cd "$src_dir"
    fi
    
    # Generate schema files if the script exists
    if [ -f "./schema/generate.sh" ]; then
        log_info "Generating MNN schema files..."
        chmod +x ./schema/generate.sh
        ./schema/generate.sh || log_warn "Schema generation skipped (may already be generated)"
    fi
    
    log_info "Configuring MNN..."
    mkdir -p build && cd build
    
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        -DMNN_BUILD_SHARED_LIBS=ON \
        -DMNN_BUILD_CONVERTER=OFF \
        -DMNN_BUILD_DEMO=OFF \
        -DMNN_BUILD_QUANTOOLS=OFF \
        -DMNN_BUILD_BENCHMARK=OFF \
        -DMNN_BUILD_TEST=OFF \
        -DMNN_OPENMP=ON \
        -DMNN_USE_SYSTEM_LIB=OFF
    
    log_info "Building MNN with $JOBS jobs..."
    cmake --build . --parallel "$JOBS"
    
    log_info "Installing MNN..."
    $SUDO_CMD cmake --build . --target install
    
    cd "$WORKDIR"
    log_success "MNN installed successfully"
}

install_yaml_cpp() {
    if check_installed "yaml-cpp" \
        "$PREFIX/include/yaml-cpp/yaml.h" \
        "$PREFIX/lib/cmake/yaml-cpp/yaml-cpp-config.cmake"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing yaml-cpp..."
    log_info "=========================================="
    
    local repo_url="https://github.com/jbeder/yaml-cpp.git"
    local src_dir="$WORKDIR/yaml-cpp"
    
    if [ -d "$src_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$src_dir"
        git pull
    else
        log_info "Cloning yaml-cpp repository..."
        git clone --depth 1 "$repo_url" "$src_dir"
        cd "$src_dir"
    fi
    
    log_info "Configuring yaml-cpp..."
    mkdir -p build && cd build
    
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        -DYAML_BUILD_SHARED_LIBS=ON \
        -DYAML_CPP_BUILD_TESTS=OFF \
        -DYAML_CPP_BUILD_TOOLS=OFF
    
    log_info "Building yaml-cpp with $JOBS jobs..."
    cmake --build . --parallel "$JOBS"
    
    log_info "Installing yaml-cpp..."
    $SUDO_CMD cmake --build . --target install
    
    cd "$WORKDIR"
    log_success "yaml-cpp installed successfully"
}

install_eigen() {
    if check_installed "Eigen3" \
        "pkg:libeigen3-dev" \
        "/usr/include/eigen3/Eigen/Core"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing Eigen3 (via apt)..."
    log_info "=========================================="

    if ! command -v apt-get &> /dev/null; then
        log_warn "apt-get not found; skipping Eigen3 (install libeigen3-dev manually)"
        return
    fi
    sudo apt-get install -y libeigen3-dev
    log_success "Eigen3 installed successfully"
}

install_spdlog() {
    if check_installed "spdlog" \
        "$PREFIX/include/spdlog/spdlog.h" \
        "$PREFIX/lib/cmake/spdlog/spdlogConfig.cmake"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing spdlog..."
    log_info "=========================================="
    
    local repo_url="https://github.com/gabime/spdlog.git"
    local src_dir="$WORKDIR/spdlog"
    
    if [ -d "$src_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$src_dir"
        git pull
    else
        log_info "Cloning spdlog repository..."
        git clone --depth 1 "$repo_url" "$src_dir"
        cd "$src_dir"
    fi
    
    log_info "Configuring spdlog..."
    mkdir -p build && cd build
    
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        -DSPDLOG_BUILD_SHARED=ON \
        -DSPDLOG_BUILD_EXAMPLE=OFF \
        -DSPDLOG_BUILD_TESTS=OFF \
        -DSPDLOG_BUILD_BENCH=OFF \
        -DSPDLOG_INSTALL=ON
    
    log_info "Building spdlog with $JOBS jobs..."
    cmake --build . --parallel "$JOBS"
    
    log_info "Installing spdlog..."
    $SUDO_CMD cmake --build . --target install
    
    cd "$WORKDIR"
    log_success "spdlog installed successfully"
}

install_soem() {
    if check_installed "SOEM" \
        "$PREFIX/include/soem/ethercat.h" \
        "$PREFIX/lib/cmake/soem/soemConfig.cmake"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing SOEM..."
    log_info "=========================================="
    
    local repo_url="https://github.com/OpenEtherCATsociety/SOEM.git"
    local src_dir="$WORKDIR/SOEM"
    
    if [ -d "$src_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$src_dir"
        git pull
    else
        log_info "Cloning SOEM repository..."
        git clone --depth 1 "$repo_url" "$src_dir"
        cd "$src_dir"
    fi
    
    log_info "Configuring SOEM..."
    mkdir -p build && cd build
    
    cmake .. \
        -DCMAKE_INSTALL_PREFIX="$PREFIX" \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_TESTS=OFF
    
    log_info "Building SOEM with $JOBS jobs..."
    cmake --build . --parallel "$JOBS"
    
    log_info "Installing SOEM..."
    $SUDO_CMD cmake --build . --target install
    
    cd "$WORKDIR"
    log_success "SOEM installed successfully"
}

install_gstreamer() {
    # Probe the GLib-2.0.gir XML peel-gen actually consumes — this is the
    # newest hard dep. Probing by file path (not dpkg name) is robust across
    # Ubuntu jammy (.gir ships in libglib2.0-dev) and noble (in
    # gir1.2-glib-2.0-dev), and prevents stale installs from short-circuiting
    # when we add new GIR deps. check_installed skips when ANY probe matches,
    # so list only the newest required artifact here.
    if check_installed "GStreamer 1.24" \
        "/usr/share/gir-1.0/GLib-2.0.gir"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing GStreamer 1.24 (via apt)..."
    log_info "=========================================="

    if ! command -v apt-get &> /dev/null; then
        log_error "apt-get not found; GStreamer install requires Ubuntu/Debian"
        return 1
    fi

    sudo apt-get update
    sudo apt-get install -y \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        libgstreamer-plugins-bad1.0-dev \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-nice \
        libgirepository1.0-dev \
        libglib2.0-dev \
        gir1.2-glib-2.0-dev \
        gir1.2-gstreamer-1.0

    log_success "GStreamer 1.24 installed successfully via apt"
}

install_gst_plugins_rs() {
    if check_installed "gst-plugins-rs (webrtc)" \
        "$PREFIX/bin/gst-webrtc-signalling-server"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing gst-plugins-rs (Rust GStreamer plugins)..."
    log_info "=========================================="

    # --- Rust toolchain ---
    # Verify cargo is functional, not just present. A rustup proxy with no
    # default toolchain configured will be on PATH but error to stderr and
    # produce empty `cargo --version` output, causing silent downstream
    # no-ops (cargo install / cargo cinstall return 0 without doing work).
    # NOTE: `set -e` + `set -o pipefail` make a bare assignment from a failing
    # command substitution abort the whole script. Append `|| true` so we can
    # detect failure via the empty-string check that follows, without aborting.
    local _cargo_version=""
    if command -v cargo &> /dev/null; then
        _cargo_version="$(cargo --version 2>/dev/null)" || true
    fi

    if [ -z "$_cargo_version" ]; then
        # Try to recover an existing rustup install before doing a fresh one
        if [ -f "$HOME/.cargo/env" ]; then
            log_info "Sourcing $HOME/.cargo/env and configuring default toolchain..."
            # shellcheck source=/dev/null
            . "$HOME/.cargo/env"
            if command -v rustup &> /dev/null; then
                rustup install stable
                rustup default stable
            fi
            _cargo_version="$(cargo --version 2>/dev/null)" || true
        fi
    fi

    if [ -z "$_cargo_version" ]; then
        log_info "Installing Rust toolchain via rustup..."
        curl https://sh.rustup.rs -sSf | sh -s -- -y --default-toolchain stable
        # shellcheck source=/dev/null
        . "$HOME/.cargo/env"
        _cargo_version="$(cargo --version 2>/dev/null)" || true
    fi

    if [ -z "$_cargo_version" ]; then
        log_error "Rust toolchain still not functional after install attempt."
        log_error "  cargo path: $(command -v cargo || echo NOT FOUND)"
        log_error "  rustup path: $(command -v rustup || echo NOT FOUND)"
        log_error "  rustup show: $(rustup show 2>&1 | head -n5)"
        return 1
    fi

    # Make sure ~/.cargo/bin is on PATH for cargo-c subcommands (and for
    # any sudo invocations that go through the `env PATH=...` SUDO_CMD).
    export PATH="$HOME/.cargo/bin:$PATH"
    log_info "Rust toolchain: $_cargo_version"

    # Ensure cargo-c is installed AND functional. The previous check (grep
    # for cbuild in `cargo --list`) only confirmed *some* cargo-c artifact
    # was registered; it would pass even with a half-installed cargo-c whose
    # `cargo cinstall` binary was missing or broken — observed in the wild
    # as a silent no-op (cinstall returns 0 and produces no files).
    local _cinstall_version
    _cinstall_version="$(cargo cinstall --version 2>/dev/null | head -n1)" || true
    if [ -z "$_cinstall_version" ] || ! command -v cargo-cinstall &> /dev/null; then
        log_info "Installing cargo-c (cargo cinstall not functional)..."
        if ! cargo install --force cargo-c; then
            log_error "cargo install --force cargo-c failed (non-zero exit)"
            return 1
        fi
        _cinstall_version="$(cargo cinstall --version 2>/dev/null | head -n1)" || true
        if [ -z "$_cinstall_version" ]; then
            log_error "cargo-c install completed but 'cargo cinstall --version' is still empty"
            log_error "  cargo-cinstall path: $(command -v cargo-cinstall || echo NOT FOUND)"
            log_error "  cargo --list (cargo-c subcommands):"
            cargo --list 2>&1 | grep -E '^\s*(c?build|c?install|capi|ctest)' \
                | while read -r l; do log_error "    $l"; done
            log_error "  ~/.cargo/bin entries:"
            ls -1 "$HOME/.cargo/bin/" 2>&1 | head -n 30 \
                | while read -r l; do log_error "    $l"; done
            return 1
        fi
    fi
    log_info "cargo-c functional: $_cinstall_version"

    # Install additional build dependencies
    log_info "Installing gst-plugins-rs build dependencies..."
    sudo apt-get install -y \
        libclang-dev \
        clang \
        libssl-dev \
        libnice-dev \
        libopus-dev \
        libvpx-dev \
        libsrtp2-dev

    # --- Clone gst-plugins-rs ---
    local gst_rs_dir="$WORKDIR/gst-plugins-rs"

    if [ -d "$gst_rs_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$gst_rs_dir"
        git pull
    else
        log_info "Cloning gst-plugins-rs repository..."
        git clone https://gitlab.freedesktop.org/gstreamer/gst-plugins-rs.git "$gst_rs_dir"
        cd "$gst_rs_dir"
        git checkout 0.13
    fi

    # Use a shared target directory outside the (potentially tmpfs-backed)
    # working directory to avoid running out of space during compilation.
    local cargo_target="${CARGO_TARGET_DIR:-$HOME/.cache/robo-deps-cargo-target}"
    mkdir -p "$cargo_target"
    export CARGO_TARGET_DIR="$cargo_target"

    # --- Build and install the webrtc plugin ---
    # cinstall implies cbuild, so skip the separate cbuild step to avoid
    # compiling twice and wasting disk space.
    # cd into the package directory and omit `-p`: cargo-c picks up the
    # local Cargo.toml unambiguously. Older cargo-c versions silently
    # no-op when `-p PKG` doesn't match the way they expect.
    log_info "Building and installing gst-plugin-webrtc with cargo-c..."
    cd "$gst_rs_dir/net/webrtc"
    # Install to a staging directory as the regular user, then copy with
    # sudo.  Running cargo cinstall directly under sudo would create
    # root-owned build artifacts in the temp directory, breaking cleanup.
    local _staging="$WORKDIR/_gst_rs_staging"
    mkdir -p "$_staging"
    cargo cinstall \
        --prefix="$PREFIX" \
        --destdir="$_staging" \
        --release \
        --verbose

    # cargo-c lays files out as "$_staging$PREFIX/...". Verify that path is
    # populated before copying — older cargo-c versions occasionally honor
    # destdir without prefix, and silent no-ops have been observed. If the
    # expected layout is missing but staging has *some* content, fall back to
    # copying the staging root.
    local _src=""
    if [ -d "$_staging$PREFIX" ] && [ -n "$(ls -A "$_staging$PREFIX" 2>/dev/null)" ]; then
        _src="$_staging$PREFIX"
    elif [ -d "$_staging" ] && [ -n "$(ls -A "$_staging" 2>/dev/null)" ]; then
        log_warn "cargo cinstall did not honor --prefix layout; copying $_staging directly"
        _src="$_staging"
    fi

    if [ -z "$_src" ]; then
        log_error "cargo cinstall produced no files in $_staging"
        log_error "Check cargo-c version: $(cargo cinstall --version 2>&1 | head -n1)"
        log_error "Staging contents (if any):"
        find "$_staging" -maxdepth 5 2>/dev/null | head -n 40
        return 1
    fi

    log_info "Copying staged files from $_src to $PREFIX..."
    $SUDO_CMD cp -a "$_src/." "$PREFIX/"

    # Free space before the next build
    rm -rf "$cargo_target"
    mkdir -p "$cargo_target"

    # --- Build and install the standalone signalling server ---
    log_info "Building gst-webrtc-signalling-server..."
    cd "$gst_rs_dir/net/webrtc/signalling"
    cargo build --release

    log_info "Installing gst-webrtc-signalling-server..."
    $SUDO_CMD mkdir -p "$PREFIX/bin"
    $SUDO_CMD cp "$cargo_target/release/gst-webrtc-signalling-server" \
        "$PREFIX/bin/gst-webrtc-signalling-server"
    $SUDO_CMD chmod 755 "$PREFIX/bin/gst-webrtc-signalling-server"

    # Clean up build artifacts to reclaim disk space
    rm -rf "$cargo_target"

    # Refresh shared library cache
    $SUDO_CMD ldconfig

    cd "$WORKDIR"
    log_success "gst-plugins-rs (webrtc plugin + signalling server) installed successfully"
}

install_peel() {
    if check_installed "peel" \
        "$PREFIX/bin/peel-gen" \
        "$PREFIX/lib/cmake/peel/peelConfig.cmake" \
        "$PREFIX/lib/x86_64-linux-gnu/cmake/peel/peelConfig.cmake"; then
        return 0
    fi

    log_info "=========================================="
    log_info "Installing peel (C++ GObject bindings)..."
    log_info "=========================================="

    # peel requires meson and ninja to build
    sudo apt-get install -y ninja-build
    ensure_meson

    local repo_url="https://gitlab.gnome.org/bugaevc/peel.git"
    local src_dir="$WORKDIR/peel"

    if [ -d "$src_dir" ]; then
        log_info "Source directory exists, pulling latest..."
        cd "$src_dir"
        git pull
    else
        log_info "Cloning peel repository..."
        git clone --depth 1 "$repo_url" "$src_dir"
        cd "$src_dir"
    fi

    log_info "Configuring peel with Meson..."
    meson setup build --prefix="$PREFIX" --buildtype=release

    log_info "Building peel with $JOBS jobs..."
    ninja -C build -j "$JOBS"

    log_info "Installing peel..."
    $SUDO_CMD ninja -C build install

    # Update ldconfig so the shared library is found
    if [ -n "$SUDO_CMD" ]; then
        $SUDO_CMD ldconfig
    fi

    cd "$WORKDIR"
    log_success "peel installed successfully"
    log_info "peel-gen and peel CMake config installed to: $PREFIX"
}

# -----------------------------------------------------------------------------
# Main script
# -----------------------------------------------------------------------------

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ros2)
            INSTALL_ROS2=1
            shift
            ;;
        --ros2-distro)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                log_error "--ros2-distro requires a distro name argument"
                exit 1
            fi
            ROS2_DISTRO="$2"
            shift 2
            ;;
        --cyclonedds)
            INSTALL_CYCLONEDDS=1
            shift
            ;;
        --cyclonedds-cxx)
            INSTALL_CYCLONEDDS_CXX=1
            shift
            ;;
        --mnn)
            INSTALL_MNN=1
            shift
            ;;
        --yaml-cpp)
            INSTALL_YAML_CPP=1
            shift
            ;;
        --spdlog)
            INSTALL_SPDLOG=1
            shift
            ;;
        --eigen)
            INSTALL_EIGEN=1
            shift
            ;;
        --soem)
            INSTALL_SOEM=1
            shift
            ;;
        --gstreamer)
            INSTALL_GSTREAMER=1
            shift
            ;;
        --gst-plugins-rs)
            INSTALL_GST_PLUGINS_RS=1
            shift
            ;;
        --peel)
            INSTALL_PEEL=1
            shift
            ;;
        --all)
            INSTALL_ALL=1
            shift
            ;;
        --prefix)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                log_error "--prefix requires a path argument"
                exit 1
            fi
            PREFIX="$2"
            shift 2
            ;;
        --jobs)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                log_error "--jobs requires a number argument"
                exit 1
            fi
            JOBS="$2"
            shift 2
            ;;
        --no-cleanup)
            CLEANUP=false
            shift
            ;;
        --no-sudo)
            NO_SUDO=1
            shift
            ;;
        --reinstall)
            REINSTALL=1
            shift
            ;;
        -h|--help)
            show_help
            ;;
        *)
            log_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# If no specific flags provided, install all
if [ $INSTALL_ROS2 -eq 0 ] && \
   [ $INSTALL_CYCLONEDDS -eq 0 ] && \
   [ $INSTALL_CYCLONEDDS_CXX -eq 0 ] && \
   [ $INSTALL_MNN -eq 0 ] && \
   [ $INSTALL_YAML_CPP -eq 0 ] && \
   [ $INSTALL_SPDLOG -eq 0 ] && \
   [ $INSTALL_EIGEN -eq 0 ] && \
   [ $INSTALL_SOEM -eq 0 ] && \
   [ $INSTALL_GSTREAMER -eq 0 ] && \
   [ $INSTALL_GST_PLUGINS_RS -eq 0 ] && \
   [ $INSTALL_PEEL -eq 0 ] && \
   [ $INSTALL_ALL -eq 0 ]; then
    INSTALL_ALL=1
fi

# If --all flag is set, enable all installations
# Note: CycloneDDS and CycloneDDS C++ are excluded from default all
# since ROS 2 Humble includes its own DDS implementation
if [ $INSTALL_ALL -eq 1 ]; then
    INSTALL_ROS2=1
    INSTALL_MNN=1
    INSTALL_YAML_CPP=1
    INSTALL_SPDLOG=1
    INSTALL_EIGEN=1
    INSTALL_SOEM=1
    INSTALL_GSTREAMER=1
    INSTALL_GST_PLUGINS_RS=1
    INSTALL_PEEL=1
fi

# Determine if sudo is needed for the install prefix
if [ $NO_SUDO -eq 0 ]; then
    # Check if we can write to the prefix directory
    # Capture user environment now, before sudo may change it.
    _USER_PATH="$PATH"
    _USER_RUSTUP_HOME="${RUSTUP_HOME:-$HOME/.rustup}"
    _USER_CARGO_HOME="${CARGO_HOME:-$HOME/.cargo}"

    if [ -d "$PREFIX" ]; then
        # Check both the prefix itself and key subdirectories (bin/, lib/)
        # which may have stricter permissions than the prefix root.
        if [ ! -w "$PREFIX" ] || \
           { [ -d "$PREFIX/bin" ] && [ ! -w "$PREFIX/bin" ]; } || \
           { [ -d "$PREFIX/lib" ] && [ ! -w "$PREFIX/lib" ]; }; then
            # Preserve PATH and Rust environment so user-local tools
            # (rustc, cargo, meson, etc.) remain available under sudo.
            SUDO_CMD="sudo env PATH=$_USER_PATH RUSTUP_HOME=$_USER_RUSTUP_HOME CARGO_HOME=$_USER_CARGO_HOME"
        fi
    else
        # Check if we can create the prefix directory
        PARENT_DIR=$(dirname "$PREFIX")
        if [ ! -w "$PARENT_DIR" ]; then
            SUDO_CMD="sudo env PATH=$_USER_PATH RUSTUP_HOME=$_USER_RUSTUP_HOME CARGO_HOME=$_USER_CARGO_HOME"
        fi
    fi
fi

if [ -n "$SUDO_CMD" ]; then
    log_info "Installation requires sudo privileges for prefix: $PREFIX"
fi

# Detect (or validate) the ROS 2 distro before printing config
[ $INSTALL_ROS2 -eq 1 ] && detect_ros2_distro

# Print configuration
echo ""
log_info "=========================================="
log_info "Installation Configuration"
log_info "=========================================="
log_info "Install prefix: $PREFIX"
log_info "Parallel jobs:  $JOBS"
log_info "Components to install:"
[ $INSTALL_ROS2 -eq 1 ] && log_info "  - ROS 2 $ROS2_DISTRO (ros-${ROS2_DISTRO}-ros-base)"
[ $INSTALL_CYCLONEDDS -eq 1 ] && log_info "  - CycloneDDS"
[ $INSTALL_CYCLONEDDS_CXX -eq 1 ] && log_info "  - CycloneDDS C++"
[ $INSTALL_MNN -eq 1 ] && log_info "  - MNN"
[ $INSTALL_YAML_CPP -eq 1 ] && log_info "  - yaml-cpp"
[ $INSTALL_SPDLOG -eq 1 ] && log_info "  - spdlog"
[ $INSTALL_EIGEN -eq 1 ] && log_info "  - Eigen3"
[ $INSTALL_SOEM -eq 1 ] && log_info "  - SOEM"
[ $INSTALL_GSTREAMER -eq 1 ] && log_info "  - GStreamer 1.24 (via apt)"
[ $INSTALL_GST_PLUGINS_RS -eq 1 ] && log_info "  - gst-plugins-rs (Rust GStreamer plugins + webrtc)"
[ $INSTALL_PEEL -eq 1 ] && log_info "  - peel (C++ GObject/GStreamer bindings)"
echo ""

# Check prerequisites
check_prerequisites

# Setup working directory
setup_workdir

# Run installations in dependency order
# ROS 2 Humble should be installed first
# CycloneDDS must be installed before CycloneDDS C++

if [ $INSTALL_ROS2 -eq 1 ]; then
    install_ros2
    add_ros2_to_shell_rc "$ROS2_DISTRO"
fi

if [ $INSTALL_CYCLONEDDS -eq 1 ]; then
    install_cyclonedds
fi

if [ $INSTALL_CYCLONEDDS_CXX -eq 1 ]; then
    install_cyclonedds_cxx
fi

if [ $INSTALL_MNN -eq 1 ]; then
    install_mnn
fi

if [ $INSTALL_YAML_CPP -eq 1 ]; then
    install_yaml_cpp
fi

if [ $INSTALL_SPDLOG -eq 1 ]; then
    install_spdlog
fi

if [ $INSTALL_EIGEN -eq 1 ]; then
    install_eigen
fi

if [ $INSTALL_SOEM -eq 1 ]; then
    install_soem
fi

# GStreamer must be installed before gst-plugins-rs and peel
if [ $INSTALL_GSTREAMER -eq 1 ]; then
    install_gstreamer
fi

# gst-plugins-rs depends on GStreamer being installed
if [ $INSTALL_GST_PLUGINS_RS -eq 1 ]; then
    install_gst_plugins_rs
fi

if [ $INSTALL_PEEL -eq 1 ]; then
    install_peel
fi

# Final summary
echo ""
log_info "=========================================="
log_success "All installations completed successfully!"
log_info "=========================================="
log_info "Libraries installed to: $PREFIX"
log_info ""
log_info "You may need to update your environment:"
log_info "  export LD_LIBRARY_PATH=$PREFIX/lib:\$LD_LIBRARY_PATH"
log_info "  export CMAKE_PREFIX_PATH=$PREFIX:\$CMAKE_PREFIX_PATH"
echo ""
