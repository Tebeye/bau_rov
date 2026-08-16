#!/usr/bin/env bash
# ==============================================================================
# BAU ROV - Raspberry Pi Tek Komutla Kurulum & Derleme Scripti
# ==============================================================================
# Kullanım:
#   chmod +x build.sh
#   ./build.sh          # Sadece derle ve çalıştır (kurulum zaten yapılmışsa)
#   ./build.sh --setup  # İlk kez: Tüm sistem kurulumu + derleme + çalıştırma
# ==============================================================================
set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros2_ws"
ROS_DISTRO="humble"

# Renkli çıktı
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log_info()  { echo -e "${GREEN}[✔] $1${NC}"; }
log_warn()  { echo -e "${YELLOW}[!] $1${NC}"; }
log_error() { echo -e "${RED}[✘] $1${NC}"; }

# ==============================================================================
# SETUP: İlk kez çalıştırma (--setup bayrağı ile)
# ==============================================================================
do_setup() {
    log_info "========== ADIM 1/6: Sistem Güncelleme =========="
    sudo apt update && sudo apt upgrade -y

    log_info "========== ADIM 2/6: Locale Ayarları =========="
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    log_info "========== ADIM 3/6: ROS 2 Humble Deposu Ekleniyor =========="
    sudo apt install -y software-properties-common curl gnupg lsb-release
    sudo add-apt-repository universe -y

    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    log_info "========== ADIM 4/6: ROS 2 + MAVROS + OpenCV Paketleri =========="
    sudo apt install -y \
        build-essential \
        cmake \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        python3-opencv \
        python3-numpy \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-mavros \
        ros-${ROS_DISTRO}-mavros-extras \
        geographiclib-tools

    log_info "========== ADIM 5/6: GeographicLib Veri Setleri =========="
    sudo bash "${PROJECT_DIR}/scripts/install_geographiclib_datasets.sh" || {
        log_warn "GeographicLib script başarısız oldu, alternatif yöntem deneniyor..."
        sudo geographiclib-get-geoids egm96-5 2>/dev/null || true
        sudo geographiclib-get-gravity egm96 2>/dev/null || true
        sudo geographiclib-get-magnetic emm2015 2>/dev/null || true
    }

    log_info "========== ADIM 6/6: Seri Port Yetkileri & Bashrc =========="
    sudo usermod -a -G dialout $USER

    # Bashrc'ye ROS 2 ortam değişkenlerini ekle (zaten yoksa)
    if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
    fi

    log_info "=========================================="
    log_info "  KURULUM TAMAMLANDI!"
    log_info "  Not: Seri port yetkileri için oturumu"
    log_info "  kapatıp açmanız gerekebilir."
    log_info "=========================================="
}

# ==============================================================================
# BUILD: Workspace Derleme
# ==============================================================================
do_build() {
    log_info "ROS 2 ortamı yükleniyor..."
    source /opt/ros/${ROS_DISTRO}/setup.bash

    log_info "Eski derleme dosyaları temizleniyor..."
    rm -rf "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/log"

    log_info "Paket derleniyor..."
    cd "${WORKSPACE_DIR}"
    colcon build --packages-select rov_line_tracking

    log_info "Ortam değişkenleri yükleniyor..."
    source "${WORKSPACE_DIR}/install/setup.bash"
    export AMENT_PREFIX_PATH="${WORKSPACE_DIR}/install/rov_line_tracking:${AMENT_PREFIX_PATH}"

    log_info "=========================================="
    log_info "  DERLEME BAŞARILI!"
    log_info "=========================================="
}

# ==============================================================================
# LAUNCH: Düğümleri Başlat
# ==============================================================================
do_launch() {
    log_info "ROV düğümleri başlatılıyor..."
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source "${WORKSPACE_DIR}/install/setup.bash"
    export AMENT_PREFIX_PATH="${WORKSPACE_DIR}/install/rov_line_tracking:${AMENT_PREFIX_PATH}"

    ros2 launch rov_line_tracking line_tracking.launch.py
}

# ==============================================================================
# ANA KONTROL
# ==============================================================================
print_usage() {
    echo ""
    echo "Kullanım: ./build.sh [SEÇENEK]"
    echo ""
    echo "  --setup     İlk kez kurulum (ROS 2, MAVROS, GeographicLib, vb.)"
    echo "  --build     Sadece derle"
    echo "  --launch    Sadece çalıştır (önceden derlenmiş olmalı)"
    echo "  --all       Kurulum + Derleme + Çalıştırma (ilk kullanım)"
    echo "  (boş)       Derle + Çalıştır (kurulum zaten yapılmışsa)"
    echo ""
}

case "${1}" in
    --setup)
        do_setup
        ;;
    --build)
        do_build
        ;;
    --launch)
        do_launch
        ;;
    --all)
        do_setup
        do_build
        do_launch
        ;;
    --help|-h)
        print_usage
        ;;
    *)
        # Varsayılan: derle + çalıştır
        do_build
        do_launch
        ;;
esac
