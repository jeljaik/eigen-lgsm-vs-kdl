#!/bin/bash
########### functions ##########################################################
function cmakeConfigure {
    if [[ $OSTYPE == darwin* ]]; then
        echo "-G Xcode"
    else
      echo ""
    fi
}

function build-func {
    if [[ $OSTYPE == darwin* ]]; then
        eval "xcodebuild -j 8"
    else
      if [[ $OSTYPE == linux-gnu ]]; then
        eval "make -j8"
      fi
    fi
}

function install-func {
    if [[ $OSTYPE == darwin* ]]; then
        eval "xcodebuild -j 8 -target install"
    else
      if [[ $OSTYPE == linux-gnu ]]; then
        eval "make install -j8"
      fi
    fi
}

# Directory existence function
function dirExists {
    if [ ! -d "$1" ];then
        echo "$1 dir does not exist. Creating one for you..."
        mkdir $1
    else
        echo "$1 dir exists!"
    fi
}

# Install library
function installLib {
    redPrint " -- Installing $1"
    XCODEOPTION="$(cmakeConfigure)"
    echo "${XCODEOPTION}"
    cd ./libraries/$1
    dirExists build
    cd build
    if [ "$1" == googletest ]; then
        redPrint "Building tests by Google Test by default"
        cmake ${XCODEOPTION} -DCMAKE_INSTALL_PREFIX=../../../install -Dgtest_build_tests=ON ../
    else
        cmake ${XCODEOPTION} -DCMAKE_INSTALL_PREFIX=../../../install  ../
    fi
    if [ "$2" = 4 ]; then
        echo "I think second argument is 4"
        cmake ${XCODEOPTION} -DCMAKE_INSTALL_PREFIX=../../../../install  ../
        install-func
        cd ../../../../
    fi
    if [ -z "$2" ]; then
        cmake ${XCODEOPTION} -DCMAKE_INSTALL_PREFIX=../../../install  ../
        install-func
        cd ../../../ # Go back to project source
    fi
}

# Some coloring
RED='\033[0;31m'
NC='\033[0m' # No Color
function redPrint {
    printf "${RED}$1${NC}\n"
}
################################################################################

# Check existence of build directory
dirExists build

# Check existence of install directory
dirExists install

# Locally installing eigen
installLib eigen

# Locally install eigen_lgsm
installLib eigen_lgsm

# Locally install googletest
installLib googletest

# Locally install orocos_kdl
installLib orocos_kinematics_dynamics/orocos_kdl 4

# Configure and build project
redPrint " -- Configuring project"
cd build
XCODEOPTION="$(cmakeConfigure)"
cmake ${XCODEOPTION} ../
build-func
