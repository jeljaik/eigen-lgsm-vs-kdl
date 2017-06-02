#!/bin/bash
########### functions ###########################################
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
    cd ./libraries/$1
    dirExists build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=../../../install  ../
    if [ "$2" = 4 ]; then
        echo "I think second argument is 4"
        cmake -DCMAKE_INSTALL_PREFIX=../../../../install  ../
        make install -j 6
        cd ../../../../
    fi
    if [ -z "$2" ]; then
        cmake -DCMAKE_INSTALL_PREFIX=../../../install  ../
        make install -j 6
        cd ../../../ # Go back to project source
    fi
}

# Some coloring
RED='\033[0;31m'
NC='\033[0m' # No Color
function redPrint {
    printf "${RED}$1${NC}\n"
}
####################################################################

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
cmake ../
make -j 8
