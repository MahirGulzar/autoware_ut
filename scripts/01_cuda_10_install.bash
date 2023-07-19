#!/bin/bash
set -e

echo ""
echo "Installing required prerequisites..."
echo ""

sudo apt-get -qqq update && sudo apt-get -qqq install curl lsb-release gnupg2

UBUNTU_VERSION=$(lsb_release -c -s)
EIGEN_VERSION=3.3.7

if [ -d "/tmp/cuda" ]; then
  rm -r /tmp/cuda
fi

mkdir /tmp/cuda && cd /tmp/cuda

echo ""
echo "Downloading installer..."
echo ""

if [ "$UBUNTU_VERSION" = "xenial" ]; then
  curl -L -o cuda_repo.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/cuda-repo-ubuntu1604_10.0.130-1_amd64.deb
  sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
elif [ "$UBUNTU_VERSION" = "bionic" ]; then
  curl -L -o cuda_repo.deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
  sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
  curl -s -L -o eigen-$EIGEN_VERSION.tar.gz https://gitlab.com/libeigen/eigen/-/archive/$EIGEN_VERSION/eigen-$EIGEN_VERSION.tar.gz
fi

if [ "$UBUNTU_VERSION" = "bionic" ]; then
  # Due to a bug in Eigen 3.3.5 (included with Melodic)
  # we must also update the Eigen version for CUDA projects
  # which use Eigen to build correctly (like some Autoware Packages)
  echo ""
  echo "Updating Eigen3 version to $EIGEN_VERSION..."
  echo ""

  sudo apt-get -qqq install build-essential cmake
  tar xf eigen-$EIGEN_VERSION.tar.gz
  cd eigen-*/
  mkdir build && cd build
  cmake ..
  sudo make install
  cd ../..
fi

echo ""
echo "Installing CUDA 10.0..."
echo ""

sudo dpkg -i cuda_repo.deb
sudo apt-get update -qq && sudo apt-get install -y cuda-10-0 cuda-nvcc-10-0

echo "Done!"
