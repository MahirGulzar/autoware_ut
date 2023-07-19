#!/bin/bash

# Looking for nvcc from CUDA
if [ -d "/opt/cuda/bin" ]; then
  find /opt/cuda/bin -name nvcc | egrep '.*' > /dev/null 2>&1
  CUDA_RETCODE1=$?
else
  CUDA_RETCODE1=100
fi

if [ -d "/usr/local/bin" ]; then
  find /usr/local/bin -name nvcc | egrep '.*' > /dev/null 2>&1
  CUDA_RETCODE2=$?
else
  CUDA_RETCODE2=100
fi

if [ -d "/usr/local/cuda/bin" ]; then
  find /usr/local/cuda/bin -name nvcc | egrep '.*' > /dev/null 2>&1
  CUDA_RETCODE3=$?
else
  CUDA_RETCODE3=100
fi

if [ "$CUDA_RETCODE1" -eq 0 ] || [ "$CUDA_RETCODE2" -eq 0 ] || [ "$CUDA_RETCODE3" -eq 0 ]; then
  CUDA_FOUND=true
else
  CUDA_FOUND=false
fi

echo ""
echo "CUDA found: $CUDA_FOUND"
echo ""

grep machine-learning /etc/apt/sources.list.d/* > /dev/null 2>&1
ML_REPO_RETCODE=$?

set -e

sudo apt-get -qq update && sudo apt-get -qq --allow-change-held-packages install wget lsb-release

CODENAME=$(lsb_release -c -s)

if [ "$CODENAME" != "xenial" -a "$CODENAME" != "bionic" ]; then
  echo "Your operating system version is not supported by this script."
  exit 10
fi

if [ "$CUDA_FOUND" = true ]; then
  if [ "$CODENAME" == "xenial" ]; then
    if [ "$ML_REPO_RETCODE" != "0" ]; then
      echo ""
      echo "NVIDIA Machine Learning repo not found - adding..."
      echo ""
      wget -O /tmp/nvidia-ml-repo.deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64/nvidia-machine-learning-repo-ubuntu1604_1.0.0-1_amd64.deb
      sudo apt-get -qq --allow-change-held-packages --allow-change-held-packages install /tmp/nvidia-ml-repo.deb
    else
      echo ""
      echo "NVIDIA Machine Learning repo found. Continuing..."
      echo ""
    fi

    sudo apt-get -qq update
    sudo apt-get -qq --no-install-recommends --allow-downgrades --allow-change-held-packages install \
      libcudnn7=7.6.3.30-1+cuda10.0 \
      libcudnn7-dev=7.6.3.30-1+cuda10.0 \
      nvinfer-runtime-trt-repo-ubuntu1604-5.0.2-ga-cuda10.0
    sudo apt-get -qq update
    sudo apt-get -qq --no-install-recommends --allow-downgrades --allow-change-held-packages install \
      libnvinfer6=6.0.1-1+cuda10.0 \
      libnvinfer-dev=6.0.1-1+cuda10.0
    sudo apt-mark hold libcudnn7 libcudnn7-dev libnvinfer6 libnvinfer-dev
  elif [ "$CODENAME" == "bionic" ]; then
    if [ "$ML_REPO_RETCODE" != "0" ]; then
      echo ""
      echo "NVIDIA Machine Learning repo not found - adding...."
      echo ""
      wget -O /tmp/nvidia-ml-repo.deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/nvidia-machine-learning-repo-ubuntu1804_1.0.0-1_amd64.deb
      sudo apt-get -qq --allow-change-held-packages install /tmp/nvidia-ml-repo.deb
    else
      echo ""
      echo "NVIDIA Machine Learning repo found. Continuing..."
      echo ""
    fi

    sudo apt-get -qq update
    version="7.0.0-1+cuda10.0"
    sudo apt-get -qq --no-install-recommends --allow-downgrades --allow-change-held-packages install \
      libcudnn7=7.6.3.30-1+cuda10.0 \
      libcudnn7-dev=7.6.3.30-1+cuda10.0 \
     libnvinfer7=${version} libnvonnxparsers7=${version} libnvparsers7=${version} libnvinfer-plugin7=${version} libnvinfer-dev=${version} libnvonnxparsers-dev=${version} libnvparsers-dev=${version} libnvinfer-plugin-dev=${version} python-libnvinfer=${version}
    sudo apt-mark hold libnvinfer7 libnvonnxparsers7 libnvparsers7 libnvinfer-plugin7 libnvinfer-dev libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev python-libnvinfer libcudnn7 libcudnn7-dev
  fi
fi

echo ""
echo "Installing TensorFlow prerequisites."
echo ""

sudo apt-get -qq update
sudo apt-get -qq --no-install-recommends --allow-change-held-packages install \
  python-pip \
  python-dev \
  python-wheel \
  python-setuptools
pip install --user --upgrade -qqq setuptools
# 16.04 requires an older version of numpy
pip install --user -qqq numpy==1.16.5

if [ "$CUDA_FOUND" = true ]; then
  echo ""
  echo "CUDA support packages installed successfully. Installing TensorFlow with GPU support."
  echo ""
  pip install --user -qqq protobuf==3.17.3 tensorflow-gpu
  echo "Tensorflow installed!"
else
  echo ""
  echo "CUDA is not installed. Installing TensorFlow without GPU support."
  echo ""
  pip install --user -qqq protobuf==3.17.3 tensorflow
  echo "Tensorflow installed!"
fi
