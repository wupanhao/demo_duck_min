#!/bin/bash
set -e

set -x

apt update && apt install -y \
    libxslt-dev \
    libxml2-dev \
    apt-file \
    iftop \
    atop \
    ntpdate \
    libatlas-base-dev \
    ipython \
    python-dev \
    python-lxml \
    python-bs4 \
    python-tables \
    python-sklearn \
    python-termcolor \
    python-sklearn \
    python-smbus \
    libturbojpeg \
    python-frozendict \
    python-cffi \
    python-skimage

apt remove -y \
	python-ruamel.yaml \
	python-ruamel.ordereddict

# These don't have an APT package

pip install --user --upgrade -r requirements.txt
pip install --user jpeg4py

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel

# apt install ros-kinetic-compressed-image-transport ros-kinetic-usb-cam
