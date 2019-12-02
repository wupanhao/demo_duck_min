# Note: this container will have the name wupanhao/duckie_base
# docker build . -f dockerfile -t wupanhao/duckie_base:v0.1
FROM ros:kinetic

#RUN export http_proxy=http://192.168.50.162:1080 && export https_proxy=http://192.168.50.162:1080

ENV http_proxy http://192.168.50.162:10809
ENV https_proxy http://192.168.50.162:10809

RUN apt update && apt install -y \
    libxslt-dev \
    libxml2-dev \
    libatlas-base-dev \
    libturbojpeg \
    python-frozendict \
    python-lxml \
    python-bs4 \
    python-tables \
    python-termcolor \
#    python-sklearn \
    python-dev \
    python-smbus \
    python-cffi \
#    python-skimage \
    python-pip \
    nano
#    ipython \
#    atop \
#    iftop \
#    ntpdate \
#    apt-file \

RUN apt install ros-kinetic-cv-bridge  ros-kinetic-image-transport ros-kinetic-tf ros-kinetic-geometry ros-kinetic-image-geometry libyaml-cpp-dev -y
#COPY requirements.txt /requirements.txt

RUN apt install -y ros-kinetic-rosbridge-server ros-kinetic-web-video-server ros-kinetic-joy 


#COPY ./caches/vc /opt/vc
#COPY ./caches/scipy-1.2.0-cp27-cp27mu-linux_armv7l.whl /tmp/
#COPY ./caches/00-vmcs.conf /etc/ld.so.conf.d/

# mkdir caches && cp -r /opt/vc /etc/ld.so.conf.d/00-vmcs.conf ./caches/
WORKDIR /demo_duck

RUN pip install --upgrade pip 

COPY . .

#COPY ./docker/publishers.py /opt/ros/kinetic/lib/python2.7/dist-packages/rosbridge_library/internal/publishers.py

#RUN mv ./caches/pip /usr/bin/pip && pip install ./caches/scipy-1.2.0-cp27-cp27mu-linux_armv7l.whl && \
#	 mv ./caches/vc /opt/vc && mv ./caches/00-vmcs.conf /etc/ld.so.conf.d/ && ldconfig && \
#	mv ./docker/publishers.py /opt/ros/kinetic/lib/python2.7/dist-packages/rosbridge_library/internal/publishers.py
#RUN pip install RPi.GPIO spidev picamera reprep ruamel.yaml==0.15.34 

# RUN pip install ./caches/scikit_learn-0.20.4-cp27-cp27mu-linux_armv7l.whl  ./caches/numpy-1.14.5-cp27-cp27mu-linux_armv7l.whl && \
#	pip install ./caches/tensorflow-1.14.0-cp27-none-linux_armv7l.whl ./caches/h5py-2.10.0-cp27-cp27mu-linux_armv7l.whl  keras

# otherwise installation of Picamera fails https://github.com/resin-io-projects/resin-rpi-python-picamera/issues/8
#ENV READTHEDOCS True
#RUN pip install -r /demo_duck/requirements.txt

#RUN mkdir /home/software
#COPY . /home/software/
#COPY docker/machines.xml /home/software/catkin_ws/src/00-infrastructure/duckietown/machines

# ENV ROS_LANG_DISABLE=gennodejs:geneus:genlisp
# RUN /bin/bash -c "cd /demo_duck/catkin_ws/ && source /opt/ros/kinetic/setup.bash &&  \
#     catkin_make_isolated -j4"
#     rosdep install --from-paths src --ignore-src -y &&  catkin_make_isolated -j4"

#RUN echo "source /home/software/docker/env.sh" >> ~/.bashrc

# We make sure that all dependencies are installed
# by trying to import the duckietown_utils package
#RUN bash -c "source /home/software/docker/env.sh && python -c 'import duckietown_utils'"

# Most of these will fail, but might be useful to debug some issues.
# Leave it here to run it manually.
# RUN bash -c "source /home/software/docker/env.sh && /home/software/what-the-duck"

CMD [ "/bin/bash" ]

#ENV DISABLE_CONTRACTS=1

#LABEL maintainer="Panhao Wu wupanhao@qq.com"
