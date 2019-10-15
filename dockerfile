from wupanhao/duckie_base:v0.1
COPY ./ /demo_duck
RUN bash -c "source /opt/ros/kinetic/setup.bash && cd /demo_duck/catkin_ws && catkin_make_isolated"
CMD ["/bin/bash"]
