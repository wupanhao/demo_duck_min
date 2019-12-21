# Note: this container will have the name wupanhao/demo_duck:v0.1
# docker build . -f dockerfile -t wupanhao/demo_duck:v0.1
FROM wupanhao/demo_duck:kinetic

WORKDIR /home/pi/workspace/demo_duck_min

COPY ./ ./

#RUN pip install ./


