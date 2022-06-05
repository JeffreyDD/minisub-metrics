FROM ros:galactic

ADD . /opt/ros/minisub_ws/src/minisub_metrics
WORKDIR /opt/ros/minisub_ws
RUN apt update && apt install -y python3-pip
RUN . /opt/ros/galactic/setup.sh 
RUN rosdep update && rosdep install -y --from-paths src/minisub_metrics/
RUN pip install prometheus-client
RUN colcon build --packages-select minisub_metrics
#RUN echo ". /opt/ros/minisub_ws/install/local_setup.sh" >> ~/.bashrc

CMD ./src/minisub_metrics/start-exporter.sh