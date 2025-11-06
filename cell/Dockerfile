FROM ros:humble-ros-core
WORKDIR /app
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install fastapi uvicorn httpx
COPY app/robot_api.py /app/robot_api.py
EXPOSE 8081
CMD ["bash","-lc",". /opt/ros/humble/setup.bash && python3 /app/robot_api.py"]