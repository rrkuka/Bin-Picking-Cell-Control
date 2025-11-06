FROM ros:humble-ros-core
WORKDIR /app
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install fastapi uvicorn httpx
COPY app/wms_api.py /app/wms_api.py
EXPOSE 8080
CMD ["bash","-lc",". /opt/ros/humble/setup.bash && python3 /app/wms_api.py"]