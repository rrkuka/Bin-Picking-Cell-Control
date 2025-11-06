FROM ros:humble-ros-core
WORKDIR /app

# context = ./cell
COPY door_node/door.py             /app/door.py
COPY ebutton_node/ebutton.py       /app/ebutton.py
COPY scanner_node/scanner.py       /app/scanner.py
COPY stacklight_node/stacklight.py /app/stacklight.py

CMD ["bash","-lc",". /opt/ros/humble/setup.bash; \
  python3 /app/door.py & \
  python3 /app/ebutton.py & \
  python3 /app/scanner.py & \
  python3 /app/stacklight.py; \
  wait"]