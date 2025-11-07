# Bin-Picking Cell Control
A self-contained **ROS 2 + FastAPI + WebHMI** demo showing an automated bin-picking workflow.  
Everything runs in Docker — no manual ROS setup required.

---

## Requirements

Before running, install:
- [Docker Desktop](https://www.docker.com/products/docker-desktop/)


Once installed, clone this repo and run the commands below.
- git clone https://github.com/rrkuka/Bin-Picking-Cell-Control.git

---

##  Quick Start

### 1. Build and Run
```bash
docker compose up -d --build
```
Wait ~30 seconds for all containers to start.

Verify:
```bash
docker compose ps
# expect: nodes, robot, wms, rosbridge, webhmi → Up
```

---

### 2. Open the HMI
Open your browser to:  
**http://localhost:8089**

You’ll see live ROS 2 data for:
- Door status  
- E-stop status  
- Pick Request / Response  

---

### 3. Send a Test Pick
Run this in your terminal:
```bash
curl -X POST http://localhost:8080/pick   -H "Content-Type: application/json"   -d '{"pickId":145,"quantity":2}'
```
The HMI will update both **Request** and **Response** panels.

---
