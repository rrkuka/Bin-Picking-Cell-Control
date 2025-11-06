# WMS API
from fastapi import FastAPI
import httpx
import uvicorn
import json, time, rclpy
from std_msgs.msg import String

#setup ros nose and pub for hmi
rclpy.init(args=None)
wms_node = rclpy.create_node("wms_hmi")
req_pub  = wms_node.create_publisher(String, "request_info", 10)




# uncomment if running locally 
# CELL_PICK_URL = "http://localhost:8081/pick"
CELL_PICK_URL = "http://robot:8081/pick"
app = FastAPI()



# Send a pick request to the robot and display on HMI
@app.post("/pick")
def send_pick(req: dict):
    pick_id = int(req["pickId"])
    qty = int(req.get("quantity", 1))

    # publish to HMI first
    req_pub.publish(String(data=json.dumps({
        "ts": time.time(),
        "path": "/pick",
        "pickId": pick_id,
        "quantity": qty
    })))

    # then forward to the robot
    r = httpx.post(CELL_PICK_URL, json={"pickId": pick_id, "quantity": qty})
    return {"sent": True, "cell_status": r.status_code}

# Robot calls this to confirm
@app.post("/confirmPick")
def confirm_pick(payload: dict):
    print("Got confirmation:", payload)
    return {"received": True}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)

