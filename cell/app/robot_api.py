# Robot api
from fastapi import FastAPI
import uvicorn, httpx
import rclpy
from rclpy.node import Node
import json, time
from std_msgs.msg import String
from std_srvs.srv import Trigger
#uncomment when running locally
# WMS_CONFIRM_URL = "http://localhost:8080/confirmPick"
WMS_CONFIRM_URL = "http://wms:8080/confirmPick"
# ---- ROS: init once, reuse one node for all service calls ----
rclpy.init()
ros = Node("cell_client")

# create publisher to send request info to hmi
resp_pub = ros.create_publisher(String, "response_info", 10)

def svc(name):
    """Call a std_srvs/Trigger service and return (ok, msg)."""
    client = ros.create_client(Trigger, name)
    
    if not client.wait_for_service(timeout_sec=1.0):
        ros.destroy_client(client)
        return False, "service_unavailable"
    
    fut = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(ros, fut)
    ros.destroy_client(client)
    
    if fut.result() is None:
        return False, "no_response"
    res = fut.result()
    
    ok = bool(res.success)
    msg = str(res.message)
    return ok, msg

app = FastAPI()

@app.post("/pick")
def pick(data: dict):
    pick_id = int(data["pickId"])
    # 1) safety via stack-light
    ok, msg = svc("/get_stacklight_state")
    if not ok or msg == "unknown":
        httpx.post(WMS_CONFIRM_URL, json={"pickId": pick_id,"pickSuccessful": False,"errorMessage": "unknown stack state","itemBarcode": 0})
        return {"accepted": True}
    if msg.strip() == "-1":  # e-stop
        httpx.post(WMS_CONFIRM_URL, json={"pickId": pick_id,"pickSuccessful": False,"errorMessage": "e-button pressed","itemBarcode": 0})
        # publish estop state to hmi response
        resp_pub.publish(String(data=json.dumps({"ts": time.time(),"result":"fail","confirm":{"pickId": pick_id,"pickSuccessful": False,"errorMessage":"e-button pressed","itemBarcode":0}})))
        return {"accepted": True}
    if msg.strip() == "1":   # door open
        httpx.post(WMS_CONFIRM_URL, json={"pickId": pick_id,"pickSuccessful": False,"errorMessage": "door open","itemBarcode": 0})
        # publish door state to hmi response
        resp_pub.publish(String(data=json.dumps({"ts": time.time(),"result":"fail","confirm":{"pickId": pick_id,"pickSuccessful": False,"errorMessage":"door open","itemBarcode":0}})))
        return {"accepted": True}

    # 2) barcode from scanner
    ok, code = svc("/last_barcode")
    code = code.strip()
    item_barcode = int(code) if ok and code.isdigit() else 0
    pick_success = item_barcode != 0

    # 3) confirm back to WMS and post to HMI
    httpx.post(WMS_CONFIRM_URL, json={
    "pickId": pick_id,
    "pickSuccessful": pick_success,
    "errorMessage": None if pick_success else "no barcode",
    "itemBarcode": item_barcode
})
    # Robot response to hmi
    resp_pub.publish(String(data=json.dumps({
        "ts": time.time(),
        "result": "ok" if pick_success else "fail",
        "confirm": {
            "pickId": pick_id,
            "pickSuccessful": pick_success,
            "errorMessage": None if pick_success else "no barcode",
            "itemBarcode": item_barcode
        }
    })))
    return {"accepted": True}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8081)







