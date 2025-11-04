import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import random

# Create door node     
class Door(Node): 
    def __init__(self):
        super().__init__('door')

        # Create topic 
        self.publisher = self.create_publisher(Bool,'door_status',10)

        # publish current door status
        self.create_timer(0.5,self.publish_door_status)


    # Publish random events of door status
    def publish_door_status(self):
        msgs = Bool()
        msgs.data = random.choice([True,False])
        self.publisher.publish(msgs)
        print('Door Status:',msgs.data)

#    
def main():
        rclpy.init()
        node = Door()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()