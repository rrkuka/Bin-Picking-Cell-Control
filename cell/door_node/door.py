import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random

# Create door node     
class Door(Node): 
    def __init__(self):
        super().__init__('door')

        # Create pub 
        self.publisher = self.create_publisher(Bool,'door_status',10)

        # Create timer that publishes every T=time
        self.create_timer(5.0,self.publish_door_status)


    # Publish random events of door status
    def publish_door_status(self):
        msgs = Bool()
        msgs.data = random.choice([True,False])
        self.publisher.publish(msgs)
        print('Door Status:',msgs.data)

    
def main():
        rclpy.init()
        node = Door()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()