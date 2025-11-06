import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, Int8, String

# Create door node     
class Stacklight(Node): 
    def __init__(self):
        super().__init__('stack_light')

        # Intial states
        self.estop_pressed = None
        self.door_open = None
        self.current_state = None

        # Create topic 
        self.stack_light_pub = self.create_publisher(Int8,'/stack_light_state',10)

        # Create subscribers for ebutton and door nodes
        self.create_subscription(Bool,'/door_status',self.door_msg,10)
        self.create_subscription(Bool,'/e_button_status',self.estop_msg,10)
        
        # create service to calling last barcode 
        self.create_service(Trigger, '/get_stacklight_state',self.stacklight_state)



    # Publish random events of door status
    def estop_msg(self,msgs):
        self.estop_pressed = msgs.data
        self.get_logger().info(f'E-stop: {self.estop_pressed}')
        self.update_state()


    def door_msg(self,msgs):
        self.door_open = msgs.data
        self.get_logger().info(f'Door: {self.door_open}')
        self.update_state()

    def stacklight_state(self, request, response):
        if self.current_state is None:
            response.success = False
            response.message = "unknown"   # no state
            return response

        response.success = True
        response.message = str(self.current_state)  # as string "-1", "0", or "1"
        return response
    
    def update_state(self):
         if self.door_open is None or self.estop_pressed is None:
              return None
         if self.estop_pressed:
              state = -1 #emergency 
         elif self.door_open:
              state = 1 #paused
         else:
              state = 0 #operational
         
         if state != self.current_state:
            self.current_state = state
            output = Int8()
            output.data = state
            self.stack_light_pub.publish(output)
            self.get_logger().info(f'Published stack light state: {state}')

#    
def main():
        rclpy.init()
        node = Stacklight()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()