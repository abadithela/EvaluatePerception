import rospy

from std_msgs.msg import Int32, Float64, String

class PlannerInterface():
    def __init__(self, robot_name) -> None:
        
        self.robot_name = robot_name
        
        self.target_abstract_speed = -255
        self.target_abstract_state = -255
        
        self.curr_abstract_speed = -255
        self.curr_abstract_state = -255
        
        self.state_pub = rospy.Publisher(f"{robot_name}/planner/abstract_state", Int32)
        self.speed_pub = rospy.Publisher(f"{robot_name}/planner/abstract_speed", Int32)
        
        self.state_sub = rospy.Subscriber(f"{robot_name}/abstract_state/pose", String, callback=self.vrpn_handler)
        
    def vrpn_handler(self, data:String):
        raw_in = data.data
        if "N/A" not in raw_in:
            self.curr_abstract_state = int(raw_in)
            

    def run(self):
        rate = rospy.Rate(10)
        while rospy.is_shutdown():
            abs_state = Int32()
            abs_state.data = self.target_abstract_state
            self.state_pub(abs_state)
            
            abs_speed = Int32()
            abs_speed.data = self.target_abstract_speed
            self.speed_pub(abs_speed)
            rate.sleep


if __name__ == "__main__":
    node = PlannerInterface("duck7")
    node.run()
    rospy.spin()