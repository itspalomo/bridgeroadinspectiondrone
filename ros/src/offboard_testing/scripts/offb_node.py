#! /usr/bin/env python

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rclpy.init_node("offb_node_py")

    state_sub = rclpy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rclpy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rclpy.wait_for_service("/mavros/cmd/arming")
    arming_client = rclpy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rclpy.wait_for_service("/mavros/set_mode")
    set_mode_client = rclpy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rclpy.Rate(20)

    # Wait for Flight Controller connection
    while(not rclpy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rclpy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rclpy.Time.now()

    while(not rclpy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rclpy.Time.now() - last_req) > rclpy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rclpy.loginfo("OFFBOARD enabled")
            
            last_req = rclpy.Time.now()
        else:
            if(not current_state.armed and (rclpy.Time.now() - last_req) > rclpy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rclpy.loginfo("Vehicle armed")
            
                last_req = rclpy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()