# robot state constists of joint state/ joint values
# This message contains information about the robot state, i.e. the positions of its joints and links
sensor_msgs/JointState joint_state

#std_msgs/Float64MultiArray state_data
#std_msgs/Float64MultiArray control_data

# This messange containts predicted state data at one time interval eg. at time step=0.0, state_data=[....]
# # This messange containts predicted control data at one time interval eg. at time step=0.0, control_data=[....]
float64[] pd_data

# time step where we get data od state and control, optimization time where pd_state_data store, eg. pd_state_data[0] arrived at time 0.00 
float64[] pd_time
