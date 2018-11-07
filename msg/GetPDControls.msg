# # This messange containts predicted control data at each time interval eg. at time step=0.0, control_data=[....]
RobotState[] pd_control_array

# This messange containts predicted control data at first time interval eg. at time step=0.0, control_data=[t=0.0]
# We called it as optimal control state
float64[] opt_control_state
