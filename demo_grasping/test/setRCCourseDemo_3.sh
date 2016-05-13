rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /apple/apple_hqp_vel_controller/reset_hqp_control 

rosservice call /apple/iiwa_hw_interface/set_stiffness "{sx: 1000.0, sy: 1000.0, sz: 1000.0, sa: 200.0, sb: 200.0, sc: 200.0}"

#load the persistent tasks
rosservice call /apple/apple_hqp_vel_controller/load_tasks "task_definitions"

#visualize the loaded tasks
rosservice call /apple/apple_hqp_vel_controller/visualize_task_geometries '{ids: [7, 8, 9, 10, 11, 12]}'

#set the joint setpoint tasks
rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{t_type: 3, priority: 2, name: joint_setpoints, is_equality_task: true, task_frame: "world", ds: 0.0, di: 1.0, dynamics: {d_type: 1, d_data: [-0.8]}, t_links: [{link_frame: "lbr_iiwa_link_1", geometries: [{g_type: 6, g_data: [1.274]} ]}, {link_frame: "lbr_iiwa_link_2", geometries: [{g_type: 6, g_data: [-2]}]}, {link_frame: "lbr_iiwa_link_3", geometries: [{g_type: 6, g_data: [-0.367]} ]}, {link_frame: "lbr_iiwa_link_4", geometries: [{g_type: 6, g_data: [-1.885]} ]}, {link_frame: "lbr_iiwa_link_5", geometries: [{g_type: 6, g_data: [-0.122]} ]}, {link_frame: "lbr_iiwa_link_6", geometries: [{g_type: 6, g_data: [0.91]} ]}, {link_frame: "lbr_iiwa_link_7", geometries: [{g_type: 6, g_data: [-1.483]} ]} ]} ]}'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control true
