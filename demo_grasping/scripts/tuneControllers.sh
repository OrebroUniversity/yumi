rosservice call /lwr/lwr_velvet_hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /lwr/lwr_velvet_hqp_vel_controller/reset_hqp_control 

#rosservice call /lwr_velvet/iiwa_hw_interface/set_stiffness "{sx: 1000.0, sy: 1000.0, sz: 1000.0, sa: 200.0, sb: 200.0, sc: 200.0}"

#load the persistent tasks
#rosservice call /lwr/lwr_velvet_hqp_vel_controller/load_tasks "task_definitions"

#visualize the loaded tasks
#rosservice call /lwr/lwr_velvet_hqp_vel_controller/visualize_task_geometries '{ids: [7, 8, 9, 10, 11, 12]}'

#set the joint setpoint tasks
rosservice call /lwr/lwr_velvet_hqp_vel_controller/set_tasks '{tasks: [{t_type: 3, priority: 2, name: joint_setpoints, is_equality_task: true, task_frame: "world", ds: 0.0, di: 1.0, dynamics: {d_type: 1, d_data: [-0.4]}, t_links: [{link_frame: "lwr_1_link", geometries: [{g_type: 6, g_data: [0.958]} ]}, {link_frame: "lwr_2_link", geometries: [{g_type: 6, g_data: [0.88]}]}, {link_frame: "lwr_3_link", geometries: [{g_type: 6, g_data: [-0.25]} ]}, {link_frame: "lwr_4_link", geometries: [{g_type: 6, g_data: [-1.068]} ]}, {link_frame: "lwr_5_link", geometries: [{g_type: 6, g_data: [2.9]} ]}, {link_frame: "lwr_6_link", geometries: [{g_type: 6, g_data: [0.2]} ]}, {link_frame: "lwr_7_link", geometries: [{g_type: 6, g_data: [0.35]} ]} ]} ]}'

rosservice call /lwr/lwr_velvet_hqp_vel_controller/activate_hqp_control true

