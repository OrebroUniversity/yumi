rosservice call /yumi/hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /yumi/hqp_vel_controller/reset_hqp_control 

#load the persistent tasks
rosservice call /yumi/hqp_vel_controller/load_tasks "task_definitions"

#set the joint setpoint tasks
rosservice call /yumi/hqp_vel_controller/set_tasks '{tasks: [{t_type: 3, priority: 2, name: joint_setpoints, is_equality_task: true, task_frame: "world", ds: 0.0, di: 1.0, dynamics: {d_type: 1, d_data: [-0.8]}, t_links: [{link_frame: "yumi_link_1_r", geometries: [{g_type: 6, g_data: [0.42]} ]}, {link_frame: "yumi_link_2_r", geometries: [{g_type: 6, g_data: [-1.48]}]}, {link_frame: "yumi_link_3_r", geometries: [{g_type: 6, g_data: [-1.21]} ]}, {link_frame: "yumi_link_4_r", geometries: [{g_type: 6, g_data: [0.37]} ]}, {link_frame: "yumi_link_5_r", geometries: [{g_type: 6, g_data: [-3.41]} ]}, {link_frame: "yumi_link_6_r", geometries: [{g_type: 6, g_data: [-0.23]} ]}, {link_frame: "yumi_link_7_r", geometries: [{g_type: 6, g_data: [0.13]} ]} ]} ]}'

#visualize the tasks
rosservice call /yumi/hqp_vel_controller/visualize_task_geometries '{ids: [7, 8, 9]}'

rosservice call /yumi/hqp_vel_controller/activate_hqp_control true
