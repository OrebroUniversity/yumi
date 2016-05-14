rosservice call /yumi/hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /yumi/hqp_vel_controller/reset_hqp_control 

#load the persistent tasks
rosservice call /yumi/hqp_vel_controller/load_tasks "task_definitions"

#set ee on plane
rosservice call /yumi/hqp_vel_controller/set_tasks '{tasks: [ {t_type: 1, priority: 2, is_equality_task: true, task_frame: "world", ds: 0.0, di: 0.02, dynamics: {d_type: 1, d_data: [-1.0]}, t_links: [{link_frame: "gripper_r_base", geometries: [{g_type: 1, g_data: [0, 0, 0.3]}]}, {link_frame: "gripper_l_base", geometries: [{g_type: 1, g_data: [0, 0, 0.3]}]}]} ]}'

#visualize the tasks
rosservice call /yumi/hqp_vel_controller/visualize_task_geometries '{ids: [13]}'

rosservice call /yumi/hqp_vel_controller/activate_hqp_control true


