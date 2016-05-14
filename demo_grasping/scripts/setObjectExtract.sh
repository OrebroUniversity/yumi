#disable gravity in gazebo
rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

rosservice call /yumi/hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /yumi/hqp_vel_controller/reset_hqp_control 

#load the persistent tasks
rosservice call /yumi/hqp_vel_controller/load_tasks "task_definitions"

#set the joint setpoint tasks
rosservice call /yumi/hqp_vel_controller/set_tasks '{tasks: [{t_type: 1, priority: 2, name: "ee_above_lower_plane", is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.02, dynamics: {d_type: 1, d_data: [-1.0]}, t_links: [{link_frame: "world", geometries: [{g_type: 3, g_data: [0, 0, 1, 0.3]}]}, {link_frame: "gripper_r_base", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}]}, {t_type: 2, priority: 2, name: "gripper_horizontal", is_equality_task: false, task_frame: "world", ds: 0.0, di: 1, dynamics: {d_type: 1, d_data: [-2.5]}, t_links: [{link_frame: "world", geometries: [{g_type: 8, g_data: [0, 0, 0, 0, 0, 1, 0.01]}]}, {link_frame: "gripper_r_base", geometries: [{g_type: 2, g_data: [0, 0, 0, 0, -1, 0]}]}]}, {t_type: 1, priority: 2, name: "ee_inside_larger_cylinder", is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.05, dynamics: {d_type: 1, d_data: [-0.5]}, t_links: [{link_frame: "gripper_r_base", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}, {link_frame: "world", geometries: [{g_type: 9, g_data: [0.45, 0.0, 0.115, 0, 0, 1, 0.11]}]}]} ]}'


#visualize the tasks
rosservice call /yumi/hqp_vel_controller/visualize_task_geometries '{ids: [7,8,9,10,11,12,13,14,15]}'

rosservice call /yumi/hqp_vel_controller/activate_hqp_control true

#
