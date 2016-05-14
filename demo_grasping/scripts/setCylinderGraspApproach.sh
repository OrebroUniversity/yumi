#disable gravity in gazebo
rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

rosservice call /yumi/hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /yumi/hqp_vel_controller/reset_hqp_control 

#load the persistent tasks
rosservice call /yumi/hqp_vel_controller/load_tasks "task_definitions"

#set the joint setpoint tasks
rosservice call /yumi/hqp_vel_controller/set_tasks '{tasks: [{t_type: 1, priority: 2, name: "ee_below_upper_plane", is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.05, dynamics: {d_type: 1, d_data: [-0.5]}, t_links: [{link_frame: "world", geometries: [{g_type: 3, g_data: [0, 0, -1, -0.16]}]}, {link_frame: "gripper_r_base", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}]}, {t_type: 2, priority: 2, name: "gripper_horizontal", is_equality_task: false, task_frame: "world", ds: 0.0, di: 1, dynamics: {d_type: 1, d_data: [-1.0]}, t_links: [{link_frame: "world", geometries: [{g_type: 8, g_data: [0, 0, 0, 0, 0, 1, 0.005]}]}, {link_frame: "gripper_r_base", geometries: [{g_type: 2, g_data: [0, 0, 0, 0, -1, 0]}]}]}, {t_type: 5, priority: 2, name: "gripper_axis_coplanar_cylinder_axis", is_equality_task: true, task_frame: "world", ds: 0.0, di: 1, dynamics: {d_type: 1, d_data: [-1.0]}, t_links: [{link_frame: "world", geometries: [{g_type: 2, g_data: [0.45, 0.0, 0.115, 0.0, 0.0, 1.0]} ]}, {link_frame: "gripper_r_base", geometries: [{g_type: 8, g_data: [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.005]}]}]}, {t_type: 1, priority: 2, name: "ee_inside_larger_cylinder", is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.05, dynamics: {d_type: 1, d_data: [-0.25]}, t_links: [{link_frame: "gripper_r_base", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}, {link_frame: "world", geometries: [{g_type: 9, g_data: [0.45, 0.0, 0.115, 0, 0, 1, 0.11]}]}]}, {t_type: 1, priority: 2, name: "ee_outside_smaller_cylinder", is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.015, dynamics: {d_type: 1, d_data: [-0.1]}, t_links: [{link_frame: "world", geometries: [{g_type: 9, g_data: [0.45, 0.0, 0.115, 0, 0, 1, 0.06]}]}, {link_frame: "gripper_r_base", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}]} ]}'


#visualize the tasks
rosservice call /yumi/hqp_vel_controller/visualize_task_geometries '{ids: [7,8,9,10,11,12,13,14]}'

rosservice call /yumi/hqp_vel_controller/activate_hqp_control true

#
