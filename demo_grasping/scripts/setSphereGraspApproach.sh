<<<<<<< HEAD
=======
#disable gravity in gazebo
rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

>>>>>>> 43f642bcee2079ada8f1669293e83f73cbad0633
rosservice call /lwr/lwr_velvet_hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /lwr/lwr_velvet_hqp_vel_controller/reset_hqp_control 

#load the persistent tasks
rosservice call /lwr/lwr_velvet_hqp_vel_controller/load_tasks "task_definitions"

#disable gravity in gazebo
rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

#set the joint setpoint tasks
rosservice call /lwr/lwr_velvet_hqp_vel_controller/set_tasks '{tasks: [{t_type: 1, priority: 2, name: "ee_inside_bigger_sphere", is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.02, dynamics: {d_type: 1, d_data: [-0.5]}, t_links: [{link_frame: "velvet_fingers_palm", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}, {link_frame: "world", geometries: [{g_type: 10, g_data: [-0.2, 0.2, 0.2, 0.2]}]}]}, {t_type: 1, priority: 2, name: "ee_outside_smaller_sphere", is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.02, dynamics: {d_type: 1, d_data: [-0.1]}, t_links: [{link_frame: "world", geometries: [{g_type: 10, g_data: [-0.2, 0.2, 0.2, 0.1]}]}, {link_frame: "velvet_fingers_palm", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}]}, {t_type: 5, priority: 2, name: "sphere_center_vertical", is_equality_task: false, task_frame: "world", ds: 0.0, di: 1, dynamics: {d_type: 1, d_data: [-1.0]}, t_links: [{link_frame: "world", geometries: [{g_type: 2, g_data: [-0.2, 0.2, 0.2, 0.0, 0.0, 1.0]} ]}, {link_frame: "velvet_fingers_palm", geometries: [{g_type: 8, g_data: [0, 0.0, 0, 1.0, 0.0, 0.0, 0.1]}]}]}, {t_type: 5, priority: 2, name: "sphere_center_horizontal", is_equality_task: false, task_frame: "world", ds: 0.0, di: 1, dynamics: {d_type: 1, d_data: [-1.0]}, t_links: [{link_frame: "world", geometries: [{g_type: 2, g_data: [-0.2, 0.2, 0.2, 1.0, 0.0, 0.0]} ]}, {link_frame: "velvet_fingers_palm", geometries: [{g_type: 8, g_data: [0, 0.0, 0, 1.0, 0.0, 0.0, 0.1]}]}]}, {t_type: 2, priority: 2, name: "gripper_lateral_axis_parallel_tabletop", is_equality_task: false, task_frame: "world", ds: 0.0, di: 10, dynamics: {d_type: 1, d_data: [-0.2]}, t_links: [{link_frame: "world", geometries: [{g_type: 3, g_data: [0, 0, 1, 0.5]}]}, {link_frame: "velvet_fingers_palm", geometries: [{g_type: 8, g_data: [0, 0, 0, 0, 1, 0, 0.1]}]}]} ]}'

#visualize the tasks
rosservice call /lwr/lwr_velvet_hqp_vel_controller/visualize_task_geometries '{ids: [7,8,9,10,11,12,13,14,15,16]}'

rosservice call /lwr/lwr_velvet_hqp_vel_controller/activate_hqp_control true
