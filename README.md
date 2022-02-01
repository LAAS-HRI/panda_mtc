# Planning requests
This node allows the supervisor to ask the planning of differents tasks using a ROS action server called "plan" (action files available in the pr2_motion_tasks_msgs package) :

- **Pick** with custom grasp pose task by providing the *name* of the action (action) which is a "pick", the *id* of the object (objId), the *arm* to be used (planGroup) and the *grasp pose* (pose). If no arm is defined, the one closer to the object side will be used.

- **Pick & Place** task by providing the *name* of the action (action) which is a "pickPlace", the *id* of the object (objId), the *arm* to be used (planGroup), and the *place pose* (pose). The grasp pose for the pick will be automatically generated.

- **Dual arm pick** task by providing the *name* of the action (action) which is a "pickDual", the *name* of the object (objId), the *arm* to be used (planGroup). The planGroup must be defined as first_arm+second_arm (the + 'plus' sign between the two plan group name is important.

- **Pick with auto-generated grasp pose** task by providing the *name* of the action (action) which is a "pickAuto", the *id* of the object (objId), the *arm* to be used (planGroup). The grasp pose for the pick will be automatically generated.

- **Place** task by providing the *name* of the action (action) which is a "place", the *pose* where to place the object (pose) and the *arm* to be used (planGroup).

- **Move** task by providing the *name* of the action (action) which is a "move", and either a *pose* (pose) as a ROS PoseStamped type, or the name of a pre-defined one (predefined_pose_id). The pre-defined pose can be defined using Moveit Setup Assistant or by directly editing the pr2_moveit_config/config/pr2.srdf file located in the moveit_pr2 repository.

- **Drop** task by providing the *name* of the action (action) which is a "drop", the *arm* to be used (planGroup), the *pose* where to put the object if automatically generated, the user only need to define the *frame_id*. 

Before a task is planned, the world handled by Moveit is udpated by asking Ontologenius about all the furnitures in the scene and all the objects that are on these furnitures and their meshes (defined as URI to dae files). The pose is then obtained by asking situation assesment service "GetPose" and providing the list of the objects ids. 

Action | Return value | Meaning 
------------ | ------------- | -------------
Plan a pick, place, move, drop | 1 | Success
Plan a pick, place, move, drop | -1 | Planning of the task failed (no solution or object wasn't on any surface, or planning cancelled by user) 
Plan a pick, place, move, drop | -6 | Update of the world failed (failed to get poses from situation assesment service)

A feedback containing a progress level as a number going from 0 to 100(%) to give info on the planning status.

# Execution requests
When supervisor asks for execution, no goal is provided because it will ask for the execution of the last planned task.

The following table recap the different return value that can be encountered when asking for the execution of a task.

Action | Return value | Meaning 
------------ | ------------- | -------------
Execute last task | 1 | Success
Execute last task | -2 | Execution of the task failed
Execute last task | -3 | Last planned task have no solutions

A feedback is returned during the execution process as an increasing number to show that the executing is still happening.

