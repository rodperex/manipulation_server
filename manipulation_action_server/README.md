# manipulation_server
Manipulation subsystem for robot in ROS 2

## test :

```bash
ros2 action send_goal /pick manipulation_interfaces/action/Pick "object_goal:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: 'base_link'
  pose:
    position:
      x: 0.54
      y: 0.0
      z: 0.75
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  id: 'object'
  type:
    key: ''
    db: ''
  primitives: [{type: 3, dimensions: [0.14, 0.035]}]
  primitive_poses: []
  meshes: []
  mesh_poses: []
  planes: []
  plane_poses: []
  subframe_names: []
  subframe_poses: []
  operation: 0x01"
```