#!/usr/bin/python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from yolov8_msgs.msg import DetectionArray, Detection, BoundingBox2D, BoundingBox3D
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
from manipulation_interfaces.action import Pick, MoveJoint, MoveToPredefined, Place, PickAndPlace


class ManipulationTest(Node):
    def __init__(self):
        super().__init__('detection_publisher_node')
        # self.publisher_ = self.create_publisher(CollisionObject, 'collition_object', 10)
        self.pick_client_ = ActionClient(self, Pick, 'pick')
        self.place_client_ = ActionClient(self, Place, 'place')
        self.move_joint_client_ = ActionClient(self, MoveJoint, 'move_joint')
        self.move_to_predefined_client_ = ActionClient(self, MoveToPredefined, 'move_to_predefined')
        self.pick_and_place_client_ = ActionClient(self, PickAndPlace, 'pick_and_place')

        self.timer = self.create_timer(1, self.publish_detection)
        


    def publish_detection(self):
        # collition object:

        self.get_logger().info('Published object')
        shape_msg = SolidPrimitive()
        shape_msg.type = SolidPrimitive.CYLINDER
        shape_msg.dimensions = [0.14, 0.025]
        obj_msg = CollisionObject()

        obj_msg.header.frame_id = "base_link"
        obj_msg.id = "test_object"
        obj_msg.primitives = [shape_msg]
        obj_msg.pose.position.x = 0.5
        obj_msg.pose.position.y = 0.0
        obj_msg.pose.position.z = 0.7

        # pick nd place test:         
        # if self.pick_and_place_client_.wait_for_server(2):
        #     request = PickAndPlace.Goal()
        #     request.object = obj_msg
        #     request.place_pose.header.frame_id = "base_link"
        #     request.place_pose.pose.position.x = 0.5
        #     request.place_pose.pose.position.y = 0.3
        #     request.place_pose.pose.position.z = 0.7
        #     request.place_pose.pose.orientation.w = 1.0
        #     future = self.pick_and_place_client_.send_goal_async(request)

        # pick then place:
        
        if self.pick_client_.wait_for_server(7):
            request = Pick.Goal()
            request.object_goal = obj_msg
            future = self.pick_client_.send_goal_async(request)

        # if self.place_client_.wait_for_server(2):
        #     request = Place.Goal()
        #     request.attached_object = obj_msg
        #     request.place_pose.header.frame_id = "base_link"
        #     request.place_pose.pose.position.x = 0.5
        #     request.place_pose.pose.position.y = 0.3
        #     request.place_pose.pose.position.z = 0.7
        #     request.place_pose.pose.orientation.w = 1.0
        #     future = self.place_client_.send_goal_async(request)

        # move joint test:
        # if self.move_joint_client_.wait_for_server(2):
        #     request = MoveJoint.Goal()
        #     request.joint_name = "arm_7_joint"
        #     request.joint_value = 0.0
        #     future = self.move_joint_client_.send_goal_async(request)



 


       
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
