#!/usr/bin/env python3
from __future__ import annotations

import rospy
from tf.transformations import euler_from_quaternion

from mrs_visitor.msg import VisitAction, VisitGoal, VisitResult
from mrs_msgs.msg import TrajectoryReference, Reference, UavState
from mrs_msgs.srv import TrajectoryReferenceSrv
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from trajectories import sample_trajectory
from utils import ActionServer, ActionError
from free_space import FreeSpace
from visualization import visualize_space, visualize_goal, visualize_trajectory


class VisitorNode(ActionServer):
    def __init__(self):
        super().__init__("mrs_visitor", VisitAction)

        rospy.loginfo("Initializing...")

        # Create the space free from collisions
        self.free_space = FreeSpace(obstacles_filepath=rospy.get_param("~obstacles_file"),  # type: ignore
                                    bounds_filepath=rospy.get_param("~world_bounds_file"))  # type: ignore

        self.uav_id: str = rospy.get_param("~uav_id")

        rospy.loginfo(f"UAV ID: {self.uav_id}")

        # Create the publishers of trajectories. We'll change them on each new request
        self.trajectory_publisher = rospy.Publisher(
            f"~{self.uav_id}/visitor/trajectory",
            TrajectoryReference,
            queue_size=1,
            latch=True)

        # Create the services to call the control manager
        self.control_manager = rospy.ServiceProxy(
            f"{self.uav_id}/control_manager/trajectory_reference",
            TrajectoryReferenceSrv)

        # Publish the world bounds and obstacles to the RViz
        visualize_space(self.free_space)

        rospy.loginfo("Initialization done. Waiting for requests")

    def on_request(self, goal: VisitGoal):
        """Handler of the only request of this node"""

        # Update visualizations
        visualize_goal(goal)
        visualize_space(self.free_space)

        reference, dt, safety_distance = self._process_request(goal)
        self.free_space.safety_distance = safety_distance  # Not really needed as set in the self._process_request()

        # Subscribe to the odometry topics of each UAV to get their starting positions and headings
        start = self._get_start()

        rospy.loginfo(f"Request received. Solving trajectory")
        trajectory = sample_trajectory([start, reference], dt=dt)

        # Visualize trajectories
        visualize_trajectory(trajectory, self.uav_id)

        rospy.loginfo("Collisions resolved. Publishing trajectory...")

        # Publish the trajectories
        header = Header()
        header.stamp = rospy.Time.now()
        points = [Reference(Point(*p.position), p.heading) for p in trajectory]
        trajectory_reference = TrajectoryReference(header=header,
                                                   points=points,
                                                   dt=0.2,
                                                   fly_now=True,
                                                   use_heading=True)

        self.trajectory_publisher.publish(trajectory_reference)

        rospy.loginfo("Trajectory published and latched.")

        # Perform calls to the uav{id}/control_manager/trajectory_reference
        try:
            self.control_manager(trajectory_reference)
            rospy.loginfo("Trajectory sent to control manager.")
        except rospy.ServiceException as e:
            rospy.logerror(f"Could not call service {self.uav_id}/control_manager/trajectory_reference. Reason: {e}")

        # Return the result
        return VisitResult(success=True)

    def _process_request(self, goal: VisitGoal):
        reference = goal.reference
        safety_distance = goal.safety_distance
        dt = goal.dt

        self.free_space.safety_distance = safety_distance

        error_messages = []

        if reference not in self.free_space:
            position = reference.position
            x, y, z = position.x, position.y, position.z
            error_messages.append(f"Reference [{x:.2f}, {y:.2f}, {z:.2f}] is not in the free space")

        if dt <= 0:
            error_messages.append(f"dt must be positive, got {dt}")

        if safety_distance <= 0:
            error_messages.append(f"safety_distance must be positive, got {safety_distance}")

        if len(error_messages) > 0:
            raise ActionError("Error(s) in request:\n - " + "\n - ".join(error_messages))

        position = reference.position
        return (position.x, position.y, position.z, reference.heading), dt, safety_distance

    def _get_start(self):
        topic = f"{self.uav_id}/odometry/uav_state"
        rospy.loginfo(f"Subscribing to {topic}")

        try:
            odometry: UavState = rospy.wait_for_message(topic, UavState, timeout=1.0)  # type: ignore
            position = odometry.pose.position
            orientation = odometry.pose.orientation
            _, _, heading = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            return position.x, position.y, position.z, heading
        except rospy.exceptions.ROSException as e:
            raise ActionError(f"Could not get starting position for UAV {self.uav_id}") from e


if __name__ == "__main__":
    rospy.init_node("mrs_visitor")

    # Create the node, it should read the config, read the obstacles, and await request
    VisitorNode()

    rospy.spin()
