from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from trajectories import Trajectory

from .functions import draw_path


def visualize_trajectory(trajectory: Trajectory, uav_id: str):
    draw_path(
        points=[
            Pose(
                position=Point(*p.position),
                orientation=Quaternion(*quaternion_from_euler(0, 0, p.heading))
            )
            for p in trajectory
        ],
        topic=f"trajectories/{uav_id}",
        text=False
    )
