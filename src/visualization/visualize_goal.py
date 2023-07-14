from __future__ import annotations

from mrs_visitor.msg import VisitGoal
from std_msgs.msg import ColorRGBA

from .functions import draw_shapes


def visualize_goal(goal: VisitGoal):

    draw_shapes(
        points=[goal.reference.position],
        shape="sphere",
        topic="inspection_points",
        color=ColorRGBA(0.3, 0.8, 0.6, 1.0),
        scale=0.5,
    )
