"""Base class for the devkit driver handlers."""

import rclpy
from rclpy.node import Node


class Handler:
    """Common base for handlers that bridge RoSys hardware modules to ROS2 topics.

    Owns the references shared by every handler: the ROS node and its logger.
    """

    def __init__(self, node: Node) -> None:
        self.node = node
        self.log = node.get_logger()

    @property
    def active(self) -> bool:
        """Whether ROS is still running.

        During shutdown RoSys keeps emitting hardware events after rclpy has been shut down
        and the publishers have been destroyed. Guarding the event callbacks with this flag
        turns those late calls into no-ops instead of a flood of ``InvalidHandle`` errors.
        """
        return rclpy.ok()
