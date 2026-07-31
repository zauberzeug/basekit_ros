"""
recon_dem_logger.py
────────────────────
Logs RTK-fixed position (lat, lon, alt) at fixed distance intervals during a
recon drive, for later interpolation into a DEM (see devkit_ui/terrain_mask.py
and repo issue #110).

Off by default — this is a recon-mode tool, not something that should log on
every normal mission. Enable per-recon-drive via the `recon_logging.enabled`
parameter.

Distance gating is computed from /odom (metric, local frame), not from
lat/lon deltas — cheaper and avoids doing geodesy just to decide whether to
log. Only points with an RTK-fixed status are kept (STATUS_GBAS_FIX, matching
the mapping rtk_navsatfix_shim.py already applies to /gnss/fix) — DEM quality
degrades fast on float/DGPS-quality altitude.

Subscriptions
─────────────
  /gnss/fix   sensor_msgs/NavSatFix   RTK-corrected position (see
                                       rtk_navsatfix_shim.py for status codes)
  /odom       nav_msgs/Odometry       local x,y for distance-interval gating

Output
──────
  CSV at recon_logging.output_path, columns: stamp,lat,lon,alt,x,y
"""

import csv
import os
from pathlib import Path

from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus


class ReconDEMLogger:
    """Log RTK-fixed points at fixed distance intervals for DEM building."""

    def __init__(self, node: Node):
        self.log = node.get_logger()
        self._node = node

        node.declare_parameter('recon_logging.enabled', False)
        node.declare_parameter('recon_logging.min_interval_m', 0.5)
        node.declare_parameter('recon_logging.output_path',
                                os.path.expanduser('~/recon_logs/recon.csv'))

        self._enabled = node.get_parameter('recon_logging.enabled').value
        self._min_interval_m = float(
            node.get_parameter('recon_logging.min_interval_m').value)
        self._output_path = Path(
            node.get_parameter('recon_logging.output_path').value)

        self._last_logged_xy: tuple[float, float] | None = None
        self._latest_fix: NavSatFix | None = None
        self._csv_file = None
        self._csv_writer = None

        if not self._enabled:
            self.log.info('ReconDEMLogger disabled (recon_logging.enabled=false)')
            return

        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        write_header = not self._output_path.exists()
        self._csv_file = open(self._output_path, 'a', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)
        if write_header:
            self._csv_writer.writerow(['stamp', 'lat', 'lon', 'alt', 'x', 'y'])
        self.log.info(f'ReconDEMLogger active, writing to {self._output_path} '
                       f'every {self._min_interval_m}m')

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(NavSatFix, '/gnss/fix', self._store_fix, sensor_qos)
        node.create_subscription(Odometry, '/odom', self._handle_odom, sensor_qos)

    def _store_fix(self, msg: NavSatFix) -> None:
        self._latest_fix = msg

    def _handle_odom(self, msg: Odometry) -> None:
        if not self._enabled:
            return
        if self._latest_fix is None:
            return
        if self._latest_fix.status.status < NavSatStatus.STATUS_GBAS_FIX:
            # Not RTK-fixed (see rtk_navsatfix_shim.py) — altitude isn't
            # trustworthy enough to feed a DEM.
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self._last_logged_xy is not None:
            dx = x - self._last_logged_xy[0]
            dy = y - self._last_logged_xy[1]
            if (dx * dx + dy * dy) ** 0.5 < self._min_interval_m:
                return

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._csv_writer.writerow([
            stamp,
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            self._latest_fix.altitude,
            x, y,
        ])
        self._csv_file.flush()
        self._last_logged_xy = (x, y)

    def close(self) -> None:
        if self._csv_file is not None:
            self._csv_file.close()
