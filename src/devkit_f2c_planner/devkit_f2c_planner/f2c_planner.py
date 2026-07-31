"""
f2c_planner.py
──────────────
Field2Cover planning core: lat/lon <-> local-XY projection and the swath
generator. Standalone package (devkit_f2c_planner) with no NiceGUI/UI
dependency, so any caller can use it.

Originally lived inside devkit_ui/ui_node.py, coupled to the manual-drawing
mission workflow. First pulled into devkit_ui/f2c_planner.py so the
terrain-aware pipeline (devkit_ui/terrain_mask.py) could share the same
projection — anchored at the same lat0/lon0 — without reaching into
ui_node's internals. Promoted to its own package because "planning core"
and "web UI" are different concerns with different reasons to change, and
nothing about this module actually depends on devkit_ui.

devkit_ui still owns the manual-drawing UI and calls _run_f2c() the same
way it always did, now via the devkit_f2c_planner dependency instead of a
same-package import.
"""

import math
import sys

# pylint: disable=import-error
import fields2cover as f2c
from shapely.geometry import LineString, MultiLineString, Polygon
from shapely.ops import unary_union


def _f2c_latlon_to_xy(lat: float, lon: float,
                      lat0: float, lon0: float) -> tuple[float, float]:
    R = 6_378_137.0
    x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R
    return x, y


def _f2c_xy_to_latlon(x: float, y: float,
                      lat0: float, lon0: float) -> tuple[float, float]:
    R   = 6_378_137.0
    lat = lat0 + math.degrees(y / R)
    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
    return lat, lon


# OBSTACLE: shapely post-clipping for swath/obstacle avoidance.
#
# We do not trust F2C's interior-ring handling. Across F2C builds and
# Python-binding versions the behaviour of SG_BruteForce with respect to
# Cell holes is inconsistent — sometimes swaths are clipped against
# holes, sometimes they aren't, with no error either way. Field tests
# showed swaths running straight through marked obstacles even with CW-
# wound interior rings.
#
# The fix: generate swaths against the outer boundary (F2C's job), then
# compute swath_line.difference(union_of_obstacle_polygons) ourselves
# (shapely's job). This is observable, deterministic, and guarantees the
# lines you see on the Mission map are the lines the robot will follow.
#
# Obstacle rings are still added to the F2C Cell as a hint — belt and
# braces — but the safety net is the shapely post-clip.
#
# HEADLAND: optional Minkowski erosion of the field by headland_width_m
# before swath generation, so swaths don't start/end exactly at the
# field boundary. Done via F2C's HG_Const_gen; wrapped in try/except
# because the headland call can fail on degenerate / very narrow fields.
#
# SNAKE: optional boustrophedon ordering — every second swath reversed
# so end-of-swath-N is near start-of-swath-N+1. Done in Python rather
# than via f2c.RP_Snake to avoid depending on which route-planner API
# the local F2C build exposes. Snake-flip happens *before* shapely
# clipping so direction is preserved per-fragment when an obstacle
# splits a swath into pieces.
def _run_f2c(corners_ll: list,
             obstacle_rings: list,
             tool_width: float,
             angle_deg: float,
             obstacle_pad_m: float = 0.0,
             headland_width_m: float = 0.0,
             snake_order: bool = True) -> list:

    def _log(msg):
        print(f'[F2C] {msg}', file=sys.stderr, flush=True)

    _log(f'called: {len(corners_ll)} boundary pts, '
         f'{len(obstacle_rings)} obstacles, pad={obstacle_pad_m}m, '
         f'headland={headland_width_m}m, snake={snake_order}, '
         f'width={tool_width}m, angle={angle_deg}°')

    try:
        has_shapely = True
        _log('shapely OK')
    except ImportError as e:
        has_shapely = False
        _log(f'shapely MISSING: {e}')

    lat0, lon0 = corners_ll[0]

    # ── Outer boundary ────────────────────────────────────────────────
    outer = f2c.LinearRing()
    for lat, lon in corners_ll:
        x, y = _f2c_latlon_to_xy(lat, lon, lat0, lon0)
        outer.addPoint(f2c.Point(x, y, 0))
    outer.closeRing()
    cell = f2c.Cell()
    cell.addRing(outer)

    # ── Obstacles: shapely polys for post-clip + F2C hole hints ──────
    obstacle_polys_xy: list = []
    for idx, ring_ll in enumerate(obstacle_rings):
        if len(ring_ll) < 3:
            _log(f'obstacle {idx}: skipped (only {len(ring_ll)} pts)')
            continue
        pts_xy = [_f2c_latlon_to_xy(lat, lon, lat0, lon0)
                  for lat, lon in ring_ll]
        _log(f'obstacle {idx}: {len(pts_xy)} pts, '
             f'xy bbox=({min(p[0] for p in pts_xy):.1f},{min(p[1] for p in pts_xy):.1f}) '
             f'to ({max(p[0] for p in pts_xy):.1f},{max(p[1] for p in pts_xy):.1f})')

        if has_shapely:
            poly = Polygon(pts_xy)
            _log(f'obstacle {idx}: shapely poly valid={poly.is_valid} '
                 f'empty={poly.is_empty} area={poly.area:.3f}m²')
            if obstacle_pad_m > 0:
                poly = poly.buffer(obstacle_pad_m, join_style=2, resolution=8)
                _log(f'obstacle {idx}: after buffer({obstacle_pad_m}m) '
                     f'empty={poly.is_empty} type={poly.geom_type} '
                     f'area={poly.area:.3f}m²')
            if poly.is_empty or poly.geom_type != 'Polygon':
                _log(f'obstacle {idx}: DROPPED (empty or non-Polygon)')
                continue
            obstacle_polys_xy.append(poly)
            pts_xy = list(poly.exterior.coords)

        hole = f2c.LinearRing()
        for x, y in reversed(pts_xy):
            hole.addPoint(f2c.Point(x, y, 0))
        hole.closeRing()
        cell.addRing(hole)

    _log(f'built cell with {len(obstacle_polys_xy)} shapely obstacles')

    # ── Headland inset ───────────────────────────────────────────────
    # Shrinks the cover area by headland_width_m on all sides so the
    # robot has room to turn at field edges instead of starting/ending
    # swaths at the boundary itself. Skipped when 0 — preserves the
    # original behaviour for backwards compatibility with saved fields.
    swath_cell = cell
    if headland_width_m > 0:
        try:
            hg = f2c.HG_Const_gen()
            inner = hg.generateHeadlands(f2c.Cells(cell), headland_width_m)
            if inner.size() > 0:
                swath_cell = inner.getGeometry(0)
                _log(f'headland: inset {headland_width_m}m → '
                     f'{inner.size()} sub-cell(s)')
            else:
                _log(f'headland: inset {headland_width_m}m produced 0 '
                     f'cells (too wide?) — using full boundary')
        except Exception as e:
            _log(f'headland generation failed: {e} — using full boundary')

    # ── Swath generation ─────────────────────────────────────────────
    angle_rad = math.radians(angle_deg % 180)
    sg     = f2c.SG_BruteForce()
    swaths = sg.generateSwaths(angle_rad, tool_width, swath_cell)

    raw_xy: list = []
    for i in range(swaths.size()):
        path = swaths.at(i).getPath()
        pts  = []
        for j in range(path.size()):
            pt = path.getGeometry(j)
            pts.append((pt.getX(), pt.getY()))
        if len(pts) >= 2:
            raw_xy.append(pts)
    _log(f'F2C produced {len(raw_xy)} raw swaths')

    # ── Snake ordering (Python-side boustrophedon) ───────────────────
    # F2C's BruteForce returns swaths spatially sorted along the
    # perpendicular to `angle`. Reversing every other one means the end
    # of swath N is near the start of swath N+1 — minimising inter-row
    # travel and giving the topo graph a natural chain order.
    #
    # Done before shapely clipping so direction is preserved when an
    # obstacle splits a swath into multiple fragments.
    if snake_order and raw_xy:
        raw_xy = [list(reversed(pts)) if i % 2 == 1 else list(pts)
                  for i, pts in enumerate(raw_xy)]
        _log(f'snake-flipped {len(raw_xy)} swaths')

    # ── Post-clip swaths against obstacle union ──────────────────────
    if has_shapely and obstacle_polys_xy:
        obstacles_union = unary_union(obstacle_polys_xy)
        _log(f'obstacle union: type={obstacles_union.geom_type} '
             f'area={obstacles_union.area:.3f}m² '
             f'bounds={obstacles_union.bounds}')
        if raw_xy:
            sample = raw_xy[0]
            _log(f'first swath: {len(sample)} pts, '
                 f'from ({sample[0][0]:.1f},{sample[0][1]:.1f}) '
                 f'to ({sample[-1][0]:.1f},{sample[-1][1]:.1f})')
        clipped: list = []
        clipped_count, dropped_count = 0, 0
        for pts in raw_xy:
            line = LineString(pts)
            intersects = line.intersects(obstacles_union)
            remaining = line.difference(obstacles_union)
            if remaining.is_empty:
                dropped_count += 1
                continue
            geoms = (list(remaining.geoms)
                     if isinstance(remaining, MultiLineString)
                     else [remaining])
            for sub in geoms:
                if sub.length > tool_width * 0.5:
                    clipped.append(list(sub.coords))
                    if intersects:
                        clipped_count += 1
        _log(f'clipped {clipped_count} swaths against obstacles, '
             f'{dropped_count} fully dropped, final={len(clipped)}')
        raw_xy = clipped
    else:
        _log(f'NO CLIPPING: has_shapely={has_shapely}, '
             f'obstacle_polys_xy={len(obstacle_polys_xy)}')

    # ── Project back to lat/lon ──────────────────────────────────────
    result: list = []
    for pts_xy in raw_xy:
        pts_ll = [_f2c_xy_to_latlon(x, y, lat0, lon0) for x, y in pts_xy]
        if len(pts_ll) >= 2:
            result.append(pts_ll)
    _log(f'returning {len(result)} swaths to UI')
    return result
