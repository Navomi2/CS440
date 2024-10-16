# geometry.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Joshua Levine (joshua45@illinois.edu)
# Inspired by work done by James Gao (jamesjg2@illinois.edu) and Jongdeog Lee (jlee700@illinois.edu)

"""
This file contains geometry functions necessary for solving problems in MP5
"""

import numpy as np
from alien import Alien
from typing import List, Tuple
from copy import deepcopy
import math

#some code is written for rectangle shape not 
def does_alien_touch_wall(alien: Alien, walls: List[Tuple[int]]):
    # print("---NEW ALIEN---")
    """Determine whether the alien touches a wall

        Args:
            alien (Alien): Instance of Alien class that will be navigating our map
            walls (list): List of endpoints of line segments that comprise the walls in the maze in the format
                         [(startx, starty, endx, endx), ...]

        Return:
            True if touched, False if not
    """
    (x,y) = alien.get_centroid()
    head, tail = alien.get_head_and_tail()
    length = alien.get_length()

    for wall in walls:
        # print(x,y)
        (startx, starty, endx, endy) = wall
        wall_segment = ((startx, starty), (endx, endy))
        # print(wall)
        if alien.is_circle():
            radius = alien.get_width()
            # print(radius)
            # print("Ball")
            if (point_segment_distance((x,y), wall_segment) <= radius):    
                # print("TRUE")
                return True
        else: #vertical or horizontal
            width = alien.get_width()
            # print(length, width)
            # print(head, tail)
            midline = (head, tail)
            if (segment_distance(midline, wall_segment) <= width):
                return True
    
    return False

def is_alien_within_window(alien: Alien, window: Tuple[int]):
    # print("---NEW ALIEN---")
    """Determine whether the alien stays within the window

        Args:
            alien (Alien): Alien instance
            window (tuple): (width, height) of the window
    """
    (x,y) = alien.get_centroid()
    head, tail = alien.get_head_and_tail()
    length = alien.get_length()

    # print(x, y)
    # print(window)
    if alien.is_circle():
        # print("Ball")
        radius = alien.get_width()
        # print(radius)
        if (x - radius > 0) and (x + radius < window[0]) and (y - radius > 0) and (y + radius < window[1]):
            return True
    else:
        width = alien.get_width()
        # print(length, width)
        # IDK why im adding width to either end when checking if the long side is in bounds it just works that way
        if head[0] == tail[0]: #vertical
            # print("vertical")
        # if (head[0] - tail[0] < window[0]) and (head[1] - tail[1] < window[1]):
            if (x - width > 0) and (x + width < window[0]) and (y - (length/2) - width > 0) and (y + (length/2) + width < window[1]):
                return True
        else:
            # print("horizontal")
            if (x - (length/2) - width > 0) and (x + (length/2) + width < window[0]) and (y - width > 0) and (y + width < window[1]):
                return True

    # print("false")
    return False

def is_point_in_polygon(point, polygon):
    """Determine whether a point is in a parallelogram.
    Note: The vertex of the parallelogram should be clockwise or counter-clockwise.

        Args:
            point (tuple): shape of (2, ). The coordinate (x, y) of the query point.
            polygon (tuple): shape of (4, 2). The coordinate (x, y) of 4 vertices of the parallelogram.
    """
    o1 = get_orientation(polygon[0], polygon[1], point)
    o2 = get_orientation(polygon[1], polygon[2], point)
    o3 = get_orientation(polygon[2], polygon[3], point)
    o4 = get_orientation(polygon[3], polygon[0], point)

    #if point is on one of the lines (orientation = 0) then True
    if (point_segment_distance(point, (polygon[0], polygon[1])) == 0
        or point_segment_distance(point, (polygon[1], polygon[2])) == 0
        or point_segment_distance(point, (polygon[2], polygon[3])) == 0
        or point_segment_distance(point, (polygon[3], polygon[0])) == 0):
        return True

    
    #if all orientations cw or ccw then True
    if (o1 == o2 == o3 == o4):
        if (o1 != 0):
            return True

    return False

#use in_polygon?? the shape created with the midlines at every point along the path to waypoint is a polygon
def does_alien_path_touch_wall(alien: Alien, walls: List[Tuple[int]], waypoint: Tuple[int, int]):
    # print("---NEW ALIEN---")
    """Determine whether the alien's straight-line path from its current position to the waypoint touches a wall

        Args:
            alien (Alien): the current alien instance
            walls (List of tuple): List of endpoints of line segments that comprise the walls in the maze in the format
                         [(startx, starty, endx, endx), ...]
            waypoint (tuple): the coordinate of the waypoint where the alien wants to move

        Return:
            True if touched, False if not
    """
    # print(alien.get_centroid())
    # print(waypoint)

    already_touching = does_alien_touch_wall(alien, walls) #if already touching, does touch
    if (already_touching):
        # print("already touching")
        return True

    center = alien.get_centroid()
    head, tail = alien.get_head_and_tail()
    # if (np.linalg.norm(np.array(center) - np.array(waypoint), 2)): #if not moving, does not touch
    #     return False
    # waypoint_path = [waypoint[0] - center[0], waypoint[1] - center[1]]
    for wall in walls:
        (startx, starty, endx, endy) = wall
        wall_segment = ((startx, starty), (endx, endy))
        if alien.is_circle():
            radius = alien.get_width()
            if segment_distance((center, waypoint), wall_segment) <= radius:
                return True
        else:
            #Note: this only works is length is up to 4x width
            width = alien.get_width()
            length = alien.get_length()
            # midline = alien.get_head_and_tail() #midline = (head, tail)
            if head[0] == tail[0]: #if vertical
                if (segment_distance(((center[0], center[1] + length/2), (waypoint[0], waypoint[1] + length/2)), wall_segment) <= width
                    or segment_distance(((center[0], center[1] - length/2), (waypoint[0], waypoint[1] - length/2)), wall_segment) <= width
                    or segment_distance(((center[0], center[1]), (waypoint[0], waypoint[1])), wall_segment) <= width):
                    return True
            else: #if horizontal
                if (segment_distance(((center[0] + length/2, center[1]), (waypoint[0] + length/2, waypoint[1])), wall_segment) <= width 
                    or segment_distance(((center[0] - length/2, center[1]), (waypoint[0] - length/2, waypoint[1])), wall_segment) <= width
                    or segment_distance(((center[0], center[1]), (waypoint[0], waypoint[1])), wall_segment) <= width):
                # if (segment_distance(((center[0], center[1] - length/2), (waypoint[0], waypoint[1] + length/2)), wall_segment) <= length/2):
                    return True

    return False

def point_segment_distance(p, s):
    """Compute the distance from the point to the line segment.

        Args:
            p: A tuple (x, y) of the coordinates of the point.
            s: A tuple ((x1, y1), (x2, y2)) of coordinates indicating the endpoints of the segment.

        Return:
            Euclidean distance from the point to the line segment.
    """
    s0 = np.array(s[0])
    s1 = np.array(s[1])
    s = np.array(s)
    p = np.array(p)

    len_s0tos1 = np.sqrt((s1[0] - s0[0])**2 + (s1[1] - s0[1])**2)
    len_s0top = np.sqrt((p[0] - s0[0])**2 + (p[1] - s0[1])**2)
    len_s1top = np.sqrt((p[0] - s1[0])**2 + (p[1] - s1[1])**2)
    # print(f"len_a = {len_a}, len_b = {len_b}")
    s0tos1 = s1 - s0 #s0 to s1
    s0top = p - s0 #s0 to p
    s1top = p - s1 #s1 to p
    # print(f"vector_a = {vector_a}, vector_b = {vector_b}")
    if np.dot(s0tos1, s1top) > 0: #if angle < 90 then min dist is len_s1top
        return len_s1top
    if np.dot(s0tos1, s0top) < 0: #if angle > 90 then min dist is len_s0top
        return len_s0top
    else: #cross a and b then divide by |b| to get a*sin(alpha)
        if (len_s0tos1 == 0):
            return len_s0top
        return abs(np.cross(s0top, s0tos1) / len_s0tos1)

#0 = colinear, 1 = cw, 2 = ccw
def get_orientation(line_start, line_end, point):
    start_to_end = [line_start[0] - line_end[0], line_start[1] - line_end[1]]
    end_to_p = [line_end[0] - point[0], line_end[1] - point[1]]

    cross_product = np.cross(start_to_end, end_to_p)

    if cross_product > 0:
        return 1
    elif cross_product < 0:
        return 2
    else:
        return 0

def do_segments_intersect(s1, s2):
    """Determine whether segment1 intersects segment2.

        Args:
            s1: A tuple of coordinates indicating the endpoints of segment1.
            s2: A tuple of coordinates indicating the endpoints of segment2.

        Return:
            True if line segments intersect, False if not.
    """
    s1 = np.array(s1)
    s2 = np.array(s2)
    s1[0] = np.array(s1[0])
    s1[1] = np.array(s1[1])
    s2[0] = np.array(s2[0])
    s2[1] = np.array(s2[1])

    # s1_vect = s1[1] - s1[0]
    # s2_vect = s2[1] - s2[0]
    # cross_product = np.cross(s1_vect, s2_vect)

    #find orientation of each point with respect to line s1 or s2
    o_s20_with_s1 = get_orientation(s1[0], s1[1], s2[0]) 
    o_s21_with_s1 = get_orientation(s1[0], s1[1], s2[1]) 
    o_s10_with_s2 = get_orientation(s2[0], s2[1], s1[0]) 
    o_s11_with_s2 = get_orientation(s2[0], s2[1], s1[1]) 
  
    #both orientations have same rule
    if ((o_s20_with_s1 != o_s21_with_s1) and (o_s10_with_s2 != o_s11_with_s2)): 
        return True
  
    # o_s20_with_s1 collinear and s2[0] on line s1
    if (o_s20_with_s1 == 0) and (point_segment_distance(tuple(s2[0]), (tuple(s1[0]), tuple(s1[1]))) == 0): 
        return True
    # o_s21_with_s1 collinear and s2[1] on line s1 
    if (o_s21_with_s1 == 0) and (point_segment_distance(tuple(s2[1]), (tuple(s1[0]), tuple(s1[1]))) == 0): 
        return True
    # o_s10_with_s2 collinear and s1[0] on line s2
    if (o_s10_with_s2 == 0) and (point_segment_distance(tuple(s1[0]), (tuple(s2[0]), tuple(s2[1]))) == 0): 
        return True
    # o_s11_with_s2 collinear and s1[1] on line s2 
    if (o_s11_with_s2 == 0) and (point_segment_distance(tuple(s1[1]), (tuple(s2[0]), tuple(s2[1]))) == 0): 
        return True
  
    return False

def segment_distance(s1, s2):
    """Compute the distance from segment1 to segment2.  You will need `do_segments_intersect`.

        Args:
            s1: A tuple of coordinates indicating the endpoints of segment1.
            s2: A tuple of coordinates indicating the endpoints of segment2.

        Return:
            Euclidean distance between the two line segments.
    """
    if (do_segments_intersect(s1, s2) == True):
        return 0

    s1_start_to_s2 = point_segment_distance(s1[0], s2)
    s1_end_to_s2 = point_segment_distance(s1[1], s2)
    s2_start_to_s1 = point_segment_distance(s2[0], s1)
    s2_end_to_s1 = point_segment_distance(s2[1], s1)

    return min(s1_start_to_s2, s1_end_to_s2, s2_start_to_s1, s2_end_to_s1)  

if __name__ == '__main__':

    from geometry_test_data import walls, goals, window, alien_positions, alien_ball_truths, alien_horz_truths, \
        alien_vert_truths, point_segment_distance_result, segment_distance_result, is_intersect_result, waypoints


    # Here we first test your basic geometry implementation
    def test_point_segment_distance(points, segments, results):
        num_points = len(points)
        num_segments = len(segments)
        for i in range(num_points):
            p = points[i]
            for j in range(num_segments):
                seg = ((segments[j][0], segments[j][1]), (segments[j][2], segments[j][3]))
                cur_dist = point_segment_distance(p, seg)
                assert abs(cur_dist - results[i][j]) <= 10 ** -3, \
                    f'Expected distance between {points[i]} and segment {segments[j]} is {results[i][j]}, ' \
                    f'but get {cur_dist}'


    def test_do_segments_intersect(center: List[Tuple[int]], segments: List[Tuple[int]],
                                   result: List[List[List[bool]]]):
        for i in range(len(center)):
            for j, s in enumerate([(40, 0), (0, 40), (100, 0), (0, 100), (0, 120), (120, 0)]):
                for k in range(len(segments)):
                    cx, cy = center[i]
                    st = (cx + s[0], cy + s[1])
                    ed = (cx - s[0], cy - s[1])
                    a = (st, ed)
                    b = ((segments[k][0], segments[k][1]), (segments[k][2], segments[k][3]))
                    if do_segments_intersect(a, b) != result[i][j][k]:
                        if result[i][j][k]:
                            assert False, f'Intersection Expected between {a} and {b}.'
                        if not result[i][j][k]:
                            assert False, f'Intersection not expected between {a} and {b}.'


    def test_segment_distance(center: List[Tuple[int]], segments: List[Tuple[int]], result: List[List[float]]):
        for i in range(len(center)):
            for j, s in enumerate([(40, 0), (0, 40), (100, 0), (0, 100), (0, 120), (120, 0)]):
                for k in range(len(segments)):
                    cx, cy = center[i]
                    st = (cx + s[0], cy + s[1])
                    ed = (cx - s[0], cy - s[1])
                    a = (st, ed)
                    b = ((segments[k][0], segments[k][1]), (segments[k][2], segments[k][3]))
                    distance = segment_distance(a, b)
                    assert abs(result[i][j][k] - distance) <= 10 ** -3, f'The distance between segment {a} and ' \
                                                                        f'{b} is expected to be {result[i]}, but your' \
                                                                        f'result is {distance}'


    def test_helper(alien: Alien, position, truths):
        alien.set_alien_pos(position)
        config = alien.get_config()

        touch_wall_result = does_alien_touch_wall(alien, walls)
        in_window_result = is_alien_within_window(alien, window)

        assert touch_wall_result == truths[
            0], f'does_alien_touch_wall(alien, walls) with alien config {config} returns {touch_wall_result}, ' \
                f'expected: {truths[0]}'
        assert in_window_result == truths[
            2], f'is_alien_within_window(alien, window) with alien config {config} returns {in_window_result}, ' \
                f'expected: {truths[2]}'


    def test_check_path(alien: Alien, position, truths, waypoints):
        alien.set_alien_pos(position)
        config = alien.get_config()

        for i, waypoint in enumerate(waypoints):
            path_touch_wall_result = does_alien_path_touch_wall(alien, walls, waypoint)

            assert path_touch_wall_result == truths[
                i], f'does_alien_path_touch_wall(alien, walls, waypoint) with alien config {config} ' \
                    f'and waypoint {waypoint} returns {path_touch_wall_result}, ' \
                    f'expected: {truths[i]}'

            # Initialize Aliens and perform simple sanity check.

    alien_ball = Alien((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Ball', window)
    test_helper(alien_ball, alien_ball.get_centroid(), (False, False, True))

    alien_horz = Alien((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Horizontal', window)
    test_helper(alien_horz, alien_horz.get_centroid(), (False, False, True))

    alien_vert = Alien((30, 120), [40, 0, 40], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Vertical', window)
    test_helper(alien_vert, alien_vert.get_centroid(), (True, False, True))

    edge_horz_alien = Alien((50, 100), [100, 0, 100], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Horizontal',
                            window)
    edge_vert_alien = Alien((200, 70), [120, 0, 120], [11, 25, 11], ('Horizontal', 'Ball', 'Vertical'), 'Vertical',
                            window)

    # Test validity of straight line paths between an alien and a waypoint
    test_check_path(alien_ball, (30, 120), (False, True, True), waypoints)
    test_check_path(alien_horz, (30, 120), (False, True, False), waypoints)
    test_check_path(alien_vert, (30, 120), (True, True, True), waypoints)

    centers = alien_positions
    segments = walls
    test_point_segment_distance(centers, segments, point_segment_distance_result)
    test_do_segments_intersect(centers, segments, is_intersect_result)
    test_segment_distance(centers, segments, segment_distance_result)
    for i in range(len(alien_positions)):
        test_helper(alien_ball, alien_positions[i], alien_ball_truths[i])
        test_helper(alien_horz, alien_positions[i], alien_horz_truths[i])
        test_helper(alien_vert, alien_positions[i], alien_vert_truths[i])

    # Edge case coincide line endpoints
    test_helper(edge_horz_alien, edge_horz_alien.get_centroid(), (True, False, False))
    test_helper(edge_horz_alien, (110, 55), (True, True, True))
    test_helper(edge_vert_alien, edge_vert_alien.get_centroid(), (True, False, True))

    print("Geometry tests passed\n")
