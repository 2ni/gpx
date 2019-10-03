import numpy as np
import math

EARTH_RADIUS = 6371 * 1000
ONE_DEGREE = (2 * math.pi * EARTH_RADIUS) / 360

def haversine_distance(start, end):
    """
    calculate distance of 2 points with haversine
    http://www.movable-type.co.uk/scripts/latlong.html
    SLOW
    """

    d_lat = math.radians(start.lat - end.lat)
    d_lon = math.radians(start.lon - end.lon)
    lat1 = math.radians(start.lat)
    lat2 = math.radians(end.lat)

    a = math.sin(d_lat/2) * math.sin(d_lat/2) + math.sin(d_lon/2) * math.sin(d_lon/2) * math.cos(lat1) * math.cos(lat2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = EARTH_RADIUS * c

    return d

def euclid_distance(start, end):
    """
    approximation of haversine
    by converting coordinates to cartesian system
    and use lenght = sqrt(x^2 + y^2)
    works for small distances

    see http://jonisalonen.com/2014/computing-distance-between-coordinates-can-be-simple-and-fast/
    https://www.movable-type.co.uk/scripts/latlong.html#equirectangular
    might use geopy.distance.distance()
    """

    x = start.lat - end.lat
    y = (start.lon - end.lon) * math.cos(math.radians(end.lat))

    distance = ONE_DEGREE * math.sqrt(x*x + y*y)

    if getattr(start, "elevation", None) and getattr(end, "elevation", None):
        math.sqrt(distance ** 2, (end.elevation - start.elevation) ** 2)

    return distance

def latlon2xy(point):
    """
    convert lat/lon to 2d cartesian coordinates
    """

    x = ONE_DEGREE * point.lat
    y = ONE_DEGREE * point.lon * math.cos(math.radians(point.lat))

    return (x, y)

def latlon2xyz(point):
    """
    convert lat/lon to 3d cartesian
    not yet used
    """
    lat = math.radians(point.lat)
    lon = math.radians(point.lon)
    x = EARTH_RADIUS * math.cos(lat) * math.cos(lon)
    y = EARTH_RADIUS * math.cos(lat) * math.sin(lon)
    z = EARTH_RADIUS * math.sin(lat)

    return (x, y, z)

def xyz2latlon(coords):
    """
    convert 3d cartesian to lat/lon
    not yet used
    """
    x, y, z = coords
    lat = math.asin(z/EARTH_RADIUS)
    lon = math.atan2(y, x)
    return (lat, lon)

def distance_to_line(point, start, end):
    """
    caclulate perpendicular distance between point and line defined by start-end
    2d

    see https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points
    """

    p1 = np.array(latlon2xy(start))
    p2 = np.array(latlon2xy(end))
    p3 = np.array(latlon2xy(point))

    # use p1 as base
    # p2-p1 = a
    # p3-p1 = b
    # cross(a, b) gives vector
    # norm(p2-p1) gives distance of p2 to p1
    distance = np.abs(np.cross(p2 - p1, p3 - p1) / np.linalg.norm(p2 - p1))

    return distance


def reduce(points, distance=10):
    """
    points: list of points to simplify
    distance: minimum distance in m to keep the point
    keeps points having a type set
    """

    if len(points) < 3:
        return points

    start, end = points[0], points[-1]
    max_distance = 0
    max_index = 1

    """
    use simplified (faster) distance calculation by
    using cartesian coordinates (ok for our case)
    see https://en.m.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    (it's not the exact distance, we only use it to *find* the point with the max distance)
    y2 = end.lat
    y1 = start.lat
    y0 = point.lat
    x2 = end.lon
    x1 = start.lon
    x0 = point.lon
    distance ~ abs(a*point.lon - b*point.lat + c)
    distance ~ abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
    """

    a = end.lat - start.lat
    b = end.lon - start.lon
    c = end.lon*start.lat - end.lat*start.lon

    # find point with max distance
    for index in range(1, len(points)-1):
        point = points[index]
        cur_distance = abs(a*point.lon - b*point.lat +c)
        if cur_distance > max_distance:
            max_distance = cur_distance
            max_index = index

    # calculate real distance now that we found point with max distance
    max_distance = distance_to_line(points[max_index], start, end)

    # maximum calculated distance is smaller than allowed distance
    # -> internal points can be ignored
    # except waypoints and peaks
    if max_distance < distance:
        valid_points = [ point for point in points if getattr(point, "type", None) and start.distance < point.distance < end.distance ]
        valid_points.insert(0, start)
        valid_points.append(end)
        return valid_points

    return (reduce(points[:max_index + 1], distance) + reduce(points[max_index:], distance)[1:])
