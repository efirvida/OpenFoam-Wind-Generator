import timeit
from math import cos, sin, atan, sqrt

try:
    from OCC.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge,
                                    BRepBuilderAPI_MakeVertex,
                                    BRepBuilderAPI_MakeWire,
                                    BRepBuilderAPI_MakeFace)
except:
    pass
import numpy as np
from jinja2 import Environment, FileSystemLoader
from scipy import spatial


def renderTemplate(template, output, data):
    def filter_1(val):
        return str(val).replace(',', '').replace(']]', ')').replace('[[', '(').replace(')]', ')').replace('[(',
                                                                                                          '(').replace(
            '[', '(').replace(']', ')')

    def filter_2(val):
        return str(val).replace(')', '').replace('(', '')

    def filter_3(val):
        if type(val) == np.ndarray or type(val) == np.matrix:
            if val.any():
                return True
        elif val:
            return True

    template_path = 'templates'
    env = Environment(loader=FileSystemLoader(template_path))
    env.filters['f1'] = filter_1
    env.filters['f2'] = filter_2
    env.filters['f3'] = filter_3
    tpl = env.get_template(template)

    output_from_parsed_template = tpl.render(data=data)

    with open(output, "wb") as fh:
        fh.write(output_from_parsed_template)


# GEOMETRY
def make_edge(*args):
    edge = BRepBuilderAPI_MakeEdge(*args)
    result = edge.Edge()
    return result


def make_vertex(*args):
    vert = BRepBuilderAPI_MakeVertex(*args)
    result = vert.Vertex()
    return result


def make_wire(*args):
    # if we get an iterable, than add all edges to wire builder
    if isinstance(args[0], list) or isinstance(args[0], tuple):
        wire = BRepBuilderAPI_MakeWire()
        for i in args[0]:
            wire.Add(i)
        wire.Build()
        return wire.Wire()
    wire = BRepBuilderAPI_MakeWire(*args)
    return wire.Wire()


def make_face(*args):
    face = BRepBuilderAPI_MakeFace(*args)
    assert (face.IsDone())
    result = face.Face()
    return result


# MATHS
def rotate2d(coordinates, angle, center=(0, 0,)):
    """
    p'x = cos(angle) * (px-ox) - sin(angle) * (py-oy) + ox
    p'y = sin(angle) * (px-ox) + cos(angle) * (py-oy) + oy

    :param coordinates:
    :param angle:
    :param center:
    :return:

    """
    rotated = np.array([
        np.cos(np.radians(angle)) * (coordinates[:, 0] - center[0]) - np.sin(np.radians(angle)) * (
        coordinates[:, 1] - center[1]) + center[0],
        np.sin(np.radians(angle)) * (coordinates[:, 0] - center[0]) + np.cos(np.radians(angle)) * (
        coordinates[:, 1] - center[1]) + center[1]])

    return rotated.T


def rotate3d(coordinates, angle, axis):
    axis = str(axis).lower()
    if axis == 'x':
        M = np.matrix([
            [1, 0, 0],
            [0, np.cos(np.deg2rad(angle)), -np.sin(np.deg2rad(angle))],
            [0, np.sin(np.deg2rad(angle)), np.cos(np.deg2rad(angle))]
        ])
    elif axis == 'y':
        M = np.matrix([
            [np.cos(np.deg2rad(angle)), 0, np.sin(np.deg2rad(angle))],
            [0, 1, 0],
            [-np.sin(np.deg2rad(angle)), 0, np.cos(np.deg2rad(angle))]
        ])
    elif axis == 'z':
        M = np.matrix([
            [np.cos(np.deg2rad(angle)), -np.sin(np.deg2rad(angle)), 0],
            [np.sin(np.deg2rad(angle)), np.cos(np.deg2rad(angle)), 0],
            [0, 0, 1],
        ])
    else:
        raise Exception('Error axis must be \'x\' , \'y\' or \'z\'')

    coordinates = coordinates * M
    return coordinates.tolist()


def offset(coordinates, distance):
    # def add_semicircle(x_origin, y_origin, radius, num_x=50):
    #     points = []
    #     for index in range(num_x):
    #         x = radius * index / num_x
    #         y = (radius ** 2 - x ** 2) ** 0.5
    #         points.append((x, -y))
    #     points += [(x, -y) for x, y in reversed(points)]
    #     return [(x + x_origin, y + y_origin) for x, y in points[4:]]

    def round_data(data):
        # Add infinitesimal rounding of the envelope
        data = data.tolist()
        assert data[-1] == data[0]
        x0, y0 = data[0]
        x1, y1 = data[1]
        xe, ye = data[-2]

        x = x0 - (x0 - x1) * .01
        y = y0 - (y0 - y1) * .01
        yn = (x - xe) / (x0 - xe) * (y0 - ye) + ye
        data[0] = x, y
        data[-1] = x, yn
        # data.extend(add_semicircle(x, (y + yn) / 2, abs((y - yn) / 2)))
        # del data[-10:]
        return data

    def clean(data):
        rows = []
        for i in range(1, len(data) - 1):
            backward_angle = np.arctan2(data[i][1] - data[i - 1][1], data[i][0] - data[i - 1][0])
            forward_angle = np.arctan2(data[i + 1][1] - data[i][1], data[i + 1][0] - data[i][0])
            angle_difference = (backward_angle - forward_angle + np.pi) % (2 * np.pi) - np.pi
            if np.absolute(angle_difference) > np.pi / 2:
                rows.append(i)

        data = data if not rows else np.delete(data, rows, axis=0)
        return data[~np.isnan(data).any(axis=1)]

    creo_point = coordinates[0]
    coordinates = round_data(coordinates)
    coordinates = iter(coordinates)
    x1, y1 = coordinates.next()
    z = distance
    points = []
    for x2, y2 in coordinates:
        # tangential slope approximation
        try:
            slope = (y2 - y1) / (x2 - x1)
            # perpendicular slope
            pslope = -1 / slope  # (might be 1/slope depending on direction of travel)
        except ZeroDivisionError:
            continue
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2

        sign = ((pslope > 0) == (x1 > x2)) * 2 - 1

        # if z is the distance to your parallel curve,
        # then your delta-x and delta-y calculations are:
        #   z**2 = x**2 + y**2
        #   y = pslope * x
        #   z**2 = x**2 + (pslope * x)**2
        #   z**2 = x**2 + pslope**2 * x**2
        #   z**2 = (1 + pslope**2) * x**2
        #   z**2 / (1 + pslope**2) = x**2
        #   z / (1 + pslope**2)**0.5 = x

        delta_x = sign * z / ((1 + pslope ** 2) ** 0.5)
        delta_y = pslope * delta_x

        points.append((mid_x + delta_x, mid_y + delta_y))
        x1, y1 = x2, y2

    data = clean(np.array(points))
    # return points

    return np.append(data, arc2points_np(creo_point, points[-1], points[0]), axis=0)


def mid_point(pair1, pair2):
    x = (pair1[0][0] + pair1[1][0]) / 2
    y = (pair1[0][0] + pair1[1][0]) / 2
    return x, y


def divide_line(points, parts):
    tmp = [np.linspace(i, j, parts) for i, j in zip(points[0], points[1])]
    return zip(tmp[0], tmp[1])


def getExtemePoints(coordinates):
    """



    :rtype : object
    :return:
    """
    # distance, index = spatial.KDTree(self.coordinates).query([0, 0])
    max_x = np.argmax(coordinates[:, 0])
    max_y = np.argmax(coordinates[:, 1])
    min_x = np.argmin(coordinates[:, 0])
    min_y = np.argmin(coordinates[:, 1])
    return (max_x, max_y, min_x, min_y)


def nearest(arr0, arr1):
    ptos = []
    j = 0
    for i in arr0:
        distance, index = spatial.KDTree(arr1).query(i)
        ptos.append([distance, index, j])
        j += 1
    ptos.sort()
    return (arr1[ptos[0][1]].tolist(), ptos[0][1], ptos[0][2])


def intersection(x0, y0, slope, airfoil, x1=5):
    y1 = (x1 - x0) * np.tan(np.deg2rad(slope))
    x = np.linspace(x0, x1, 100)
    y = np.linspace(y0, y1, 100)
    return nearest(zip(x, y), airfoil)


def arc2points(center, pt1, pt2, resolution=100):
    """
    Genera las coordenadas de un arco entre los puntos 'pt1' y 'pt2' con centro en 'center'
    y numero de puntos igual a 'resolution'

    :param center: list o tuple
    :param pt1: list o tuple
    :param pt2: list o tuple
    :param resolution: int
    :return: list
    """
    X, Y = center
    Xa = pt1[0]
    Xc = pt2[0]
    Ya = pt1[1]
    Yc = pt2[1]

    radius = sqrt((Xa - X) ** 2 + (Ya - Y) ** 2)

    theta1 = atan((Ya - Y) / (Xa - X))
    theta2 = atan((Yc - Y) / (Xc - X))

    theta = [theta1 + x * (theta2 - theta1) / resolution for x in range(resolution)]

    x = [radius * cos(i) + center[0] for i in theta]
    y = [radius * sin(i) + center[1] for i in theta]

    return zip(x, y)


def arc2points_np(center, pt1, pt2, resolution=100):
    """
    Genera las coordenadas de un arco entre los puntos 'pt1' y 'pt2' con centro en 'center'
    y numero de puntos igual a 'resolution'

    :param center: list o tuple
    :param pt1: list o tuple
    :param pt2: list o tuple
    :param resolution: int
    :return: numpy.ndarray
    """
    X, Y = center
    Xa = pt1[0]
    Xc = pt2[0]
    Ya = pt1[1]
    Yc = pt2[1]

    radius = np.sqrt((Xa - X) ** 2 + (Ya - Y) ** 2)

    theta1 = np.arctan2(Ya - Y, Xa - X)
    theta2 = np.arctan2(Yc - Y, Xc - X)

    theta = np.linspace(theta1, theta2, resolution)
    return np.vstack((radius * np.cos(theta) + center[0], radius * np.sin(theta) + center[1])).T


def arc3points(pt1, pt2, pt3):
    Xa = pt1[0]
    Xb = pt2[0]
    Xc = pt3[0]
    Ya = pt1[1]
    Yb = pt2[1]
    Yc = pt3[1]
    a = [
        [2 * (Xa - Xb), 2 * (Ya - Yb)],
        [2 * (Xa - Xc), 2 * (Ya - Yc)],
    ]
    b = [-(Xb ** 2 + Yb ** 2 - Xa ** 2 - Ya ** 2), -(Xc ** 2 + Yc ** 2 - Xa ** 2 - Ya ** 2)]
    X, Y = np.linalg.lstsq(a, b)[0]

    points = arc2points((X, Y), pt1, pt3)

    return points


def blend(coords, r, plane='xy'):
    if plane == 'xy':
        x = np.array([i[0] for i in coords])
        y = r * np.sin(np.arccos(x / r))
        z = np.array([i[2] for i in coords])
    elif plane == 'yz':
        x = np.array([i[0] for i in coords])
        y = np.array([i[1] for i in coords])
        z = r * np.sin(np.arccos(y / r))
    elif plane == 'xz':
        z = np.array([i[0] for i in coords])
        y = np.array([i[1] for i in coords])
        x = r * np.sin(np.arccos(z / r))

    return zip(x, y, z)


def midLine(coords):
    x0 = len(coords) / 2
    upProfile = coords[:x0]
    downProfile = coords[x0:][::-1]

    if len(upProfile) - len(downProfile) > 0:
        upProfile = coords[:x0 - (len(upProfile) - len(downProfile))]
    else:
        downProfile = coords[x0 - (len(upProfile) - len(downProfile)):][::-1]

    return (upProfile + downProfile) / 2.
