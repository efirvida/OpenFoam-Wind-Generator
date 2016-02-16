import copy
import os
import matplotlib.pyplot as plt

from core_utils import midLine

try:
    # PYTHON OCC IMPORT
    from OCC.BRepAlgoAPI import BRepAlgoAPI_Fuse
    from OCC.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCC.BRepOffsetAPI import BRepOffsetAPI_ThruSections
    from OCC.BRepPrimAPI import BRepPrimAPI_MakeSphere
    from OCC.Geom2dAPI import Geom2dAPI_PointsToBSpline
    from OCC.GeomAPI import geomapi
    from OCC.STEPControl import STEPControl_Writer
    from OCC.StlAPI import StlAPI_Writer
    from OCC.TColgp import TColgp_Array1OfPnt2d
    from OCC.IFSelect import IFSelect_RetDone
    from OCC.Interface import Interface_Static_SetCVal
    from OCC.STEPControl import STEPControl_AsIs
    from OCC.gp import gp_Pnt, gp_Pln, gp_Dir, gp_Pnt2d, gp_Trsf, gp_OZ, gp_OY
    from OCC.Display.SimpleGui import init_display
    # PYTHON OCC IMPORT
except:
    print 'Warning... No python-occ libs found the visualization functions may not work'

import subprocess
from core_utils import *
from naca import naca4, naca5


class Airfoils(object):
    def __init__(self, name, **kwargs):
        """

        :param name: 
        :param kwargs: 
        :raise Exception: 
        """
        denomination = kwargs['denomination'] if 'denomination' in kwargs else None
        radius = kwargs['radius'] if 'radius' in kwargs else 0.5
        radius = float(radius)
        n_points = 500  # number of on the profiles

        self.coordinates = np.array([])

        if name == 'circle':
            self.coordinates = self.__circle(radius, (0.5, 0), n_points)
            self.aerodynamic_center = np.array((0.5, 0))
            self.type = 'circle'

        elif name == 'naca4' or name == 'naca5':
            self.type = name
            self.coordinates = self.__naca(name, denomination, n_points)
            self.aerodynamic_center = self.chordLine()[len(self.chordLine()) / 4]
        else:
            self.type = 'from_file'
            self.coordinates = self.__from_file(name)
            self.aerodynamic_center = self.chordLine()[len(self.chordLine()) / 4]

        if not self.coordinates.any():
            raise Exception('Something Went\'s Wrong')
        else:
            self.coordinates[-1, :] = self.coordinates[0, :]

    def extremePoint(self):
        return getExtemePoints(self.coordinates)

    def midPoint(self):
        return self.midLine()[len(self.midLine()) / 2]

    def chordLine(self, points=50):
        x = np.linspace(self.coordinates[getExtemePoints(self.coordinates)[2]][0], self.coordinates[0][0], points)
        y = np.linspace(self.coordinates[getExtemePoints(self.coordinates)[2]][1], self.coordinates[0][1], points)
        return np.dstack((x, y))[0]

    def plot(self):
        """

        

        :rtype : object
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        ax.plot(self.coordinates[:, 0], self.coordinates[:, 1], '-r')
        ax.plot(self.midLine()[:, 0], self.midLine()[:, 1], '-b')
        ax.plot(self.chordLine()[:, 0], self.chordLine()[:, 1], '-y')
        ax.plot(self.aerodynamic_center[0], self.aerodynamic_center[1], 'o')

        ax.grid()
        plt.show()

    def __from_file(self, file):
        """
        :rtype : object
        :param name: 
        :return: :raise Exception: 
        """
        with open(file, 'rb') as f:
            if f:
                f.next()
                return np.array([[float(x) for x in line.split()] for line in f if line.strip()])
            else:
                raise Exception('ERROR! Not found "' + file + '" in the airfoils database')

    def __naca(self, airfoilType, denomination, n_points=120):
        """
        :rtype : object
        :param airfoilType: 
        :param denomination: 
        :return: 
        """

        if airfoilType == 'naca4':
            return naca4(denomination, n_points)
        elif airfoilType == 'naca5':
            return naca5(denomination, n_points)

    @staticmethod
    def __circle(r, c=(0.5, 0), n_points=120):
        """
        :rtype : object
        :param r: 1
        :param c: 
        :param n: 
        :return: 
        """
        x_y = [[c[0] + r * np.cos(theta), c[1] + r * np.sin(theta)] for theta in
               np.linspace(0, 2 * np.pi, n_points + 1)]
        x_y[n_points] = x_y[0]
        return np.array(x_y)


class Blade(object):
    def __init__(self, blade):
        self.blade = blade
        self.__modifyAirfoils()
        self.length = self.blade[-1]['position'][2]

    def __modifyAirfoils(self):

        """



        :rtype : object
        :return:
        """
        airfoils = []
        aerodynamic_center = []
        for i in self.blade['airfoils']:
            tmp = copy.deepcopy(i)

            # scale the airfoil
            tmp['airfoil'].coordinates = tmp['airfoil'].coordinates * tmp['scale']
            tmp['airfoil'].aerodynamic_center = tmp['airfoil'].aerodynamic_center * tmp['scale']

            # aerodynamic_center.append(tmp['airfoil'].aerodynamic_center * tmp['scale'])
            # make transformations twist the airfoil, by its center
            if not i['airfoil'].type is 'circle':
                if tmp['twist']:
                    tmp['airfoil'].coordinates = rotate2d(tmp['airfoil'].coordinates, tmp['twist'],
                                                          tmp['airfoil'].aerodynamic_center)
                if self.blade['blade_twist']:
                    tmp['airfoil'].coordinates = rotate2d(tmp['airfoil'].coordinates, self.blade['blade_twist'], (0.5, 0.))

            # align the airfoil by aerodynamic center
            tmp['airfoil'].coordinates[:, 0] = (tmp['airfoil'].coordinates[:, 0] - tmp['airfoil'].aerodynamic_center[
                0]) + i['position'][0]
            tmp['airfoil'].coordinates[:, 1] = (tmp['airfoil'].coordinates[:, 1] - tmp['airfoil'].aerodynamic_center[
                1]) + i['position'][1]
            tmp['airfoil'].aerodynamic_center = (i['position'][0], i['position'][1])
            tmp['position'][2] = i['position'][2]
            tmp['airfoil'].coordinates = np.insert(tmp['airfoil'].coordinates, 1, i['position'][2], axis=1)

            airfoils.append(tmp)
        self.blade = airfoils
        self.blade_coords = [i['airfoil'].coordinates for i in airfoils]

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')

        for i in self.blade:
            x = i['airfoil'].coordinates[:, 0]
            z = i['airfoil'].coordinates[:, 2]
            ax.plot(x, z)
            # ax.plot(i['airfoil'].aerodynamic_center[0], i['airfoil'].aerodynamic_center[2], 'o')
        ax.grid()
        plt.show()

    def view(self):
        display, start_display, add_menu, add_function_to_menu = init_display()
        blade = self.__OCCBladeModel()
        display.DisplayShape(blade)
        display.FitAll()
        start_display()

    def vtkview(self):
        from mayavi import mlab
        x, y, z = [], [], []
        # poner todos los putos de los perfiles, en listas x,y,z
        for i in self.blade:
            for point in i['airfoil'].coordinates:
                x.append(point[0])
                y.append(i['position'][2])
                z.append(point[1])

        fig = mlab.figure(1, bgcolor=(1, 0.7, 1), fgcolor=(0.5, 0.5, 0.5))

        # configuacion de la vista, de la ventana o algo por el estilo
        vtk_source = mlab.points3d(x, y, z, opacity=0.3, mode='2dvertex')
        vtk_source.actor.property.point_size = 3

        # mayar y visualizar
        delaunay = mlab.pipeline.delaunay2d(vtk_source)
        edges = mlab.pipeline.extract_edges(delaunay)
        surf = mlab.pipeline.surface(edges, colormap='jet')
        mlab.show()

    def __OCCBladeModel(self):
        #FIXME revisar completo
        """



        :rtype : object
        :return:
        """
        faces = []
        for i in self.blade:
            coordinates = i['airfoil'].coordinates
            plane = gp_Pln(gp_Pnt(0., i['position'][2], 0.), gp_Dir(0., 1., 0.))
            j = 1
            array = TColgp_Array1OfPnt2d(1, len(coordinates))
            for point in coordinates:
                x = point[0]
                z = point[2]
                array.SetValue(j, gp_Pnt2d(x, z))
                j += 1

            curve = Geom2dAPI_PointsToBSpline(array).Curve()
            spline = geomapi.To3d(curve, plane)
            edge = make_edge(spline)
            wire = make_wire(edge)
            faces.append(wire)

        blade = BRepOffsetAPI_ThruSections(True, False)
        map(blade.AddWire, faces)
        blade.Build()

        my_Trsf = gp_Trsf()
        blade = BRepBuilderAPI_Transform(blade.Shape(), my_Trsf).Shape()

        return blade

    def exportToStl(self, file):
        """

        :param file:
        """
        blade = self.__OCCBladeModel()
        stl_output_file = file
        stl_ascii_format = False

        stl_export = StlAPI_Writer()
        stl_export.Write(blade, stl_output_file, stl_ascii_format)

    def exportToSTEP(self, file, schema="AP214"):
        """

        :param file:
        """
        print('Creating "' + file + '" this may take awhile')
        blade = self.__OCCBladeModel

        # initialize the STEP exporter
        step_writer = STEPControl_Writer()
        Interface_Static_SetCVal("write.step.schema", schema)

        # transfer shapes and write file
        step_writer.Transfer(blade, STEPControl_AsIs)
        status = step_writer.Write(file)
        assert (status == IFSelect_RetDone)

    def exportToAnsys(self, file='coordinates.txt'):
        f = open(file, 'w')
        for i in self.blade:
            for point in i['airfoil'].coordinates:
                f.write('1 %s %s %s\n' % (point[0], i['position'][2], point[1]))


class Rotor(object):
    def __init__(self, rotor):
        self.rotor = rotor
        self.radius = self.rotor['blade'].length + self.rotor['hub_radius']
        self.diameter = self.radius * 2
        self.hub_length = self.rotor['hub_length']
        self.hub_radius = self.rotor['hub_radius']

    def __OCCRotorModel(self):
        angular = np.radians(360 / self.rotor['n_blades'])
        blade = self.rotor['blade']
        bladeShape = blade.__OCCBladeModel()
        shapes = []
        my_Trsf = gp_Trsf()

        theSphere = BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0),
                                           self.rotor['blade'].blade[0]['airfoil'].coordinates.max()).Shape()
        shapes.append(theSphere)

        for i in range(0, self.rotor['n_blades']):
            my_Trsf.SetRotation(gp_OZ(), i * angular)
            # my_Trsf.SetDisplacement(self.rotor['hub_radius'],gp_OZ())
            shapes.append(BRepBuilderAPI_Transform(bladeShape, my_Trsf).Shape())
        return shapes

    def view(self):
        display, start_display, add_menu, add_function_to_menu = init_display()
        display.DisplayShape(self.__OCCRotorModel())
        display.FitAll()
        start_display()

    def exportToSTEP(self, file, schema="AP214"):
        print('Creating "' + file + '" this may take awhile')
        shapes = self.__OCCRotorModel()

        rotor = shapes[0]
        for shape in shapes:
            rotor = BRepAlgoAPI_Fuse(rotor, shape).Shape()

        # initialize the STEP exporter
        step_writer = STEPControl_Writer()
        Interface_Static_SetCVal("write.step.schema", schema)

        # transfer shapes and write file
        step_writer.Transfer(rotor, STEPControl_AsIs)
        status = step_writer.Write(file)
        assert (status == IFSelect_RetDone)

    def exportToStl(self, file):
        shapes = self.__OCCRotorModel()

        rotor = shapes[0]
        for shape in shapes:
            rotor = BRepAlgoAPI_Fuse(rotor, shape).Shape()

        stl_output_file = file
        stl_ascii_format = False

        stl_export = StlAPI_Writer()
        stl_export.Write(rotor, stl_output_file, stl_ascii_format)


class Foam(object):
    def __init__(self, rotor, path='OpenFoamCase'):
        self.dirs = {}
        self.path = path
        self.rotor = rotor
        self.rotorObj = Rotor(rotor)

        # PpenFoam case directories
        self.dirs['main'] = path if path.endswith('/') else path + '/'
        self.dirs['system'] = path + '/system/'
        self.dirs['constant'] = path + '/constant/'
        self.dirs['geometry'] = path + '/geometry/'
        self.dirs['polyMesh'] = path + '/constant/polyMesh/'
        self.dirs['triSurface'] = path + '/constant/triSurface/'

        self.__openFoamInit()
        self.mesh = OpenFoamBlockMesh(self)

    def __openFoamInit(self):
        try:
            os.mkdir(self.dirs['main'])
            os.mkdir(self.dirs['system'])
            os.mkdir(self.dirs['constant'])
            os.mkdir(self.dirs['polyMesh'])
        except:
            pass

        # FIXME crear clase para generar la configuracion del caso
        renderTemplate('controlDict.jinja', self.dirs['system'] + 'controlDict', '')
        renderTemplate('fvSchemes.jinja', self.dirs['system'] + 'fvSchemes', '')
        renderTemplate('fvSolution.jinja', self.dirs['system'] + 'fvSolution', '')
        # FIXME END

        # Generar y dar permiso de ejecucion a los ficheros para correr el caso de openfoam
        renderTemplate('Allclean.jinja', self.dirs['main'] + 'Allclean', '')
        subprocess.call(['chmod', '+x', self.dirs['main'] + 'Allclean'])


class OpenFoamBlockMesh(object):
    def __init__(self, study):
        self.study = study
        self.blade = self.study.rotor['blade'].blade
        self.hub = self.study.rotor['hub_radius']
        self.hub_length = self.study.rotor['hub_length']
        self.n_blades = self.study.rotor['n_blades']

        self.convert_to_meters = 1.0  # factor de conversion de unidades a metros
        self.tunnel_radius = 80  # Radio del tunel
        self.tunnel_length = [80, 80]  # Longitud del tunel, [frontal, trasera]
        self.airfoil_offset = 0.5 # distancia del perfil al perfil offset

        self.leading_edge_skewness_factor = 30
        self.leading_edge_region = 10

        self.__div_angle = np.deg2rad(360 / self.n_blades - 90)

    def topology_1(self, n_point=0):
        blade = self.blade  # coordenadas de los perfiles de la pala


        for airfoil in self.blade:
            coords = airfoil['airfoil'].coordinates
            midline = midLine(coords)

            offset_airfoil = offset(coords[:, [0, 2]], self.airfoil_offset)
            if airfoil['airfoil'].type is 'circle':
                offset_airfoil = offset_airfoil[:-100]

            p0 = midline[len(midline) * 5 / 6]
            p1 = midline[len(midline) / 2]
            p2 = midline[len(midline) * 1 / 6]
            p3 = coords[0]
            p6 = coords[len(coords)/2]


            fig = plt.figure()
            ax = fig.add_subplot(111, aspect='equal')
            ax.plot(coords[:, 0], coords[:, 2])
            ax.plot(midline[:, 0], midline[:, 2])
            ax.plot(coords[len(coords)/2, 0], coords[len(coords)/2, 2],'mo')
            ax.plot(coords[0, 0], coords[0, 2],'mo')
            ax.plot(offset_airfoil[:,0], offset_airfoil[:,1],'m')
            ax.plot(p0[0], p0[2],'mo')
            ax.plot(p1[0], p1[2],'ro')
            ax.plot(p2[0], p2[2],'bo')
            ax.plot(p3[0], p3[2],'bo')
            ax.plot(p6[0], p6[2],'bo')
            ax.grid()
            plt.show()


    def h_topology(self, init_point):
        # init_point = 0
        inc = 16
        spline_points = 0 + init_point
        arc_points = 0 + init_point
        ptos = []
        blocks = []
        splines = []
        arcss = []
        blade_face = []
        in_face = []
        out_face = []
        periodic_face_1 = []
        periodic_face_2 = []


        # bloques
        b0 = [np.array([0, 4, 5, 1] + [x + inc for x in [0, 4, 5, 1]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b1 = [np.array([1, 5, 6, 2] + [x + inc for x in [1, 5, 6, 2]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b2 = [np.array([2, 6, 7, 3] + [x + inc for x in [2, 6, 7, 3]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b3 = [np.array([3, 7, 4, 0] + [x + inc for x in [3, 7, 4, 0]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b4 = [np.array([7, 11, 8, 4] + [x + inc for x in [7, 11, 8, 4]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b5 = [np.array([4, 8, 9, 5] + [x + inc for x in [4, 8, 9, 5]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b6 = [np.array([5, 9, 10, 6] + [x + inc for x in [5, 9, 10, 6]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b7 = [np.array([6, 10, 11, 7] + [x + inc for x in [6, 10, 11, 7]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b8 = [np.array([8, 12, 13, 9] + [x + inc for x in [8, 12, 13, 9]]) + init_point, [10, 10, 10], [1, 1, 1]]
        b9 = [np.array([11, 10, 14, 15] + [x + inc for x in [11, 10, 14, 15]]) + init_point, [10, 10, 10], [1, 1, 1]]

        # dominio de la pala
        # FIXME cerrar el hub con un wall en alcho definido
        f0 = np.array([0, 1] + [x + inc for x in [1, 0]]) + init_point
        f1 = np.array([1, 2] + [x + inc for x in [2, 1]]) + init_point
        f2 = np.array([2, 3] + [x + inc for x in [3, 2]]) + init_point
        f3 = np.array([3, 0] + [x + inc for x in [0, 3]]) + init_point
        hub_face = np.array([
            [0, 4, 5, 1],
            [4, 8, 9, 5],
            [1, 5, 6, 2],
            [5, 9, 10, 6],
            [2, 6, 7, 3],
            [6, 10, 11, 7],
            [3, 7, 4, 0],
            [7, 11, 8, 4],
        ]) + init_point

        # dominio de entrada
        f4 = np.array([9, 10] + [x + inc for x in [10, 9]]) + init_point
        f5 = np.array([9, 13] + [x + inc for x in [13, 9]]) + init_point
        f6 = np.array([10, 14] + [x + inc for x in [14, 10]]) + init_point

        # dominio de salida
        f7 = np.array([8, 11] + [x + inc for x in [11, 8]]) + init_point
        f8 = np.array([8, 12] + [x + inc for x in [12, 8]]) + init_point
        f9 = np.array([11, 15] + [x + inc for x in [15, 11]]) + init_point

        # dominio periodico
        f10 = np.array([14, 15] + [x + inc for x in [15, 14]]) + init_point
        f11 = np.array([12, 13] + [x + inc for x in [13, 12]]) + init_point

        for i in self.blade:
            radial_position = self.hub + i['position'][2]

            coord = np.insert(i['airfoil'].coordinates, 2, values=radial_position, axis=1)
            offset_airfoil = offset(coord[:, [0, 1]], 0.03)
            offset_airfoil = np.insert(offset_airfoil, 2, values=radial_position, axis=1)

            control_points = self.__getControlPoint(coord, [self.leading_edge_region, 0, 0, self.leading_edge_region])

            if i['airfoil'].type is 'circle':
                offset_airfoil = offset_airfoil[:-84]

            midline = i['airfoil'].midLine()
            cp0 = midline[len(midline) * 1 / 6]
            cp1 = midline[len(midline) / 2]
            cp2 = midline[len(midline) * 5 / 6]

            # FIXME parametrizar 60 grados
            x1 = max(offset_airfoil[:, 0])
            p0 = np.array(intersection(cp0[0], cp0[1], 60, coord[:, [0, 1]], x1=x1))
            p1 = np.array(intersection(cp2[0], cp2[1], -60, coord[:, [0, 1]], x1=-x1))
            p2 = np.array(intersection(cp2[0], cp2[1], 60, coord[:, [0, 1]], x1=-x1))
            p3 = np.array(intersection(cp0[0], cp0[1], -60, coord[:, [0, 1]], x1=x1))

            p8 = np.array([self.hub_length / 2, self.hub * np.cos(self.__div_angle)])
            p9 = np.array([-self.hub_length / 2, self.hub * np.cos(self.__div_angle)])
            p10 = np.array([-self.hub_length / 2, -self.hub * np.cos(self.__div_angle)])
            p11 = np.array([self.hub_length / 2, -self.hub * np.cos(self.__div_angle)])
            # p8 = np.array([1.2 * self.hub_length / 2, self.hub * np.cos(self.__div_angle)])
            # p9 = np.array([1.2 * -self.hub_length / 2, self.hub * np.cos(self.__div_angle)])
            # p10 = np.array([1.2 * -self.hub_length / 2, -self.hub * np.cos(self.__div_angle)])
            # p11 = np.array([1.2 * self.hub_length / 2, -self.hub * np.cos(self.__div_angle)])

            arc0 = arc3points(cp0, p0[0], p8)
            arc1 = arc3points(cp2, p1[0], p9)
            arc2 = arc3points(cp2, p2[0], p10)
            arc3 = arc3points(cp0, p3[0], p11)

            arc4 = np.matrix(
                zip(np.linspace(p8[0], p11[0], 100), np.linspace(p8[1], p11[1], 100), [radial_position] * 100))
            arc5 = np.matrix(
                [np.linspace(p9[0], p10[0], 100), np.linspace(p9[1], p10[1], 100), [radial_position] * 100, ]).T

            arc0 = arc0[~np.isnan(arc0).any(axis=1)]
            arc1 = arc1[~np.isnan(arc1).any(axis=1)]
            arc2 = arc2[~np.isnan(arc2).any(axis=1)]
            arc3 = arc3[~np.isnan(arc3).any(axis=1)]

            p4 = nearest(arc0, offset_airfoil[:, [0, 1]])
            p5 = nearest(arc1, offset_airfoil[:, [0, 1]])
            p6 = nearest(arc2, offset_airfoil[:, [0, 1]])
            p7 = nearest(arc3, offset_airfoil[:, [0, 1]])

            cp3 = nearest(arc0, coord[:, [0, 1]])[2]
            cp4 = nearest(arc1, coord[:, [0, 1]])[2]
            cp5 = nearest(arc2, coord[:, [0, 1]])[2]
            cp6 = nearest(arc3, coord[:, [0, 1]])[2]

            p12 = -radial_position * np.cos(self.__div_angle), radial_position * np.sin(self.__div_angle), p9[0]
            p13 = -radial_position * np.cos(self.__div_angle), radial_position * np.sin(self.__div_angle), p8[0]
            p14 = radial_position * np.cos(self.__div_angle), radial_position * np.sin(self.__div_angle), p8[0]
            p15 = radial_position * np.cos(self.__div_angle), radial_position * np.sin(self.__div_angle), p9[0]

            # fig = plt.figure()
            # ax = fig.add_subplot(111, aspect='equal')
            # ax.plot(coord[:, 0], coord[:, 1])
            # ax.plot(offset_airfoil[:, 0], offset_airfoil[:, 1])
            # ax.plot(midline[:, 0], midline[:, 1])
            #
            # # HUB
            # ax.plot([self.hub_length / 2, -self.hub_length / 2], [self.hub / 2, self.hub / 2],
            #         [-self.hub_length / 2, -self.hub_length / 2], [self.hub / 2, -self.hub / 2],
            #         [-self.hub_length / 2, self.hub_length / 2], [-self.hub / 2, -self.hub / 2],
            #         [self.hub_length / 2, self.hub_length / 2], [-self.hub / 2, self.hub / 2])
            #
            #
            # # DOMINIO
            # ax.plot([p8[0], p9[0]], [p8[1], p9[1]],
            #         [p9[0], p10[0]], [p9[1], p10[1]],
            #         [p10[0], p11[0]], [p10[1], p11[1]],
            #         [p11[0], p8[0]], [p11[1], p8[1]])
            #
            # ax.plot(p0[0][0], p0[0][1], 'o')
            # ax.plot(p1[0][0], p1[0][1], 'o')
            # ax.plot(p2[0][0], p2[0][1], 'o')
            # ax.plot(p3[0][0], p3[0][1], 'o')
            # #
            # ax.plot(cp0[0], cp0[1], 'ro')
            # ax.plot(cp1[0], cp1[1], 'bo')
            # ax.plot(cp2[0], cp2[1], 'mo')
            # ax.plot(arc0[cp3:, 0], arc0[cp3:, 1], 'r--')
            # ax.plot(arc1[cp4:, 0], arc1[cp4:, 1], 'r--')
            # ax.plot(arc2[cp5:, 0], arc2[cp5:, 1], 'r--')
            # ax.plot(arc3[cp6:, 0], arc3[cp6:, 1], 'r--')
            # #
            # ax.plot(p4[0][0], p4[0][1], 'mo')
            # ax.plot(p5[0][0], p5[0][1], 'mo')
            # ax.plot(p6[0][0], p6[0][1], 'mo')
            # ax.plot(p7[0][0], p7[0][1], 'mo')
            # ax.plot(p8[0], p8[1], 'mo')
            # ax.plot(p9[0], p9[1], 'mo')
            # ax.plot(p10[0], p10[1], 'mo')
            # ax.plot(p11[0], p11[1], 'mo')
            #
            # ax.grid()
            # plt.show()

            coord = rotate3d(coord, 90, 'x')
            coord = rotate3d(coord, -90, 'y')
            offset_airfoil = rotate3d(offset_airfoil, 90, 'x')
            offset_airfoil = rotate3d(offset_airfoil, -90, 'y')

            coord = blend(coord, radial_position)
            offset_airfoil = blend(offset_airfoil, radial_position)

            pt = map(lambda x: np.insert(x, 2, values=radial_position),
                     [p0[0], p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8, p9, p10, p11])
            pt = rotate3d(pt, 90, 'x')
            pt = rotate3d(pt, -90, 'y')
            pt = blend(pt, radial_position)
            pt = np.array(pt + [p12, p13, p14, p15])

            arcs = map(lambda x: np.insert(x, 2, values=radial_position, axis=1), [arc0, arc1, arc2, arc3])
            arcs = map(lambda x: rotate3d(x, 90, 'x'), arcs)
            arcs = map(lambda x: rotate3d(x, -90, 'y'), arcs)
            arcs = map(lambda x: blend(x, radial_position), arcs)

            ptos.append(pt)

            blocks.append(
                (b0, b1, b2, b3, b4, b5, b6, b7, b8, b9)
            )

            b0 = [b0[0] + inc, b0[1], b0[2]]
            b1 = [b1[0] + inc, b1[1], b1[2]]
            b2 = [b2[0] + inc, b2[1], b2[2]]
            b3 = [b3[0] + inc, b3[1], b3[2]]
            b4 = [b4[0] + inc, b4[1], b4[2]]
            b5 = [b5[0] + inc, b5[1], b5[2]]
            b6 = [b6[0] + inc, b6[1], b6[2]]
            b7 = [b7[0] + inc, b7[1], b7[2]]
            b8 = [b8[0] + inc, b8[1], b8[2]]
            b9 = [b9[0] + inc, b9[1], b9[2]]
            #
            splines.append(
                (
                    ((spline_points, spline_points + 1),
                     (
                         coord[p0[1]:p1[1]]
                     )),
                    ((spline_points + 1, spline_points + 2),
                     (
                         coord[p1[1]:p2[1]]
                     )),
                    ((spline_points + 2, spline_points + 3),
                     (
                         coord[p2[1]:p3[1]]
                     )),
                    ((spline_points + 3, spline_points),
                     (
                         [i for i in [coord[p3[1]:] + coord[:p0[1]]]][0]
                     )),
                    ((spline_points + 4, spline_points + 5),
                     (
                         offset_airfoil[p4[1]:p5[1]]
                     )),
                    ((spline_points + 5, spline_points + 6),
                     (
                         offset_airfoil[p5[1]:p6[1]]
                     )),
                    ((spline_points + 6, spline_points + 7),
                     (
                         offset_airfoil[p6[1]:p7[1]]
                     )),
                    ((spline_points + 7, spline_points + 4),
                     (
                         [i for i in [offset_airfoil[p7[1]:] + offset_airfoil[:p4[1]]]][0]
                     )),
                    ((spline_points, spline_points + 4),
                     (
                         arcs[0][cp3:p4[2]]
                     )),
                    ((spline_points + 4, spline_points + 8),
                     (
                         arcs[0][p4[2]:]
                     )),
                    ((spline_points + 1, spline_points + 5),
                     (
                         arcs[1][cp4:p5[2]]
                     )),
                    ((spline_points + 5, spline_points + 9),
                     (
                         arcs[1][p5[2]:]
                     )),
                    ((spline_points + 2, spline_points + 6),
                     (
                         arcs[2][cp5:p6[2]]
                     )),
                    ((spline_points + 6, spline_points + 10),
                     (
                         arcs[2][p6[2]:]
                     )),
                    ((spline_points + 3, spline_points + 7),
                     (
                         arcs[3][cp6:p7[2]]
                     )),
                    ((spline_points + 7, spline_points + 11),
                     (
                         arcs[3][p7[2]:]
                     )),
                    # ((spline_points + 8, spline_points + 11),
                    #  (
                    #      arc4
                    #  )),
                    # ((spline_points + 9, spline_points + 10),
                    #  (
                    #      arc5
                    #  ))
                )
            )
            spline_points += inc

            arcss.append([(arc_points + 8, arc_points + 11), (0, radial_position, pt[8][2])])
            arcss.append([(arc_points + 9, arc_points + 10), (0, radial_position, pt[9][2])])

            if not (round(pt[9][0], 3), round(pt[9][1], 3)) == (round(p13[0], 3), round(p13[1], 3)):
                tmp_center = arc2points((0, 0), (pt[9][0], pt[9][1]), p13[:2])[25].tolist()
                arcss.append([(arc_points + 9, arc_points + 13), (tmp_center[0], tmp_center[1], pt[9][2])])
            if not (round(pt[8][0], 3), round(pt[9][1], 3)) == (round(p12[0], 3), round(p12[1], 3)):
                tmp_center = arc2points((0, 0), (pt[8][0], pt[9][1]), p12[:2])[25].tolist()
                arcss.append([(arc_points + 8, arc_points + 12), (tmp_center[0], tmp_center[1], pt[8][2])])
            #
            if not (round(pt[10][0], 3), round(pt[10][1], 3)) == (round(p14[0], 3), round(p14[1], 3)):
                tmp_center = arc2points((0, 0), (pt[10][0], pt[10][1]), p14[:2])[25].tolist()
                arcss.append([(arc_points + 10, arc_points + 14), (tmp_center[0], tmp_center[1], pt[9][2])])
            if not (round(pt[11][0], 3), round(pt[11][1], 3)) == (round(p15[0], 3), round(p15[1], 3)):
                tmp_center = arc2points((0, 0), (pt[11][0], pt[11][1]), p15[:2])[25].tolist()
                arcss.append([(arc_points + 11, arc_points + 15), (tmp_center[0], tmp_center[1], pt[8][2])])

            arc_points += inc

            blade_face.append([f0, f1, f2, f3])
            in_face.append([f4, f5, f6])
            out_face.append([f7, f8, f9])
            periodic_face_1.append(f10)
            periodic_face_2.append(f11)

            f0 = f0 + inc
            f1 = f1 + inc
            f2 = f2 + inc
            f3 = f3 + inc
            f4 = f4 + inc
            f5 = f5 + inc
            f6 = f6 + inc
            f7 = f7 + inc
            f8 = f8 + inc
            f9 = f9 + inc
            f10 = f10 + inc
            f11 = f11 + inc

            ggi = [j for i in [[j.tolist() for i in in_face[:-1] for j in i],  # entrada
                               [j.tolist() for i in out_face[:-1] for j in i],  # salida
                               [i[0][0:4].tolist() for i in blocks[-1]]]  # parte de arriba
                   for j in i]
        faces = [
            (('Blade'), ('wall'),
             (
                 hub_face.tolist() + [j.tolist() for i in blade_face[:-1] for j in i]
             )
             ),
            (('Blade_GGI'), ('patch'),
             (
                 ggi,
             )
             ),
            (('Blade_Periodic_1'), ('cyclicGgi'),
             (
                 periodic_face_1[:-1]
             )
             ),
            (('Blade_Periodic_2'), ('cyclicGgi'),
             (
                 periodic_face_2[:-1]
             )
             ),
        ]

        data = {
            'convertToMeters': self.convert_to_meters,
            'vertices': [j for i in ptos for j in i],
            'arcs': arcss,
            'splines': [j for i in splines for j in i],
            'blocks': [j for i in blocks[:-1] for j in i],
            'faces': faces
        }
        # renderTemplate('blockMeshDict.jinja', self.study.dirs['polyMesh'] + 'blockMeshDict_pala', data)
        return data

    def tunnel(self):

        r = self.tunnel_radius
        l = self.tunnel_length
        gap = .01

        r1 = gap + (self.study.rotor['blade'].length + self.hub)
        l1 = gap + self.hub_length / 2

        p0 = 0., 0., l[0]
        p1 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l[0]
        p2 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l[0]
        p3 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l[0]
        p4 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l[0]

        p5 = 0., 0., l1
        p6 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l1
        p7 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l1
        p8 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l1
        p9 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l1

        p10 = 0., 0., -l1
        p11 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l1
        p12 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l1
        p13 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l1
        p14 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l1

        p15 = 0., 0., -l[1]
        p16 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l[1]
        p17 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l[1]
        p18 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l[1]
        p19 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l[1]

        ptos = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17, p18, p19]

        b0 = [(0, 5, 5, 0, 1, 6, 9, 4), (20, 20, 20), (1, 1, 1)]
        b1 = [(1, 6, 9, 4, 2, 7, 8, 3), (20, 20, 20), (1, 1, 1)]
        b2 = [(6, 11, 14, 9, 7, 12, 13, 8), (20, 20, 20), (1, 1, 1)]
        b3 = [(11, 16, 19, 14, 12, 17, 18, 13), (20, 20, 20), (1, 1, 1)]
        b4 = [(10, 15, 15, 10, 11, 16, 19, 14), (20, 20, 20), (1, 1, 1)]

        arc0 = [(1, 4), (p0[0], r1, p0[2])]
        arc1 = [(2, 3), (p0[0], r, p0[2])]
        arc2 = [(6, 9), (p5[0], r1, p5[2])]
        arc3 = [(7, 8), (p5[0], r, p5[2])]
        arc4 = [(11, 14), (p10[0], r1, p10[2])]
        arc5 = [(12, 13), (p10[0], r, p10[2])]
        arc6 = [(16, 19), (p15[0], r1, p15[2])]
        arc7 = [(17, 18), (p15[0], r, p15[2])]

        faces = [
            (('Tunel_Inlet'), ('patch'),
             (
                 (0, 1, 4, 0),
                 (1, 2, 3, 4),
             )
             ),
            (('Tunel_Outlet'), ('patch'),
             (
                 (15, 16, 19, 15),
                 (16, 17, 18, 19),
             )
             ),
            (('Tunel_Tip'), ('patch'),
             (
                 (2, 7, 8, 3),
                 (7, 12, 13, 8),
                 (12, 17, 18, 13),
             )
             ),
            (('Tunel_Periodic'), ('patch'),
             (
                 # right
                 (0, 5, 6, 1),
                 (1, 6, 7, 2),
                 (6, 11, 12, 7),
                 (10, 15, 16, 11),
                 (11, 16, 17, 12),

                 # left
                 (0, 5, 9, 4),
                 (4, 9, 8, 3),
                 (9, 14, 13, 8),
                 (14, 19, 18, 13),
                 (10, 15, 19, 14),
             )
             ),
            (('Tunel_GGI'), ('patch'),
             (
                 (5, 6, 9, 5),
                 (10, 11, 14, 10),
                 (6, 11, 14, 9),
             )
             ),
            # (('Tunel_Blade_Domain_Inlet'), ('patch'),
            #  (
            #      (5, 6, 9, 5),
            #  )
            #  ),
            # (('Tunel_Blade_Domain_Outlet'), ('patch'),
            #  (
            #      (10, 11, 14, 10),
            #  )
            #  ),
            # (('Tunel_Blade_Domain_Tip'), ('patch'),
            #  (
            #      (6, 11, 14, 9),
            #  )
            #  ),
        ]

        blade = self.h_topology(len(ptos))
        # blade = self.h_topology(0)

        data = {
            'convertToMeters': self.convert_to_meters,
            'vertices': ptos + blade['vertices'],
            'blocks': [b0, b1, b2, b3, b4] + blade['blocks'],
            'arcs': [arc0, arc1, arc2, arc3, arc4, arc5, arc6, arc7] + blade['arcs'],
            'splines': blade['splines'],
            'faces': faces + blade['faces'],
        }
        # data = {
        #     'convertToMeters': self.convert_to_meters,
        #     'vertices': ptos,
        #     'blocks': [b0, b1, b2, b3, b4],
        #     'arcs': [arc0, arc1, arc2, arc3, arc4, arc5, arc6, arc7],
        #     'faces': faces,
        # }
        renderTemplate('blockMeshDict.jinja', self.study.dirs['polyMesh'] + 'blockMeshDict', data)

        rundata = {
            'n_blades': self.n_blades - 1,
            'rotate': [(round(np.cos(np.deg2rad(i * 360 / self.n_blades)), 8),
                        round(np.sin(np.deg2rad(i * 360 / self.n_blades)), 8),
                        0) for i in range(1, self.n_blades)]
        }

        renderTemplate('Allrun.jinja', self.study.dirs['main'] + 'Allrun', rundata)
        subprocess.call(['chmod', '+x', self.study.dirs['main'] + 'Allrun'])

    def tunnel_con_hueco(self):
        r = self.tunnel_radius
        rh = self.study.rotor['hub_radius']
        l = self.tunnel_length
        gap = .01

        r1 = gap + (self.study.rotor['blade'].length[2] + self.hub)
        l1 = gap + self.hub_length * 0.5
        l2 = gap + self.hub_length / 2

        p0 = 0., 0., l[0]
        p1 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l[0]
        p2 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l[0]
        p3 = 0, -r1, l[0]
        p4 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l[0]
        p5 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l[0]
        p6 = 0, -r, l[0]

        p7 = 0., 0., l2
        p8 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l2
        p9 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l2
        p10 = 0, -r1, l2
        p11 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l2
        p12 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l2
        p13 = 0, -r, l2

        p14 = 0., 0., -l1
        p15 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l1
        p16 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l1
        p17 = 0, -r1, -l1
        p18 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l1
        p19 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l1
        p20 = 0, -r, -l1

        p21 = 0., 0., -l[1]
        p22 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l[1]
        p23 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l[1]
        p24 = 0, -r1, -l[1]
        p25 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l[1]
        p26 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l[1]
        p27 = 0, -r, -l[1]

        p28 = rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), l[0]
        p29 = -rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), l[0]

        p30 = rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), l2
        p31 = -rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), l2

        p32 = rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), -l1
        p33 = -rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), -l1

        p34 = rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), -l[1]
        p35 = -rh * np.cos(self.__div_angle), rh * np.sin(self.__div_angle), -l[1]

        ptos = [p0, p1, p2, p3, p4, p5, p6,
                p7, p8, p9, p10, p11, p12, p13,
                p14, p15, p16, p17, p18, p19, p20,
                p21, p22, p23, p24, p25, p26, p27,
                p28, p29, p30, p31, p32, p33, p34, p35]

        b0 = [(28, 30, 31, 29, 1, 8, 9, 2), (20, 20, 20), (1, 1, 1)]
        b1 = [(1, 8, 9, 2, 4, 11, 12, 5), (20, 20, 20), (1, 1, 1)]
        b2 = [(0, 7, 7, 0, 2, 9, 10, 3), (20, 20, 20), (1, 1, 1)]
        b3 = [(2, 9, 10, 3, 5, 12, 13, 6), (20, 20, 20), (1, 1, 1)]
        b4 = [(0, 7, 7, 0, 3, 10, 8, 1), (20, 20, 20), (1, 1, 1)]
        b5 = [(3, 10, 8, 1, 6, 13, 11, 4), (20, 20, 20), (1, 1, 1)]

        b6 = [(8, 15, 16, 9, 11, 18, 19, 12), (20, 20, 20), (1, 1, 1)]
        b7 = [(9, 16, 17, 10, 12, 19, 20, 13), (20, 20, 20), (1, 1, 1)]
        b8 = [(10, 17, 15, 8, 13, 20, 18, 11), (20, 20, 20), (1, 1, 1)]

        b9 = [(32, 34, 35, 33, 15, 22, 23, 16), (20, 20, 20), (1, 1, 1)]
        b10 = [(15, 22, 23, 16, 18, 25, 26, 19), (20, 20, 20), (1, 1, 1)]
        b11 = [(14, 21, 21, 14, 16, 23, 24, 17), (20, 20, 20), (1, 1, 1)]
        b12 = [(16, 23, 24, 17, 19, 26, 27, 20), (20, 20, 20), (1, 1, 1)]
        b13 = [(14, 21, 21, 14, 17, 24, 22, 15), (20, 20, 20), (1, 1, 1)]
        b14 = [(17, 24, 22, 15, 20, 27, 25, 18), (20, 20, 20), (1, 1, 1)]

        arc0 = [(1, 2), (p0[0], r1, p0[2])]
        arc1 = [(2, 3), (-r1, p0[1], p0[2])]
        arc2 = [(1, 3), (r1, p0[1], p0[2])]
        arc3 = [(4, 5), (p0[0], r, p0[2])]
        arc4 = [(5, 6), (-r, p0[1], p0[2])]
        arc5 = [(4, 6), (r, p0[1], p0[2])]

        arc6 = [(8, 9), (p7[0], r1, p7[2])]
        arc7 = [(9, 10), (-r1, p7[1], p7[2])]
        arc8 = [(8, 10), (r1, p7[1], p7[2])]
        arc9 = [(11, 12), (p7[0], r, p7[2])]
        arc10 = [(12, 13), (-r, p7[1], p7[2])]
        arc11 = [(11, 13), (r, p7[1], p7[2])]

        arc12 = [(15, 16), (p14[0], r1, p14[2])]
        arc13 = [(16, 17), (-r1, p14[1], p14[2])]
        arc14 = [(15, 17), (r1, p14[1], p14[2])]
        arc15 = [(18, 19), (p14[0], r, p14[2])]
        arc16 = [(19, 20), (-r, p14[1], p14[2])]
        arc17 = [(20, 18), (r, p14[1], p14[2])]

        arc18 = [(22, 23), (p21[0], r1, p21[2])]
        arc19 = [(23, 24), (-r1, p21[1], p21[2])]
        arc20 = [(22, 24), (r1, p21[1], p21[2])]
        arc21 = [(25, 26), (p21[0], r, p21[2])]
        arc22 = [(26, 27), (-r, p21[1], p21[2])]
        arc23 = [(25, 27), (r, p21[1], p21[2])]

        arc24 = [(28, 29), (-rh, p0[1], p0[2])]
        arc25 = [(30, 31), (-rh, p7[1], p7[2])]
        arc26 = [(32, 33), (-rh, p14[1], p14[2])]
        arc27 = [(34, 35), (-rh, p21[1], p21[2])]

        faces = [
            (('Tunel_Inlet'), ('patch'),
             (
                 (1, 4, 5, 2),
                 (28, 29, 2, 1),
             )
             ),
            (('Tunel_Outlet'), ('patch'),
             (
                 (22, 25, 26, 23),
                 (34, 35, 23, 22),
             )
             ),
            (('Tunel_Tip'), ('patch'),
             (
                 (4, 11, 12, 5),
                 (11, 18, 19, 12),
                 (18, 25, 26, 19),
             )
             ),
            (('Tunel_GGI'), ('patch'),
             (
                 (8, 15, 16, 9),
                 (30, 31, 9, 8),
                 (32, 33, 16, 15),
             )
             ),
            (('Tunel_Periodic'), ('patch'),
             (
                 (28, 30, 31, 29),
                 (28, 30, 8, 1),
                 (1, 4, 11, 8),
                 (32, 33, 35, 34),
                 (32, 34, 22, 15),
                 (15, 22, 25, 18),
                 (8, 15, 18, 11),
             )
             ),
        ]

        # blade = self.h_topology(len(ptos))
        # blade = self.h_topology_completa(0)

        # 'blocks': [b0, b1, b2, b3, b4, b5,
        #            b6, b7, b8,
        #            b9, b10, b11, b12, b13, b14] + blade['blocks'],

        # data = {
        #     'convertToMeters': self.convert_to_meters,
        #     'vertices': ptos + blade['vertices'],
        #     'blocks': [b0, b1, b9,
        #                b6,
        #                b10] + blade['blocks'],
        #     'arcs': [arc0, arc1, arc2, arc3, arc4, arc5,
        #              arc6, arc7, arc8, arc9, arc10, arc11,
        #              arc12, arc13, arc14, arc15, arc16, arc17,
        #              arc18, arc19, arc20, arc21, arc22, arc23, arc24, arc25, arc26, arc27,
        #              ] + blade['arcs'],
        #     'splines': blade['splines'],
        #     'faces': faces + blade['faces'],
        # }
        #
        data = {
            'convertToMeters': self.convert_to_meters,
            'vertices': ptos,
            'blocks': [b0, b1, b9,
                       b6,
                       b10],
            'arcs': [arc0, arc1, arc2, arc3, arc4, arc5,
                     arc6, arc7, arc8, arc9, arc10, arc11,
                     arc12, arc13, arc14, arc15, arc16, arc17,
                     arc18, arc19, arc20, arc21, arc22, arc23, arc24, arc25, arc26, arc27,
                     ],
            'faces': faces,
        }
        renderTemplate('blockMeshDict.jinja', self.study.dirs['polyMesh'] + 'blockMeshDict', data)

    def h_topology_completa(self, init_point):
        # init_point = 0
        inc = 12
        r = []
        spline_points = 0 + init_point
        arc_points = 0 + init_point
        ptos = []
        blocks = []
        splines = []
        arcss = []
        blade_face = []
        in_face = []
        out_face = []
        periodic_face = []

        # bloques
        b0 = [np.array([0, 4, 5, 1] + [x + inc for x in [0, 4, 5, 1]]) + init_point, [20, 20, 20], [1, 1, 1]]
        b1 = [np.array([1, 5, 6, 2] + [x + inc for x in [1, 5, 6, 2]]) + init_point, [20, 20, 20], [1, 1, 1]]
        b2 = [np.array([2, 6, 7, 3] + [x + inc for x in [2, 6, 7, 3]]) + init_point, [20, 20, 20], [1, 1, 1]]
        b3 = [np.array([3, 7, 4, 0] + [x + inc for x in [3, 7, 4, 0]]) + init_point, [20, 20, 20], [1, 1, 1]]
        b4 = [np.array([7, 11, 8, 4] + [x + inc for x in [7, 11, 8, 4]]) + init_point, [20, 20, 20], [1, 1, 1]]
        b5 = [np.array([4, 8, 9, 5] + [x + inc for x in [4, 8, 9, 5]]) + init_point, [20, 20, 20], [1, 1, 1]]
        b6 = [np.array([5, 9, 10, 6] + [x + inc for x in [5, 9, 10, 6]]) + init_point, [20, 20, 20], [1, 1, 1]]
        b7 = [np.array([6, 10, 11, 7] + [x + inc for x in [6, 10, 11, 7]]) + init_point, [20, 20, 20], [1, 1, 1]]

        # dominio de la pala
        # FIXME cerrar el hub con un wall en alcho definido
        f0 = np.array([0, 1] + [x + inc for x in [1, 0]]) + init_point
        f1 = np.array([1, 2] + [x + inc for x in [2, 1]]) + init_point
        f2 = np.array([2, 3] + [x + inc for x in [3, 2]]) + init_point
        f3 = np.array([3, 0] + [x + inc for x in [0, 3]]) + init_point
        hub_face = np.array([
            [0, 4, 5, 1],
            [4, 8, 9, 5],
            [1, 5, 6, 2],
            [5, 9, 10, 6],
            [2, 6, 7, 3],
            [6, 10, 11, 7],
            [3, 7, 4, 0],
            [7, 11, 8, 4],
        ]) + init_point

        tmp = inc * len(self.blade)
        # dominio de entrada
        f4 = np.array([9, 10] + [x + inc for x in [10, 9]]) + init_point
        f5 = np.array([9 + tmp, 10] + [x + inc for x in [10, 9 + tmp]]) + init_point
        f6 = np.array([9 + tmp, 10 + tmp] + [x + inc for x in [10 + tmp, 9 + tmp]]) + init_point
        f7 = np.array([9 + 2 * tmp, 10 + tmp] + [x + inc for x in [10 + tmp, 9 + 2 * tmp]]) + init_point
        f8 = np.array([9 + 2 * tmp, 10 + 2 * tmp] + [x + inc for x in [10 + 2 * tmp, 9 + 2 * tmp]]) + init_point
        f9 = np.array([9, 10 + 2 * tmp] + [x + inc for x in [10 + 2 * tmp, 9]]) + init_point

        # # dominio de salida
        f10 = np.array([8, 11] + [x + inc for x in [11, 8]]) + init_point
        f11 = np.array([8 + tmp, 11] + [x + inc for x in [11, 8 + tmp]]) + init_point
        f12 = np.array([8 + tmp, 11 + tmp] + [x + inc for x in [11 + tmp, 8 + tmp]]) + init_point
        f13 = np.array([8 + 2 * tmp, 11 + tmp] + [x + inc for x in [11 + tmp, 8 + 2 * tmp]]) + init_point
        f14 = np.array([8 + 2 * tmp, 11 + 2 * tmp] + [x + inc for x in [11 + 2 * tmp, 8 + 2 * tmp]]) + init_point
        f15 = np.array([8, 11 + 2 * tmp] + [x + inc for x in [11 + 2 * tmp, 8]]) + init_point

        for i in self.blade:
            radial_position = self.hub + i['position'][2]

            coord = np.insert(i['airfoil'].coordinates, 2, values=radial_position, axis=1)
            offset_airfoil = offset(coord[:, [0, 1]], 0.05)
            offset_airfoil = np.insert(offset_airfoil, 2, values=radial_position, axis=1)

            if i['airfoil'].type is 'circle':
                offset_airfoil = offset_airfoil[:-84]

            midline = i['airfoil'].midLine()
            cp0 = midline[len(midline) * 1 / 6]
            cp1 = midline[len(midline) / 2]
            cp2 = midline[len(midline) * 5 / 6]

            # FIXME parametrizar 60 grados
            x1 = max(offset_airfoil[:, 0])
            p0 = intersection(cp0[0], cp0[1], 60, coord[:, [0, 1]], x1=x1)
            p1 = intersection(cp2[0], cp2[1], -60, coord[:, [0, 1]], x1=-x1)
            p2 = intersection(cp2[0], cp2[1], 60, coord[:, [0, 1]], x1=-x1)
            p3 = intersection(cp0[0], cp0[1], -60, coord[:, [0, 1]], x1=x1)

            # FIXME parametrizar tamanno del hub en dependencia
            # del tamanno de los perfiles de las palas
            p8 = [self.hub_length * .75, self.hub * np.cos(self.__div_angle)]
            p9 = [-self.hub_length / 1.5, self.hub * np.cos(self.__div_angle)]
            p10 = [-self.hub_length / 1.5, -self.hub * np.cos(self.__div_angle)]
            p11 = [self.hub_length * .75, -self.hub * np.cos(self.__div_angle)]

            arc0 = arc3points(cp0, p0[0], p8)
            arc1 = arc3points(cp2, p1[0], p9)
            arc2 = arc3points(cp2, p2[0], p10)
            arc3 = arc3points(cp0, p3[0], p11)

            arc0 = arc0[~np.isnan(arc0).any(axis=1)].tolist()
            arc1 = arc1[~np.isnan(arc1).any(axis=1)].tolist()
            arc2 = arc2[~np.isnan(arc2).any(axis=1)].tolist()
            arc3 = arc3[~np.isnan(arc3).any(axis=1)].tolist()

            p4 = nearest(arc0, offset_airfoil[:, [0, 1]])
            p5 = nearest(arc1, offset_airfoil[:, [0, 1]])
            p6 = nearest(arc2, offset_airfoil[:, [0, 1]])
            p7 = nearest(arc3, offset_airfoil[:, [0, 1]])

            cp3 = nearest(arc0, coord[:, [0, 1]])[2]
            cp4 = nearest(arc1, coord[:, [0, 1]])[2]
            cp5 = nearest(arc2, coord[:, [0, 1]])[2]
            cp6 = nearest(arc3, coord[:, [0, 1]])[2]

            coord = rotate3d(coord, 90, 'x')
            coord = rotate3d(coord, -90, 'y')
            offset_airfoil = rotate3d(offset_airfoil, 90, 'x')
            offset_airfoil = rotate3d(offset_airfoil, -90, 'y')

            coord = blend(coord, radial_position)
            offset_airfoil = blend(offset_airfoil, radial_position)

            map(lambda x: x.insert(2, radial_position),
                [p0[0], p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8, p9, p10, p11])
            pt = [p0[0], p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8, p9, p10, p11]
            pt = rotate3d(pt, 90, 'x')
            pt = rotate3d(pt, -90, 'y')
            pt = blend(pt, radial_position)

            spline_arcs = map(lambda x: np.insert(x, 2, values=radial_position, axis=1).tolist(),
                              [arc0, arc1, arc2, arc3])
            spline_arcs = map(lambda x: rotate3d(x, 90, 'x'), spline_arcs)
            spline_arcs = map(lambda x: rotate3d(x, -90, 'y'), spline_arcs)
            spline_arcs = map(lambda x: blend(x, radial_position), spline_arcs)

            ptos.append(pt)

            blocks.append(
                (b0, b1, b2, b3, b4, b5, b6, b7)
            )

            b0 = [b0[0] + inc, b0[1], b0[2]]
            b1 = [b1[0] + inc, b1[1], b1[2]]
            b2 = [b2[0] + inc, b2[1], b2[2]]
            b3 = [b3[0] + inc, b3[1], b3[2]]
            b4 = [b4[0] + inc, b4[1], b4[2]]
            b5 = [b5[0] + inc, b5[1], b5[2]]
            b6 = [b6[0] + inc, b6[1], b6[2]]
            b7 = [b7[0] + inc, b7[1], b7[2]]
            #
            splines.append(
                (
                    ((spline_points, spline_points + 1),
                     (
                         coord[p0[1]:p1[1]]
                     )),
                    ((spline_points + 1, spline_points + 2),
                     (
                         coord[p1[1]:p2[1]]
                     )),
                    ((spline_points + 2, spline_points + 3),
                     (
                         coord[p2[1]:p3[1]]
                     )),
                    ((spline_points + 3, spline_points),
                     (
                         [i for i in [coord[p3[1]:] + coord[:p0[1]]]][0]
                     )),
                    ((spline_points + 4, spline_points + 5),
                     (
                         offset_airfoil[p4[1]:p5[1]]
                     )),
                    ((spline_points + 5, spline_points + 6),
                     (
                         offset_airfoil[p5[1]:p6[1]]
                     )),
                    ((spline_points + 6, spline_points + 7),
                     (
                         offset_airfoil[p6[1]:p7[1]]
                     )),
                    ((spline_points + 7, spline_points + 4),
                     (
                         [i for i in [offset_airfoil[p7[1]:] + offset_airfoil[:p4[1]]]][0]
                     )),
                    ((spline_points, spline_points + 4),
                     (
                         spline_arcs[0][cp3:p4[2]]
                     )),
                    ((spline_points + 4, spline_points + 8),
                     (
                         spline_arcs[0][p4[2]:]
                     )),
                    ((spline_points + 1, spline_points + 5),
                     (
                         spline_arcs[1][cp4:p5[2]]
                     )),
                    ((spline_points + 5, spline_points + 9),
                     (
                         spline_arcs[1][p5[2]:]
                     )),
                    ((spline_points + 2, spline_points + 6),
                     (
                         spline_arcs[2][cp5:p6[2]]
                     )),
                    ((spline_points + 6, spline_points + 10),
                     (
                         spline_arcs[2][p6[2]:]
                     )),
                    ((spline_points + 3, spline_points + 7),
                     (
                         spline_arcs[3][cp6:p7[2]]
                     )),
                    ((spline_points + 7, spline_points + 11),
                     (
                         spline_arcs[3][p7[2]:]
                     )),
                )
            )
            spline_points += inc
            arcss.append([(arc_points + 8, arc_points + 11), (0, radial_position, pt[8][2])])
            arcss.append([(arc_points + 9, arc_points + 10), (0, radial_position, pt[9][2])])
            r.append(radial_position)
            arc_points += inc

            blade_face.append([f0, f1, f2, f3])
            in_face.append([f4, f5, f6, f7, f8, f9])
            out_face.append([f10, f11, f12, f13, f14, f15])
            # periodic_face.append([f10, f11])

            f0 = f0 + inc
            f1 = f1 + inc
            f2 = f2 + inc
            f3 = f3 + inc
            f4 = f4 + inc
            f5 = f5 + inc
            f6 = f6 + inc
            f7 = f7 + inc
            f8 = f8 + inc
            f9 = f9 + inc
            f10 = f10 + inc
            f11 = f11 + inc
            f12 = f12 + inc
            f13 = f13 + inc
            f14 = f14 + inc
            f15 = f15 + inc

        vertices = []
        arcos = []
        all_splines = []
        bloques = []
        ptos = [j for i in ptos for j in i]
        n_ptos = len(ptos)
        splines = [j for i in splines for j in i]

        tip_face = [i[0][0:4].tolist() for i in blocks[-1]]
        blocks = [j for i in blocks[:-1] for j in i]

        blade_faces = []
        tip_faces = []
        for i in range(self.n_blades):
            vertices.append(rotate3d(ptos, i * 360 / self.n_blades, 'z'))
            arcos.append(
                [(tuple(np.array(arc[0]) + i * n_ptos), rotate3d(arc[1], i * 360 / self.n_blades, 'z')) for arc in
                 arcss])
            all_splines.append(
                [(tuple(np.array(spline[0]) + i * n_ptos), rotate3d(spline[1], i * 360 / self.n_blades, 'z')) for spline
                 in splines])
            bloques.append([(tuple(np.array(bloque[0]) + i * n_ptos), bloque[1], bloque[2]) for bloque in blocks])
            blade_faces.append(
                np.array(hub_face.tolist() + [j.tolist() for k in blade_face[:-1] for j in k]) + i * n_ptos)
            tip_faces.append([np.array(k) + i * n_ptos for k in tip_face])

        ocho = [8 + n_ptos, 8 + 2 * n_ptos, 8]
        nueve = [9 + n_ptos, 9 + 2 * n_ptos, 9]
        dies = [10, 10 + n_ptos, 10 + 2 * n_ptos]
        once = [11, 11 + n_ptos, 11 + 2 * n_ptos]

        nueve1 = [9 + n_ptos + inc, 9 + 2 * n_ptos + inc, 9 + inc]
        dies1 = [10 + inc, 10 + n_ptos + inc, 10 + 2 * n_ptos + inc]
        ocho1 = [8 + n_ptos + inc, 8 + 2 * n_ptos + inc, 8 + inc]
        once1 = [11 + inc, 11 + n_ptos + inc, 11 + 2 * n_ptos + inc]

        block = []
        arcs = []
        r.pop(0)
        vertices = [j for i in vertices for j in i]
        for j in xrange(len(self.blade) - 1):
            for i in range(self.n_blades):
                block.append([[once[i] + init_point + j * inc, dies[i] + init_point + j * inc,
                               nueve[i] + init_point + j * inc, ocho[i] + init_point + j * inc,
                               once[i] + init_point + j * inc + inc, dies[i] + init_point + j * inc + inc,
                               nueve[i] + init_point + j * inc + inc,
                               ocho[i] + init_point + j * inc + inc], [20, 20, 20], [1, 1, 1]])
                arcs.append([(nueve1[i] + init_point + j * inc, dies1[i] + init_point + j * inc),
                             (r[j] * np.cos(np.deg2rad(-i * 120 + 30)),
                              r[j] * np.sin(np.deg2rad(-i * 120 + 30)),
                              vertices[nueve[i] + j * inc][2])])
                arcs.append([(ocho1[i] + init_point + j * inc, once1[i] + init_point + j * inc),
                             (r[j] * np.cos(np.deg2rad(-i * 120 + 30)),
                              r[j] * np.sin(np.deg2rad(-i * 120 + 30)),
                              vertices[ocho[i] + j * inc][2])])

        ggi = [j for i in [[j.tolist() for i in in_face[:-1] for j in i],  # entrada
                           [j.tolist() for i in out_face[:-1] for j in i],
                           [j.tolist() for i in tip_faces for j in i],
                           [i[0][4:] for i in [block[-j] for j in range(1, 4)]]]  # parte de arriba
               for j in i]

        faces = [
            (('Blade'), ('wall'),
             (
                 [j.tolist() for i in blade_faces for j in i],
                 [i + init_point for i in np.array([8, 9, 10 + 2 * tmp, 11 + 2 * tmp]).tolist()]
             )
             ),
            (('Blade_GGI'), ('patch'),
             (
                 ggi
             )
             ),
        ]

        data = {
            'convertToMeters': self.convert_to_meters,
            'vertices': vertices,
            'arcs': [j for i in arcos for j in i] + arcs,
            'splines': [j for i in all_splines for j in i],
            'blocks': [j for i in bloques for j in i] + block,
            'faces': faces
        }
        # renderTemplate('blockMeshDict.jinja', self.study.dirs['polyMesh'] + 'blockMeshDict', data)
        return data

    def tunnel_completo(self):
        r = self.tunnel_radius
        l = self.tunnel_length
        gap = .001

        r1 = gap + (self.study.rotor['blade'].length[2] + self.hub)
        l1 = gap + self.hub_length * 0.75
        l2 = gap + self.hub_length / 1.5

        p0 = 0., 0., l[0]
        p1 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l[0]
        p2 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l[0]
        p3 = 0, -r1, l[0]
        p4 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l[0]
        p5 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l[0]
        p6 = 0, -r, l[0]

        p7 = 0., 0., l2
        p8 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l2
        p9 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), l2
        p10 = 0, -r1, l2
        p11 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l2
        p12 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), l2
        p13 = 0, -r, l2

        p14 = 0., 0., -l1
        p15 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l1
        p16 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l1
        p17 = 0, -r1, -l1
        p18 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l1
        p19 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l1
        p20 = 0, -r, -l1

        p21 = 0., 0., -l[1]
        p22 = r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l[1]
        p23 = -r1 * np.cos(self.__div_angle), r1 * np.sin(self.__div_angle), -l[1]
        p24 = 0, -r1, -l[1]
        p25 = r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l[1]
        p26 = -r * np.cos(self.__div_angle), r * np.sin(self.__div_angle), -l[1]
        p27 = 0, -r, -l[1]

        ptos = [p0, p1, p2, p3, p4, p5, p6,
                p7, p8, p9, p10, p11, p12, p13,
                p14, p15, p16, p17, p18, p19, p20,
                p21, p22, p23, p24, p25, p26, p27]

        b0 = [(0, 7, 7, 0, 1, 8, 9, 2), (20, 20, 20), (1, 1, 1)]
        b1 = [(1, 8, 9, 2, 4, 11, 12, 5), (20, 20, 20), (1, 1, 1)]
        b2 = [(0, 7, 7, 0, 2, 9, 10, 3), (20, 20, 20), (1, 1, 1)]
        b3 = [(2, 9, 10, 3, 5, 12, 13, 6), (20, 20, 20), (1, 1, 1)]
        b4 = [(0, 7, 7, 0, 3, 10, 8, 1), (20, 20, 20), (1, 1, 1)]
        b5 = [(3, 10, 8, 1, 6, 13, 11, 4), (20, 20, 20), (1, 1, 1)]

        b6 = [(8, 15, 16, 9, 11, 18, 19, 12), (20, 20, 20), (1, 1, 1)]
        b7 = [(9, 16, 17, 10, 12, 19, 20, 13), (20, 20, 20), (1, 1, 1)]
        b8 = [(10, 17, 15, 8, 13, 20, 18, 11), (20, 20, 20), (1, 1, 1)]

        b9 = [(14, 21, 21, 14, 15, 22, 23, 16), (20, 20, 20), (1, 1, 1)]
        b10 = [(15, 22, 23, 16, 18, 25, 26, 19), (20, 20, 20), (1, 1, 1)]
        b11 = [(14, 21, 21, 14, 16, 23, 24, 17), (20, 20, 20), (1, 1, 1)]
        b12 = [(16, 23, 24, 17, 19, 26, 27, 20), (20, 20, 20), (1, 1, 1)]
        b13 = [(14, 21, 21, 14, 17, 24, 22, 15), (20, 20, 20), (1, 1, 1)]
        b14 = [(17, 24, 22, 15, 20, 27, 25, 18), (20, 20, 20), (1, 1, 1)]

        arc0 = [(1, 2), (p0[0], r1, p0[2])]
        arc1 = [(2, 3), (-r1, p0[1], p0[2])]
        arc2 = [(1, 3), (r1, p0[1], p0[2])]
        arc3 = [(4, 5), (p0[0], r, p0[2])]
        arc4 = [(5, 6), (-r, p0[1], p0[2])]
        arc5 = [(4, 6), (r, p0[1], p0[2])]

        arc6 = [(8, 9), (p7[0], r1, p7[2])]
        arc7 = [(9, 10), (-r1, p7[1], p7[2])]
        arc8 = [(8, 10), (r1, p7[1], p7[2])]
        arc9 = [(11, 12), (p7[0], r, p7[2])]
        arc10 = [(12, 13), (-r, p7[1], p7[2])]
        arc11 = [(11, 13), (r, p7[1], p7[2])]

        arc12 = [(15, 16), (p14[0], r1, p14[2])]
        arc13 = [(16, 17), (-r1, p14[1], p14[2])]
        arc14 = [(15, 17), (r1, p14[1], p14[2])]
        arc15 = [(18, 19), (p14[0], r, p14[2])]
        arc16 = [(19, 20), (-r, p14[1], p14[2])]
        arc17 = [(20, 18), (r, p14[1], p14[2])]

        arc18 = [(22, 23), (p21[0], r1, p21[2])]
        arc19 = [(23, 24), (-r1, p21[1], p21[2])]
        arc20 = [(22, 24), (r1, p21[1], p21[2])]
        arc21 = [(25, 26), (p21[0], r, p21[2])]
        arc22 = [(26, 27), (-r, p21[1], p21[2])]
        arc23 = [(25, 27), (r, p21[1], p21[2])]

        faces = [
            (('Tunel_Inlet'), ('patch'),
             (
                 (0, 1, 2, 0),
                 (1, 4, 5, 2),
                 (0, 2, 3, 0),
                 (2, 5, 6, 3),
                 (0, 3, 1, 0),
                 (3, 6, 4, 1),
             )
             ),
            (('Tunel_Outlet'), ('patch'),
             (
                 (21, 22, 23, 21),
                 (22, 25, 26, 23),
                 (21, 23, 24, 21),
                 (23, 26, 27, 24),
                 (21, 24, 22, 21),
                 (22, 24, 27, 25),
             )
             ),
            (('Tunel_Tip'), ('patch'),
             (
                 (4, 11, 12, 5),
                 (11, 18, 19, 12),
                 (18, 25, 26, 19),
                 (6, 13, 12, 5),
                 (13, 20, 19, 12),
                 (20, 27, 26, 19),
                 (6, 13, 11, 4),
                 (13, 20, 18, 11),
                 (20, 27, 25, 18),
             )
             ),
            (('Tunel_GGI'), ('patch'),
             (
                 (7, 8, 9, 7),
                 (7, 9, 10, 7),
                 (7, 10, 8, 7),
                 (14, 15, 16, 14),
                 (14, 16, 17, 14),
                 (14, 17, 15, 14),
                 (8, 15, 16, 9),
                 (9, 16, 17, 10),
                 (15, 8, 10, 17),
             )
             ),
            # (('Tunel_Blade_Domain_Inlet'), ('patch'),
            #  (
            #      (5, 6, 9, 5),
            #  )
            #  ),
            # (('Tunel_Blade_Domain_Outlet'), ('patch'),
            #  (
            #      (10, 11, 14, 10),
            #  )
            #  ),
            # (('Tunel_Blade_Domain_Tip'), ('patch'),
            #  (
            #      (6, 11, 14, 9),
            #  )
            #  ),
        ]

        blade = self.h_topology_completa(len(ptos))
        # blade = self.h_topology_completa(0)

        data = {
            'convertToMeters': self.convert_to_meters,
            'vertices': ptos + blade['vertices'],
            'blocks': [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14] + blade['blocks'],
            'arcs': [arc0, arc1, arc2, arc3, arc4, arc5,
                     arc6, arc7, arc8, arc9, arc10, arc11,
                     arc12, arc13, arc14, arc15, arc16, arc17,
                     arc18, arc19, arc20, arc21, arc22, arc23,
                     ] + blade['arcs'],
            'splines': blade['splines'],
            'faces': faces + blade['faces'],
        }

        # data = {
        #     'convertToMeters': self.convert_to_meters,
        #     'vertices': ptos,
        #     'blocks': [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14],
        #     'arcs': [arc0, arc1, arc2, arc3, arc4, arc5,
        #              arc6, arc7, arc8, arc9, arc10, arc11,
        #              arc12, arc13, arc14, arc15, arc16, arc17,
        #              arc18, arc19, arc20, arc21, arc22, arc23,
        #              ],
        #     'faces': faces,
        # }
        renderTemplate('blockMeshDict.jinja', self.study.dirs['polyMesh'] + 'blockMeshDict', data)

    def __getControlPoint(self, coordinates, data, n=0):
        """


        :rtype : object
        :param f_up:
        :param f_down:
        :param l_up:
        :param l_down:
        :return:
        """
        arr = [i / 100. for i in data]
        p = getExtemePoints(coordinates)[2]

        len_up_profile = len(coordinates[:p])
        len_down_profile = len(coordinates[p:]) - n

        # meshDictGen airfoil vertices {6,7,13,14,18,19}
        control_f_up = int(abs(arr[0] * len_up_profile - len_up_profile))
        control_l_up = int(arr[1] * len_up_profile)
        control_f_down = int(abs(arr[2] * len_down_profile - len_down_profile))
        control_l_down = int(arr[3] * len_down_profile)
        control_m_up = int(0.5 * len_up_profile)
        control_m_down = int(0.5 * len_down_profile)

        return (control_f_up, control_m_up, control_l_up, control_f_down, control_m_down, control_l_down)
