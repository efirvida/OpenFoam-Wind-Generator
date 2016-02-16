from windturbine import *


#DDefinir perfiles
airfoil = Airfoils
NACA64618 = airfoil('naca5', denomination='64618')
circle = airfoil('circle')

#plotear perfiles
# NACA64618.plot()
# circle.plot()


#Definicion de la pala
blade = {
    'blade_twist': 0,
    'airfoils': [
        # {'airfoil': circle, 'position': [x,y,r], 'twist': angle, 'scale': scale},
        {'airfoil': circle, 'position': [0.00, 0.00, 0.00], 'twist': 0.0, 'scale': 0.6},
        {'airfoil': circle, 'position': [0.00, 0.00, 0.05], 'twist': 0.0, 'scale': 0.6},
        {'airfoil': NACA64618, 'position': [-0.15, -0.05, 0.70], 'twist': 10.0, 'scale': 2.9},
        {'airfoil': NACA64618, 'position': [-0.15, -0.05, 5.50], 'twist': 5.0, 'scale': 1.5},
    ],
}
myBlade = Blade(blade)

# myBlade.plot()

rotor = {
    'blade': myBlade,
    'n_blades': 3,
    'hub_radius': 0.4,
    'hub_length': 0.8,
}

myRotor = Rotor(rotor)
study1 = Foam(rotor=rotor, path='OpenFoamCase_Prueba_periodico')
study1.mesh.airfoil_offset = 0.1
study1.mesh.topology_1()

# study1.mesh.tunnel_radius = 10
# study1.mesh.tunnel_length = [20, 20]
# study1.mesh.tunnel()
