from windturbine import *

# DDefinir perfiles
airfoil = Airfoils
perfil = airfoil('S809.dat')
# perfil = airfoil('naca4', denomination='9523')

# plotear perfiles
# perfil.plot()

# Definicion de la pala
blade = {
    'blade_twist': 0.,
    'airfoils': [
        # {'airfoil': circle, 'position': [x,y,r], 'twist': angle, 'scale': scale},
        {'airfoil': perfil, 'position': [0.0, 0.0, 0.0000], 'twist': 0.0, 'scale': 0.218},
        {'airfoil': perfil, 'position': [0.0, 0.0, 0.5083], 'twist': 0.0, 'scale': 0.218},
        {'airfoil': perfil, 'position': [0.0, 0.0, 0.6604], 'twist': 0.0, 'scale': 0.218},
        {'airfoil': perfil, 'position': [0.0, 0.0, 0.8835], 'twist': 0.0, 'scale': 0.283},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.0085], 'twist': 6.7, 'scale': 0.349},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.0675], 'twist': 9.9, 'scale': 0.441},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.1335], 'twist': 13.4, 'scale': 0.544},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.2575], 'twist': 20.04, 'scale': 0.737},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.343], 'twist': 18.074, 'scale': 0.728},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.510], 'twist': 14.292, 'scale': 0.711},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.648], 'twist': 11.909, 'scale': 0.697},
        {'airfoil': perfil, 'position': [0.0, 0.0, 1.952], 'twist': 7.979, 'scale': 0.666},
        {'airfoil': perfil, 'position': [0.0, 0.0, 2.257], 'twist': 5.308, 'scale': 0.636},
        {'airfoil': perfil, 'position': [0.0, 0.0, 2.343], 'twist': 4.715, 'scale': 0.627},
        {'airfoil': perfil, 'position': [0.0, 0.0, 2.562], 'twist': 3.425, 'scale': 0.605},
        {'airfoil': perfil, 'position': [0.0, 0.0, 2.867], 'twist': 2.083, 'scale': 0.574},
        {'airfoil': perfil, 'position': [0.0, 0.0, 3.172], 'twist': 1.15, 'scale': 0.543},
        {'airfoil': perfil, 'position': [0.0, 0.0, 3.185], 'twist': 1.115, 'scale': 0.542},
        {'airfoil': perfil, 'position': [0.0, 0.0, 3.476], 'twist': 0.494, 'scale': 0.512},
        {'airfoil': perfil, 'position': [0.0, 0.0, 3.781], 'twist': -0.015, 'scale': 0.482},
        {'airfoil': perfil, 'position': [0.0, 0.0, 4.023], 'twist': -0.381, 'scale': 0.457},
        {'airfoil': perfil, 'position': [0.0, 0.0, 4.086], 'twist': -0.475, 'scale': 0.451},
        {'airfoil': perfil, 'position': [0.0, 0.0, 4.391], 'twist': -0.92, 'scale': 0.42},
        {'airfoil': perfil, 'position': [0.0, 0.0, 4.696], 'twist': -1.352, 'scale': 0.389},
        {'airfoil': perfil, 'position': [0.0, 0.0, 4.780], 'twist': -1.469, 'scale': 0.381},
        {'airfoil': perfil, 'position': [0.0, 0.0, 5.000], 'twist': -1.775, 'scale': 0.358},
        {'airfoil': perfil, 'position': [0.0, 0.0, 5.305], 'twist': -2.191, 'scale': 0.328},
        {'airfoil': perfil, 'position': [0.0, 0.0, 5.532], 'twist': -2.5, 'scale': 0.305},
    ],
}
myBlade = Blade(blade)

# myBlade.plot()

rotor = {
    'blade': myBlade,
    'n_blades': 3,
    'hub_radius': 2.,
    'hub_length': 1.5,
}

myRotor = Rotor(rotor)
study1 = Foam(rotor=rotor, path='OpenFoamCase_Tesis')
study1.mesh.airfoil_offset = 0.02 #<- TODO offset tiene que proporcional al tama;o del perfil
study1.mesh.rotor_disk_length = [2, 2.5]  # Longitud del disco del rotor, [frontal, trasera]
study1.mesh.topology_O()

# study1.mesh.tunnel_radius = 10
# study1.mesh.tunnel_length = [20, 20]
# study1.mesh.tunnel()
