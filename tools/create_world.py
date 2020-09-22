# This script adds a random (but configurable) number of spheres to an SDF Gazebo World file
# See command line usage below (in __main__) for syntax details

import xml.etree.ElementTree as ET
import random
import time
import sys


# This a laundry list of xml elements need to create a sphere model in Gazebo
# Mostly boilerplate code. Be careful editing. Consult this website if you must make a change:
# http://sdformat.org/spec
def sphere_tree(model, pos, sphere_radius, mass_val):

    pose = ET.SubElement(model, 'pose')
    pose.set('frame', '')
    pose.text =  pos

    static = ET.SubElement(model, 'static')
    static.text = 'false'

    link = ET.SubElement(model, 'link')
    link.set('name', 'link')
    inertial = ET.SubElement(link, 'inertial')
    mass = ET.SubElement(inertial, 'mass')
    mass.text = mass_val
    inertia = ET.SubElement(inertial, 'inertia')

    ixx = ET.SubElement(inertia, 'ixx')
    ixx.text = "0.1"

    ixy = ET.SubElement(inertia, 'ixy')
    ixy.text = "0"

    ixz = ET.SubElement(inertia, 'ixz')
    ixz.text = "0"

    iyy = ET.SubElement(inertia, 'iyy')
    iyy.text = "0.1"

    iyz = ET.SubElement(inertia, 'iyz')
    iyz.text = "0"

    izz = ET.SubElement(inertia, 'izz')
    izz.text = "0.1"

    pose = ET.SubElement(inertial, 'pose')
    pose.set('frame', '')
    pose.text = '0 0 0 0 -0 0'


    collision = ET.SubElement(link, 'collision')
    collision.set('name', 'collision')

    geometry = ET.SubElement(collision, 'geometry')
    sphere = ET.SubElement(geometry, 'sphere')
    radius = ET.SubElement(sphere, 'radius')
    radius.text = sphere_radius

    max_contacts = ET.SubElement(collision, 'max_contacts')
    max_contacts.text = "10"
    
    surface = ET.SubElement(collision, 'surface')
    contact = ET.SubElement(surface, 'contact')
    ode = ET.SubElement(contact, 'ode')
    bounce = ET.SubElement(surface, 'bounce')


    friction = ET.SubElement(surface, 'contact')
    torsional = ET.SubElement(friction, 'torsional')
    ode2 = ET.SubElement(torsional, 'ode')


    visual = ET.SubElement(link, 'visual')
    visual.set('name', 'visual')

    geometry2 = ET.SubElement(visual, 'geometry')
    sphere2 = ET.SubElement(geometry2, 'sphere')
    radius2 = ET.SubElement(sphere2, 'radius')
    radius2.text = sphere_radius

    material = ET.SubElement(visual, 'material')
    script =  ET.SubElement(material, 'script')
    uri = ET.SubElement(script, 'uri')
    uri.text = "file://media/materials/scripts/gazebo.material"
    name = ET.SubElement(script, 'name')
    name.text = "Gazebo/Red"


    self_collide = ET.SubElement(link, 'self_collide')
    self_collide.text = "0"
    enable_wind = ET.SubElement(link, 'enable_wind')
    enable_wind.text = "0"
    kinematic = ET.SubElement(link, 'kinematic')
    kinematic.text = "0"

# Adds a random number of spheres to the specified gazebo files.
# randomness can be explicitly overridden on the command line with command: -n [number of spheres]
def addRandomSpheres(fileName, rand_num_spheres, x_range, y_range, z_range, radius, mass):

    print("Spheres generated: " + str(rand_num_spheres))
    et = ET.parse(fileName)

    root = et.getroot()
    child_list = []
    child_list = root.getchildren()

    world_root = ''

    counter = 0
    for i in child_list:
        if child_list[counter].tag == 'world':

            # update world root
            world_root = child_list[counter]

            sphere_number = 3 #Hard coded to start at sphere 3. This is for testing purposes 
            for x in range(rand_num_spheres):

                x_coord = random.uniform(-1 * x_range/2, x_range/2)
                y_coord = random.uniform(-1 * y_range/2, y_range/2)
                z_coord = random.uniform(-1 * z_range/2, z_range/2)
                coords_str = str(x_coord) + " " + str(y_coord) + " " + str(z_coord) + " 0 -0 0" #we hardcode pitch and yaw since they are spheres
    
                sphere_model = ET.SubElement(i, 'model')
                sphere_tree(sphere_model, coords_str, str(radius), str(mass))


                sphere_name = 'unit_sphere_' + str(sphere_number)
                sphere_model.attrib['name'] = sphere_name  # must be str; cannot be an int
                sphere_number = sphere_number + 1

            
        counter = counter + 1
    
    et.write(fileName)



# Syntax for calling program: 
#
# create_world.py [filename] 
#
#  The above will creates a random number (1 -20 by default)
#  of spheres in the file withon the coordinate
#  grid of (-100,100)
# 
# Optional Syntax:
#
# -n [number]
#
# the above overrides randomess. Generates a specific number of spheres
#
#
# -[x/y/z] [number]
#
# The above sets the range (ie xmax-xmin), for the coordinates of the sphere
# 
# -r [number]
#
# The above sets the radius of the spheres to a specific value
# 
# -nr [min] [max]
#
# The above sets the range of the random spheres to be generated
# 
#
# -rr [min] [max]
#
# The above sets the range of the radius spheres to be generated
#
# -m [number]
#
# The above sets the mass of the spheres (large value ie 1000 won't roll)
if __name__ == "__main__":

    # default values. Change with parameters (syntax above)
    rand_num_spheres = random.randint(1,20)
    x_range = 20
    y_range = 20
    z_range = 20
    radius = random.uniform(0.1,1)
    mass = 1000

    if len(sys.argv) < 2:
        print("Syntax: create_world.py [filename] [-optional_cmds]")

    elif len(sys.argv) > 3:

        counter = 0
        called = False

        # check command line configurable args
        for i in sys.argv:
            if i == '-n':
                rand_num_spheres = int(sys.argv[counter + 1])

            if i == '-nr':
                random_min = int(sys.argv[counter + 1])
                random_max = int(sys.argv[counter + 2])
                rand_num_spheres = random.randint(random_min, random_max)

            if i == '-x':
                x_range = int(sys.argv[counter + 1])
    

            if i == '-y':
                y_range = int(sys.argv[counter + 1])
          

            if i == '-z':
                z_range = int(sys.argv[counter + 1])
          

            if i == '-r':
                radius = float(sys.argv[counter + 1])

            if i == '-rr':
                random_min = int(sys.argv[counter + 1])
                random_max = int(sys.argv[counter + 2])
                radius = random.uniform(random_min,random_max)

            if i == '-m':
                mass = int(sys.argv[counter + 1])
             
            
            counter = counter + 1

       
        addRandomSpheres(sys.argv[1], rand_num_spheres, x_range, y_range, z_range, radius, mass)
    
    #default case
    else:
        addRandomSpheres(sys.argv[1], rand_num_spheres, x_range, y_range, z_range, radius, mass)


