from __future__ import print_function
import os 
import sys 
import math
import argparse
import random
import numpy as np 
import openalea.lpy as lpy
import openalea.plantik.tools.config
from modeling import *
from openalea.plantgl.all import *
from tools.fileHandler import fileHandler
from tools.prepareSynthetic import prepareSynthetic


def get_geometry(sh):
    if hasattr(sh, 'geometry'):
        return get_geometry(sh.geometry)
    elif hasattr(sh, 'primitive'):
        return get_geometry(sh.primitive)
    else:
        return sh

def synthetic_LiDAR(scenePlantGL, posLiDAR, pos2look=[0,0,1], jitter = 0.1, raywidth = 2, ann=False):
    """
    Generate a synthetic point cloud based on the actual scene of the trees 
    INPUT:
        scenePlantGL: PlantGL scene with the mapplet synthetic tree inside 
        posLiDAR: List of the estimate position ([x,y,z]) of the lidar in the scene 
        pos2look: Cartesian points to where the camera have to look 
        raywidht: Area took by the simulated light bim
        jitter: ----
        ann: If you wish to pre annotad the measurements 
    OUTPUT:
        point cloud set of points numpy array 
    """
    # Conver the input positon vector in Vector3 for plantGL camera handling 
    _pos_lidar = pgl.Vector3( posLiDAR[0], posLiDAR[1], posLiDAR[2])
    _pos2look  = pgl.Vector3( pos2look[0], pos2look[1], pos2look[2])
    black = Material((0, 0, 0))
    red   = Material((255, 0, 0))
    # Get the camera position and orientation 
    _cam_p, _cam_r, _ = pgl.Viewer.camera.getPosition()
    # Preprocess the scene -- High light the appeles and set the other elements in black  
    sc2w = Scene()
    if(ann):
        # Scene to work 
        sc2w = Scene()
        # Color the shape 
        for shape in scenePlantGL:
            if(isinstance( get_geometry(shape), Sphere)):
                sc2w.add(Shape(shape.geometry,
                                      red,
                                      shape.id,
                                      shape.parentId) )
            else:
                sc2w.add(Shape(shape.geometry,
                                      black,
                                      shape.id,
                                      shape.parentId)) 
    Viewer.grids.set(False, False, False, False)
     # Shut down the light 
    Viewer.light.enabled = False
    Viewer.display(sc2w)
    # Init the object where are going to be saved lidar scans  
    pts = pgl.PointSet([], [])
    #print(_pos2look)
    pgl.Viewer.camera.lookAt(_pos_lidar, _pos2look)
    # Get the Point cloud base on the observed geometries s
    try:
        pi, ci = pgl.Viewer.frameGL.grabZBufferPoints(jitter, raywidth) # jitter, raywidth
    except: # Argument error launched from C++
        pi, ci = pgl.Viewer.frameGL.grabZBufferPoints()
    # Save the measures point cloud 
    pts.pointList += pi
    pts.colorList += ci
    # Convert to annumpy array and save them 
    np_pts = np.array(pts.pointList)
    colors = np.array([(c.red, c.green, c.blue) for c in pts.colorList])
    # Merge all in the same file 
    data = np.concatenate([np_pts, colors], axis=1) # X,Y,Z,R,G,B
    return data

def singleTree_sideMeasurements(lst_scenes):
    """
    From a given tree, take 4 Lidar measurements from each side 
    INPUT:
        lst_scenes: List of PlantGL scenes 
    OUTPUT:
        dict with the 4 measurements 
    """
    sc2ret = Scene()
    data2ret = []

    return sc2ret, data2ret

def singleTree_Line(lst_tree, dx, ntree=5, quality=0, d_l2t=5, angles=[45, -45], sens_height=1.5):
    """
    Single tree line 
    INPUT:
        lst_tree: List of PlantGL scenes with the MAppleT inside 
        dx      : Distance between the tree around the x axis 
        ntree   : Number of trees to use 
        d_l2t   : Distance from the sensor to the tree, double 
        angles  : Pair of angles to evaluate , Only took in care when quality is 0
        sens_height: Height of the sensor to make the measurement 
        quality :  In the simple draw of above, 1 represent the LiDAR and T the tree
            0: Low ressolution 
                        101
                        0T0
                        0T0
                        0T0
                        0T0
                        0T0
                        101
            1: Hight resolution 
                        010
                        0T0
                        101
                        0T0
                        101
                        0T0
                        101
                        0T0
                        101
                        0T0
                        010
        OUTPUT:
            PlantGL generated scene and a list of numpy arrays [Point cloud]
    """
    # Verify the the list of trees is consistenet with the requested scene 
    if(len(lst_tree)<ntree):
        print("ERROR: Merged scene can not be generated, input trees %i requested trees %i" %(len(lst_tree), ntree))
        sys.exit()
    # coord -- Ensure that the tree are draw from the center of the scene and that 
    # in the first iterations they begin from the half distance positive and negative
    x = [dx/2, -dx/2]
    lcoord = []
    data2wrte = []
    s2r = Scene()
    # Load and merge each tree in one scene 
    for idx, tree in enumerate(lst_tree, start=1):
        # Merge the tree in the scene, Walk over all shape an put them in the new scene 
        for a_shape in tree:
            if( (idx%2) == 0 ):
                s2r.add( Shape( Translated(x[0],0,0,a_shape.geometry),
                                a_shape.appearance, 
                                a_shape.id,
                                a_shape.parentId) )
            else:
                s2r.add( Shape( Translated(x[1],0,0,a_shape.geometry),
                                a_shape.appearance, 
                                a_shape.id,
                                a_shape.parentId) )                
        # Updated distance to translate 
        if( (idx%2) == 0 ):
            x[0] += dx
        else:
            x[1] -= dx
        if(idx == ntree):
            lcoord.append([x[0], 0])
            lcoord.append([x[1], 0])
            break
    # The viewes has to be activated before launch the scan  
    Viewer.grids.set(False, False, False, False)
     # Shut down the light 
    Viewer.light.enabled = False
    Viewer.display(s2r)
    # Setting the sensor over the scene 
    if(quality == 0): # Low res
        # Set the lidar to make the measurements 
        for ang in angles:
            # This element gives two measurements at time 
            lidar_coord = estimate_LiDAR2Tree_coord(lcoord, d_l2t, angle=ang)
            # Walk over the available measurement  
            for idx, a_coord in enumerate( lidar_coord ):
                sens_data = synthetic_LiDAR(s2r, [a_coord[0], a_coord[1], sens_height*10], pos2look=[0,0,sens_height], ann=True)
                # Data to return - reference of the point cloud and the point cloud as np array
                data2wrte.append( [ "idx_%s_ang_%s"%(str(idx),str(ang)),sens_data] )
                #np.savetxt('lidar_idx_%s_ang_%s.txt'%(str(idx), str(ang)), sens_data)
    elif(quality==1): # High res 
        # Get the lidar coordinates between trees and top and front 
        lidar_coord = estimate_LiDAR2Tree_coord(lcoord, d_l2t, angle=90, m_type=1, d_t2t=dx)
        # Draw the obtained coordinates 
        for idx, a_coord in enumerate(lidar_coord):
            # The first two positions are the extreme centered elements 
            # the other positions are the ones who avance over the crop 
            if(idx>1):
                # It was multiplied by ten because the measurement is in decimenter. and the multiplication by 5 is to ensure that the camera point to
                # the half of the tree .5
                sens_data = synthetic_LiDAR(s2r, [a_coord[0], a_coord[1], sens_height*10], pos2look=[a_coord[0], 0, sens_height*5], ann=True )
            else:
                sens_data = synthetic_LiDAR(s2r, [a_coord[0], a_coord[1], sens_height*10], pos2look=[0, a_coord[1], sens_height*5], ann=True )
            data2wrte.append([ "idx_%s"%str(idx), sens_data ])
            #np.savetxt('lidar_idx_%s.txt'%str(idx), sens_data)
    else:
        pass
    #Viewer.display(s2r)
    return s2r, data2wrte

def estimate_LiDAR2Tree_coord(finalTrees, d_l2t, m_type=0, angle=45., d_t2t=3.):
    """
    Estimate the coordinates where the lidar have to be set
    based on the coordinates of the last trees added in the scene 
    and the desired distance a position from the lidar to the tree

    INPUT:
        finalTree: List of the coordinated of the bottom left and top right trees [ [x,y], [x,y]]
        m_type: integer
            0 -> Low Resolution measurement
            1 -> High Resolution measurement 
        d_l2t: Distance from the sensor to the tree, float
        angle: The angles is only use when m_type=0, It define the angle from to set the lidar from the tree, float
        d_t2t: Distance between trees float
    OUTPUT:
        LiDAR list of coordinates  
    """
    # Init var 
    l_coord = []
    # Walk over the give coordinates and estimate the LiDAR positions 
    if(m_type==0):
        for a_coord in finalTrees:
            # LiDAR coordinates - from 0,0  
            l_x = d_l2t*math.cos(math.radians(angle)) 
            l_y = d_l2t*math.sin(math.radians(angle)) 
            # Apply the offset given by the tree position 
            if( a_coord[0] < 0  and a_coord[1]>=0):
                l_x = a_coord[0] - l_x
                l_y = a_coord[1] + l_y
            elif(a_coord[0] >= 0  and a_coord[1]<0):
                l_x = a_coord[0] + l_x
                l_y = a_coord[1] - l_y
            elif(a_coord[0] < 0  and a_coord[1]<0):
                l_x = a_coord[0] - l_x
                l_y = a_coord[1] - l_y
            elif (a_coord[0] >= 0  and a_coord[1]>=0):
                l_x = a_coord[0] + l_x
                l_y = a_coord[1] + l_y
            else:
                l_x = -1
                l_y = -1
            l_coord.append([l_x, l_y])
    elif(m_type==1):
        # Find the min a and max coordinate over the X axis 
        max_x = int(np.amax( np.array( [finalTrees[0][0], finalTrees[1][0]] ) ))
        min_x = int(np.amin( np.array( [finalTrees[0][0], finalTrees[1][0]] ) ))
        # Eval value 
        e_val = float(max_x)+(d_t2t/2.)
        gd= int(abs(round((abs(max_x)+abs(min_x))/2)))
        l_coord.append( [max_x+d_l2t, 0] )
        l_coord.append( [min_x-d_l2t, 0] )
        for idx, _ in enumerate(range(int(gd))):
            # Update the coordinate arround the x axis 
            e_val -= d_t2t 
            # Keep The coordinates 
            if(idx > 0 and idx < len(range(int(max_x+(d_t2t/2.)), int(min_x+(d_t2t*2)), int(-d_t2t)))):
                l_coord.append( [e_val, d_l2t] )
                l_coord.append( [e_val, -d_l2t] )
    return l_coord

def prepare_exp(lst_scenes, dxy_tree, ntree=5, d_s2t=2.5, measure_type=0, exp_quality=0):
    """
    Merge the given synthetic trees in one scene base on the initial Field Measurements

    INPUT:
        lst_scene: list of PlantGL scenes with the MAppleT generated trees
        dxy_tree : Distance along the X and Y axis between the trees
        ntree = Number of trees to take in care 
        d_s2t: Distance from the sensor to the tree
        exp_quality: High resolution 1 or Low resolution 0
        measure_type: Protocol used to make the measurement, in the following draw 0-> Empty, 1->LiDAR, T->Tree
            0: 
                010
                1T1
                010
            1:

    OUTPUT:
        PlantGL generated scene and the list of point clouds 
    """
    # Create the empty scene where the trees are going to be merged 
    scn2ret = Scene()
    pc = []
    # Evaluate the different cases 
    if(measure_type==0):
        scn2ret, pc = singleTree_sideMeasurements(lst_scenes)
    elif(measure_type==1):
       scn2ret, pc = singleTree_Line(lst_scenes, dxy_tree[0], quality=exp_quality, ntree=ntree, d_l2t=d_s2t) 
    #Viewer.display(scn2ret)
    return scn2ret, pc

def main(argv):
    print("-> Experiment generator")
    parser = argparse.ArgumentParser(description='Generate different quality measurements')
    parser.add_argument("quality", type=int, help="Type of experiment that want to be simulated")
    parser.add_argument("ntree", type=int, help="Number of trees to be loaded")
    parser.add_argument("path2trees", type=str, help="Path to the folder where are located the synthetic trees")
    parser.add_argument("--stOmt", type=int, help="Single tree [0] or a row of trees [1]", default=1)
    parser.add_argument("--output", type=str, default="output", help="Path where you want to write the measured point cloud")
    parser.add_argument("--d_s2t", type=float, help="Distance from the sensor to the tree [meters], double", default=10)
    parser.add_argument("--d_t2t", type=float, help="Distance between trees [meters], double", default=1.8)
    args = parser.parse_args()
    # Init my obj 
    fh = fileHandler()
    pst = prepareSynthetic()
    # Init var
    lst_tree = []
    # Look for the bgeom files of MAppleT
    lst_bgeom = fh.get_FileList("bgeom", args.path2trees)
    # Verify that the number of found files is bigger or equal than the requested to put in the same scene 
    if(len(lst_bgeom)<args.ntree):
        print("ERROR: Not enough tree to successfully represent the requested scene, found files %i, requested files %i" %(len(lst_bgeom), args.ntree))
        sys.exit()
    # Load the files and set the requested scene 
    for idx, usrScene in enumerate(lst_bgeom, start=1):
        print("\r-> Loading[%i/%i]: %s" %(idx,len(lst_bgeom), usrScene))
        # Path and name of the actual scene 
        _nme_scene = os.path.join(args.path2trees, usrScene)
        # Load the indepentend scene 
        a_scene = Scene(_nme_scene)
        # Remove the floor and the text from the scene 
        cleanTree = pst.removeTextFromScene(a_scene)
        # Apped the tree 
        lst_tree.append(cleanTree)
    print("-> Loaded scenes: %i" %len(lst_tree))
    print("-> Trees to set: %s" %str(args.ntree))
    print("-> Selected quality: %s" %("High resolution" if args.quality == 1 else "Low resolution"))
    print("-> Experiment type : %s" %("Tree row" if args.stOmt == 1 else "Single tree"))
    print("-> Distance between trees[m]: %s" %str(args.d_t2t))
    print("-> Distance from the Sensor to the trees[m]: %s" %args.d_s2t)
    # The variables are multiplied by 10, because the virtual environment work on decimeters 
    exp, pcs = prepare_exp(lst_tree, (args.d_t2t*10, args.d_t2t*10), measure_type=args.stOmt, exp_quality=args.quality, ntree=args.ntree, d_s2t=args.d_s2t*10)
    # Display final scene
    Viewer.display(exp)
    # Write the point clouds -- The retured list pcs = [ idxSensor, Pointcloud ]
    for pc in pcs:
        print(" -> Writing file: %s.txt" %pc[0])
        np.savetxt( "%s.txt"%str(pc[0]), pc[1])
    print("Exit")
if(__name__=="__main__"):
    main(sys.argv)