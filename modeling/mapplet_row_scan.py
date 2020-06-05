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

def my_lidarscan(scene,   posCam, a=90, z=1):
    # pgl.Viewer.display(scene)
    # sc = pgl.Viewer.getCurrentScene()
    bbx = pgl.BoundingBox(scene)
    c = bbx.getCenter()
    p, h, u = pgl.Viewer.camera.getPosition()
    pts = pgl.PointSet([], [])

    #pgl.Viewer.camera.setPosition(pgl.Vector3( posCam[0],  posCam[1], 1))
    # p, h, u = pgl.Viewer.camera.getPosition()
    # print(p, h, u)
    # pgl.Viewer.camera.lookAt(pgl.Vector3(100, 100, 8),
    #                          c)
    # pi, ci = pgl.Viewer.frameGL.grabZBufferPoints()
    # pts.pointList += pi
    # pts.colorList += ci

    for a in arange(0, 360, a):
        print(posCam)
        mp = (c + pgl.Matrix3.axisRotation(pgl.Vector3(0, 0, 1), np.double(a)) # r
              * pgl.Vector3(1, 0, 0) # Zoom?
              * pgl.norm(p-c)) # Distance
        print(c)
        Viewer.display(scene)
        
    #    print(mp)
        pgl.Viewer.camera.lookAt(mp, c)
        jitter = 0.1
        raywidth = 2
        pi, ci = pgl.Viewer.frameGL.grabZBufferPoints() # jitter, raywidth
        # pi, ci = pgl.Viewer.frameGL.grabZBufferPoints()
        #pts.pointList += pi
        #pts.colorList += ci
    #    print(type(pts))
    return 0#pts

def singleTree_sideMeasurements(lst_scenes):
    """
    From a given tree, take 4 Lidar measurements from each side 
    INPUT:
        lst_scenes: List of PlantGL scenes 
    OUTPUT:
        dict with the 4 measurements 
    """

    return 0

def singleTree_Line(lst_tree, dx, ntree=5, quality=0, d_l2t=5):
    """
    Single tree line 
    INPUT:
        lst_tree: List of PlantGL scenes with the MAppleT inside 
        dx      : Distance between the tree around the x axis 
        ntree   : Number of trees to use 
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
                        1T1
                        1T1
                        1T1
                        1T1
                        1T1
                        010
    """
    # Verify the the list of trees is consistenet with the requested scene 
    if(len(lst_tree)<ntree):
        print("ERROR: Merged scene can not be generated, input trees %i requested trees %i" %(len(lst_tree), ntree))
        sys.exit()
    # coord -- Ensure that the tree are draw from the center of the scene and that 
    # in the first iterations they begin from the half distance positive and negative
    x = [dx/2, -dx/2]
    lcoord = []
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
    # Setting the sensor over the scene 
    if(quality == 0):
        # Angles 
        angs = [45, -45]
        # Set the lidar to make the measurements 
        for ang in angs:
            lidar_coord = estimate_LiDAR2Tree_coord(lcoord, d_l2t, angle=ang)
            # 
            s1 = Sphere(0.2)
            s1 = Translated(lidar_coord[0][0], lidar_coord[0][1], 1,  s1)
            m = Material("red", Color3(150,0,0))
            Shape1 = Shape(s1,m)

            s2r.add( Shape( Shape1.geometry,
                            Shape1.appearance, 
                            Shape1.id,
                            Shape1.parentId) )

            s2 = Sphere(0.2)
            s2 = Translated(lidar_coord[1][0], lidar_coord[1][1], 1,  s2)
            m2 = Material("blue", Color3(0,0,150))
            Shape2 = Shape(s2,m2)

            s2r.add( Shape( Shape2.geometry,
                            Shape2.appearance, 
                            Shape2.id,
                            Shape2.parentId) )
        my_lidarscan(s2r, [lidar_coord[0][0], lidar_coord[0][1], 1], a=90, z=1)

    elif(quality==1):
        # Get the lidar coordinates between trees and top and front 
        lidar_coord = estimate_LiDAR2Tree_coord(lcoord, d_l2t, angle=90, m_type=1, d_t2t=dx)
        # Draw the obtained coordinates 
        for a_coord in lidar_coord:
            
            s1 = Sphere(0.2)
            s1 = Translated(a_coord[0], a_coord[1], 1,  s1)
            m = Material("green", Color3(0,150,0))
            Shape1 = Shape(s1,m)

            s2r.add( Shape( Shape1.geometry,
                            Shape1.appearance, 
                            Shape1.id,
                            Shape1.parentId) )
    else:
        pass
    Viewer.display(s2r)
    return s2r 

def estimate_LiDAR2Tree_coord(finalTrees, d_l2t, m_type=0, angle=45., d_t2t=3.):
    """
    Estimate the coordinates where the lidar have to be set
    based on the coordinates of the last trees added in the scene 
    and the desired distance a position from the lidar to the tree

    INPUT:
        finalTree: List of the coordinated of the bottom left and top right trees [ [x,y], [x,y]]
        m_type: 
            0 -> Low Resolution measurement
            1 -> High Resolution measurement  
    OUTPUT:
        LiDAR list of coordinates  
    """
    # Init var 
    l_coord = []
    # Walk over the give coordinates and estimate the LiDAR positions 
    if(m_type==0):
        for a_coord in finalTrees:
            # LiDAR coordinates - from 0,0  
            l_x = d_l2t*math.cos(math.degrees(angle)) 
            l_y = d_l2t*math.sin(math.degrees(angle)) 
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
        d_s2t: Distance from the sensor to the tree
        measure_type: Protocol used to make the measurement, in the following draw 0-> Empty, 1->LiDAR, T->Tree
            0: 
                010
                1T1
                010
            1:

    OUTPUT:
        List with the desired measurements
    """
    # Create the empty scene where the trees are going to be merged 
    scn2ret = Scene()
    # Evaluate the different cases 
    if(measure_type==0):
        scn2ret = singleTree_sideMeasurements(lst_scenes)
    elif(measure_type==1):
       scn2ret = singleTree_Line(lst_scenes, dxy_tree[0], quality=exp_quality, ntree=ntree, d_l2t=d_s2t) 
    #Viewer.display(scn2ret)
    return 0

def main(argv):
    print("-> Experiment generator")
    parser = argparse.ArgumentParser(description='Generate different quality measurements')
    parser.add_argument("quality", type=int, help="Type of experiment that want to be simulated")
    parser.add_argument("ntree", type=int, help="Number of trees to be loaded")
    parser.add_argument("path2trees", type=str, help="Path to the folder where are located the synthetic trees")
    parser.add_argument("--output", type=str, default="output", help="Path where you want to write the measured point cloud")
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
    exp = prepare_exp(lst_tree, (5.8, 1.8), measure_type=1, exp_quality=1, ntree=5, d_s2t=5*10)

if(__name__=="__main__"):
    main(sys.argv)