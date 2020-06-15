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

def split_pointCloud(pointCloud, objCentralPos=(0,0), d_c2lp=20):
    """
    Crop one segment of a point cloud given center of the object and the radius of the cylinder 

    INPUT:  
        pointCloud: numpy array with the points inside (N,M) N-> Number of points and M is the X,Y,Z and their features 
        objCentralPos: Center of the object [X,Y]
        d_c2lp: Distance from the center of the object to the last point wanted from there 
    OUTPUT:
        Croped Object
    """
    # Init variables 
    obj2ret = np.array([])
    lck = 0
    # Walk over the points get the distance and know if have to append the point to the new obj element 
    for pt in pointCloud:
        # Find the distance between the two points 
        d = math.sqrt(((pt[0]-objCentralPos[0])**2)+((pt[1]-objCentralPos[1])**2))
        # Create the new objec base on the trashold given by the user 
        if(d<=d_c2lp):
            if(lck==0):
                obj2ret = np.array([pt])
                lck = 1
            else:
                obj2ret = np.concatenate( (obj2ret, np.array([pt])) , axis=0)
    return obj2ret

def synthetic_LiDAR(scenePlantGL, posLiDAR, pos2look=[0,0,1], jitter = 0.1, raywidth = 2, ann=False, colorTree = [0,0,0], colorApple=[0,0,255], bAw=True, scale=0.1):
    """
    Generate a synthetic point cloud based on the actual scene of the trees 
    INPUT:
        scenePlantGL: PlantGL scene with the mapplet synthetic tree inside 
        posLiDAR: List of the estimate position of the lidar in the scene, np array ([x,y,z]) 
        pos2look: Cartesian points to where the camera have to look, np array([x,y,z])
        raywidht: Area took by the simulated light bim
        jitter: ----
        ann: If you wish to pre annotad the measurements 
    OUTPUT:
        point cloud set of points numpy array 
    """
    # Conver the input positon vector in Vector3 for plantGL camera handling 
    _pos_lidar = pgl.Vector3( posLiDAR[0], posLiDAR[1], posLiDAR[2])
    _pos2look  = pgl.Vector3( pos2look[0], pos2look[1], pos2look[2])
    black = Material((colorTree))
    red   = Material((colorApple))
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
                if(bAw==True):
                    sc2w.add(Shape(shape.geometry,
                                        black,
                                        shape.id,
                                        shape.parentId)) 
                else:
                    sc2w.add(Shape(shape.geometry,
                                        shape.appearance,
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
    # Convert to annumpy array and save them --- The points with mappleT are scaled by ten because all their 
    # measurements are in decimeters 
    np_pts = np.array(pts.pointList)*scale
    colors = np.array([(c.red, c.green, c.blue) for c in pts.colorList])
    # Merge all in the same file 
    data = np.concatenate([np_pts, colors], axis=1) # X,Y,Z,R,G,B
    return data

def singleTree_sideMeasurements(lst_scenes, d_s2t=3, sense_height=1.5, topView=60):
    """
    From a given tree, take 4 Lidar measurements from each side 
    INPUT:
        lst_scenes: List of PlantGL scenes 
        d_s2t: Distance from the sensoor to the tree
        sense_height: Z position of the sensor 
        topView: Height to the top view scan 
    OUTPUT:
        Point cloud from different views of the tree
    """
    data2ret = []
    l_coord = []
    merged = np.array([])
    # Name list 
    names = ["front", "back", "right", "left", "top"]
    # Walk over each tree scene, find the sensor position and scan 
    for tree in lst_scenes:
        # It's assume that the single tree is located in the 0,0 coordinates
        coord0 = [ d_s2t, 0, sense_height] # X aligned
        coord1 = [-d_s2t, 0, sense_height]
        coord2 = [ 0, d_s2t, sense_height] # Y alinged 
        coord3 = [ 0, -d_s2t, sense_height]
        coord4 = [ 0, 0, topView]
        # Merge the coordinates 
        l_coord.append(coord0)
        l_coord.append(coord1)
        l_coord.append(coord2)
        l_coord.append(coord3)
        l_coord.append(coord4)
        # Pos to look 
        _p2l = [0, 0, sense_height*5] 
        # 
        for idx, view in enumerate(l_coord):     
            sens_data = np.array([])
            # Make the scan 
            if(idx < 4):
                sens_data = synthetic_LiDAR(tree, view, pos2look=_p2l, ann=True)
            else:
                sens_data = synthetic_LiDAR(tree, view, [0,0,0], ann=True)
            # Append 
            data2ret.append( ["%s_%s"%(names[idx], idx), sens_data] )
            # Merge all 
            if(idx==0):
                merged = sens_data
            else:
                merged = np.concatenate( (merged, sens_data), axis=0 )
        # Reset var 
        l_coord = []
    return lst_scenes, data2ret, merged 

def singleTree_Line(lst_tree, dx, ntree=5, quality=0, d_l2t=5, angles=[45, -45], sens_height=1.5, _colors=None):
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
    merged = []
    s2r = Scene()
    t2l = Scene()
    if(_colors is None):
        _colors = [ [0, 25, 0], [0, 75, 0], [0, 150, 0], [0, 0, 50], [0, 0, 100] ]
    # Load and merge each tree in one scene 
    for idx, tree in enumerate(lst_tree, start=1):
        # Merge the tree in the scene, Walk over all shape an put them in the new scene 
        for a_shape in tree:
            if( (idx%2) == 0 ):
                # Original tree
                s2r.add( Shape( Translated(x[0],0,0,a_shape.geometry),
                                a_shape.appearance, 
                                a_shape.id,
                                a_shape.parentId) )
                # Independet tree colored 
                t2l.add( Shape( Translated(x[0],0,0,a_shape.geometry),
                                Material(_colors[idx-1]), 
                                a_shape.id,
                                a_shape.parentId) )
            else:
                s2r.add( Shape( Translated(x[1],0,0,a_shape.geometry),
                                a_shape.appearance, 
                                a_shape.id,
                                a_shape.parentId) )    
                # Independet tree colored 
                t2l.add( Shape( Translated(x[1],0,0,a_shape.geometry),
                                Material(_colors[idx-1]), 
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
    Viewer.display(t2l)
    cntr = 0
    # Setting the sensor over the scene 
    if(quality == 0): # Low res
        merged = []
        # Set the lidar to make the measurements 
        for aidx, ang in enumerate(angles):
            # This element gives two measurements at time 
            lidar_coord = estimate_LiDAR2Tree_coord(lcoord, d_l2t, angle=ang, topView=180)
            # Walk over the available measurement  
            for idx, a_coord in enumerate( lidar_coord ):
                sens_data = synthetic_LiDAR(t2l, [a_coord[0], a_coord[1], sens_height*10], pos2look=[0,0,sens_height], ann=True, colorApple=[255,0,0], bAw=False)
                # Data to return - reference of the point cloud and the point cloud as np array
                data2wrte.append( [ "idx_%s_ang_%s"%(str(idx),str(ang)),sens_data] )
                #np.savetxt('lidar_idx_%s_ang_%s.txt'%(str(idx), str(ang)), sens_data)
                if(aidx==0 and idx == 0):
                    merged = sens_data
                else:
                    merged = np.concatenate((merged, sens_data), axis=0)
                cntr+=1
        #np.savetxt('lidar_idx_merged_single_lowres.txt', merged)
    elif(quality==1): # High res 
        # Get the lidar coordinates between trees and top and front 
        lidar_coord = estimate_LiDAR2Tree_coord(lcoord, d_l2t, angle=90, m_type=1, d_t2t=dx, sens_height=sens_height*10, topView=180)
        # Draw the obtained coordinates 
        for idx, a_coord in enumerate(lidar_coord):
            # The first two positions are the extreme centered elements 
            # the other positions are the ones who avance over the crop 
            if(idx>2):
                # It was multiplied by ten because the measurement is in decimenter. and the multiplication by 5 is to ensure that the camera point to
                # the half of the tree .5
                sens_data = synthetic_LiDAR(t2l, [a_coord[0], a_coord[1], a_coord[2]], pos2look=[a_coord[0], 0, sens_height*5], ann=True, bAw=False )
            else:
                if(idx == 2):
                    sens_data = synthetic_LiDAR(t2l, [a_coord[0], a_coord[1], a_coord[2]], pos2look=[0, 0, 0], ann=True, bAw=False )
                else: 
                    sens_data = synthetic_LiDAR(t2l, [a_coord[0], a_coord[1], a_coord[2]], pos2look=[0, a_coord[1], sens_height*5], ann=True, bAw=False )
            # Merge the obtained data 
            if(idx == 0):
                merged = sens_data
            else:
                merged = np.concatenate( (merged, sens_data), axis=0 )
            data2wrte.append([ "idx_%s"%str(idx), sens_data ])
        #print("merged--split")
        #split_pointCloud(merged)
        #np.savetxt('lidar_idx_merged_single.txt', merged)
    else:
        pass
    #sys.exit()
    #Viewer.display(s2r)
    return s2r, data2wrte, merged

def estimate_LiDAR2Tree_coord(finalTrees, d_l2t, m_type=0, angle=45., d_t2t=3., topView=60, sens_height=1.5):
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
        sens_height: Z position of the sensor 
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
            l_coord.append([l_x, l_y, sens_height])
    elif(m_type==1):
        # Find the min a and max coordinate over the X axis 
        max_x = int(np.amax( np.array( [finalTrees[0][0], finalTrees[1][0]] ) ))
        min_x = int(np.amin( np.array( [finalTrees[0][0], finalTrees[1][0]] ) ))
        # Eval value 
        e_val = float(max_x)+(d_t2t/2.)
        gd= int(abs(round((abs(max_x)+abs(min_x))/2)))
        l_coord.append( [max_x+d_l2t, 0, sens_height] )
        l_coord.append( [min_x-d_l2t, 0, sens_height] )
        l_coord.append( [0, 0, topView] )
        for idx, _ in enumerate(range(int(gd))):
            # Update the coordinate arround the x axis 
            e_val -= d_t2t 
            # Keep The coordinates 
            if(idx > 0 and idx < len(range(int(max_x+(d_t2t/2.)), int(min_x+(d_t2t*2)), int(-d_t2t)))):
                l_coord.append( [e_val, d_l2t, sens_height] )
                l_coord.append( [e_val, -d_l2t, sens_height] )
    return l_coord

def prepare_exp(lst_scenes, dxy_tree, ntree=5, d_s2t=2.5, measure_type=0, exp_quality=0, scale=0.1):
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
    if(len(lst_scenes)< ntree):
        print("> Error: The number of scenes is less than number of trees")
        sys.exit()
    # Create the empty scene where the trees are going to be merged 
    scn2ret = Scene()
    pc = []
    merged = []
    # Evaluate the different cases 
    if(measure_type==0):
        scn2ret, pc, merged = singleTree_sideMeasurements(lst_scenes, d_s2t=d_s2t)
    elif(measure_type==1):
       scn2ret, pc, merged = singleTree_Line(lst_scenes, dxy_tree[0], quality=exp_quality, ntree=ntree, d_l2t=d_s2t) 
    #Viewer.display(scn2ret)
    return scn2ret, pc, merged

def split_trees(treePointCloud, label=None, rowLabel=[4,6]):
    """
    Split the trees based on their defined colors 
    """
    evalColor = [] 
    dicPos = {}
    if(label is None):
        evalColor = [ 25, 75, 150, 50, 100  ] # G G G B B
    else:
        evalColor = label
    # Walk over the points 
    for pt in treePointCloud:
        # Walk over the label 
        for idx, lbl in enumerate(evalColor):
            #print(lbl, pt[rowLabel[0]:rowLabel[1]])
            if(lbl in pt[rowLabel[0]:rowLabel[1]]):
                if(idx not in dicPos):
                    dicPos[idx] = np.array([pt])
                else:
                    dicPos[idx] = np.concatenate( ( dicPos[idx], np.array([pt]) ), axis=0 )
        #sys.exit()
    #print(dicPos.keys())
    #for i in dicPos.keys():
    #    np.savetxt( "%stest.txt"%str(i), dicPos[i])
    return dicPos


def get_singleOrgan(MAT_PGL_scene, str_organ, action=0):
    """
    INPUT:
        MAT_PGL_scene = MappleT's PlantGL scene
        str_organ = String with the possible element to return 
                'wood' -> Return the trunk and branches of the syntetic tree
                'apple'-> Return the available apples
                'leaf' -> Return the available leafs 
        action = 
                0 -> return apples
                1 -> Remove apples 
    OUPUT:
        PlantGL scene with just the tree's leafs
    """
    # Init var
    scn2ret = Scene()
    organ = get_Definition(str_organ)
    # Walk over the shapes of the scene 
    for shape in MAT_PGL_scene:
        # Verify the Obj primitive
        if(action == 0):        
            if isinstance(get_geometry(shape), organ):
                scn2ret.add( shape )
        elif(action == 1):
            if not isinstance(get_geometry(shape), organ):
                scn2ret.add( shape )
    return scn2ret 

def get_Definition(str_Organ):
    """
    INPUT:
        str_Organ: String with the possible element to return 
                'wood' -> Return the trunk and branches of the syntetic tree
                'apple'-> Return the available apples
                'leaf' -> Return the available leafs 
    OUTPUT:
        Object type that represent the desired organ in plantGL
    """
    if(str_Organ.lower() == "wood"):
        return Cylinder
    elif(str_Organ.lower() == "apple"):
        return Sphere
    elif(str_Organ.lower() == "leaf"):
        return BezierPatch
    else:
        raise IOError("Unexpected value")

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
    parser.add_argument("--winter", type=int, help="Winter tree 0, Summer tree 1", default=1)
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
        if(args.winter==1):
            cleanTree = get_singleOrgan(cleanTree, "wood")
        else:
            pass
        #Viewer.display(cleanTree)
        # Apped the tree 
        lst_tree.append(cleanTree)
    print("-> Loaded scenes: %i" %len(lst_tree))
    print("-> Trees to set: %s" %str(args.ntree))
    print("-> Selected quality: %s" %("High resolution" if args.quality == 1 else "Low resolution"))
    print("-> Experiment type : %s" %("Tree row" if args.stOmt == 1 else "Single tree"))
    print("-> Distance between trees[m]: %s" %str(args.d_t2t))
    print("-> Distance from the Sensor to the trees[m]: %s" %args.d_s2t)
    # Path to write 
    if(not os.path.isdir(args.output)):
        os.mkdir(args.output)
    # Split the cleaned trees to get the dataset 
    for i in range(0, len(lst_tree), args.ntree):
        # Path to actual set 
        p2s = os.path.join( args.output, str(i) )
        if(i+args.ntree < len(lst_tree)):
            if( not os.path.isdir( p2s ) ):
                os.mkdir(p2s)
            # Prepare the test with the splited elements 
            _, pcs, merged = prepare_exp(lst_tree[i:i+args.ntree], (args.d_t2t*10, args.d_t2t*10), measure_type=args.stOmt, exp_quality=args.quality, ntree=args.ntree, d_s2t=args.d_s2t*10)
            # Split the merged scene -- Each tree is set in a dictionary
            singTree = split_trees(merged)
            # Get the folder to splited elements
            p2splt = os.path.join( p2s, "splited/" )
            # If doesnt exist create folder 
            if(not os.path.isdir( p2splt ) ):
                os.mkdir(p2splt)
            # Write the splited trees 
            for d_idx in singTree.keys():
                # Get the tree
                s_a_pc = singTree[d_idx]
                np.savetxt( "%s/splited_tree_idx_%s.txt"%(p2splt, d_idx), s_a_pc)
        else:
            continue
        # Write the indepented elements 
        for g_idx, pc in enumerate(pcs):
            print(" -> Writing file: %s/%s_%s.txt" %(p2s,str(pc[0]), g_idx))
            np.savetxt( "%s/%s_%s_%s.txt"%(p2s,str(i),str(pc[0]), g_idx), pc[1])
        if(args.quality==0 and args.stOmt == 1):
            np.savetxt( "%s/%s_LowResolution_treeRow_merged.txt"%(p2s, str(i)), merged)
        elif(args.quality==1 and args.stOmt == 1):
            np.savetxt( "%s/%s_HighResolution_treeRow_merged.txt"%(p2s,str(i)), merged)
        elif( (args.quality==1 or args.quality==0) and  args.stOmt == 1):
            np.savetxt( "%s/%s_singleTree_merged.txt"%(p2s,str(i)), merged)

    print("Exit")
    sys.exit(0)
if(__name__=="__main__"):
    main(sys.argv)