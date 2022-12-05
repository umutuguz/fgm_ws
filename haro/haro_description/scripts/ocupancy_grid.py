#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import SimpleOccupancyGridArray, SimpleOccupancyGrid
from geometry_msgs.msg import Point, Vector3

import random
import numpy as np
import tf

OCP_DIM_ATRIX = 10
CELL_SEP_MAX = 0.5
CELL_OFFSET_MAX = CELL_SEP_MAX/2.0

"""
[jsk_recognition_msgs/SimpleOccupancyGridArray]:                                                                                                                          
std_msgs/Header header                                                                                                                                                    
  uint32 seq                                                                                                                                                              
  time stamp                                                                                                                                                              
  string frame_id                                                                                                                                                         
jsk_recognition_msgs/SimpleOccupancyGrid[] grids                                                                                                                          
  std_msgs/Header header                                                                                                                                                  
    uint32 seq                                                                                                                                                            
    time stamp                                                                                                                                                            
    string frame_id                                                                                                                                                       
  float32[4] coefficients                                                                                                                                                 
  float32 resolution                                                                                                                                                      
  geometry_msgs/Point[] cells                                                                                                                                             
    float64 x                                                                                                                                                             
    float64 y                                                                                                                                                             
    float64 z 
"""

def cells(x_offset, cell_separation=0.05, random_z=False, z_init = 0):
    """
    Z value is referneces to the one stated in the coeficients -array[3]
    """
    ret = []
    for i in range(0, 20):
        for j in range(0, 20):
            if random_z:
                z_value = random.random() + z_init
            else:
                z_value = z_init
            
            ret.append(Point(x = cell_separation * i + x_offset, y = cell_separation * j, z = z_value))
    return ret



def update_free_ocup_cellarrays(occupancy_matrix, cell_separation=CELL_SEP_MAX, offset=CELL_OFFSET_MAX):
    """
    
    occupancy_matrix = np.zeros((dim,dim), dtype = 'bool')
    """
    free_cell_array, ocup_cell_array = init_cells()
    
    for (x,y), value in np.ndenumerate(occupancy_matrix):
        if value:
            ocup_cell_array.append(Point(x = cell_separation * x + offset, y = cell_separation * y + offset, z = 0))
        else:
            free_cell_array.append(Point(x = cell_separation * x + offset, y = cell_separation * y + offset, z = 0))
    

    return free_cell_array, ocup_cell_array



def update_occupancy_matrix(matrix_pos):

    occupancy_matrix = init_occupancy_matrix(matrix_dim=OCP_DIM_ATRIX)
    x_len = occupancy_matrix.shape[0]
    y_len = occupancy_matrix.shape[1]
    
    if not(matrix_pos >= x_len*y_len):
    
        x_pos = matrix_pos / x_len
        y_pos = matrix_pos % x_len
        occupancy_matrix[x_pos][y_pos] = True
    
    return occupancy_matrix


def get_haro_world_pos():
    haro_pose = None
    
    try:                
        haro_pose = rospy.wait_for_message('/haro_world_baselink_tf_translation', Vector3, timeout=1)
    except:
        rospy.loginfo("Current haro pose not ready yet, retrying")
        
    return haro_pose

def update_occupancy_matrix_tf():
    """
    Be carefull with the grid size.
    """
    x_pos = 0
    y_pos = 0
    occupancy_matrix = init_occupancy_matrix(matrix_dim=OCP_DIM_ATRIX)
    haro_pose = get_haro_world_pos()
    if haro_pose:
        
        x_len = occupancy_matrix.shape[0]
        y_len = occupancy_matrix.shape[1]
        CELL_SEP_MAX
        
        x_pos_raw = int(haro_pose.x / CELL_SEP_MAX)
        y_pos_raw = int(haro_pose.y / CELL_SEP_MAX)
        x_pos = min([x_pos_raw,x_len-1])
        y_pos = min([y_pos_raw,y_len-1])
    
    occupancy_matrix[x_pos][y_pos] = True
        
    return occupancy_matrix

def init_occupancy_matrix(matrix_dim=OCP_DIM_ATRIX):
    return np.zeros((matrix_dim,matrix_dim), dtype = 'bool')

def init_cells():
    return [], []


def ocup_matrix_test():
    r = rospy.Rate(1)
    

    while not rospy.is_shutdown():
        occupancy_matrix = update_occupancy_matrix_tf()
        print occupancy_matrix
        r.sleep()



def integrated_twogrids_demo():
    p = rospy.Publisher("/occupancy_grid", SimpleOccupancyGridArray,  queue_size=1)
    r = rospy.Rate(10)
    

    
    free_cell_array, ocup_cell_array = init_cells()
    
    matrix_pos = 0
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        occupancy_grid_array = SimpleOccupancyGridArray()

        #occupancy_matrix = update_occupancy_matrix(matrix_pos)
        occupancy_matrix = update_occupancy_matrix_tf()
        
        free_cell_array, ocup_cell_array = update_free_ocup_cellarrays(occupancy_matrix, cell_separation=CELL_SEP_MAX)

        # Free
        free_grid = SimpleOccupancyGrid()
        free_grid.header.frame_id = "world"
        free_grid.header.stamp = now
        # Orientation of plane normal, [x_vector_value, x_vector_value, x_vector_value, -height], practical effects
        free_grid.coefficients = [0,0, 1, 0]
        # Size of each square side in the grid
        free_grid.resolution = CELL_SEP_MAX    #5cm resolution
        # Here we specify the position of each square in the grid
        free_grid.cells = free_cell_array
        
        # Ocup
        oc_grid = SimpleOccupancyGrid()
        oc_grid.header.frame_id = "world"
        oc_grid.header.stamp = now
        # Orientation of plane normal, [x_vector_value, x_vector_value, x_vector_value, -height], practical effects
        oc_grid.coefficients = [0,0, 1, 0]
        # Size of each square side in the grid
        oc_grid.resolution =CELL_SEP_MAX    #5cm resolution
        # Here we specify the position of each square in the grid
        oc_grid.cells = ocup_cell_array
        
        occupancy_grid_array.grids.append(free_grid)
        occupancy_grid_array.grids.append(oc_grid)

        occupancy_grid_array.header.stamp = now
        occupancy_grid_array.header.frame_id = "world"
        p.publish(occupancy_grid_array)
        r.sleep()
        matrix_pos += 1
        if matrix_pos > occupancy_matrix.size:
            matrix_pos = 0


def dynamic_grid_demo():
    p = rospy.Publisher("/occupancy_grid", SimpleOccupancyGridArray,  queue_size=1)
    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        occupancy_grid_array = SimpleOccupancyGridArray()
        for i in range(1):
            occupancy_grid = SimpleOccupancyGrid()
            occupancy_grid.header.frame_id = "world"
            occupancy_grid.header.stamp = now
            # Orientation of plane normal, [x_vector_value, x_vector_value, x_vector_value, -height], practical effects
            occupancy_grid.coefficients = [0,0, 1, 0]
            # Size of each square side in the grid
            occupancy_grid.resolution = 0.5    #5cm resolution
            # Here we specify the position of each square in the grid
            occupancy_grid.cells = cells(x_offset=0.0, cell_separation=0.5, random_z=True, z_init = 0)
            occupancy_grid_array.grids.append(occupancy_grid)
        occupancy_grid_array.header.stamp = now
        occupancy_grid_array.header.frame_id = "world"
        p.publish(occupancy_grid_array)
        r.sleep()

def sample_demo():
    
    p = rospy.Publisher("/occupancy_grid", SimpleOccupancyGridArray,  queue_size=1)

    r = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        occupancy_grid_array = SimpleOccupancyGridArray()
        for i in range(10):
            occupancy_grid = SimpleOccupancyGrid()
            occupancy_grid.header.frame_id = "world"
            occupancy_grid.header.stamp = now
            occupancy_grid.coefficients = [0, 0, 1, i * 0.2]
            occupancy_grid.resolution = 0.05      #5cm resolution
            occupancy_grid.cells = cells(i / 2.0)
            occupancy_grid_array.grids.append(occupancy_grid)
        occupancy_grid_array.header.stamp = now
        occupancy_grid_array.header.frame_id = "world"
        p.publish(occupancy_grid_array)
        r.sleep()


if __name__ == "__main__":
    rospy.init_node("test_occupancy_grid")
    #sample_demo()
    #dynamic_grid_demo()
    integrated_twogrids_demo()
    #ocup_matrix_test()

  