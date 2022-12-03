import math
import numpy as np
import cv2
import matplotlib.pyplot as plt

class Global_Navigation:
    def __init__(self,obstacles_pos,thymio_pos,goal_pos,map_size):            
        #convert the obstacle_pos as list of lists of tuples instead list of np.ndarray of tuples for next calculations
        self.all_obstacles=[]
        for index, obstacle in enumerate(obstacles_pos):
            self.all_obstacles.append([])
            for point in obstacle:
                self.all_obstacles[index].append((point[0],point[1]))
            
        self.raw_obstacles_pos=obstacles_pos
        self.thymio_pos=(thymio_pos[0],thymio_pos[1])
        self.goal_pos=(goal_pos[0],goal_pos[1])
        self.all_nodes=[]
        self.all_nodes.append(self.thymio_pos)
        self.all_nodes.append(self.goal_pos)      
        for obstacle in self.all_obstacles:
            for node in obstacle:
                self.all_nodes.append((node[0],node[1]))
        
        self.max_valx=map_size[0]
        self.max_valy=map_size[1]
        self.nodes_neigbors={}
        self.path=[]
        
        
##################################################################      
########### functions to link nodes to their neighbors ###########   

    def onSegment(self,p, q, r):
        if ( (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and 
               (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1]))):
            return True
        return False
    
    
    def orientation(self,p, q, r):
        # to find the orientation of an ordered triplet (p,q,r)
        # function returns the following values:
        # 0 : Collinear points
        # 1 : Clockwise points
        # 2 : Counterclockwise
        val = (float(q[1] - p[1]) * (r[0] - q[0])) - (float(q[0] - p[0]) * (r[1] - q[1]))
        if (val > 0):
            # Clockwise orientation
            return 1
        elif (val < 0):
            # Counterclockwise orientation
            return 2
        else:
            # Collinear orientation
            return 0
     
    def doIntersect(self,p1,q1,p2,q2):
      
        # Find the 4 orientations required for 
        # the general and special cases
        o1 = self.orientation(p1, q1, p2)
        o2 = self.orientation(p1, q1, q2)
        o3 = self.orientation(p2, q2, p1)
        o4 = self.orientation(p2, q2, q1)
  
        # General case
        if ((o1 != o2) and (o3 != o4)):
            return True
        # Special Cases
  
        # p1 , q1 and p2 are collinear and p2 lies on segment p1q1
        if ((o1 == 0) and self.onSegment(p1, p2, q1)):
            return True
  
        # p1 , q1 and q2 are collinear and q2 lies on segment p1q1
        if ((o2 == 0) and self.onSegment(p1, q2, q1)):
            return True
  
        # p2 , q2 and p1 are collinear and p1 lies on segment p2q2
        if ((o3 == 0) and self.onSegment(p2, p1, q2)):
            return True
    
        # p2 , q2 and q1 are collinear and q1 lies on segment p2q2
        if ((o4 == 0) and self.onSegment(p2, q1, q2)):
            return True
    
        # If none of the cases
        return False

    def find_neighbors(self,all_obstacles,all_nodes,pos_thymio,goal):
    
            
        neighbors = {node: [] for node in all_nodes}
   

        #if the nodes of obstacles are too close from the edges of the maps, don't add them to the visiblity graph
        nodes_close_edges=[]
        for obstacle in all_obstacles:
            for node in obstacle:
                if (node[0]<= 38) or (node[0]>= self.max_valx-38) or (node[1]<=38) or (node[1]>=self.max_valy-38):
                    nodes_close_edges.append(node)

        for i in range(len(all_nodes)): 
            if all_nodes[i] in nodes_close_edges:
                    continue
            for j in range(len(all_nodes) - i - 1):
           
                intersection=False
                
                if all_nodes[i+j+1] in nodes_close_edges:
                    continue
                
                tmp_link1=[all_nodes[i],all_nodes[i+j+1]]
            
                for obstacle in all_obstacles:
                
                    if (all_nodes[i] in obstacle) and (all_nodes[i+j+1] in obstacle): #identify when 2 vertices of the same polygon are connected
                        if obstacle.index(all_nodes[i])==len(obstacle)-1:
                            if (obstacle.index(all_nodes[i+j+1]) != 0) and (obstacle.index(all_nodes[i])!=obstacle.index(all_nodes[i+j+1])+1):
                                o1=self.orientation(all_nodes[i],all_nodes[i+j+1],obstacle[obstacle.index(all_nodes[i])-1])
                                o2=self.orientation(all_nodes[i],all_nodes[i+j+1],obstacle[0])
                                o3=self.orientation(all_nodes[i],obstacle[0],obstacle[obstacle.index(all_nodes[i])-1])
                                if (o1==1 and o2==2)or (o1==o2 and o3==2):
                                    intersection=True
                                    break
                        elif obstacle.index(all_nodes[i])==0:
                            if (obstacle.index(all_nodes[i+j+1]) != len(obstacle)-1) and (obstacle.index(all_nodes[i+j+1]) != obstacle.index(all_nodes[i])+1):
                                o1=self.orientation(all_nodes[i],all_nodes[i+j+1],obstacle[len(obstacle)-1])
                                o2=self.orientation(all_nodes[i],all_nodes[i+j+1],obstacle[obstacle.index(all_nodes[i])+1])
                                o3=self.orientation(all_nodes[i],obstacle[obstacle.index(all_nodes[i])+1],obstacle[len(obstacle)-1])
                                if (o1==1 and o2==2)or (o1==o2 and o3==2):
                                    intersection=True
                                    break
                        else:
                            if (obstacle.index(all_nodes[i])!=obstacle.index(all_nodes[i+j+1])+1) and (obstacle.index(all_nodes[i+j+1]) != obstacle.index(all_nodes[i])+1):
                                o1=self.orientation(all_nodes[i],all_nodes[i+j+1],obstacle[obstacle.index(all_nodes[i])-1])
                                o2=self.orientation(all_nodes[i],all_nodes[i+j+1],obstacle[obstacle.index(all_nodes[i])+1])
                                o3=self.orientation(all_nodes[i],obstacle[obstacle.index(all_nodes[i])+1],obstacle[obstacle.index(all_nodes[i])-1])
                                if (o1==1 and o2==2)or (o1==o2 and o3==2):
                                    intersection=True
                                    break
                        
                     
                    
                        
                    for index, vertice in enumerate(obstacle):
                    
                        if index== len(obstacle)-1: #last point of the obstacle
                            tmp_link2=[(vertice[0],vertice[1]),(obstacle[0][0],obstacle[0][1])]
                        else:
                        
                            tmp_link2=[(vertice[0],vertice[1]),(obstacle[index+1][0],obstacle[index+1][1])]
                    
                    
                        if tmp_link1[0] ==tmp_link2[0] or tmp_link1[0] ==tmp_link2[1] or tmp_link1[1] == tmp_link2[0] or tmp_link1[1] ==tmp_link2[1]:
                            continue
                        
                        if self.doIntersect(tmp_link1[0],tmp_link1[1],tmp_link2[0],tmp_link2[1]):
                            intersection=True
                            break
                    
                    if intersection==True:
                        break
                    
                if intersection==False:
                    neighbors[all_nodes[i]].append(all_nodes[i+j+1])
                    neighbors[all_nodes[i+j+1]].append(all_nodes[i])
    
        return neighbors
    
####################################################################
    
####################################################################
######################### Find shortest path #######################
    def h_vertices_obstacles(self,coords,goal):
        k=[]
        for vertex in coords:
            k.append(math.dist(vertex,goal))
            h = dict(zip(coords, k))
        return h
    
    def reconstruct_path(self,cameFrom, current):
   
        total_path = [current] 
    
        while current in cameFrom.keys():

            total_path.insert(0, cameFrom[current])  
            current=cameFrom[current]
        
        return total_path


    def A_Star(self,start, goal, h_vertices_obstacles, coords, nodes_neigbors,max_valx,max_valy):
    

        for point in [start, goal]:
            assert point[0]>=0 and point[0]<max_valx and point[1]>=0 and point[1]<max_valy ,"start or end goal not contained in the map"
        #if occupancy_grid[start[0], start[1]]:
        #    raise Exception('Start node is not traversable')

        #if occupancy_grid[goal[0], goal[1]]:
        #    raise Exception('Goal node is not traversable')

        openSet = [start]

        closedSet = []

        cameFrom = dict()

        gScore = dict(zip(coords, [np.inf for x in range(len(coords))])) 
        gScore[start] = 0 

        fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
        fScore[start] = math.dist(goal,start)



        while openSet != []: 
            fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}
            current = min(fScore_openSet, key=fScore_openSet.get)
            del fScore_openSet 

            if current == goal:
                return self.reconstruct_path(cameFrom, current)

            openSet.remove(current)
            closedSet.append(current)

            for neighbor in nodes_neigbors[current]: 


                if (neighbor in closedSet): 
                    continue


                tentative_gScore = gScore[current] + math.dist(current,neighbor) 

                if neighbor not in openSet:
                    openSet.append(neighbor) 

                if tentative_gScore < gScore[neighbor]:
                    cameFrom[neighbor] = current
                    gScore[neighbor] = tentative_gScore
                    fScore[neighbor] = gScore[neighbor] + h_vertices_obstacles[neighbor]


        print("No path found to goal")
        return [], closedSet

#################################################################
    
    def create_path(self):           
        self.nodes_neigbors=self.find_neighbors(self.all_obstacles,self.all_nodes,self.thymio_pos,self.goal_pos)
        
        self.path=self.A_Star(self.thymio_pos, self.goal_pos, self.h_vertices_obstacles(self.all_nodes,self.goal_pos), self.all_nodes,self.nodes_neigbors ,self.max_valx,self.max_valy)
        return self.path

        
    
    
##############################################################

############################ Plots ###########################
    
    def plot_visibility_graph(self): #have to use method create_path before using this one
        Img=np.zeros((self.max_valy, self.max_valx,3), dtype='uint8')

        for obstacle in self.raw_obstacles_pos:
            cv2.polylines(Img, [obstacle.reshape((-1, 1, 2))], True, (255,255,0), 5) 

        cv2.circle(Img, (round(self.goal_pos[0]),round(self.goal_pos[1])), 5, (255, 0, 0), 5)
        cv2.circle(Img, (round(self.thymio_pos[0]),round(self.thymio_pos[1])), 5, (0, 0, 255), 5)

        for node in self.nodes_neigbors:
            for neighbor in self.nodes_neigbors[node]:
                image = cv2.line(Img, (round(node[0]),round(node[1])), (round(neighbor[0]),round(neighbor[1])), (100, 100, 255), 2)
        plt.imshow(image)
        
        
    def plot_shortest_path(self):#have to use method create_path before using this one
        Img=np.zeros((self.max_valy, self.max_valx,3), dtype='uint8')
        for obstacle in self.raw_obstacles_pos:
            image = cv2.polylines(Img, [obstacle.reshape((-1, 1, 2))], True, (255,255,0), 5) 
        cv2.circle(Img, (round(self.goal_pos[0]),round(self.goal_pos[1])), 5, (255, 0, 0), 5)
        cv2.circle(Img, (round(self.thymio_pos[0]),round(self.thymio_pos[1])), 5, (0, 0, 255), 5)

        cv2.polylines(Img, np.int32([np.array(self.path).reshape((-1, 1, 2))]), False, (200, 0, 255), 3) 

        plt.imshow(image)