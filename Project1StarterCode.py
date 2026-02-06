import heapq
import sys
import math
from PIL import Image

# =====IMAGE HELPERS=====
# Aside: Why do this when the functions are all just one liners?
# Consider switching to a different image library (opencv), representation, etc.
# This cleans up the code a bit by leaving the logic of the search algorithms independent of the specific image package.
# If we were to change, we would just need to update logic here once

def SetRGB(image, pixel_coordinate, color):
    image.putpixel(pixel_coordinate, color)

def GetRGB(image, pixel_coordinate):
    return image.getpixel(pixel_coordinate)

def SaveImage(image, filename, extension):
    image.save(filename, extension)

def OpenImage(filename):
    return  Image.open(filename)

# =====PATHFINDING HELPERS=====
#Dijsktera Search Algorithm
def Dijsktera(start, goal, terrain_costs, movement_type, TerrainImage, WIDTH, HEIGHT):
    tracker = []
    current = start 
    visited = set()
    heapq.heappush(tracker, (0, start))
    parents = {start: None}
    cost_so_far = {start: 0}
    while tracker:
        current_cost, current = heapq.heappop(tracker)
        if current == goal:
            break
        if current in visited:
            continue
        visited.add(current)
        if movement_type == 4:
            neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]), (current[0], current[1]+1), (current[0], current[1]-1)]
        else:
            neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]), (current[0], current[1]+1), (current[0], current[1]-1),
                         (current[0]+1, current[1]+1), (current[0]-1, current[1]-1), (current[0]+1, current[1]-1), (current[0]-1, current[1]+1)]
        
        for neighbor in neighbors:
            if neighbor[0]<WIDTH and neighbor[0]>=0 and neighbor[1]<HEIGHT and neighbor[1]>=0:
                pixel_rgb = GetRGB(TerrainImage, neighbor)
                weight = terrain_costs.get(pixel_rgb, 1)

                isDiagonal = abs(neighbor[0]-current[0]) + abs(neighbor[1]-current[1]) == 2
                if isDiagonal:
                    weight = weight * 1.414

                new_cost = current_cost + weight

                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(tracker, (new_cost, neighbor))
                    parents[neighbor] = current
    return parents, cost_so_far, visited
#
def A(start, goal, terrain_costs, movement_type, TerrainImage,WIDTH, HEIGHT):
    tracker = []
    current = start
    visited = set()
    heapq.heappush(tracker, (0, start))
    parents = {start: None}
    cost_so_far = {start: 0}
    while tracker:
        f_score, current = heapq.heappop(tracker)
        if current == goal:
            break
        if current in visited:
            continue
        visited.add(current)
        if movement_type == 4:
            neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]), (current[0], current[1]+1), (current[0], current[1]-1)]
        else:
            neighbors = [(current[0]+1, current[1]), (current[0]-1, current[1]), (current[0], current[1]+1), (current[0], current[1]-1),
                         (current[0]+1, current[1]+1), (current[0]-1, current[1]-1), (current[0]+1, current[1]-1), (current[0]-1, current[1]+1)]
        
        for neighbor in neighbors:
            if neighbor[0]<WIDTH and neighbor[0]>=0 and neighbor[1]<HEIGHT and neighbor[1]>=0:
                pixel_rgb = GetRGB(TerrainImage, neighbor)
                weight = terrain_costs.get(pixel_rgb, 1)

                isDiagonal = abs(neighbor[0]-current[0]) + abs(neighbor[1]-current[1]) == 2
                if isDiagonal:
                    weight = weight * 1.414

                new_weight = cost_so_far[current] + weight
                cost_to_goal = math.sqrt(pow(goal[0]-neighbor[0],2)+pow(goal[1]-neighbor[1],2))
                if neighbor not in cost_so_far or new_weight < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_weight
                    new_cost = new_weight+cost_to_goal
                    heapq.heappush(tracker, (new_cost, neighbor))
                    parents[neighbor] = current
    return parents, cost_so_far, visited


    
# ===== MAIN ====

if __name__ == '__main__':
    FileArg = sys.argv[1] #The name of the image file to open
    WeightArg = sys.argv[2] #The name of the weights .txt file to open
    MovementArg = sys.argv[3] #The type of movement the agent can take (Four versus Eight) directions
    Start = (int(sys.argv[4]), int(sys.argv[5]))
    Destination = (int(sys.argv[6]), int(sys.argv[7]))

    InputImage = FileArg+".png"

    TerrainImage = OpenImage(InputImage)
    WIDTH, HEIGHT = TerrainImage.width,TerrainImage.height
    
    terrain_costs = {}
    File = open(WeightArg, "r");
    for line in File:
        keys = line.strip().split(',')
        rgb = (int(keys[0]), int(keys[1]), int(keys[2]))
        cost = int(keys[3])
        terrain_costs[rgb] = cost
    File.close()

    movement_type = int(MovementArg)
    parents, cost_so_far, visited = Dijsktera(Start, Destination, terrain_costs, movement_type, TerrainImage, WIDTH, HEIGHT)
    if Destination in parents:
        DijkstraImg = TerrainImage.copy()
        current = Destination
        while current != Start:
            SetRGB(DijkstraImg, current, (255, 0, 0))
            current = parents[current]
        SetRGB(DijkstraImg, Start, (0, 255, 0))
        SaveImage(DijkstraImg, f"OutputDijkstra_{FileArg}.png", "PNG")
    
    
    DijkstraCov = TerrainImage.copy()
    for pixel in visited:
        SetRGB(DijkstraCov, pixel, (0, 0, 0))
    SaveImage(DijkstraCov, f"CoverageDijkstra_{FileArg}.png", "PNG")

    
    parents, cost_so_far, visited = A(Start, Destination, terrain_costs, movement_type, TerrainImage, WIDTH, HEIGHT)
    
    if Destination in parents:
        AStarImg = TerrainImage.copy()
        current = Destination
        while current != Start:
            SetRGB(AStarImg, current, (255, 0, 0))
            current = parents[current]
        SetRGB(AStarImg, Start, (0, 255, 0))
        SaveImage(AStarImg, f"OutputAStar_{FileArg}.png", "PNG")

    AStarCov = TerrainImage.copy()
    for pixel in visited:
        SetRGB(AStarCov, pixel, (0, 0, 0))
    SaveImage(AStarCov, f"CoverageAStar_{FileArg}.png", "PNG")
    





    
    
