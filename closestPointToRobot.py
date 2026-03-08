import json

points = []


# load the json file for points map 

def load_points(filename):
    global points
    try:
        with open(filename, "r") as f:
            points = json.load(f)
        print(f"Loaded {len(points)} points from {filename}")
    except FileNotFoundError:
        
        print("No save file found")
        
load_points("path.json")

# find distance to closest point that has the lowest i value
def find_closest_index2(robot_x, robot_y, path):
    
    best_i = 0
    best_dist = 1e9

    for s, p in enumerate(path):

        px = p[1]
        py = p[2]

        d = (px - robot_x)**2 + (py - robot_y)**2

        if d < best_dist:
            best_dist = d
            best_i = s

    return best_i

def find_closest_index(robot_x, robot_y, path):
    best_i = 0
    best_dist = 1e9
    
    for i, p in enumerate(path):
        pi, px, py = p["i"], p["x"], p["y"]
        d = (px - robot_x)**2 + (py - robot_y)**2  # squared distance multiplied by the point number
        print(points[i], i,    d)
        if d < best_dist:
            best_dist = d
            best_i = pi

    return best_i
# 20.6, 80.4 is the cordinate i want to use
print(find_closest_index(34.8, 106.4, points))


# Loaded 6 points from path.json
# {'x': 24.6, 'y': 38.6} 0 991.2399999999998
# {'x': 36.4, 'y': 38.6} 1 464.96
# {'x': 51.4, 'y': 37.4} 2 233.00000000000009
# {'x': 61.8, 'y': 38.6} 3 277.0
# {'x': 69.8, 'y': 38.8} 4 479.4400000000001
# {'x': 88.2, 'y': 39.0} 5 1438.1200000000003
# 2 <---- closest number

#honestly this part seems to work altight. It finds the closest thing pretty well


