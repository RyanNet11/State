import pygame, math, json
SCALE = 5  # pixels per inch
FIELD_SIZE = 144  # inches

WINDOW_SIZE = FIELD_SIZE * SCALE
selected_index = None
DRAG_RADIUS = 10  # pixels
FILE_NAME = "path.json"
field_image = pygame.image.load("field.png")
field_image = pygame.transform.scale(field_image, (WINDOW_SIZE, WINDOW_SIZE))
points = []
new_points = []
telemetry_frames = []
frame_index = 0

def load_telemetry(file_path):
    global telemetry_frames, new_points, frame_index

    telemetry_frames = []
    new_points = []
    frame_index = 0

    with open(file_path, "r") as f:
        text = f.read()

    text = text.replace(",]", "]")
    telemetry_frames = json.loads(text)

    for data in telemetry_frames:
        try:
            new_points.append({
                "i": len(new_points) + 1,
                "x": data["xpos"],
                "y": data["ypos"]
            })
        except KeyError:
            print("error, skipping entry")

    print(f"Loaded {len(telemetry_frames)} telemetry frames.")
      
def draw_curvature_circle(robot_x, robot_y, theta, curvature):
    if abs(curvature) < 1e-6:
        return  # straight line

    R = 1 / curvature

    cx = robot_x - math.sin(theta) * R
    cy = robot_y + math.cos(theta) * R

    sx, sy = field_to_screen(cx, cy)
    radius_pixels = min(abs(R) * SCALE, 5000)
    pygame.draw.circle(screen, (255,0,255), (int(sx), int(sy)), int(radius_pixels), 1)
    
def draw_frame_data():
    if not telemetry_frames:
        return

    data = telemetry_frames[frame_index]

    # Robot position
    rx, ry = field_to_screen(data["xpos"], data["ypos"])
    pygame.draw.circle(screen, (0,255,0), (rx, ry), 6)

    # Next point
    next_pt = data["lookahead"]
    nx, ny = field_to_screen(next_pt[1], next_pt[2])
    pygame.draw.circle(screen, (255,0,0), (nx, ny), 6)

    # Centered point
    cx, cy = field_to_screen((data["centeredPoint"][0]+(FIELD_SIZE/2)), (data["centeredPoint"][1]+(FIELD_SIZE/2)))
    pygame.draw.circle(screen, (0,255,255), (cx, cy), 6)

    # Draw pursuit line
    pygame.draw.line(screen, (255,51,255), (rx, ry), (nx, ny), 2)
    
    draw_curvature_circle(
    data["xpos"],
    data["ypos"],
    data["theta"],
    data["Curvature"]
    )
    
def draw_path(point_list, point_color, line_color):
    for i, p in enumerate(point_list):
        sx, sy = field_to_screen(p["x"], p["y"])
        
        # Draw the dot
        pygame.draw.circle(screen, point_color, (sx, sy), 3)

        # Draw the line connecting to the previous point
        if i > 0:
            prev = point_list[i - 1]
            psx, psy = field_to_screen(prev["x"], prev["y"])
            pygame.draw.line(screen, line_color, (psx, psy), (sx, sy), 2)
            
def draw_path_numbered(point_list, point_color, line_color):
    for i, p in enumerate(point_list):
        sx, sy = field_to_screen(p["x"], p["y"])
        
        # Draw the dot
        pygame.draw.circle(screen, point_color, (sx, sy), 3)

        # Draw point number
        label = font.render(str(p["i"]), True, (255,255,255))
        screen.blit(label, (sx + 5, sy - 5))

        # Draw the line connecting to the previous point
        if i > 0:
            prev = point_list[i - 1]
            psx, psy = field_to_screen(prev["x"], prev["y"])
            pygame.draw.line(screen, line_color, (psx, psy), (sx, sy), 2)            
def field_to_screen(x, y):
    return int(x * SCALE), int(WINDOW_SIZE - y * SCALE)

def screen_to_field(px, py):
    return px / SCALE, (WINDOW_SIZE - py) / SCALE

pygame.init()

screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 18)

def draw_field():
    screen.blit(field_image, (0, 0))

    # Outer border
    pygame.draw.rect(screen, (200,200,200), (0,0,WINDOW_SIZE,WINDOW_SIZE), 4)

    # 12" tile grid
    for i in range(0, FIELD_SIZE+1, 12):
        x = i * SCALE
        pygame.draw.line(screen, (70,70,70), (x,0), (x,WINDOW_SIZE))
        pygame.draw.line(screen, (70,70,70), (0,x), (WINDOW_SIZE,x))

def draw_points(points):
    for i, p in enumerate(points):
        sx, sy = field_to_screen(p["x"], p["y"])

        color = (0, 255, 0) if i == selected_index else (0, 0, 255)
        pygame.draw.circle(screen, color, (sx, sy), 3)

        if i > 0:
            prev = points[i - 1]
            psx, psy = field_to_screen(prev["x"], prev["y"])
            pygame.draw.line(screen, (255, 100, 100), (psx, psy), (sx, sy), 2)

def save_points(filename):
    with open(filename, "w") as f:
        json.dump(points, f, indent=None)
    print(f"Saved {len(points)} points to {filename}")


def load_points(filename):
    global points
    try:
        with open(filename, "r") as f:
            points = json.load(f)
        print(f"Loaded {len(points)} points from {filename}")
    except FileNotFoundError:
        print("No save file found")


def get_point_at_pos(mx, my):
    for i, p in enumerate(points):
        sx, sy = field_to_screen(p["x"], p["y"])
        if math.hypot(mx - sx, my - sy) < DRAG_RADIUS:
            return i
    return None


def handle_events():
    global running, selected_index, frame_index

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # KEYBOARD CONTROLS
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_s:
                save_points(FILE_NAME)

            if event.key == pygame.K_l:
                load_points("path.json")
            
            if event.key == pygame.K_t:
                load_telemetry("messy.json")

            if event.key == pygame.K_c:
                points.clear()
                new_points.clear()
                telemetry_frames.clear()
                
                print("Cleared points")
            if event.key == pygame.K_RIGHT:
                if frame_index < len(telemetry_frames) - 1:
                    frame_index += 1
                    print("Frame:", frame_index)

            if event.key == pygame.K_LEFT:
                if frame_index > 0:
                    frame_index -= 1
                    print("Frame:", frame_index)
                    

        # MOUSE DOWN
        if event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()

            if event.button == 1:  # LEFT CLICK
                idx = get_point_at_pos(mx, my)

                if idx is not None:
                    selected_index = idx  # start dragging
                else:
                    fx, fy = screen_to_field(mx, my)
                    points.append({"i": (len(points) + 1),"x": fx, "y": fy})
                    print(f"Added point {fx:.1f}, {fy:.1f}")

            if event.button == 3:  # RIGHT CLICK
                idx = get_point_at_pos(mx, my)
                if idx is not None:
                    removed = points.pop(idx)
                    print(f"Deleted {removed}")

        # MOUSE UP
        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                selected_index = None

        # MOUSE DRAG
        if event.type == pygame.MOUSEMOTION:
            if selected_index is not None:
                mx, my = pygame.mouse.get_pos()
                fx, fy = screen_to_field(mx, my)
                points[selected_index]["x"] = fx
                points[selected_index]["y"] = fy


running = True

while running:
    handle_events()
    draw_field()

    # Draw Manual Path (Blue dots, Red lines)
    draw_path(new_points, (255, 255, 0), (255, 255, 0))
    # Draw Telemetry Path (Yellow dots, Yellow lines)

    draw_path_numbered(points, (0, 0, 255), (255, 100, 100))
    draw_frame_data()

    pygame.display.flip()
    clock.tick(60)
pygame.quit()