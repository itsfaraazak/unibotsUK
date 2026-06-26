float32 world_x           # projected obstacle world position
float32 world_y
float32 radius_m          # estimated obstacle radius for half-plane computation
bool is_confirmed_robot   # True if YOLO classified it as a robot
float32 pixel_x           # centre pixel x in original image
float32 bearing_deg       # horizontal bearing from camera bore, positive=right
float32 distance_m        # estimated distance in metres
