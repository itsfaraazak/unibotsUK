string ball_type          # "ping_pong" or "steel"
float32 pixel_x           # centre pixel x
float32 pixel_y           # centre pixel y
float32 distance_cm       # estimated distance from pinhole model
float32 bearing_deg       # horizontal angle from camera centre, positive=right
float32 confidence        # 0.0 to 1.0
bool yolo_confirmed       # True if YOLO has validated this detection
uint32 track_id           # Kalman filter track ID
