string ball_type
float32 world_x           # meters, arena map frame (0,0 = SW corner)
float32 world_y
float32 confidence
string status             # "VISIBLE", "OCCLUDED", or "COLLECTED"
builtin_interfaces/Time last_seen
uint32 track_id
float32 predicted_x       # predicted world X at t + pred_horizon_s
float32 predicted_y       # predicted world Y at t + pred_horizon_s
float32 speed_m_s         # current estimated speed sqrt(vx^2+vy^2)
int32 density             # count of other tracked balls within density_radius_m
