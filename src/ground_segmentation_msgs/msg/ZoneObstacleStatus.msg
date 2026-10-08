# Zone obstacle detection status message
bool red_zone                   # True if obstacle detected in red (danger) zone and in path
float64 red_zone_distance       # Distance to closest obstacle in red zone (-1.0 if none)
bool yellow_zone                # True if obstacle detected in yellow (warning) zone and in path
float64 yellow_zone_distance    # Distance to closest obstacle in yellow zone (-1.0 if none)
float64 obstacle_angle_deg      # Angle of closest in-path obstacle in degrees (NaN if none)
string obstacle_side            # Side of closest in-path obstacle: 'left', 'right', 'center', or ''
