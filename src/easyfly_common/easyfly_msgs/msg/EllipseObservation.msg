# Image coordinates, pixels. No camera calibration or metric pose is implied.
float64 center_x
float64 center_y
float64 major_axis_px
float64 minor_axis_px
# Major-axis angle, image +x towards image +y, [0, pi); circles have ambiguous direction.
float64 angle_rad
float64 radial_error
# Geometric fit quality [0,1], not a calibrated probability.
float64 score
