# Original source image header, including on invalidation. No refreshed detection timestamp.
std_msgs/Header header
uint32 image_width
uint32 image_height
EllipseObservation[] observations
# true + empty = processed image with no targets; false = unusable input/result.
bool valid
string reason
