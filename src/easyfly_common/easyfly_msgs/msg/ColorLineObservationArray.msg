# Original source image header, including on invalidation.
std_msgs/Header header
uint32 image_width
uint32 image_height
ColorLineObservation[] observations
# true + empty = processed image with no targets; false = unusable input/result.
bool valid
string reason
