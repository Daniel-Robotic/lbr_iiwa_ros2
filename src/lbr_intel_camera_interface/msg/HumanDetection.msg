std_msgs/Header header

# image data
string format
uint8[] data

# bounding box (x, y, width, height)
float32 x
float32 y
float32 width
float32 height

# confidence (уверенность)
float32 confidence

# keypoints (ключевые точки)
float32[] keypoints
# distance (Растояние до точки)
float32[] kps_distance