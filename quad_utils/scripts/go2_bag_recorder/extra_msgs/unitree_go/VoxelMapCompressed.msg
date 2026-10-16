# Go2 firmware type not shipped in unitree_ros2. Field order/types copied from
# unitree_sdk2 include/unitree/idl/go2/VoxelMapCompressed_.hpp so the CDR wire
# format matches unitree_go::msg::dds_::VoxelMapCompressed_.
float64 stamp
string frame_id
float64 resolution
float64[3] origin
uint16[3] width
uint64 src_size
uint8[] data
