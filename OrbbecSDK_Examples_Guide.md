# OrbbecSDK Examples Guide for Multi-Camera Setup

This guide covers the most useful OrbbecSDK examples for working with multiple POE cameras (specifically Gemini 335Le), focusing on point cloud filtering, image format control, and camera intrinsics for ROS wrapper development.

## 🎯 Key Examples for Multi-Camera Setup

### 1. Multi-Device Management

#### `3.advanced.multi_devices` - Essential for 6 POE Cameras
**Location**: `/examples/3.advanced.multi_devices/multi_device.cpp`

**Key Features**:
- Automatically detects and manages multiple connected cameras
- Creates separate pipeline for each device
- Handles synchronized frame capture from all devices
- Thread-safe frame handling with mutex protection

**Code Highlights**:
```cpp
// Query all connected devices
auto devList = ctx.queryDeviceList();
int devCount = devList->getCount();

// Create pipeline for each device
std::map<int, std::shared_ptr<ob::Pipeline>> pipes;
for(int i = 0; i < devCount; i++) {
    auto dev = devList->getDevice(i);
    auto pipe = std::make_shared<ob::Pipeline>(dev);
    pipes.insert({ i, pipe });
}

// Enable depth and color streams for all devices
config->enableVideoStream(OB_STREAM_COLOR);
config->enableVideoStream(OB_STREAM_DEPTH);
```

**Perfect for**: Managing your 6 POE Gemini 335Le cameras (192.168.1.31-36)

#### `3.advanced.multi_devices_sync` - Hardware Synchronization
**Location**: `/examples/3.advanced.multi_devices_sync/`

**Key Features**:
- Hardware-level synchronization between multiple cameras
- Master-slave configuration for precise timing
- Essential for applications requiring temporal alignment

---

### 2. Point Cloud Processing & Filtering

#### `3.advanced.point_cloud` - Point Cloud Generation
**Location**: `/examples/3.advanced.point_cloud/point_cloud.cpp`

**Key Features**:
- Creates RGBD and depth-only point clouds
- Supports PLY file export
- Frame synchronization and alignment
- Mesh generation capabilities

**Code Highlights**:
```cpp
// Create point cloud filter
auto pointCloud = std::make_shared<ob::PointCloudFilter>();

// Create alignment filter (depth to color)
auto align = std::make_shared<ob::Align>(OB_STREAM_COLOR);

// Enable frame synchronization
pipeline->enableFrameSync();

// Generate point cloud from aligned frames
auto pointCloudFrame = pointCloud->process(alignedFrameSet);
```

**Filtering Options**:
- Spatial filtering through alignment
- Temporal filtering via frame synchronization
- Hardware depth-to-color alignment

#### `3.advanced.post_processing` - Advanced Filtering
**Location**: `/examples/3.advanced.post_processing/post_processing.cpp`

**Key Features**:
- Multiple post-processing filters available
- Real-time filter parameter adjustment
- Noise removal and spatial filtering
- Interactive filter control

**Available Filters**:
- **Spatial filters**: Edge-preserving smoothing
- **Temporal filters**: Frame-to-frame noise reduction  
- **Noise removal**: Statistical outlier removal
- **Hole filling**: Fill missing depth data
- **Decimation**: Reduce point cloud density

**Interactive Control**:
```cpp
// List available filters
printFiltersInfo(filterList);

// Enable/disable filters dynamically
filter->enable(true/false);

// Adjust filter parameters in real-time
filter->setConfigValue(configName, value);
```

---

### 3. Image Format & Stream Control

#### `1.stream.color` & `1.stream.depth` - Basic Stream Configuration
**Location**: `/examples/1.stream.color/color.cpp`

**Key Features**:
- Configure resolution, format, and FPS
- Support for various image formats
- Stream-specific parameter control

**Format Configuration**:
```cpp
// Enable specific format and resolution
config->enableVideoStream(OB_STREAM_COLOR, width, height, fps, format);
config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);

// Available formats:
// Color: OB_FORMAT_RGB, OB_FORMAT_BGR, OB_FORMAT_MJPG, OB_FORMAT_NV21
// Depth: OB_FORMAT_Y16, OB_FORMAT_Y8
```

#### `3.advanced.common_usages` - Comprehensive Camera Control
**Location**: `/examples/3.advanced.common_usages/common_usages.cpp`

**Key Features**:
- Complete camera parameter control
- Exposure, gain, and mirror settings
- Auto-exposure control
- Laser and LDP (Laser Depth Projection) control

**Image Control Examples**:
```cpp
// Exposure control
device->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, exposureValue);
device->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, exposureValue);

// Gain control  
device->setIntProperty(OB_PROP_DEPTH_GAIN_INT, gainValue);
device->setIntProperty(OB_PROP_COLOR_GAIN_INT, gainValue);

// Auto-exposure toggle
device->setBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, enable);
device->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable);

// Mirror settings
device->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, enable);
device->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, enable);
```

---

### 4. Camera Intrinsics & Calibration Parameters

#### `3.advanced.common_usages` - Camera Parameters Extraction
**Function**: `getCameraParams()`

**Key Features**:
- Extract intrinsic parameters for each sensor
- Distortion coefficients
- Essential for ROS camera_info messages

**Code Implementation**:
```cpp
void getCameraParams() {
    for(const auto &item: profilesMap) {
        auto profile = item.second;
        auto type = item.first;
        
        // Get intrinsic parameters
        auto intrinsics = profile->getIntrinsic();
        auto distortion = profile->getDistortion();
        
        std::cout << "Intrinsics: "
                  << "fx:" << intrinsics.fx << ", fy:" << intrinsics.fy 
                  << ", cx:" << intrinsics.cx << ", cy:" << intrinsics.cy << std::endl;
                  
        std::cout << "Distortion: "
                  << "k1:" << distortion.k1 << ", k2:" << distortion.k2 
                  << ", k3:" << distortion.k3 << ", k4:" << distortion.k4
                  << ", k5:" << distortion.k5 << ", k6:" << distortion.k6 
                  << ", p1:" << distortion.p1 << ", p2:" << distortion.p2 << std::endl;
    }
}
```

**ROS Integration Ready**:
- Intrinsic matrix: `[fx, 0, cx, 0, fy, cy, 0, 0, 1]`
- Distortion coefficients: `[k1, k2, p1, p2, k3, k4, k5, k6]`
- Perfect for `sensor_msgs/CameraInfo` messages

---

### 5. Additional Useful Examples

#### `2.device.control` - Device Property Management
**Location**: `/examples/2.device.control/device_control.cpp`
- Runtime parameter adjustment
- Property enumeration and control
- Device-specific settings

#### `3.advanced.coordinate_transform` - Coordinate Systems
**Location**: `/examples/3.advanced.coordinate_transform/`
- Transform between depth and color coordinate systems
- Essential for multi-camera calibration

#### `5.wrapper.pcl` - PCL Integration
**Location**: `/examples/5.wrapper.pcl/pcl/pcl.cpp`
- Direct PCL point cloud integration
- Advanced point cloud processing
- Visualization with PCL viewer

#### `4.misc.save_to_disk` - Data Recording
**Location**: `/examples/4.misc.save_to_disk/save_to_disk.cpp`
- Save depth and color frames
- Timestamp preservation
- Batch processing capabilities

---

## 🚀 Quick Start for Multi-Camera Setup

### 1. Basic Multi-Camera Detection
```bash
cd /home/contoro/ws/OrbbecSDK_v2/examples/3.advanced.multi_devices
mkdir build && cd build
cmake .. && make
./multi_device
```

### 2. Point Cloud with Filtering
```bash
cd /home/contoro/ws/OrbbecSDK_v2/examples/3.advanced.post_processing
mkdir build && cd build
cmake .. && make
./post_processing
```

### 3. Camera Parameters Extraction
```bash
cd /home/contoro/ws/OrbbecSDK_v2/examples/3.advanced.common_usages
mkdir build && cd build
cmake .. && make
./common_usages
# Use 'p' command to print camera parameters
```

---

## 🔧 ROS Wrapper Development Tips

### Camera Info Message Population
```cpp
// From getCameraParams() function
sensor_msgs::CameraInfo camera_info;
camera_info.K = {intrinsics.fx, 0, intrinsics.cx,
                 0, intrinsics.fy, intrinsics.cy,
                 0, 0, 1};
camera_info.D = {distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3};
```

### Multi-Camera Namespace Management
```cpp
// For each camera (192.168.1.31-36)
std::string camera_ns = "camera_" + std::to_string(camera_id);
ros::NodeHandle nh(camera_ns);
```

### Point Cloud Publishing
```cpp
// Convert OrbbecSDK point cloud to ROS PointCloud2
sensor_msgs::PointCloud2 cloud_msg;
// Use the point cloud filter from 3.advanced.point_cloud example
```

---

## 📝 Notes for Your Setup

- **IP Addresses**: Your cameras (192.168.1.31-36) should be automatically detected by the multi_devices example
- **Synchronization**: Consider using `multi_devices_sync` for hardware synchronization across all 6 cameras
- **Performance**: The examples include efficient multi-threading and memory management
- **Filtering**: Post-processing filters can significantly improve point cloud quality for robotics applications

This guide provides the foundation for developing a robust ROS wrapper for your 6-camera Gemini 335Le setup with comprehensive point cloud filtering and camera parameter management.
