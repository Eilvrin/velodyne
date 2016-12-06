# Raw Velodyne LIDAR packet.

time stamp              # Start of data collection + TCP transfer time +  calibration offset
uint8[1206] data        # Velodyne data blocks + timestamp + factory bytes (= one velodyne packet - header)

