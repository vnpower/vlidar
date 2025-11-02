from rplidarc1 import RPLidar, protocol
from rplidarc1.protocol import Request, Response

lidar = RPLidar("\\\\.\\COM6", 460800) # Comm port 6 on Windows

# create request
request_cmd = Request.create_request(protocol.RequestBytes.STOP_BYTE)

# send request
Request.send_request(lidar._serial, request_cmd)

# reset lidar
lidar.reset()