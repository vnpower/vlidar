from rplidarc1 import RPLidar, protocol
from rplidarc1.protocol import Request, Response

lidar = RPLidar("\\\\.\\COM6", 460800) # Comm port 6 on Windows

data_out_buffer: bytes = bytes(0)

lidar.logger.debug("Starting healthcheck.")
request = Request.create_request(protocol.RequestBytes.GET_HEALTH_BYTE)
Request.send_request(lidar._serial, request)
length, mode = Response.parse_response_descriptor(lidar._serial)
response = Response.handle_response(serial=lidar._serial, length=length)
# print("Healthcheck Response:", response)
if response[0] == 0:
    print("Lidar status: Good")

# scan_request = Request.create_request(protocol.RequestBytes.SCAN_BYTE)
scan_request = Request.create_request(protocol.RequestBytes.INFO_BYTE)
Request.send_request(lidar._serial, scan_request)
length, mode = Response.parse_response_descriptor(lidar._serial)
response = Response.handle_response(serial=lidar._serial, length=length)
# print("Mode:", mode)
# print("Lidar Info Response:", response)

# parse series and model
series = ((response[0] >> 4) & 0x0F) # Get series from high nibble
model = response[0] & 0x0F # Get model from low nibble
if series == 0x04 :
    print("This is a RPLIDAR C-series device.")
    print(f"The model is C1M{model}.")
else:
    S_series = series - 5 - 5 # Check lại chỗ này, trừ 5 một hay hai lần
    print(f"This is a RPLIDAR S{series} device.")
    print(f"The model is S{S_series}M{model}.")

print(f"Firmware version: {response[2]}.{response[1]}")
print(f"Hardware version: {response[3]}")
print(f"Serial number: {response[4:].hex()}")

lidar.reset()