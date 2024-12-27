#!/usr/bin/env python3

import socket
import depthai as dai

# # Địa chỉ và cổng UDP
# UDP_IP = "127.0.0.1"  # Địa chỉ IP máy nhận (localhost)
# UDP_PORT = 5005  # Cổng gửi dữ liệu
#
# # Tạo socket UDP
# sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Tạo pipeline DepthAI
pipeline = dai.Pipeline()

FPS = 30
colorCam = pipeline.create(dai.node.ColorCamera)
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
colorCam.setInterleaved(False)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
colorCam.setFps(FPS)

videnc = pipeline.create(dai.node.VideoEncoder)
videnc.setDefaultProfilePreset(FPS, dai.VideoEncoderProperties.Profile.H265_MAIN)
colorCam.video.link(videnc.input)

veOut = pipeline.create(dai.node.XLinkOut)
veOut.setStreamName("encoded")
videnc.bitstream.link(veOut.input)

# Lấy danh sách thiết bị DepthAI
device_infos = dai.Device.getAllAvailableDevices()
if len(device_infos) == 0:
    raise RuntimeError("No DepthAI device found!")
else:
    print("Available devices:")
    for i, info in enumerate(device_infos):
        print(f"[{i}] {info.getMxId()} [{info.state.name}]")
    if len(device_infos) == 1:
        device_info = device_infos[0]
    else:
        val = input("Which DepthAI Device you want to use: ")
        try:
            device_info = device_infos[int(val)]
        except:
            raise ValueError("Incorrect value supplied: {}".format(val))

# Cảnh báo nếu không sử dụng giao thức USB
if device_info.protocol != dai.XLinkProtocol.X_LINK_USB_VSC:
    print("Running UDP stream may be unstable due to connection... (protocol: {})".format(device_info.protocol))

# Kết nối với thiết bị và gửi luồng video qua UDP
with dai.Device(pipeline, device_info) as device:
    seqNum = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Tạo socket UDP
    encoded = device.getOutputQueue("encoded", maxSize=30, blocking=True)
    print("Setup finished")
    # print(f"Setup finished, sending H265 stream to UDP {UDP_IP}:{UDP_PORT}")
    while True:
        data = encoded.get().getData()  # Lấy dữ liệu từ DepthAI
        dataToSend = len(data)
        startPos = 0
        while startPos < dataToSend:
            dataLeft = dataToSend - startPos
            toSend = min(1400, dataLeft)  # Sử dụng min() để tìm kích thước gói
            endPos = startPos + toSend

            # Chuyển đổi phần dữ liệu từ numpy.ndarray thành bytearray
            buffer = bytearray(data[startPos:endPos].tobytes())  # Sử dụng tobytes() để chuyển mảng numpy sang bytes
            print(f"Sending {len(buffer)} / {dataLeft} {startPos} {endPos}")
            startPos = endPos
            sock.sendto(buffer, ("192.168.1.192", 5601))  # Gửi qua UDP tới địa chỉ và cổng


