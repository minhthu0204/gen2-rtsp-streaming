#!/usr/bin/env python3

import socket
import depthai as dai
import numpy as np

# Địa chỉ và cổng UDP
UDP_IP = "192.168.1.69"  # Địa chỉ IP máy nhận (localhost)
UDP_PORT = 5602  # Cổng gửi dữ liệu

# Tạo socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Tạo pipeline DepthAI
pipeline = dai.Pipeline()

FPS = 30
colorCam = pipeline.create(dai.node.ColorCamera)
colorCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
colorCam.setInterleaved(False)
colorCam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
colorCam.setFps(FPS)

videnc = pipeline.create(dai.node.VideoEncoder)
videnc.setBitrate(2000)

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
    encoded = device.getOutputQueue("encoded", maxSize=30, blocking=True)
    print(f"Setup finished, sending H265 stream to UDP {UDP_IP}:{UDP_PORT}")
    while True:
        # Lấy dữ liệu từ DepthAI
        data = encoded.get().getData()
        dataToSend = len(data)
        if(dataToSend < 60000):
            print(dataToSend)
            data = np.ascontiguousarray(data)
            sock.sendto(data, (UDP_IP, UDP_PORT))
        else:
            print(dataToSend)

