#!/usr/bin/env python3

import threading
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import depthai as dai

class UdpStream:
    def __init__(self, host='192.168.1.192', port=5601):
        Gst.init(None)
        self.host = host
        self.port = port
        self.pipeline = None
        self.data = None

    def start(self):
        t = threading.Thread(target=self._thread_udp)
        t.start()

    def _thread_udp(self):
        loop = GLib.MainLoop()
        loop.run()

    def send_data(self, data):
        self.data = data
        if self.pipeline:
            appsrc = self.pipeline.get_by_name('source')
            if appsrc:
                # Chuyển đổi dữ liệu thành Gst.Buffer
                buffer = Gst.Buffer.new_wrapped(self.data.tobytes())
                retval = appsrc.emit('push-buffer', buffer)
                if retval != Gst.FlowReturn.OK:
                    print("Error pushing buffer:", retval)

    def setup_pipeline(self):
        self.pipeline = Gst.parse_launch(
            'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ! '
            'h265parse ! rtph265pay pt=96 ! udpsink host={} port={}'.format(self.host, self.port)
        )
        appsrc = self.pipeline.get_by_name('source')
        if appsrc:
            appsrc.connect('need-data', self.on_need_data)
        self.pipeline.set_state(Gst.State.PLAYING)

    def on_need_data(self, src, length):
        if self.data is not None:
            # Chuyển đổi dữ liệu thành Gst.Buffer khi cần
            buffer = Gst.Buffer.new_wrapped(self.data.tobytes())
            retval = src.emit('push-buffer', buffer)
            if retval != Gst.FlowReturn.OK:
                print("Error pushing buffer:", retval)


if __name__ == "__main__":
    server = UdpStream(host='192.168.1.192', port=5601)
    server.setup_pipeline()

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

    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutSpatialData = pipeline.create(dai.node.XLinkOut)
    xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

    xoutDepth.setStreamName("depth")
    xoutSpatialData.setStreamName("spatialData")
    xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    spatialLocationCalculator.inputConfig.setWaitForMessage(False)

    # Create 25 ROIs (5x5 grid)
    for i in range(5):
        for j in range(5):
            config = dai.SpatialLocationCalculatorConfigData()
            config.depthThresholds.lowerThreshold = 200
            config.depthThresholds.upperThreshold = 30000
            config.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN

            config.roi = dai.Rect(
                dai.Point2f(i * 0.2, j * 0.2),
                dai.Point2f((i + 1) * 0.2, (j + 1) * 0.2)
            )
            spatialLocationCalculator.initialConfig.addROI(config)

    # Linking

    # Attach cameras to output Xlink
    # monoLeft.out.link(xoutLeft.input)

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
    stereo.depth.link(spatialLocationCalculator.inputDepth)

    spatialLocationCalculator.out.link(xoutSpatialData.input)
    xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)

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

    if device_info.protocol != dai.XLinkProtocol.X_LINK_USB_VSC:
        print("Running stream may be unstable due to connection... (protocol: {})".format(device_info.protocol))

    with dai.Device(pipeline, device_info) as device:
        device.setIrLaserDotProjectorBrightness(1000)

        # Output queues
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        # Get output queues.
        # leftQueue = device.getOutputQueue(name="left", maxSize=1)

        spatialCalcQueue = device.getOutputQueue(name="spatialData", maxSize=4, blocking=False)
        fontType = cv2.FONT_HERSHEY_TRIPLEX
        encoded = device.getOutputQueue("encoded", maxSize=30, blocking=True)
        print("Setup finished, streaming video over UDP to {}:{}".format(server.host, server.port))
        while True:
            data = encoded.get().getData()
            server.send_data(data)
            
            inDepth = depthQueue.get()
            depthFrame = inDepth.getFrame()  # Depth values in millimeters

            # inLeft = leftQueue.get()
            # LeftFrame = inLeft.getFrame()

            depth_downscaled = depthFrame[::4]
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1) if np.any(
                depth_downscaled != 0) else 0
            max_depth = np.percentile(depth_downscaled, 99)
            depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
            depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

            spatialData = spatialCalcQueue.get().getSpatialLocations()

            # Initialize a 5x5 grid to store total danger scores
            grid = np.zeros((5, 5))

            # Calculate total danger score for each ROI in the 5x5 grid
            for depthData in spatialData:
                roi = depthData.config.roi
                coords = depthData.spatialCoordinates
                # distance = math.sqrt(coords.x ** 2 + coords.y ** 2 + coords.z ** 2)
                distance = coords.z
                # Determine ROI grid position
                col = int(roi.topLeft().x * 5)
                row = int(roi.topLeft().y * 5)

                # If distance is less than 1m, increase danger score for that grid cell
                if distance / 1000 < 1:
                    grid[row, col] += 1

                # Visualize ROIs
                roi = roi.denormalize(width=depthFrameColor.shape[1], height=depthFrameColor.shape[0])
                xmin = int(roi.topLeft().x)
                ymin = int(roi.topLeft().y)
                xmax = int(roi.bottomRight().x)
                ymax = int(roi.bottomRight().y)

                color = (0, 255, 0)  # Green (safe)
                if distance / 1000 < 1:
                    color = (0, 0, 255)  # Red (danger)
                elif distance / 1000 < 2:
                    color = (0, 255, 255)  # Yellow (warning)

                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, thickness=2)
                cv2.putText(depthFrameColor, "{:.1f}m".format(distance / 1000), (xmin + 10, ymin + 20), fontType, 0.6,
                            color)

            # Sum up total scores for each zone (front, back, left, right, etc.)

            leftDanger = np.sum(grid[0:5, 0:4])
            rightDanger = np.sum(grid[0:5, 1:5])
            topDanger = np.sum(grid[0:4, 0:5])
            bottomDanger = np.sum(grid[1:5, 0:5])
            leftSafe = np.sum(grid[0:5, 0:1])
            rightSafe = np.sum(grid[0:5, 4:5])
            topSafe = np.sum(grid[0:1, 0:5])
            bottomSafe = np.sum(grid[4:5, 0:5])
            total = np.sum(grid[0:5, 0:5])

            # Quyết định hành động với trọng số cho các khu vực
            if total == 0:
                action = "Không có vật cản, tiếp tục bay"
            elif leftDanger > 0 and rightSafe == 0:
                action = "Né phải"
            elif rightDanger > 0 and leftSafe == 0:
                action = "Né trái"
            elif topDanger > 0 and bottomSafe == 0:
                action = "Bay xuống"
            elif bottomDanger > 0 and topSafe == 0:
                action = "Bay lên"
            else:
                action = "Lùi lại"  # Không có vật cản nào

            # Log the action
            print(f"Action: {action}")

            # Show the frame
            cv2.imshow("depth", depthFrameColor)
            # Show the frame
            # cv2.imshow("Left", LeftFrame)

            if cv2.waitKey(1) == ord('q'):
                break
