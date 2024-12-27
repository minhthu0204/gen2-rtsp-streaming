import depthai as dai
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

GObject.threads_init()
Gst.init(None)

class VideoStreamer:
    def __init__(self):
        self.number_frames = 0
        self.fps = 10
        self.duration = 1 / self.fps * Gst.SECOND
        # Sử dụng h265parse và rtph265pay
        self.pipe = "appsrc name=source is-live=true block=true format=GST_FORMAT_TIME " \
                    "caps=video/x-raw,format=BGR,width=1920,height=1080,framerate={}/1 " \
                    "! videoconvert ! video/x-raw,format=I420 " \
                    "! h265parse ! rtph265pay name=pay0 config-interval=1 pt=96 " \
                    "! udpsink host=127.0.0.1 port=1234".format(self.fps)
        self.pipeline = Gst.parse_launch(self.pipe)
        self.loop = None
        appsrc = self.pipeline.get_by_name('source')
        appsrc.connect('need-data', self.on_need_data)

        # Tạo pipeline DepthAI
        self.device = None
        self.encoded = None

    def setup_depthai_pipeline(self):
        # Tạo pipeline DepthAI
        pipeline = dai.Pipeline()

        FPS = 10
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

        device_infos = dai.Device.getAllAvailableDevices()
        if len(device_infos) == 0:
            raise RuntimeError("No DepthAI device found!")

        device_info = device_infos[0]
        self.device = dai.Device(pipeline, device_info)
        self.encoded = self.device.getOutputQueue("encoded", maxSize=30, blocking=True)

    def run(self):
        self.pipeline.set_state(Gst.State.READY)
        self.pipeline.set_state(Gst.State.PLAYING)

        self.loop = GObject.MainLoop()
        self.loop.run()

    def on_need_data(self, src, length):
        if self.encoded is not None:
            data = self.encoded.get().getData()

            # Push dữ liệu vào appsrc
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)
            buf.duration = self.duration
            timestamp = self.number_frames * self.duration
            buf.pts = buf.dts = int(timestamp)
            buf.offset = timestamp
            self.number_frames += 1
            retval = src.emit('push-buffer', buf)

            print(f'Pushed buffer, frame {self.number_frames}, duration {self.duration / Gst.SECOND}s')

            if retval != Gst.FlowReturn.OK:
                print(retval)

if __name__ == "__main__":
    streamer = VideoStreamer()
    streamer.setup_depthai_pipeline()
    streamer.run()
