import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
from rose.fft_structure_extraction import \
    FFTStructureExtraction as structure_extraction
from rose.fft_structure_extraction import save_simple_map
from rose_ros.srv import Save, SaveResponse
import png
import numpy


class rose_wrapper:
    def __init__(self, params):
        self.pub_map = OccupancyGrid()
        self.params = params
        self.info = None
        self.header = None
        self.rose = None

    def set_threshold(self, tr_msg):
        if not self.rose is None:
            self.rose.simple_filter_map(tr_msg.data)

            result = self.rose.analysed_map * 1
            result = np.where(result == 1, 100, result)
            result = result[:self.info.height, :self.info.width]
            self.pub_map.header = self.header
            self.pub_map.info = self.info
            self.pub_map.data = list(result.ravel())

    def filter_map(self, msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height,
                                                           msg.info.width)
        data = np.where(data == -1, 128, data)
        data = np.where(data == 0, 255, data)
        data = np.where(data == 100, 0, data)

        self.rose = structure_extraction(data,
                                         peak_height=self.params['peak_height'],
                                         smooth=self.params['smooth'],
                                         sigma=self.params['sigma'])
        self.rose.process_map()

        if self.params['filtering_tr'] > 0:
            self.rose.simple_filter_map(self.params['filtering_tr'])
        else:
            self.rose.histogram_filtering()

        result = self.rose.analysed_map * 1
        result = np.where(result == 1, 100, result)
        self.info = msg.info
        self.header = msg.header
        result = result[:self.info.height, :self.info.width]
        self.pub_map.header = self.header
        self.pub_map.info = self.info
        self.pub_map.data = list(result.ravel())

    def handle_save(self, req):
        s = self.rose.analysed_map * 255
        a = numpy.array(s, dtype=numpy.uint8)
        if not req.path=="":
            png.from_array(a, mode="L").save(req.path)
        else:
            png.from_array(a, mode="L").save(self.params['save_path'])
        return "Saved"


def listener():
    rospy.init_node('listener', anonymous=True)
    params = dict()
    params['sub_map_topic_name'] = rospy.get_param("~input_map", "map")
    params['pub_map_topic_name'] = rospy.get_param("~output_map", "o_map")
    params['filtering_tr'] = rospy.get_param("~threshold", -1.0)
    params['peak_height'] = rospy.get_param("~peak_heigh", 0.05)
    params['smooth'] = rospy.get_param("~smooth", True)
    params['sigma'] = rospy.get_param("~sigma", 0.5)
    params['tr_topic'] = rospy.get_param("~tr_topic", "threshold")
    params['save_path'] = rospy.get_param("~save_path", "")

    rw = rose_wrapper(params)

    pub = rospy.Publisher(params['pub_map_topic_name'], OccupancyGrid,
                          queue_size=10, latch=True)

    rospy.Subscriber(params['sub_map_topic_name'], OccupancyGrid, rw.filter_map)
    rospy.Subscriber(params['tr_topic'], Float64, rw.set_threshold)

    s = rospy.Service('save_map', Save, rw.handle_save)

    while not rospy.is_shutdown():
        pub.publish(rw.pub_map)

    rospy.spin()


if __name__ == '__main__':
    listener()
