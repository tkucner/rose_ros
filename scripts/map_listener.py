import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
from rose.fft_structure_extraction import FFTStructureExtraction as structure_extraction


class rose_wrapper:
    def __init__(self,params):
        self.pub_map = OccupancyGrid()
        self.params=params

    def filter_map(self,msg):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        data = np.where(data==-1, 128, data)
        data = np.where(data==0, 255, data)
        data = np.where(data==100, 0, data)

        rose = structure_extraction(data, peak_height=self.params['peak_height'], smooth=self.params['smooth'] , sigma=self.params['sigma'])
        rose.process_map()
        if self.params['filtering_tr']>0:
            rose.simple_filter_map(self.params['filtering_tr'])
        else:
            rose.histogram_filtering()
        result = rose.analysed_map*1
        result = np.where(result==1, 100, result)
        result = result[:msg.info.height, :msg.info.width]
        self.pub_map.header = msg.header
        self.pub_map.info = msg.info
        self.pub_map.data=list(result.ravel())


def listener():

    rospy.init_node('listener', anonymous=True)
    params=dict()
    params['sub_map_topic_name']=rospy.get_param("~input_map", "map")
    params['pub_map_topic_name']=rospy.get_param("~output_map", "o_map")
    params['filtering_tr']=rospy.get_param("~threshold", -1.0)
    params['peak_height']=rospy.get_param("~peak_heigh", 0.05)
    params['smooth']=rospy.get_param("~smooth", True)
    params['sigma']=rospy.get_param("~sigma", 0.5)

    rw = rose_wrapper(params)

    pub= rospy.Publisher(params['pub_map_topic_name'], OccupancyGrid, queue_size=10, latch=True)

    rospy.Subscriber(params['sub_map_topic_name'], OccupancyGrid, rw.filter_map)

    while not rospy.is_shutdown():
        pub.publish(rw.pub_map)

    rospy.spin()

if __name__ == '__main__':
    listener()
