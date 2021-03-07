import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np
from rose.fft_structure_extraction import FFTStructureExtraction as structure_extraction


pub = rospy.Publisher('filtered_map', OccupancyGrid, queue_size=10, latch=True)
pub_map = OccupancyGrid()

def occupancygrid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    data = np.where(data==-1, 128, data)
    data = np.where(data==0, 255, data)
    data = np.where(data==100, 0, data)
    rose = structure_extraction(data, peak_height=0.05, smooth=True ,sigma=0.5)
    rose.process_map()
    rose.simple_filter_map(0.35)
    result = rose.analysed_map*1
    result = np.where(result==1, 100, result)
    result=result[:msg.info.height, :msg.info.width]
    print("Procesed")
    print(result)
    return result

def callback(msg):
    map_filtered = occupancygrid_to_numpy(msg)
    pub_map.header = msg.header
    pub_map.info = msg.info
    pub_map.data=list(map_filtered.ravel())

def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('map', OccupancyGrid, callback)
    while not rospy.is_shutdown():
        pub.publish(pub_map)



    rospy.spin()

if __name__ == '__main__':
    listener()
