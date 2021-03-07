import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
import numpy as np

def occupancygrid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    data=np.where(data==-1, 0, data)
    data=np.where(data==100, 255, data)


    return data#, mask=data==-1, fill_value=-1)

def callback(msg):
    map_np=occupancygrid_to_numpy(msg)

#    np.savetxt("foo.csv", map_np, delimiter=",")

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('map', OccupancyGrid, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
