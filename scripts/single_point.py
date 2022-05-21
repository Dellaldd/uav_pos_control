import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from scipy.spatial.transform import Rotation as R
import threading

def quaternion2euler(quaternion):
    r = R.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=True)
    return euler

def thread_job():
    rospy.spin()

class Controller:
    
    def __init__(self):

        # 无人机当前状态
        self.state = State()
        # 获取无人机的当前位置
        self.local_pos = PoseStamped()
        # 获取无人机当前位置作为参考
        self.cur_pos = PoseStamped()
        
        # 订阅无人机的状态
        rospy.Subscriber('mavros/state', State, self.stateCb)
        self.sp_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size = 1)

        # 回调函数运行
        self.add_thread = threading.Thread(target = thread_job)
        self.add_thread.start()

        # 无人机目标位置
        self.target_sp = PoseStamped()
        self.target_sp.pose.position.x = 0
        self.target_sp.pose.position.y = 0
        self.target_sp.pose.position.z = 1.5
       

    # 回调函数

    def stateCb(self, msg):
        self.state = msg
        print("current mode is: ",self.state.mode)

# 主函数
def main():
    print("start!")
    rospy.init_node('offboard_test_node', anonymous=True)

    cnt = Controller()
    rate = rospy.Rate(100)

    # ROS main loop
    while not rospy.is_shutdown():
        cnt.target_sp.header.stamp = rospy.Time.now()
        cnt.sp_pub.publish(cnt.target_sp) 
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
