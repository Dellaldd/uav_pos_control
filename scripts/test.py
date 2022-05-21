#! /usr/bin/env python
import rospy

# omega=25;      
#     x_des=1*cos(t*omega/180*pi);
#     y_des=1*sin(t*omega/180*pi);
#     z_des=1; 

# ('Timer called at ', '1652256175638005971')
# ('Timer called at ', '1652256177637926101')  2 sec  unit:nsec
class Controll():
    def _init_(self):
        self.start_time = 0
        self.cur_time = 0

    def my_callback(self,event):
        # print('Timer called at ', event.current_real)
        # rospy.sleep(2)
        self.cur_time = event.current_real
        print((self.cur_time-self.start_time)/1e8)
        # print(rospy.Time.now() - event.current_real)
        print("end")


print("start")
rospy.init_node('offboard_test_node', anonymous=True)
cnt = Controll()
cnt.start_time = rospy.Time.now()
cnt.cur_time = cnt.start_time
rospy.Timer(rospy.Duration(nsecs=1e8), cnt.my_callback, oneshot=False)
rospy.spin()