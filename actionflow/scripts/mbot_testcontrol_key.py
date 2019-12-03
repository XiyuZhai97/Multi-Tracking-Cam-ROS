#!/usr/bin/env python
# Xiyu
import  sys
import  tty, termios
import rospy
from std_msgs.msg import String
def GetKey(last_cmd):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
    try :
        tty.setraw( fd )
        ch = sys.stdin.read( 1 )
    finally :
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        if ch == 'w':
            command = r"1:l=5,r=5\n" \
                    +r"2:l=5,r=5\n" \
                    +r"3:l=5,r=5\n" \
                    +r"4:l=5,r=5\n" \
                    +r"5:l=5,r=5\n" \
                    +r"6:l=5,r=5\n" \
                    +r"7:l=5,r=5\n" \
                    +r"8:l=5,r=5\n" \
                    +r"9:l=5,r=5\n" \
                    +r"10:l=5,r=5\n" \
                    +r"11:l=5,r=5\n" \
                    +r"12:l=5,r=5\n" \
                    +r"13:l=5,r=5\n" \
                    +r"14:l=5,r=5\n" \
                    +r"15:l=5,r=5\n" \
                    +r"16:l=5,r=5\n" \
                    +r"17:l=5,r=5\n" \
                    +r"18:l=5,r=5\n" \
                    +r"19:l=5,r=5\n" \
                    +r"20:l=5,r=5\n"
        
            print("Forward 20")
        elif ch == 's':
            command = "1:l=0,r=0\n" \
                    +"2:l=0,r=0\n" \
                    +"3:l=0,r=0\n" \
                    +"4:l=0,r=0\n" \
                    +"5:l=0,r=0\n" \
                    +"6:l=0,r=0\n" \
                    +"7:l=0,r=0\n" \
                    +"8:l=0,r=0\n" \
                    +"9:l=0,r=0\n" \
                    +"10:l=0,r=0\n" \
                    +"11:l=0,r=0\n" \
                    +"12:l=0,r=0\n" \
                    +"13:l=0,r=0\n" \
                    +"14:l=0,r=0\n" \
                    +"15:l=0,r=0\n" \
                    +"16:l=0,r=0\n" \
                    +"17:l=0,r=0\n" \
                    +"18:l=0,r=0\n" \
                    +"19:l=0,r=0\n" \
                   + "20:l=0,r=0" 
            print("Stop 20")

        elif ch == 'a':
            command = "11:l=5,r=0"
            print(command)
            print("Left")

        elif ch == 'd':
            command = "11:l=0,r=5"
            print(command)
            print("Right")
        elif ch == 'c':
            l = 0.114
            carid = input("Car_id:")
            sl = input("l:")
            sr = input("r:")

            command = str(carid)+":"+"l=" +str(sl) +",r="+str(sr)
            print(command)
        elif ch == 'q':
            command = "11:l=0,r=0"
            print("Stop")
        elif ch == '0':
            command = "0"
            print(command)
        elif ch == '1':
            command = "1"
            print(command)
        elif ch == '2':
            command = "2"
            print(command)
        elif ch == '3':
            command = "3"
            print(command)
        elif ch == '4':
            command = "4"
            print(command)
        elif ch == '5':
            command = "5"
            print(command)
        else:
            pass
            command  = "1:l=20,r=20"
    return command
def talker():
    pub = rospy.Publisher('axis', String, queue_size=10)
    rospy.init_node('test_control', anonymous=True)
    rate = rospy.Rate(60) # 10hz
    
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        command = GetKey("stop")
        pub.publish(command)
        rate.sleep()
if __name__ == '__main__':
#    keyboard_publisher = KeyboardPublisher()
#    keyboard_publisher.keyboard_listener()
    try:
    #   rospy.spin()
        talker()
    except rospy.ROSInterruptException:
        print("Stopping keyboard_publisher")
