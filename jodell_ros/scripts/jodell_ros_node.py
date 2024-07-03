from jodellSdk.jodellSdkDemo import ClawEpgTool
from threading import Thread,Lock,Event
import time
from jodell_ros.msg import claw_status,set_angle
import rospy


#start ros
rospy.init_node('jodell_ros_node')
rospy.loginfo("jodell ros driver")
#ros参数获取
port_name = rospy.get_param('/port_name', '/dev/ttyUSB0')
claw_salve_id = rospy.get_param("/salve_id",9)
clawTool = ClawEpgTool()


def set_angle_callback(msg):
    clawTool.runWithParam(claw_salve_id,msg.position,msg.speed,msg.torque)
    rospy.loginfo("send command: position:%d speed:%d torque:%d" %(msg.position,msg.speed,msg.torque))
    return

def main():

    flag = clawTool.serialOperation(port_name, 115200, True)
    port_list = clawTool.searchCom()
    print(port_list)
    # print(port_list)
    if flag==True:
        rospy.loginfo("Connect to port %s success" %(port_name))
        if clawTool.clawEnable(claw_salve_id,True):
            rospy.loginfo("Init claw with id: %d success" %(claw_salve_id))
            #初始化
            clawTool.runWithParam(claw_salve_id,0,255,255)
            #话题接收者
            set_angle_sub = rospy.Subscriber("/set_angle", set_angle, set_angle_callback)
            #开始更新线程
            topic_pub_thread = Thread(target=publish_msg)
            topic_pub_thread.start()
        else:
            rospy.logerr("Init claw with id: %d failed!" %(claw_salve_id))
            return
        while not rospy.is_shutdown():
            rospy.spin()

    else:
        rospy.logerr("Failed to connect to port: %s" %(port_name))

def publish_msg():
    state_pub = rospy.Publisher("/claw_status",claw_status,queue_size=5)

    while not rospy.is_shutdown():
        msg = claw_status()

        msg.position = clawTool.getClawCurrentLocation(claw_salve_id)[0]
        msg.speed = clawTool.getClawCurrentSpeed(claw_salve_id)[0]
        msg.torque = clawTool.getClawCurrentTorque(claw_salve_id)[0]
        msg.temp = clawTool.getClawCurrentTemperature(claw_salve_id)[0]
        msg.voltage = clawTool.getClawCurrentVoltage(claw_salve_id)[0]

        state_pub.publish(msg)
        time.sleep(0.1)


if __name__=='__main__' :
    main()