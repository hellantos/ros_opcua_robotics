import rospy
import asyncio
import roslib; roslib.load_manifest('urdfdom_py')
from sensor_msgs.msg import JointState
from ros_opcua_robotics.opcua_robotics import OpcuaRobotics
from urdf_parser_py.urdf import URDF

class ros_opcua_robotics_node(object):
    def __init__(self):
        rospy.init_node("ros_opcua_robotics", anonymous=True)
        rospy.Rate(100)
        ua_nodeset_path = rospy.get_param("/UA_nodeset_path")
        self.DI_path = ua_nodeset_path + "/DI/Opc.Ua.Di.NodeSet2.xml"
        self.Robotics_path = ua_nodeset_path + "/Robotics/Opc.Ua.Robotics.NodeSet2.xml"
        
    
    async def robotics_server_from_urdf(self):
        print(rospy.get_param("/robot_description"))
        self.robot = URDF.from_parameter_server()
        self.robotics_server = await OpcuaRobotics.create("{}System".format(self.robot.name), self.DI_path, self.Robotics_path)
        await self.robotics_server.add_controller("ROS-IndustrialController")
        await self.robotics_server.add_robot(self.robot.name)
        await self.robotics_server.connect_controller_to_robot()
        for joint in self.robot.joints:
            if joint.type != "fixed":
                await self.robotics_server.add_axis(joint.name)
        for transmission in self.robot.transmissions:
            powertrain_name = transmission.name
            joint_name = transmission.joints[0].name
            motor_name = transmission.actuators[0].name
            drive_name = "{}_{}".format(motor_name, transmission.joints[0].hardwareInterfaces[0].replace('hardware_interface/',''))

            await self.robotics_server.add_powertrain(powertrain_name)
            await self.robotics_server.connect_powertrain_to_axis(powertrain_name, joint_name)
            await self.robotics_server.add_motor(powertrain_name, motor_name)
            await self.robotics_server.add_drive(powertrain_name, drive_name)
            await self.robotics_server.connect_motor_to_drive(motor_name, drive_name)

    async def run(self):
        await self.robotics_server_from_urdf()
        self.subscribe()
        await self.robotics_server.start()


    def subscribe(self):
        rospy.Subscriber("joint_states", JointState, self.subscriber_callback)
    
    def subscriber_callback(self, data):
        for i in range(0, len(data.name)):
            self.robotics_server.set_axis_position(data.name[i], data.position[i])

    def spin(self):
        rospy.spin()

