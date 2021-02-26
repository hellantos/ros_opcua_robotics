import asyncio
import logging
import sys
from datetime import datetime
sys.path.insert(0, "..")

from asyncua import ua, uamethod, Server
from asyncua.common.instantiate_util import instantiate

class OpcuaDeviceObject:
    
    def get_device(self):
        return self.device

    async def get_property(self, name):
        properties = await self.device.get_properties()
        for property in properties:
            property_name = await property.read_browse_name()
            if property_name.Name == name:
                return property

    async def set_manufacturer(self, value):
        manufacturer_node = await self.get_property("Manufacturer")
        await manufacturer_node.set_value(ua.LocalizedText(text=value), ua.VariantType.LocalizedText)

    async def set_component_name(self, value):
        node = await self.get_property("ComponentName")
        await node.set_value(ua.LocalizedText(text=value), ua.VariantType.LocalizedText)
    
    async def set_serial_number(self, value):
        node = await self.get_property("SerialNumber")
        await node.set_value(value, ua.VariantType.String)
    
    async def set_model(self, value):
        node = await self.get_property("Model")
        await node.set_value(ua.LocalizedText(text=value), ua.VariantType.LocalizedText)

    async def set_product_code(self, value):
        node = await self.get_property("ProductCode")
        await node.set_value(value, ua.VariantType.String)

    async def add_parameter(self, name, type, value):
        parameter_set = await self.device.get_child("{}:ParameterSet".format(self.device_idx))
        await parameter_set.add_variable(self.idx, name, value, varianttype=type)
    
    async def set_parameter(self, name, type, value):
        parameter_set = await self.device.get_child("{}:ParameterSet".format(self.device_idx))
        parameters = await parameter_set.get_variables()
        for child in parameters:
            node_name = await child.read_browse_name()
            if node_name.Name == name:
                await child.set_value(value, type)    


class MotionDeviceSystem(OpcuaDeviceObject):
    @classmethod
    async def create(cls, parent, server, idx, name):
        self = MotionDeviceSystem()
        self.robot_idx = await server.get_namespace_index("http://opcfoundation.org/UA/Robotics/")
        self.device_idx = await server.get_namespace_index("http://opcfoundation.org/UA/DI/")
        self.idx = idx
        self.type_id = "1002"
        self.type_node = server.get_node("ns={};i={}".format(str(self.robot_idx), self.type_id))
        nodes = await instantiate(parent, self.type_node, bname="{}:{}".format(str(idx), name))
        self.device = nodes[0]
        self.server = server
        return self
    
    async def add_controller(self, name):
        self.controller_folder = await self.device.get_child("{}:Controllers".format(self.robot_idx))
        try:
            placeholder = await self.controller_folder.get_child("{}:<ControllerIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        controller = await Controller.create(self.controller_folder, self.server, self.idx, name)
        return controller

    async def add_motion_device(self, name):
        self.motion_device_folder = await self.device.get_child("{}:MotionDevices".format(self.robot_idx))
        try:
            placeholder = await self.motion_device_folder.get_child("{}:<MotionDeviceIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")

        controller = await MotionDevice.create(self.motion_device_folder, self.server, self.idx, name)
        return controller

class Controller(OpcuaDeviceObject):
    @classmethod
    async def create(cls, parent, server, idx, name):
        self = Controller()
        self.robot_idx = await server.get_namespace_index("http://opcfoundation.org/UA/Robotics/")
        self.device_idx = await server.get_namespace_index("http://opcfoundation.org/UA/DI/")
        self.idx = idx
        self.type_id = "1003"
        self.type_node = server.get_node("ns={};i={}".format(str(self.robot_idx), self.type_id))
        nodes = await instantiate(parent, self.type_node, bname="{}:{}".format(str(idx), name))
        self.device = nodes[0]
        
        self.server = server
        return self


    async def controls(self, node):
        try:
            placeholder = await self.device.get_child("{}:<MotionDeviceIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        await self.device.add_reference(node, "ns={};i=4002".format(self.robot_idx))


    
class MotionDevice(OpcuaDeviceObject):
    @classmethod
    async def create(cls, parent, server, idx, name):
        self = MotionDevice()
        self.robot_idx = await server.get_namespace_index("http://opcfoundation.org/UA/Robotics/")
        self.device_idx = await server.get_namespace_index("http://opcfoundation.org/UA/DI/")
        self.idx = idx
        self.type_id = "1004"
        self.type_node = server.get_node("ns={};i={}".format(str(self.robot_idx), self.type_id))
        nodes = await instantiate(parent, self.type_node, bname="{}:{}".format(str(idx), name))
        self.device = nodes[0]
        
        self.server = server
        return self

    async def set_motion_device_category(self, value):
        node = await self.get_property("MotionDeviceCategory")
        await node.set_value(value, ua.VariantType.UInt32)

    async def add_axis(self, name):
        self.axes_folder = await self.device.get_child("{}:Axes".format(self.robot_idx))
        try:
            placeholder = await self.axes_folder.get_child("{}:<AxisIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")

        axis = await Axis.create(self.axes_folder, self.server, self.idx, name)
        return axis
    
    async def add_powertrain(self, name):
        self.powertrains_folder = await self.device.get_child("{}:PowerTrains".format(self.robot_idx))
        try:
            placeholder = await self.powertrains_folder.get_child("{}:<PowerTrainIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        powertrain = await PowerTrain.create(self.powertrains_folder, self.server, self.idx, name)
        return powertrain
    
class Axis(OpcuaDeviceObject):
    @classmethod
    async def create(cls, parent, server, idx, name):
        self = Axis()
        self.robot_idx = await server.get_namespace_index("http://opcfoundation.org/UA/Robotics/")
        self.device_idx = await server.get_namespace_index("http://opcfoundation.org/UA/DI/")
        self.idx = idx
        self.name = name
        self.position = 0.0
        self.speed = 0.0
        self.acceleration = 0.0
        self.type_id = "16601"
        self.type_node = server.get_node("ns={};i={}".format(str(self.robot_idx), self.type_id))
        nodes = await instantiate(parent, self.type_node, bname="{}:{}".format(str(idx), name))
        self.device = nodes[0]
        self.server = server
        return self

    async def update(self):
        await self.set_parameter("ActualPosition", ua.VariantType.Double, self.position)

    async def requires(self, node):
        try:
            placeholder = await self.device.get_child("{}:<PowerTrainIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        await self.device.add_reference(node, "ns={};i=18179".format(self.robot_idx))

class PowerTrain(OpcuaDeviceObject):
    @classmethod
    async def create(cls, parent, server, idx, name):
        self = PowerTrain()
        self.robot_idx = await server.get_namespace_index("http://opcfoundation.org/UA/Robotics/")
        self.device_idx = await server.get_namespace_index("http://opcfoundation.org/UA/DI/")
        self.idx = idx
        self.name = name
        self.type_id = "16794"
        self.type_node = server.get_node("ns={};i={}".format(str(self.robot_idx), self.type_id))
        nodes = await instantiate(parent, self.type_node, bname="{}:{}".format(str(idx), name))
        self.device = nodes[0]
        self.server = server
        await self.clean()
        return self

    async def clean(self):
        try:
            placeholder = await self.device.get_child("{}:<PowerTrainIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        try:
            placeholder = await self.device.get_child("{}:<GearIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")

    async def moves(self, node):
        try:
            placeholder = await self.device.get_child("{}:<AxisIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        await self.device.add_reference(node, "ns={};i=18178".format(self.robot_idx))
    
    async def add_motor(self, name):
        try:
            placeholder = await self.device.get_child("{}:<MotorIdentifier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        motor = await Motor.create(self.device, self.server, self.idx, name)
        return motor
    
    async def add_drive(self, name):
        drive = await Drive.create(self.device, self.server, self.idx, name)
        return drive


class Motor(OpcuaDeviceObject):
    @classmethod
    async def create(cls, parent, server, idx, name):
        self = Motor()
        self.robot_idx = await server.get_namespace_index("http://opcfoundation.org/UA/Robotics/")
        self.device_idx = await server.get_namespace_index("http://opcfoundation.org/UA/DI/")
        self.idx = idx
        self.name = name
        self.type_id = "1019"
        self.type_node = server.get_node("ns={};i={}".format(str(self.robot_idx), self.type_id))
        nodes = await instantiate(parent, self.type_node, bname="{}:{}".format(str(idx), name))
        self.device = nodes[0]
        self.server = server
        return self

    async def is_driven_by(self, node):
        try:
            placeholder = await self.device.get_child("{}:<DriveIdentifiier>".format(self.robot_idx))
            await placeholder.delete()
        except ua.UaStatusCodeError:
            print("Caught Error when deleting placeholder")
        await self.device.add_reference(node, "ns={};i=18180".format(self.robot_idx))

class Drive(OpcuaDeviceObject):
    @classmethod
    async def create(cls, parent, server: Server, idx, name):
        self = Motor()
        self.robot_idx = await server.get_namespace_index("http://opcfoundation.org/UA/Robotics/")
        self.device_idx = await server.get_namespace_index("http://opcfoundation.org/UA/DI/")
        self.idx = idx
        self.name = name
        self.type_node = server.nodes.base_object_type
        nodes = await instantiate(parent, self.type_node, bname="{}:{}".format(str(idx), name))
        self.device = nodes[0]
        self.server = server
        return self


class OpcuaRobotics(object):
    @classmethod
    async def create(cls, name, di_path, robotics_path,
    server_url="opc.tcp://0.0.0.0:4840/",
    uri="http://opc-ua.ros-industrial.org", 
    manufacturer="ROS-Industrial", 
    model="ROS-Industrial", 
    product_code="ROS-Industrial", 
    serial_number="123456789"):
        self = OpcuaRobotics()

        self.name = name
        self.uri = uri
        self.manufacturer = manufacturer
        self.model = model
        self.product_code = product_code
        self.serial_number = serial_number
        self.axes = {}
        self.powertrains = {}
        self.positions = {}
        self.motors = {}
        self.drives = {}


        self.server = Server()
        await self.server.init()
        self.server.set_endpoint("{}{}/server/".format(server_url, name))
        self.server.set_server_name(name)

        await self.server.import_xml(di_path)
        await self.server.import_xml(robotics_path)
        self.idx = await self.server.register_namespace(self.uri)
        self.DeviceSet = await self.server.nodes.objects.get_child("2:DeviceSet")
        self.system = await MotionDeviceSystem.create(
            self.DeviceSet, 
            self.server, 
            self.idx, 
            self.name)
        await self.system.set_manufacturer(self.manufacturer)
        await self.system.set_model(self.model)
        await self.system.set_product_code(self.product_code)
        await self.system.set_serial_number(self.serial_number)
        return self

    async def add_controller(self, name, manufacturer="ROS-Industrial", model="ROS-Industrial", product_code="ROS-Industrial", serial_number="123456789"):
        self.controller = await self.system.add_controller(name)
        await self.controller.set_manufacturer(manufacturer)
        await self.controller.set_model(model)
        await self.controller.set_product_code(product_code)
        await self.controller.set_serial_number(serial_number)

    async def add_robot(self, name, manufacturer="ROS-Industrial", model="ROS-Industrial", product_code="ROS-Industrial", serial_number="123456789"):
        self.robot = await self.system.add_motion_device(name)
        await self.robot.set_manufacturer(manufacturer)
        await self.robot.set_model(model)
        await self.robot.set_product_code(product_code)
        await self.robot.set_serial_number(serial_number)
        await self.robot.set_motion_device_category("1")

    async def connect_controller_to_robot(self):
        await self.controller.controls(self.robot.device)

    async def add_axis(self, name):
        self.axes[name] = await self.robot.add_axis(name)
    
    async def add_powertrain(self, name):
        self.powertrains[name] = await self.robot.add_powertrain(name)

    async def connect_powertrain_to_axis(self, powertrain_name, axis_name):
        powertrain = self.powertrains[powertrain_name]
        axis = self.axes[axis_name]
        await axis.requires(powertrain.device)
        await powertrain.moves(axis.device)
    
    async def add_motor(self, powertrain_name, name):
        powertrain = self.powertrains[powertrain_name]
        self.motors[name] = await powertrain.add_motor(name)

    async def add_drive(self, powertrain_name, name):
        powertrain = self.powertrains[powertrain_name]
        self.drives[name] = await powertrain.add_drive(name)

    async def connect_motor_to_drive(self, motor_name, drive_name):
        motor = self.motors[motor_name]
        drive = self.drives[drive_name]
        await motor.is_driven_by(drive.device)



    def set_axis_position(self, name, position):
        self.axes[name].position = position


    async def start(self):
        await self.server.start()
        while True:
            await asyncio.sleep(0.05)
            for axis in self.axes:
                await self.axes[axis].update()



async def main():
    logging.basicConfig(level=logging.INFO)
    robotics_server = await OpcuaRobotics.create("ROS-IndustrialRobotSystem")
    await robotics_server.add_controller("ROS-IndustrialController")
    await robotics_server.add_robot("UR5e")
    await robotics_server.add_axis("shoulder_link")
    await robotics_server.add_powertrain("shoulder_powertrain", "shoulder_link")
    await robotics_server.start()
    '''
    server = Server()
    await server.init()
    # import some nodes from xml
    await server.import_xml("/home/christoph/Documents/opc-ua/opcua-asyncio/schemas/UA-Nodeset-master/DI/Opc.Ua.Di.NodeSet2.xml")
    await server.import_xml("/home/christoph/Documents/opc-ua/opcua-asyncio/schemas/UA-Nodeset-master/Robotics/Opc.Ua.Robotics.NodeSet2.xml")

    uri = "http://opc-ua.ros-industrial.org"
    idx = await server.register_namespace(uri)
    
    device = await MotionDeviceSystem.create(server.nodes.objects, server, idx, "ROSIndustrialRobotSystem")
    await device.set_manufacturer("ROS-Indsutrial")
    await device.set_model("Test_Version")
    await device.set_product_code("System")
    await device.set_serial_number("123123123")

    controller = await device.add_controller("ROSIndustrialController")
    await controller.set_manufacturer("ROS-Industrial")
    await controller.set_model("Test_Version")
    await controller.set_product_code("Controller")
    await controller.set_serial_number("123123123")

    await controller.set_parameter("StartUpTime", ua.VariantType.DateTime, ua.datetime.now())

    robot = await device.add_motion_device("UniversalRobot")
    await robot.set_manufacturer("UniversalRobot")
    await robot.set_model("UR5e")
    await robot.set_product_code("???")
    await robot.set_serial_number("123123123")
    await robot.set_motion_device_category("1")

    axis_1 = await robot.add_axis("base_0")
    await axis_1.set_parameter("ActualPosition", ua.VariantType.Double, 1.00)
    #controllers = await robot.get_child("3:Controllers")
    #controller_type = server.get_node("ns={}; i=1003".format(str(idx-1)))
    #nodes = await instantiate(controllers, controller_type, bname="4:ROSIndustrialController")
    #controller = nodes[0]
    #await assetId.set_value("123", ua.VariantType.String)
    print(123)

    i=0
    # starting!
    async with server:
        while True:
            await asyncio.sleep(1)
            await controller.set_serial_number("{}".format(str(i)))
            await axis_1.set_parameter("ActualPosition", ua.VariantType.Double, i)
            i=i+1
    '''


if __name__ == "__main__":
    asyncio.run(main())
