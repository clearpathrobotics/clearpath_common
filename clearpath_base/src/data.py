import roslib; roslib.load_manifest('clearpath_base')
import rospy

from clearpath_base.msg import *

# This structure maps ROS data topics to message classes
msgs = {
    'system_status': SystemStatus,
    'power_status': PowerStatus,
    'processor_status': ProcessorStatus,
    'safety_status': SafetyStatus,
    'differential_speed': DifferentialSpeed,
    'differential_control': DifferentialControl,
    'differential_output': DifferentialOutput,
    'velocity': VelocitySetpt,
    'platform_orientation': Orientation,
    'platform_rotation': RotateRate,
    'platform_magnetometer': Magnetometer,
    'encoders': Encoders,
    'raw_encoders': RawEncoders,
    'distance': Distance
}

# Helper function for copying from payload to message.
def copy_attributes(source, dest, attrs):
    for attr in attrs:
        setattr(dest, attr, getattr(source, attr))

def pkg_robot_info(platform_name, platform_info, firmware_info):
    msg = ClearpathRobot();
    msg.name = platform_name.name
    msg.model = platform_info.model
    msg.platform_revision = platform_info.revision
    msg.serial = platform_info.serial
    msg.horizon_version = firmware_info.version
    msg.firmware_version = firmware_info.firmware
    msg.write_date = str(firmware_info.written)
    return msg

def pkg_system_status(payload):
    msg = SystemStatus()
    copy_attributes(payload, msg, ['uptime', 'voltages', 
                                   'currents', 'temperatures']) 
    return msg

def pkg_power_status(payload):
    msg = PowerStatus()
    charges = payload.charges
    capacities = payload.capacities
    descriptions = payload.descriptions
    for i in range(len(charges)):
        msg_s = PowerSource(charge=charges[i], 
                            capacity=capacities[i])
        msg_s.present, msg_s.in_use, msg_s.description = descriptions[i]
        msg.sources.append(msg_s)
    return msg

def pkg_processor_status(payload):
    msg = ProcessorStatus()
    msg.errors = payload.errors
    return msg

def pkg_differential_speed(payload):
    msg = DifferentialSpeed()
    copy_attributes(payload, msg, ['left_speed', 'right_speed',
                                   'left_accel', 'right_accel'])
    return msg

def pkg_differential_control(payload):
    msg = DifferentialControl()
    copy_attributes(payload, msg, ['left_p', 'left_i', 'left_d',
                                   'left_ffwd', 'left_stic', 'left_sat',
                                   'right_p', 'right_i', 'right_d',
                                   'right_ffwd', 'right_stic', 'right_sat'])
    return msg

def pkg_differential_output(payload):
    msg = DifferentialOutput()
    copy_attributes(payload, msg, ['left', 'right'])
    return msg

def pkg_ackermann_output(payload):
    msg = AckermannOutput()
    copy_attributes(payload, msg, ['steering', 'throttle', 'brake'])
    return msg

def pkg_velocity(payload):
    msg = VelocitySetpt()
    copy_attributes(payload, msg, ['trans', 'rot', 'accel'])
    return msg

def pkg_platform_orientation(payload):
    msg = Orientation()
    copy_attributes(payload, msg, ['roll', 'pitch', 'yaw'])
    return msg

def pkg_platform_rotation(payload):
    msg = RotateRate()
    copy_attributes(payload, msg, ['roll', 'pitch', 'yaw'])
    return msg

def pkg_platform_magnetometer(payload):
    msg = Magnetometer()
    copy_attributes(payload, msg, ['x', 'y', 'z'])
    return msg

def pkg_encoders(payload):
    msg = Encoders()
    travels = payload.travel
    speeds = payload.speed
    for i in range(len(travels)):
        msg.encoders.append(Encoder(travel=travels[i],
                                    speed=speeds[i]))
    return msg

def pkg_raw_encoders(payload):
    msg = RawEncoders()
    msg.ticks = payload.ticks
    return msg

def pkg_safety_status(payload):
    msg = SafetyStatus()
    msg.flags = payload.flags
    msg.estop = payload.flags & 0x8;
    return msg

def pkg_distance(payload):
    msg = Distance()
    msg.distances = payload.distances
    return msg
