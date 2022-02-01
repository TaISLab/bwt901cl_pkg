import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature, MagneticField
from geometry_msgs.msg import Vector3
import subprocess

from .src.bwt901cl import BWT901CL

class Imu901cl(Node):
    def __init__(self, time_interval=1.0):
        super().__init__('imu_bwt901cl')
        self.pub_imu = self.create_publisher(Imu, '/sensor/bwt901cl/Imu', 10)
        self.pub_mag = self.create_publisher(MagneticField, '/sensor/bwt901cl/MagneticField', 10)
        self.pub_tmp = self.create_publisher(Temperature, '/sensor/bwt901cl/Temperature', 10)
        self.pub_ang = self.create_publisher(Vector3, '/sensor/bwt901cl/Angle', 10)
        self.tmr = self.create_timer(time_interval, self.timer_callback)
        #subprocess.call("sudo chmod 777 /dev/ttyUSB0", shell=True)
        self.imu_sensor =  BWT901CL("/dev/witmotion")

        self.msg_imu = Imu()
        self.msg_mag = MagneticField()
        self.msg_tmp = Temperature()
        self.msg_ang = Vector3()        
        self.msg_imu.header.frame_id =  self.msg_mag.header.frame_id =  self.msg_tmp.header.frame_id =  'witmotion' 

        # TODO this is a crappy hack... again
        small_value = 0.0001
        dummy_cov = [small_value, 0,           0, 
                     0,           small_value, 0, 
                     0,           0,           small_value]        
        self.msg_imu.orientation_covariance = self.msg_imu.angular_velocity_covariance = self.msg_imu.linear_acceleration_covariance = dummy_cov

    def timer_callback(self):
        self.msg_imu.header.stamp =  self.msg_mag.header.stamp =  self.msg_tmp.header.stamp =  self.get_clock().now().to_msg()
        
        angle, angular_velocity, accel, temp, magnetic, quaternion, time = self.imu_sensor.getData()

        self.msg_tmp.temperature = temp
        self.pub_tmp.publish(self.msg_tmp)

        self.msg_mag.magnetic_field.x = float(magnetic[0])
        self.msg_mag.magnetic_field.y = float(magnetic[1])
        self.msg_mag.magnetic_field.z = float(magnetic[2])
        self.pub_mag.publish(self.msg_mag)

        self.msg_imu.orientation.x = quaternion[0]
        self.msg_imu.orientation.y = quaternion[1]
        self.msg_imu.orientation.z = quaternion[2]
        self.msg_imu.orientation.w = quaternion[3]
        self.msg_imu.angular_velocity.x = angular_velocity[0]
        self.msg_imu.angular_velocity.y = angular_velocity[1]
        self.msg_imu.angular_velocity.z = angular_velocity[2]
        self.msg_imu.linear_acceleration.x = accel[0]
        self.msg_imu.linear_acceleration.y = accel[1]
        self.msg_imu.linear_acceleration.z = accel[2]
        self.pub_imu.publish(self.msg_imu)

        self.msg_ang.x = angle[0]
        self.msg_ang.y = angle[1]
        self.msg_ang.z = angle[2]
        self.pub_ang.publish(self.msg_ang)

        #print("Time:", time)
        #print("th:", angle)
        #print("d_th: ", angular_velocity)
        #print("d_x: ", accel)
        #print("mag: ", magnetic)
        #print("tmp: ", temp)
        #print(quaternion)

def main(args=None):    
    print('Hi from bwt901cl_pkg.')

    rclpy.init(args=args)
    node_imu_bwt901cl = Imu901cl(time_interval=0.1)
    rclpy.spin(node_imu_bwt901cl)

    node_imu_bwt901cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
