from ros_depth_pid.pid import PID
import rclpy
from rclpy.node import Node
from rov_msg.msg import PidMsg
from std_msgs.msg import Float32

class DEPTH_PID_CONTROLLER(Node):
    def __init__(self):
        super().__init__('depth_pid_node')
        self.pid_out = PID() 
        self.pid_in = PID()
        self.pid_in.MAXOutput = 1000.0
        self.pid_in.MAXIntegral = 10.0
        self.pid_out.MAXOutput = 20.0
        self.pid_out.MAXIntegral = 3.0
        self.F = 0   #电机推力
        self.a = 0   #小小rov加速度
        self.m = 10  #机器人质量
        self.kv = 1  #阻力系数
        self.f = 1   #静态阻力
        self.d_target = 0  #目标d
        self.v_target = 0   #目标v
        self.d_current = 0  #当前深度
        self.v_current = 0  #当前速度
        self.PID_DT = 0.01  #pid周期，10ms一次
        self.delta_d = 0.0  #目标delta_d
        self.v_max = 5.0
        self.v_min = -5.0
        self.timer = self.create_timer(self.PID_DT, self.depth_pid_callback) #10ms一次pid
        self.create_subscription(PidMsg, 'depth_pid', self.depth_param_callback, 10)
        self.depth_pub = self.create_publisher(Float32, 'current_depth', 10)
    
    #改参数
    def depth_param_callback(self, msg):
        self.pid_in.Set_Param(msg, 1)
        self.pid_out.Set_Param(msg, 2)
        self.d_target = self.pid_out.target_value
        self.pid_out.integral_sum = 0
        
    def depth_pid_callback(self):
        #外环位置环（增量式）输出delta_d
        self.delta_d = self.pid_out.Incremental_PID(self.d_target, self.d_current)
        self.v_target += self.delta_d / self.PID_DT
        if self.v_target > self.v_max:
            self.v_target = self.v_max
        elif self.v_target < self.v_min:
            self.v_target = self.v_min
        #内环速度环（位置式）输出F
        self.F = self.pid_in.Position_PID(self.v_target, self.v_current)
        self.a = (self.F - self.f - self.kv * self.v_current) / self.m
        self.v_current += self.a * self.PID_DT
        self.d_current += self.v_current * self.PID_DT
        msg = Float32()
        msg.data = self.d_current
        self.depth_pub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = DEPTH_PID_CONTROLLER()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
