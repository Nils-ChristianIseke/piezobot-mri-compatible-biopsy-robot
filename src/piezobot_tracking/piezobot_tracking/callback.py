import sys
import array
import time

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, JointConstraint
from sensor_msgs.msg import JointState
import math
import random

from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool

import numpy as np

import rclpy
from rclpy.node import Node
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class IntroCTPlanningNode(Node):

    def __init__(self):
        self.point_entry = Point()
        self.point_target = Point()
        self.target_pose = Pose()
        self.current_states_sim = JointState()
        client_cb_group = ReentrantCallbackGroup()
        subscriber_cb_group = ReentrantCallbackGroup()
        super().__init__('introct_planning_node')
        self.cli = self.create_client(GetPositionIK, 'compute_ik',callback_group=client_cb_group)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.req = GetPositionIK.Request()
        
        self.subscription = self.create_subscription(Bool,'introct/action_calculate_ik',self.calculate_ik_callback,10,callback_group=subscriber_cb_group)
        self.subscription = self.create_subscription(Bool,'introct/action_calculate_random_ik',self.calculate_ik_random_callback,10,callback_group=subscriber_cb_group)
        self.subscription = self.create_subscription(Bool,'introct/action_calculate_fine_random_ik',self.calculate_ik_fine_callback,10,callback_group=subscriber_cb_group)
        
        self.subscription_JointStates = self.create_subscription(JointState, 'joint_states', self.read_joint_states,10)
        


        self.current_states = JointState()

        self.publisher_joint_goal = self.create_publisher(JointState, 'introct/manipulator/joint_goal', 10)
        self.publisher_target_pose = self.create_publisher(Pose, 'introct/pose_target', 10)

        self.subscription = self.create_subscription(Point, 'introct/navigation/point_entry', self.point_entry_callback,10)
        self.subscription = self.create_subscription(Point, 'introct/navigation/point_target', self.point_target_callback,10)
        
        self.subscription  # prevent unused variable warning
    
        self.get_logger().info('Planning node running')
        

    def send_request(self, req):
        self.req.ik_request = req
        result = self.cli.call(self.req)
        return result
    
    def point_entry_callback(self, msg):
        self.get_logger().info('Received entry point: '+str(msg))
        self.point_entry = msg
        #self.point_entry.x = -self.point_entry.x
        #self.point_entry.y = -self.point_entry.y


    def point_target_callback(self, msg):
        self.get_logger().info('Received target point: '+str(msg))
        self.point_target = msg
        #self.point_target.x = -self.point_target.x
        #self.point_target.y = -self.point_target.y

    def read_joint_states(self, msg):
        self.current_states = msg
    
    def calculate_ik_random_callback(self, msg):
        self.get_logger().info('Setting random target and entry points')
        self.point_entry.x = 0.0+random.randrange(-20,20)/1000
        self.point_entry.y =  0.2+random.randrange(0,100)/1000
        self.point_entry.z = -random.randrange(1200,1500)/1000    
        self.point_target.x = self.point_entry.x+random.randrange(-5,5)/1000
        self.point_target.y = self.point_entry.y-0.01
        self.point_target.z = self.point_entry.z+random.randrange(-5,5)/1000


    def calculate_ik_callback(self, msg):

        self.get_logger().info('Starting IK calculations')

        IK_calc_stop_criteria = 100 # after 20 IK calculations it will finish with no solution 
        IK_solution_flag = False  # will get True if solution was found
        i = 0
        while(i < IK_calc_stop_criteria and IK_solution_flag == False):
        
            time_start = time.time()          

            entry_point = self.point_entry
            target_point = self.point_target
            
            req = PositionIKRequest()
            req.group_name = 'g1_all'
            req.ik_link_name = 'needle'
            req.robot_state.joint_state.name = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J6_nh', 'J6_needle', 'J6_needle_lin', 'J7', 'J7_nh']
            req.robot_state.joint_state.position = [0.01]*11



            constraint2 = JointConstraint()
            constraint2.joint_name = 'J2'
            constraint2.position = 1.4
            constraint2.tolerance_above = 1.35
            constraint2.tolerance_below = 1.35
            constraint2.weight = 0.3
            req.constraints.joint_constraints.append(constraint2)

            constraint3 = JointConstraint()
            constraint3.joint_name = 'J3'
            constraint3.position = math.pi/2
            constraint3.tolerance_above = 1.5
            constraint3.tolerance_below = 1.5
            constraint3.weight = 0.5
            req.constraints.joint_constraints.append(constraint3)

            constraint4 = JointConstraint()
            constraint4.joint_name = 'J4'
            constraint4.position = 0.0
            constraint4.tolerance_above = 1.5
            constraint4.tolerance_below = 1.5
            constraint4.weight = 0.7
            req.constraints.joint_constraints.append(constraint4)

            constraint5 = JointConstraint()
            constraint5.joint_name = 'J5'
            constraint5.position = 0.034
            constraint5.tolerance_above = 0.02
            constraint5.tolerance_below = 0.02
            constraint5.weight = 0.9
            req.constraints.joint_constraints.append(constraint5)
           
            # entry_point = Point()
            # entry_point.x = 0.0
            # entry_point.y =  -0.1
            # entry_point.z = -1.5    

            # target_point = Point()
            # target_point.x = 0.0
            # target_point.y =  -0.05
            # target_point.z = -1.5 

            # entry_point = Point()
            # entry_point.x = 0.0+random.randrange(-20,20)/1000
            # entry_point.y =  -0.1-random.randrange(0,100)/1000
            # entry_point.z = -random.randrange(1200,1500)/1000    

            # target_point = Point()
            # target_point.x = entry_point.x
            # target_point.y = entry_point.y+0.01
            # target_point.z = entry_point.z 

            self.target_pose = calculate_target_pose(entry_point,target_point)

            req.pose_stamped.pose.position = self.target_pose.position
            req.pose_stamped.pose.orientation = self.target_pose.orientation

        
            response = self.send_request(req)
            self.get_logger().info(str(response))

            msg = JointState()
            joint_state = response.solution.joint_state
            if len(joint_state.position) == 0:
                self.get_logger().error('No IK solution found after iteration '+str(i+1))
            else:        
                joint_state = self.calculate_j7(joint_state,True) #calculate missing joint
                msg = joint_state

                self.publisher_joint_goal.publish(msg)

                result = list(response.solution.joint_state.name)
                self.get_logger().info('IK solution found after iteration '+str(i+1))

                self.get_logger().info("joint names: "+str(result))
                result = list(array.array("d", response.solution.joint_state.position))
                self.get_logger().info("Calculated joint positions: "+str(result))
                self.current_states_sim = joint_state
                IK_solution_flag = True
            i  = i+1

        time_end = time.time()
        self.get_logger().info('Elapsed time: '+str(time_end-time_start)+' s')

    
    def calculate_ik_fine_callback(self, msg):

        # To test:
        # ros2 topic pub -1 /introct/action_calculate_fine_random_ik std_msgs/msg/Bool "{data: true}"
        
        self.get_logger().info('Starting IK calculations for fine positioning')
        IK_fine_calc_stop_criteria = 100 # after 20 IK calculations it will finish with no solution 
        IK_fine_solution_flag = False  # will get True if solution was found
        i = 0
        while(i < IK_fine_calc_stop_criteria and IK_fine_solution_flag == False):
            time_start = time.time()
            
            # Read current state of joints  
            states = self.current_states.position
            #states = self.calculate_j7(self.current_states,False) #calculate missing joint
            #Problem: alles 0 in joint [0.3799999952316284, 1.7073314189910889, 1.194427251815796, -0.7341312766075134, 3.5000000934815034e-05, 0.018336666747927666, 0.0, 0.0, 0.0, 0.018336666747927666, 0.0]
            
            #states = self.current_states_sim.position
            #states[0] = states[0] +0.001
            #states[2] = states[2] +0.01
            
            req = PositionIKRequest()
            req.group_name = 'g2_fine'
            req.ik_link_name = 'needle_linear'
            req.robot_state.joint_state.name = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'J6_nh', 'J6_needle', 'J6_needle_lin', 'J7', 'J7_nh']
            req.robot_state.joint_state.position = states
            req.pose_stamped.pose.position = self.target_pose.position
            req.pose_stamped.pose.orientation = self.target_pose.orientation
            
            self.get_logger().info('Without correction' + str(states))
            # Calculate new Joints for the target pose

        

            response = self.send_request(req)
            if response.solution.joint_state.position.tolist()==[]:
                joint_state = self.current_states
                self.get_logger().error('Did not find IK solution for fine positioning during interation ' +str(i+1))
            else:
                
                self.get_logger().info('IK solution found after iteration '+str(i+1))

                self.get_logger().info('IK solution found: ' + str(response.solution.joint_state))
                joint_state = response.solution.joint_state

                time_end = time.time()
                self.get_logger().info('Elapsed time: '+str(time_end-time_start)+' s')

                joint_state = self.calculate_j7(joint_state,False) #calculate missing joint
                msg = joint_state

                self.publisher_joint_goal.publish(msg)
                IK_fine_solution_flag = True
            i = i+1
        
        
                


    def calculate_j7(self,js,improve_positioning):
        self.get_logger().info('Received joint state: '+str(list(js.position)))

        if len(js.position) == 0:
            self.get_logger().error('Joint states empty. Cannot calculate J7 state')
        else:

            j1 = js.position[0]
            j6 = js.position[5]
            j6_nh = js.position[6]

            distance_j6_j7 = 0.018


            if improve_positioning:
                stroke_j6 = 0.044 # maximum movement of j6 and j7 motors
                
                j6_target = stroke_j6/2 + math.tan(j6_nh)*distance_j6_j7/2
                j7_target = stroke_j6/2 - math.tan(j6_nh)*distance_j6_j7/2
                offset = j6-j6_target
                j1_target = j1-offset
        
                if 0 <= j6_target <= stroke_j6 and 0 <= j7_target <= stroke_j6:
                    js.position[0] = j1_target
                    js.position[5] = j6_target
                    js.position[9] = j7_target
                    self.get_logger().info('New joint state (improved): '+str(list(js.position)))

                else:
                    self.get_logger().info(['Unable to find a solution for j6 & j7 improvement.  Results: j6='+str(j6)+' , j7='+str(j7)])

            else:
                j7_target = j6 - math.tan(j6_nh)*distance_j6_j7/2
                js.position[9] = j7_target
                self.get_logger().info('New joint state: '+str(list(js.position)))

        return js        


def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix.

    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True

    """
    q = np.empty((4, ), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q


def calculate_target_pose(entry_point,target_point):
    target_pose = Pose()
 
    direction_vector = Point()
    direction_vector.x = target_point.x-entry_point.x
    direction_vector.y = target_point.y-entry_point.y
    direction_vector.z = target_point.z-entry_point.z

    a = np.array([entry_point.x, entry_point.y, entry_point.z])
    b = np.array([target_point.x, target_point.y, target_point.z])
    
    entry_point_vector = a
    target_vector = a-b
    c_vector = np.cross(entry_point_vector,target_vector)

    x_vector = target_vector/np.linalg.norm(target_vector)

    y_vector = c_vector/np.linalg.norm(c_vector)

    z_vector = np.cross(x_vector, y_vector)

    #Calculate Coordinate transformation

    R = np.array([x_vector, y_vector, z_vector])
    R = np.transpose(R)

    trans_matrix = np.eye(4)
    trans_matrix[0:3, 0:3] = R
    trans_matrix[0,3] = a[0]
    trans_matrix[1,3] = a[1]
    trans_matrix[2,3] = a[2]
    
    quaternion = quaternion_from_matrix(trans_matrix)
    

    target_pose.position = entry_point
    target_pose.orientation.x = quaternion[0]
    target_pose.orientation.y = quaternion[1]
    target_pose.orientation.z = quaternion[2]
    target_pose.orientation.w = quaternion[3]

    return target_pose
    

def main(args=None):
    rclpy.init(args=args)
    node = IntroCTPlanningNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.\n')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()