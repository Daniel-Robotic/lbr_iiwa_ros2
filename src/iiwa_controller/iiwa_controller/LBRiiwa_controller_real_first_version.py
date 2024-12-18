import argparse
import array
import rclpy
import numpy as np
import pyfri as fri

from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from iiwa_interfaces.srv import ChangeTrajectory


class LBRControllerReal(fri.LBRClient):
    def __init__(self, robot_name:str="LBRiiwa7R800"):
        super().__init__()

        rclpy.init(args=None)
        self.__node = rclpy.create_node(f"real_robot_node")
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                depth=1)
        
        self.__node.create_subscription(JointTrajectory,
                                        f"{robot_name}/cmd_positions", # TODO: Поменять
                                        self.__cmd_positions_callback,
                                        qos_profile)
        
        self.__current_positions_pub = self.__node.create_publisher(msg_type=JointTrajectoryPoint,
                                                                    topic=f"{robot_name}/current_positions/real",
                                                                    qos_profile=qos_profile)
        self.__node.create_service(srv_type=Trigger,
                                    srv_name=f"{robot_name}/stop_move",
                                    callback=self.__change_state_callback)

        self.__node.create_service(srv_type=ChangeTrajectory,
                                    srv_name=f"{robot_name}/change_trajectory",
                                    callback=self.__change_trajectory_callback)
        
        self.__base_joint_names = ["lbr_A1", "lbr_A2", "lbr_A3",
                                   "lbr_A4", "lbr_A5", "lbr_A6",
                                   "lbr_A7"]
        
        self.__points = []
        self.__joint_names = []
        self.__position = np.array([])

        self.__curent_point_index = 0 # Текущий индекс позиции
        self.__stop_movement = False # Флаг остановки выполнения перемещения

    def __change_trajectory_callback(self,
                                      request: ChangeTrajectory.Request,
                                      response: ChangeTrajectory.Response) -> ChangeTrajectory.Response:
        try:
            if not self.__points:
                raise ValueError("Отсутсвуют точки для перемещения")

            current_index = self.__curent_point_index
            new_index = max(0, current_index - request.step)
            points = self.__points[current_index:new_index:-1]

            self.__curent_point_index = 0
            self.__points = points
            
            response.position = array.array('f', points[-1].positions)
            response.joint_names = self.__joint_names

            response.status = True
            response.message = "Trajectory changed successfully"

        except Exception as e:
            response.message = str(e)
            response.status = False

        return response
    
    def __change_state_callback(self, 
                                request: Trigger.Request, 
                                response: Trigger.Response) -> Trigger.Response:
        """Остановка движения робота движения робота"""

        try:
            self.__stop_movement = True
            self.__reset_positions()
            response.success = True
            response.message = "The robot's movement is stopped"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    def __cmd_positions_callback(self, msg: JointTrajectory):
        """Установка позиций перемещения робота

        Args:
            msg (JointTrajectory): Траектория перемещения
        """
        if msg:
            self.__stop_movement = False
            self.__reset_positions(points=msg.points, 
                                    joint=msg.joint_names, 
                                    point_index=0)
    
    def __reset_positions(self,
                          points:list=[],
                          joint:list=[],
                          point_index:int=0) -> None:

        self.__points = points
        self.__joint_names = joint
        self.__curent_point_index  = point_index

    def monitor(self):
        pass
    
    def onStateChange(self, old_state, new_state):
        self.__node.get_logger().info(f"State changed from {old_state} to {new_state}")
        self.robotState().getControlMode()
        
    def waitForCommand(self):
        if not self.__position.tolist():
            self.__position = self.robotState().getMeasuredJointPosition()

        msg = JointTrajectoryPoint(positions=self.__position.tolist())
        self.__current_positions_pub.publish(msg)
        
        self.robotCommand().setJointPosition(self.__position.astype(np.float32))
        
    def command(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        if self.__stop_movement:
            self.__position = self.robotState().getCommandedJointPosition()
            # getCommandedJointPosition

        if self.__points and self.__curent_point_index < len(self.__points):
            point = self.__points[self.__curent_point_index]
            self.__position = np.array(point.positions)

            self.__curent_point_index += 1

        else:
            self.__position = self.robotState().getCommandedJointPosition()

        current_pos = self.robotState().getMeasuredJointPosition()
        msg = JointTrajectoryPoint(positions=current_pos.tolist())
        self.__current_positions_pub.publish(msg)
        
        self.robotCommand().setJointPosition(self.__position.astype(np.float32))

        
def main(args=None):

    

    print("Running FRI Version:", fri.FRI_CLIENT_VERSION)
    
    client = LBRControllerReal()
    app = fri.ClientApplication(client)
    
    success = app.connect(30200, "192.168.21.147")
    if not success:
        print("Не удалось подключиться к контроллеру KUKA.")
        return 1

    print("Подключение к контроллеру KUKA установлено.")
    
    try:
        while success:
            success = app.step()
            
            # Проверка состояния сессии, если IDLE — завершить
            if client.robotState().getSessionState() == fri.ESessionState.IDLE:
                break

    except KeyboardInterrupt:
        pass
    
    finally:
        rclpy.shutdown()

        app.disconnect()
        print("Соединение завершено")

    return 0
    
if __name__ == "__main__":
    main()