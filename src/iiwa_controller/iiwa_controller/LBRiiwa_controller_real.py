import array
import rclpy
import numpy as np
import pyfri as fri

from rclpy.node import Node
from std_srvs.srv import Trigger
from scipy.interpolate import CubicSpline, make_interp_spline
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from iiwa_interfaces.srv import ChangeTrajectory


class LBRControllerReal(fri.LBRClient):
    def __init__(self):
        super().__init__()

        self.__points = []
        self.__position = np.array([])
        self.__curent_pos = np.array([])
        
        self.__curent_point_index = 0 # Текущий индекс позиции

    @property
    def points(self):
        return self.__points
    
    @property
    def position(self):
        return self.__position
    
    @property
    def curent_point_index(self):
        return self.__curent_point_index
    
    @property
    def current_pos(self):
        return self.__curent_pos
    
    @points.setter
    def points(self, value):
        self.__points = value
    
    @position.setter
    def position(self, value):
        self.__position = value
    
    @curent_point_index.setter
    def curent_point_index(self, value):
        self.__curent_point_index = value

        
    def monitor(self):
        pass
    
    def onStateChange(self, old_state, new_state):
        print(f"State changed from {old_state} to {new_state}")
        self.robotState().getControlMode()

    def waitForCommand(self):
            
        self.__position = self.robotState().getCommandedJointPosition()
        self.robotCommand().setJointPosition(self.__position.astype(np.float32))
    
    def command(self):
        if self.__points and self.__curent_point_index < len(self.__points):
            point = self.__points[self.__curent_point_index]
            self.__position = np.array(point.positions)

            self.__curent_point_index += 1

        else:
            self.__points = []
            self.__curent_point_index = 0
            self.__position = self.robotState().getCommandedJointPosition()
            print(self.__position.astype(np.float32))
            
        self.__curent_pos = self.robotState().getMeasuredJointPosition()
        
        self.robotCommand().setJointPosition(self.__position.astype(np.float32))

    
    
class LBRControllerNode(Node):
    # 100Hz - 0.01
    def __init__(self, robot_name:str="LBRiiwa7R800"):
        super().__init__(f"{robot_name}Driver")

        # Параметры сети
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth=1)

        # Параметры для работы контроллера
        robot_ip = self.declare_parameter("robot_ip", "").get_parameter_value().string_value
        robot_port = self.declare_parameter("robot_port", "").get_parameter_value().string_value
        self.__recive_move_step = int(self.declare_parameter("recive_move_step", "50").get_parameter_value().string_value)




        self.create_subscription(JointTrajectory,
                                topic=f"{robot_name}/cmd_positions", # TODO: Поменять
                                callback=self.__cmd_positions_callback,
                                qos_profile=qos_profile)
        
        self.__current_positions_pub = self.create_publisher(msg_type=JointTrajectoryPoint,
                                                            topic=f"{robot_name}/current_positions/real",
                                                            qos_profile=qos_profile)
        
        self.create_timer(0.01, 
                          callback=self.__publish_current_position_callback)
        
        self.create_service(srv_type=Trigger,
                            srv_name=f"{robot_name}/stop_move",
                            callback=self.__stop_robot_callback)

        self.create_service(srv_type=ChangeTrajectory,
                            srv_name=f"{robot_name}/reverse_movement",
                            callback=self.__reverse_movement_callback)
        
        self.__controller = LBRControllerReal()
        
        self.__app = fri.ClientApplication(self.__controller)
        success = self.__app.connect(30200, "192.168.21.147")
        
        if not success:
            self.get_logger().error("Не удалось подключиться к контроллеру KUKA.")
            return 1

        self.get_logger().info("Подключение установлено")
        
    def __publish_current_position_callback(self):
        """Метод отвечающий за непосредсвенный шаг движения робота
           и публикацию его текущего положения
        """
        self.__app.step()
        
        msg = JointTrajectoryPoint(positions=self.__controller.current_pos.tolist())
        # self.get_logger().warn(str(self.__controller.points))

        self.__current_positions_pub.publish(msg)
    
    
    def __reverse_movement_callback(self,
                                      request: ChangeTrajectory.Request,
                                      response: ChangeTrajectory.Response) -> ChangeTrajectory.Response:
        """Сервис позволяющий двигать робота по обратной траектории
        Необходим для перерасчёта ОЗК 
        """
        
        self.get_logger().info("Получен запрос на обратное перемещение робота")
        
        # TODO: Интерполяция нужна для управления временем робота
        try:
            if not self.__controller.points:
                raise ValueError("Отсутсвуют точки для перемещения")

            current_index = self.__controller.curent_point_index
            new_index = max(0, current_index - self.__recive_move_step)
            points = self.__controller.points[current_index:new_index:-1]
            
            
            interpol_points = self.__interpolate_points(self.__controller.current_pos.tolist(),
                                                        points[-1].positions,
                                                        duration=request.time)
            
            trajectory_msg = JointTrajectory()
            
            points_msg = [JointTrajectoryPoint(positions=pos.tolist()) for pos in interpol_points]
            trajectory_msg.points = points_msg
            
            self.__controller.curent_point_index = 0
            self.__controller.points = trajectory_msg.points
            
            response.position = array.array('f', interpol_points[-1])

            response.status = True
            response.message = "Траектория успешно изменена"

        except Exception as e:
            self.get_logger().error(str(e))
            response.message = str(e)
            response.status = False

        return response
    
    
    # TODO: Попробовать не интерполировать траектории, а просто продолжить выполнение на 10 шагов
    def __stop_robot_callback(self, 
                              request: Trigger.Request, 
                              response: Trigger.Response) -> Trigger.Response:
        """Остановка движения робота"""
        
        self.get_logger().info("Получен запрос на остановку движения робота")
        
        try:
            trajectory_msg = JointTrajectory()
            trajectory_msg.points = [JointTrajectoryPoint(positions=self.__controller.current_pos.tolist())]
            
            self.__controller.curent_point_index = 0
            self.__controller.points = trajectory_msg.points
            
            
            response.success = True
            response.message = "Движение робота остановлено"
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(response.message)

        return response

    def __cmd_positions_callback(self, msg: JointTrajectory):
        """Установка позиций перемещения робота

        Args:
            msg (JointTrajectory): Траектория перемещения
        """
        if msg:
            self.get_logger().info("Получено сообщения для смены траектории робота")
            self.__reset_positions(points=msg.points, 
                                    point_index=0)
    
    def __reset_positions(self,
                          points:list=[],
                          point_index:int=0) -> None:
        """Сброс параметров контроллера

        Args:
            points (list, optional): Точки робота. По умолчанию [].
            joint (list, optional): Названия соеденений. По умолчанию [].
            point_index (int, optional): Индекс точки, с которой необходимо начать движение. По умолчанию 0.
        """

        self.__controller.points = points
        self.__controller.curent_point_index = point_index
        
    def __interpolate_points(self, start_point, end_point, duration) -> np.ndarray:
        """Интерполяция точек перемещения

        Args:
            start_point (_type_): Стартовая точка
            waypoints (_type_): Промежуточные точки
            end_point (_type_): Конечная точка
            duration (_type_): Время расчёта

        Returns:
            np.ndarray: Интерполируемые точки
        """
        
        all_positions = np.vstack((start_point, end_point))
        
        # Время для каждой точки (в мс)
        time_points = np.linspace(0, duration * 1000, len(all_positions))

        # Время для интерполяции
        interp_time = np.arange(0, duration * 1000, 10)

        # Интерполируем углы с использованием сплайнов Безье
        interpolated_angles = np.array([make_interp_spline(time_points, all_positions[:, i], bc_type='natural')(interp_time) for i in range(all_positions.shape[1])]).T

        return interpolated_angles

    

def main(args=None):
    rclpy.init(args=args)
    
    node = LBRControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
    
if __name__== "__main__":
    main()