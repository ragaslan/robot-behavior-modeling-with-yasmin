import time
import os
import subprocess
import rclpy
from rclpy.node import Node
from yasmin import State, Blackboard, StateMachine
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub

class InitialState(State):
    def __init__(self) -> None:
        super().__init__(["start_gazebo","start_slam_operation","run_teleoperation","save_map","create_station","end"])

    def execute(self,blackboard: Blackboard) -> str:
        # burada ana durumda komutlar alarak diğer durumlara geçiş yap
        print("initial state is active")
        
        flag = True
        while(flag):
            print("\nSelect Process\n1) Load Gazebo World\n2) Start Slam Operation\n3) Run Teleoperation Node\n4)Save Map\n5)Create Station Point\nQ)Exit")
            choice = input("Your Choice: ")
            
            #print("gazeboIsStarted" in blackboard)
            
            if choice == "1":
                if "gazeboIsStarted" in blackboard and blackboard["gazeboIsStarted"]:
                    print("\nGazebo World is already started! ")
                else:
                    blackboard["gazeboIsStarted"] = True
                    return "start_gazebo"
            if choice == "2":
                if "slamOperationIsStarted" in blackboard and blackboard["slamOperationIsStarted"]:
                    print("\nSLAM operation is already started!")
                else:
                    blackboard["slamOperationIsStarted"] = True
                    return "start_slam_operation"
            if choice == "3":
                if  "teleoperationIsStarted" in blackboard and blackboard["teleoperationIsStarted"]:
                    print("\nTeleoperation is already started! ")
                else:
                    blackboard["teleoperationIsStarted"] = True
                    return "run_teleoperation"
            if choice == "4":
                if  "mapSavingIsStarted" in blackboard and blackboard["mapSavingIsStarted"]:
                    print("\nMap saving process is already started! ")
                else:
                    blackboard["mapSavingIsStarted"] = True
                    return "save_map"
            if choice == "5":
                if "createStationIsStarted" in blackboard and blackboard["createStationIsStarted"]:
                    print("\nCreate Station is already started")
                else:
                    blackboard["createStationIsStarted"]  = True
                    return "create_station"
            if choice == "q" or choice == "Q":
                return "end"



# Define a state to load the Gazebo world
class GazeboWorldState(State):
    def __init__(self) -> None:
        super().__init__(["gazebo_started"])
        
    def execute(self, blackboard: Blackboard) -> str:
        print("Loading Gazebo world...")
        #subprocess.Popen(["ros2", "launch", "turtlebot3_gazebo", "turtlebot3_house.launch.py"])
        time.sleep(5)
        #os.system("ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py")
        subprocess.Popen(["ros2", "launch", "turtlebot3_gazebo", "turtlebot3_house.launch.py"])
        time.sleep(15)
        blackboard["gazebo_started"] = True
        return "gazebo_started"

    
#Define other states (if any) as needed
class SlamOperationState(State):
    def __init__(self) -> None:
        super().__init__(["slam_operation_started"])
    def execute(self, blackboard: Blackboard) -> str:
        print("Performing slam operation...")
        time.sleep(5)
        # #subprocess.Popen(["export TURTLEBOT3_MODEL=waffle"])
        created_process = subprocess.Popen(["ros2", "launch", "turtlebot3_cartographer", "cartographer.launch.py","use_sim_time:=True"])
        
        time.sleep(15)
        blackboard["slam_operation_started"] = True
        return "slam_operation_started"


class TeleoperationState(State):
    def __init__(self) -> None:
        super().__init__(["teleoperation_started"])
    
    def execute(self, blackboard: Blackboard) -> str:
        print("Running teleoperation node...")
        time.sleep(5)
        subprocess.call('ros2 run turtlebot3_teleop teleop_keyboard',shell=True)
        time.sleep(5)
        blackboard["teleoperation_started"] = True
        return "teleoperation_started"

class SavingMapState(State):
    def __init__(self) -> None:
        super().__init__(["map_saved"])
    
    def execute(self, blackboard: Blackboard) -> str:
        print("map saving state..")
        time.sleep(5)
        subprocess.call('ros2 run nav2_map_server map_saver_cli -f ~/yasmin_ws/map',shell=True)
        time.sleep(5)
        blackboard["map_saved"] = True
        return "map_saved"

class CreateStationState(State):
    # teleop yaparken o anki robot konumunun istasyon olarak kaydedilmesi için istasyon kaydetme durumuna geçip istasyon oluşturulması
    # initial state kayıt sonrası dönüş
    def __init__(self):
        super().__init__(["station_created"])
    
    def execute(self,blackboard : Blackboard) -> str:
        print("station creating is active")
        # listen /clicked_point topic by starting subscriber program
        clickedPointSubscriberProcess = subprocess.Popen(["ros2","run","robot","clicked_point_subscriber"])
        
        choice = input("Press Q to exit station creating state")
        while(choice != "Q" and choice != "q"):
            # wait until exit command from cli
            choice = input("Press Q to exit station creating state")
        
        # kill listener process
        clickedPointSubscriberProcess.kill()
        
        # go back to initial state
        return "station_created"
        

class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_node")
        # Create a FSM
        self.slam_process = None
        print("FSM with Gazebo World Loading Demo")
        self.sm = StateMachine(outcomes=["end"])
        # Add states with transitions
        self.sm.add_state(
            "InitialState",
            InitialState(),
            transitions = {
                "start_gazebo" : "GazeboWorldState",
                "start_slam_operation" : "SlamOperationState",
                "run_teleoperation" : "TeleoperationState",
                "save_map" : "SavingMapState",
                "create_station" : "CreateStationState",
                "end" : "end"
            }
        )
        self.sm.add_state(
            "GazeboWorldState",
            GazeboWorldState(),
            transitions= {
                "gazebo_started" : "InitialState"
            }
        )

        self.sm.add_state(
            "SlamOperationState",
            SlamOperationState(),
            transitions = {
                "slam_operation_started" : "InitialState"
            }
        )

        self.sm.add_state(
            "TeleoperationState",
            TeleoperationState(),
            transitions = {
                "teleoperation_started" : "InitialState"
            }
        )

        self.sm.add_state(
            "SavingMapState",
            SavingMapState(),
            transitions = {
                "map_saved" : "InitialState"
            }
        )

        self.sm.add_state(
            "CreateStationState",
            CreateStationState(),
            transitions= {
                "station_created" : "InitialState"
            }
        )
        # Publish FSM info
        #YasminViewerPub("CreateStationState", self.sm)
        
        # Execute the FSM
        outcome = self.sm()
        print("FSM finished with outcome:", outcome)
        



# Main function
def main(args=None):
    
    # Initialize ROS 2
    rclpy.init(args=args)
    set_ros_loggers()

    node = RobotNode()
    rclpy.spin(node=node)

    # Shutdown ROS 2
    rclpy.shutdown()

if __name__ == "__main__":
    main()



