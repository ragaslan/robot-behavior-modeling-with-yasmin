import time
import os
import subprocess
import rclpy
from rclpy.node import Node
from yasmin import State, Blackboard, StateMachine
from yasmin_ros.ros_logs import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from utils.kill_sub_process import kill_sub_process


class InitialState(State):
    def __init__(self) -> None:
        super().__init__(["start_gazebo","start_slam_operation","run_teleoperation","save_map","create_station","start_navigation","end"])

    def execute(self,blackboard: Blackboard) -> str:
        # burada ana durumda komutlar alarak diğer durumlara geçiş yap
        print("initial state is active")
        
        # set battery value in blackboard
        # max value is 100
        blackboard["battery_level"] = 90

        flag = True
        while(flag):
            print("\nSelect Process\n1) Load Gazebo World\n2) Start Slam Operation\n3) Run Teleoperation Node\n4)Save Map\n5)Create Station Point\n6)Navigation Start\nQ)Exit")
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
            if choice == "6":
                if "navigationIsStarted" in blackboard and blackboard["navigationIsStarted"]:
                    print("\nNavigation is already started")
                else:
                    blackboard["navigationIsStarted"] = True
                    return "start_navigation"
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
        gazeboWorldProcess = subprocess.Popen(["ros2", "launch", "turtlebot3_gazebo", "turtlebot3_house.launch.py"])
        time.sleep(15)
        blackboard["gazebo_started"] = True
        blackboard["gazebo_world_process"] = gazeboWorldProcess.pid
        return "gazebo_started"

    
#Define other states (if any) as needed
class SlamOperationState(State):
    def __init__(self) -> None:
        super().__init__(["slam_operation_started"])
    def execute(self, blackboard: Blackboard) -> str:
        print("Performing slam operation...")
        time.sleep(5)
        # #subprocess.Popen(["export TURTLEBOT3_MODEL=waffle"])
        slam_operation_process = subprocess.Popen(["ros2", "launch", "turtlebot3_cartographer", "cartographer.launch.py","use_sim_time:=True"])
        time.sleep(15)
        blackboard["slam_operation_started"] = True
        blackboard["slam_operation_process"] = slam_operation_process
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

class NavigationSetupState(State):
    def __init__(self):
        super().__init__(["nav2_started"])

    def execute(self, blackboard):
        
        navigationProcess = subprocess.Popen(
            f"ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=/home/agaslan/yasmin_ws/map.yaml",
            shell=True
        )
        
        blackboard["navigation_process"] = navigationProcess
        
        time.sleep(10)

        return "nav2_started"

class TaskListenerState(State):
    def __init__(self):
        super().__init__(["task_accepted"])
    
    def show_menu(self,stations):
        print("*******************\n")
        print("Stations: \n")

        for i in range(len(stations)):
            print(f" Station {i}\n")
        print("*******************\n")

        print("GIVE COMMAND TO ROBOT\n")

        # 1 noktaya gitme görevleri için
        print(" 1) Go to Station\n")




    def execute(self,blackboard : Blackboard) -> str:
        print("***********")
        # burada bir dinleme mekanizması olmalı seçim ekran ya da
        # istasyon bilgilerini versin ve görevler gözüksün
        ## istasyon bilgileri
        stations = []
        # dosyayı okuma 
        stations_file = open("stations.txt","r")
        while True:
            content = stations_file.readline().rstrip("\n")
            if not content:
                break
            arr = content.split(',')
            
            station = dict()

            for arr_item in arr:
                # key değeri ile x,y,z 
                # value ile bu x,y,z ye karşılık gelen değer alınır
                key = arr_item.split(":")[0]
                value = arr_item.split(":")[1]
                station[key] = value
            
            stations.append(station)
        

        # menu goster ve input al
        while True:
            self.show_menu(stations)
            print("Select command: ")
            choice = input()
            # verilen inputa blackboard a görev tanımla
            if choice == "1":
                print("Which station you want to go ? ")
                station_choice = input()
                blackboard["mission"] = dict()
                blackboard["mission"]["type"] = 1
                blackboard["mission"]["target"] = stations[int(station_choice)]
                #print(blackboard["mission"])
                break
        # yapılması için navigasyon durumuna geçiş yap
        return "task_accepted"

class NavigationState(State):
    def __init__(self):
        super().__init__(["succeed","failed","canceled","low_battery"])
    
    def execute(self, blackboard : Blackboard) -> str:
        # task listener state tarafından görevi alır (blackboard sayesinde)
        if "mission" not in blackboard:
            print("mission could not find")
            return "failed"
        else:
            # görevi yap
            # görev tipine göre görev yap - görev tipi 1 direkt istasyona gitme - görev tipi 2 ise iki nokta arasında bir tur gidip gelme
            #print(blackboard["mission"])
            mission = blackboard["mission"]

            mission_x = mission["target"]["x"]
            mission_y = mission["target"]["y"]
            mission_z = mission["target"]["z"]
            mission_w = 1.0
            cli_command_str = "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 3.0}, orientation: {z: 0.0, w: 1.0}}}}\""
            cli_command_str = cli_command_str[:129] + str(mission_x) + cli_command_str[132:137] + str(mission_y) + cli_command_str[140:160] + str(mission_z) + cli_command_str[163:168] + str(mission_w) + cli_command_str[171:]
            print(cli_command_str)
            subprocess.call(cli_command_str,shell=True)
            blackboard["mission"] = None
            # duruma göre state değiştir ya da şarj sorun olursa şarja git 

            return "succeed"

class ChargeState(State):
    def __init__(self):
        super().__init__(["battery_full"])
    
    def execute(self, blackboard : Blackboard) -> str:
        # todo
        battery_level = blackboard["battery_level"]
        while(battery_level < 100):
            battery_level += 1
            time.sleep(1)
        
        if battery_level == 100:
            return "battery_full"


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
                "start_navigation" : "NavigationSetupState",
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
        self.sm.add_state(
            "NavigationSetupState",
            NavigationSetupState(),
            transitions = {
                "nav2_started" : "TaskListenerState"
            }
        )
        self.sm.add_state(
            "TaskListenerState",
            TaskListenerState(),
            transitions = {
                "task_accepted" : "NavigationState"
            }
        )

        self.sm.add_state(
            "NavigationState",
            NavigationState(),
            transitions = {
                "succeed" : "TaskListenerState",
                "failed" : "TaskListenerState",
                "canceled" : "TaskListenerState",
                "low_battery" : "ChargeState"
            }
        )

        self.sm.add_state(
            "ChargeState",
            ChargeState(),
            transitions= {
                "battery_full" : "TaskListenerState"
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



