import csv, shutil, json, os, time, threading
import numpy as np
import ALSLib.ALSClient, ALSLib.TCPClient
from ALSLib.ALSHelperFunctionLibrary import get_sensordata_path

ALS_SERVER_IP_ADDRESS = '127.0.0.1'
###############################################################################
class SensorDataThread(threading.Thread):
    def __init__(self, ThreadID, client, function):
        threading.Thread.__init__(self)
        self.threadID = ThreadID
        self.client = client
        self.function = function

    def run(self):
        print("Starting ", self.threadID, "\n")
        self.function(self.client)
        print("Exiting ", self.threadID, "\n")


###############################################################################
class ThreadedSensorRecieve:
    def __init__(self, sensorDefinitions):
        self.sensorThreads = []
        [self.addSensor(sensor[0], sensor[1], sensor[2])
         for sensor in sensorDefinitions]
        self.startAllThreads()

    def addSensor(self, socketAddress, socketPort, recieveFunction):
        socket = ALSLib.TCPClient.TCPClient(socketAddress, socketPort, 15)
        socket.connect(10)
        thread = SensorDataThread(len(self.sensorThreads), socket, recieveFunction)
        self.sensorThreads.append([socket, thread])

    def startAllThreads(self):
        [thread[1].start() for thread in self.sensorThreads]

    def waitAllThreads(self):
        [thread[1].join() for thread in self.sensorThreads]


###############################################################################
class VehicleStatus:
    def __init__(self, vehicle_id):
        self.vehicle_id = vehicle_id
        self.SimulationTime = 0.0
        self.StatusString = ''
        self.CurrentDestination = [0.0, 0.0, 0.0]
        self.CurrentPosition = [0.0, 0.0, 0.0]
        self.num_waypoints_reached = 0
        self.num_waypoints_sensed = 0
        self.current_target_waypoint = -1  # Initialize as -1 to indicate no target
        self.waypoint_list = []
        self.last_waypoint = ''
        self.distance_to_target = float('inf')

    def update_destination(self):
        if self.num_waypoints_reached < len(self.waypoint_list):
            self.CurrentDestination = self.waypoint_list[self.num_waypoints_reached]
            self.distance_to_target = self.compute_distance_to_target()

    def compute_distance_to_target(self):
        if self.CurrentDestination and self.CurrentPosition:
            dx = self.CurrentDestination[0] - self.CurrentPosition[0]
            dy = self.CurrentDestination[1] - self.CurrentPosition[1]
            dz = self.CurrentDestination[2] - self.CurrentPosition[2]
            distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
            return distance
        return float('inf')



###############################################################################
def messageHandlerBoat(rawMessage):
    str = rawMessage.decode('utf-8')
    commandList = str.split(" ")
    if commandList[0].startswith('EndCondition'):
        if commandList[1].startswith("TimeEnded"):
            SimulationContext.lock.acquire()
            SimulationContext.should_end_simulation = True
            print("Timeout received .. declaring end of test")
            SimulationContext.lock.release()
    if commandList[0].startswith('Status'):
        SimulationContext.lock.acquire()
        SimulationContext.vehicleStatus['Ego_1'].SimulationTime = float(commandList[1])
        SimulationContext.vehicleStatus['Ego_1'].StatusString = commandList[2]
        SimulationContext.vehicleStatus['Ego_2'].SimulationTime = float(commandList[1])
        SimulationContext.vehicleStatus['Ego_2'].StatusString = commandList[2]
        SimulationContext.lock.release()


###############################################################################
class SimulationContext:
    lock = threading.Lock()
    should_end_simulation = False
    ALS_simulation_control_port = 9000
    client = ALSLib.ALSClient.Client((ALS_SERVER_IP_ADDRESS, ALS_simulation_control_port), messageHandlerBoat)
    vehicleStatus = {
        'Ego_1': VehicleStatus('Ego_1'),
        'Ego_2': VehicleStatus('Ego_2')
    }
    #vehicleStatus = VehicleStatus()
    #waypoint_list = []
    #last_waypoint = ''


###############################################################################
def SteerTowardsGoal(position, forward, right, destination):
    d = np.asarray(destination)
    p = np.asarray(position)
    targetDir = d - p
    distance = np.linalg.norm(targetDir)
    if distance != 0.0:
        targetDir = targetDir / distance
    else:
        targetDir = np.asarray([1.0, 0.0, 0.0])

    Throttle = 0
    Steering = np.dot(targetDir, right)
    dot = np.dot(forward, targetDir)

    if distance > 100:
        if dot > 0:
            Throttle = 0.5
        else:
            Throttle = 0
            Steering = -1.0
    else:
        Throttle = distance / 100

    return (Throttle, Steering)


###############################################################################
# Initialize the global lists
def recieveObjectList(FilteredObject_Socket):
    def decodeJson(jsonstring):
        msg = json.loads(jsonstring)
        return msg

    global ego1_waypoints, ego2_waypoints
    global waypoints_needs_sorting, sorted_ego1_positions, sorted_ego2_positions

    while True:
        try:
            data = FilteredObject_Socket.read()
        except Exception as e:  # Be specific with the exception if possible
            SimulationContext.should_end_simulation = True
            print("Connection closed: declaring end of test, reason:", str(e))
            break

        message = data.decode('utf-8')
        decoded = decodeJson(message)

        waypoints_updated = False

        # Process each element in the message
        for num, element in enumerate(decoded["MSG"]):
            waypoint_pos = element["Pos"]
            dist = element["Dist"]
            if element['Alias'].startswith('Ego1'):
                if waypoint_pos not in SimulationContext.vehicleStatus['Ego_1'].waypoint_list:
                    ego1_waypoints.append((waypoint_pos, dist))
                    waypoints_updated = True
            elif element['Alias'].startswith('Ego2'):
                if waypoint_pos not in SimulationContext.vehicleStatus['Ego_2'].waypoint_list:
                    ego2_waypoints.append((waypoint_pos, dist))
                    waypoints_updated = True

        if waypoints_updated:
            # Sort the waypoints based on distance
            ego1_waypoints.sort(key=lambda x: x[1])
            ego2_waypoints.sort(key=lambda x: x[1])

            # Extract the sorted positions
            sorted_ego1_positions = [wp[0] for wp in ego1_waypoints]
            sorted_ego2_positions = [wp[0] for wp in ego2_waypoints]

            # Update the waypoint lists in SimulationContext
            SimulationContext.vehicleStatus['Ego_1'].waypoint_list = sorted_ego1_positions
            SimulationContext.vehicleStatus['Ego_2'].waypoint_list = sorted_ego2_positions
            SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed = len(sorted_ego1_positions)
            SimulationContext.vehicleStatus['Ego_2'].num_waypoints_sensed = len(sorted_ego2_positions)

            # Set the first destination if not already set
            if SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint == -1 and sorted_ego1_positions:
                SimulationContext.vehicleStatus['Ego_1'].update_destination()
                print("First destination for Ego1: ", SimulationContext.vehicleStatus['Ego_1'].CurrentDestination)

            if SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint == -1 and sorted_ego2_positions:
                SimulationContext.vehicleStatus['Ego_2'].update_destination()
                print("First destination for Ego2: ", SimulationContext.vehicleStatus['Ego_2'].CurrentDestination)

        # Always spawn spheres for current destinations
        if sorted_ego1_positions:
            SimulationContext.client.request(
                'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(
                    pos1=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[0],
                    pos2=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[1],
                    pos3=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[2] + 250,
                    color="bluesphere"))

        if sorted_ego2_positions:
            SimulationContext.client.request(
                'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(
                    pos1=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[0],
                    pos2=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[1],
                    pos3=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[2] + 250,
                    color="redsphere"))

        if SimulationContext.should_end_simulation:
            break

def LaunchWorkshop():
    SimulationContext.client.connect()

    print("Loading Turku, a boat and waypoints")
    SimulationContext.client.request('LoadScenario Workshop_BoatControl')
    SimulationContext.client.wait_for_task_complete()

    print("Connecting to the control socket")
    remoteControlSocket_1 = ALSLib.TCPClient.TCPClient(ALS_SERVER_IP_ADDRESS, 7700, 5)
    remoteControlSocket_1.connect(5)

    remoteControlSocket_2 = ALSLib.TCPClient.TCPClient(ALS_SERVER_IP_ADDRESS, 7701, 5)
    remoteControlSocket_2.connect(5)

    print("Starting the sensors collection Threads")
    sensorDefinintion_1 = [
        [ALS_SERVER_IP_ADDRESS, 8880, recieveObjectList]
    ]
    sensorDefinintion_2 = [
        [ALS_SERVER_IP_ADDRESS, 8881, recieveObjectList]
    ]

    sensorThread1 = ThreadedSensorRecieve(sensorDefinintion_1)
    sensorThread2 = ThreadedSensorRecieve(sensorDefinintion_2)

    lastStatus1 = SimulationContext.vehicleStatus['Ego_1'].SimulationTime
    lastStatus2 = SimulationContext.vehicleStatus['Ego_2'].SimulationTime

    while True:
        if SimulationContext.should_end_simulation:
            print('End of test received')
            break
        time.sleep(0.05)

        position_1, forward_1, right_1 = None, None, None
        position_2, forward_2, right_2 = None, None, None

        if lastStatus1 != SimulationContext.vehicleStatus['Ego_1'].SimulationTime:
            lastStatus1 = SimulationContext.vehicleStatus['Ego_1'].SimulationTime
            parsedJson = json.loads(SimulationContext.vehicleStatus['Ego_1'].StatusString)
            position_1 = parsedJson['Position']
            forward_1 = parsedJson['Forward']
            right_1 = parsedJson['Right']
            SimulationContext.vehicleStatus['Ego_1'].CurrentPosition = position_1
            SimulationContext.vehicleStatus['Ego_1'].distance_to_target = SimulationContext.vehicleStatus['Ego_1'].compute_distance_to_target()

        ego_1_dist_2_target = SimulationContext.vehicleStatus['Ego_1'].distance_to_target
        print(f"Ego_1 distance to target= {ego_1_dist_2_target}")
        print(f"Visited targets {SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached}")

        if ego_1_dist_2_target < 3500.0:  # Threshold for considering waypoint reached
            SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached += 1
            SimulationContext.vehicleStatus['Ego_1'].update_destination()

        if position_1 and forward_1 and right_1:
            (throttle_1, steering_1) = SteerTowardsGoal(position_1, forward_1, right_1,
                                                        SimulationContext.vehicleStatus['Ego_1'].CurrentDestination)
            command_string = 'SetControl t:%f s:%f' % (throttle_1, steering_1)
            remoteControlSocket_1.write(command_string.encode('utf-8'))

        if lastStatus2 != SimulationContext.vehicleStatus['Ego_2'].SimulationTime:
            lastStatus2 = SimulationContext.vehicleStatus['Ego_2'].SimulationTime
            parsedJson = json.loads(SimulationContext.vehicleStatus['Ego_2'].StatusString)
            position_2 = parsedJson['Position']
            forward_2 = parsedJson['Forward']
            right_2 = parsedJson['Right']
            SimulationContext.vehicleStatus['Ego_2'].CurrentPosition = position_2
            SimulationContext.vehicleStatus['Ego_2'].distance_to_target = SimulationContext.vehicleStatus['Ego_2'].compute_distance_to_target()

        ego_2_dist_2_target = SimulationContext.vehicleStatus['Ego_2'].distance_to_target
        print(f"Ego_2 distance to target= {ego_2_dist_2_target}")

        if ego_2_dist_2_target < 3500.0:  # Threshold for considering waypoint reached
            SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached += 1
            SimulationContext.vehicleStatus['Ego_2'].update_destination()

        if position_2 and forward_2 and right_2:
            (throttle_2, steering_2) = SteerTowardsGoal(position_2, forward_2, right_2,
                                                        SimulationContext.vehicleStatus['Ego_2'].CurrentDestination)
            command_string = 'SetControl t:%f s:%f' % (throttle_2, steering_2)
            remoteControlSocket_2.write(command_string.encode('utf-8'))

    SimulationContext.client.execute('DestroySituation')
    sensorThread1.waitAllThreads()
    sensorThread2.waitAllThreads()

print("Start the demo ")

LaunchWorkshop()

print("--- end of script ---")
