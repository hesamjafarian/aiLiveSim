import csv, shutil, json, os, time, threading
import numpy as np
import ALSLib.ALSClient, ALSLib.TCPClient
from ALSLib.ALSHelperFunctionLibrary import get_sensordata_path
import random
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
        self.JsonData = {}
        self.CurrentDestination = [0.0, 0.0, 0.0]
        self.CurrentPosition = [0.0, 0.0, 0.0]
        self.num_waypoints_reached = 0
        self.num_waypoints_sensed = 0
        self.current_target_waypoint = []
        # Hesam Modifications
        self.forward = []
        self.right = []
        self.waypoint_list = []
        self.last_waypoint = ''
        self.waypoints_needs_sorting = True
        self.sorted_waypoint_list = None

    def Copy(self, other):
        self.SimulationTime = other.SimulationTime
        self.JsonData = other.JsonData
        self.CurrentDestination = other.CurrentDestination
        self.CurrentPosition = other.CurrentPosition
        self.num_waypoints_reached = other.num_waypoints_reached
        self.num_waypoints_sensed = other.num_waypoints_sensed
        self.current_target_waypoint = other.current_target_waypoint
        # Hesam Modifications
        self.forward = other.forward
        self.right = other.right
        self.waypoint_list = other.waypoint_list
        self.last_waypoint = other.last_waypoint
        self.waypoints_needs_sorting = other.waypoints_needs_sorting
        self.sorted_waypoint_list = other.sorted_waypoint_list


    def get_distance_to_target(self):
        if self.CurrentDestination and self.CurrentPosition:
            dx = self.CurrentDestination[0] - self.CurrentPosition[0]
            dy = self.CurrentDestination[1] - self.CurrentPosition[1]
            distance = np.sqrt(dx ** 2 + dy ** 2)
            return distance
        else:
            return None


###############################################################################
def messageHandlerBoat(rawMessage):
    str = rawMessage.decode('utf-8')
    commandList = str.split(" ")

    if commandList[0].startswith('EndCondition'):
        if (commandList[1].startswith("TimeEnded")):
            SimulationContext.lock.acquire()
            SimulationContext.should_end_simulation = True
            print("Timout recieved .. declaring end of test")
            SimulationContext.lock.release()

    elif commandList[0].startswith('Status'):
        parsedJson = json.loads(commandList[2])
        SimulationContext.lock.acquire()
        if parsedJson["ID"] == "Ego_1":
            SimulationContext.vehicleStatus['Ego_1'].SimulationTime = float(commandList[1])
            SimulationContext.vehicleStatus['Ego_1'].JsonData = parsedJson
        elif parsedJson["ID"] == "Ego_2":
            SimulationContext.vehicleStatus['Ego_2'].SimulationTime = float(commandList[1])
            SimulationContext.vehicleStatus['Ego_2'].JsonData = parsedJson

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
###############################################################################
def SteerTowardsGoal(position, forward, right, destination,vehicle_id):
    print(f'Vehicle {vehicle_id} Pos= {position} Forward={forward}, Right={right}, Destination={destination}')
    d = np.asarray(destination)
    p = np.asarray(position)
    targetDir = d - p
    distance = np.linalg.norm(targetDir)

    if distance != 0.0:
        targetDir = targetDir / distance
    else:
        return (0, 0)  # No movement needed

    Steering = np.dot(targetDir, right)
    dot = np.dot(forward, targetDir)

    if distance > 100:
        if dot > 0:
            Throttle = 0.5
        else:
            Throttle = 0
            Steering = -1.0 if Steering <= 0 else 1.0
    else:
        Throttle = min(distance / 200, 0.5)

    # Debug output
    #print(f"Debug Info -- Ego_X Position: {position}, Forward: {forward}, Right: {right}, Destination: {destination}, Throttle: {Throttle}, Steering: {Steering}")

    return (Throttle, Steering)
# def SteerTowardsGoal(position, forward, right, destination):
#     d = np.asarray(destination)
#     p = np.asarray(position)
#     targetDir = d - p
#     distance = np.linalg.norm(targetDir)
#     if distance != 0.0:
#         targetDir = targetDir / distance
#     else:
#         targetDir = np.asarray([1.0, 0.0, 0.0])
#
#     Throttle = 0
#     Steering = np.dot(targetDir, right)
#     dot = np.dot(forward, targetDir)
#
#     if distance > 100:
#         if dot > 0:
#             Throttle = 0.5
#         else:
#             Throttle = 0
#             Steering = -1.0
#     else:
#         Throttle = distance / 100
#
#     return (Throttle, Steering)


###############################################################################
# Initialize the global lists
# ego1_waypoints = []
# ego2_waypoints = []
#
# waypoints_needs_sorting = True
# sorted_ego1_positions = None
# sorted_ego2_positions = None


def recieveObjectList(FilteredObject_Socket):
    def decodeJson(jsonstring):
        msg = json.loads(jsonstring)
        return msg

    # global ego1_waypoints, ego2_waypoints
    # global waypoints_needs_sorting, sorted_ego1_positions, sorted_ego2_positions

    while True:
        try:
            data = FilteredObject_Socket.read()
        except Exception as e:  # Be specific with the exception if possible
            SimulationContext.should_end_simulation = True
            print("Connection closed: declaring end of test, reason:", str(e))
            break

        message = data.decode('utf-8')
        decoded = decodeJson(message)
        waypoints1_needs_sorting = SimulationContext.vehicleStatus['Ego_1'].waypoints_needs_sorting
        waypoints2_needs_sorting = SimulationContext.vehicleStatus['Ego_2'].waypoints_needs_sorting

        if waypoints1_needs_sorting or waypoints2_needs_sorting :
            # Process each element in the message
            for num, element in enumerate(decoded["MSG"]):
                waypoint_pos = element["Pos"]
                dist = element["Dist"]
                if element['Alias'].startswith('Ego1'):
                    if waypoint_pos not in SimulationContext.vehicleStatus['Ego_1'].waypoint_list:
                        SimulationContext.vehicleStatus['Ego_1'].waypoint_list.append((waypoint_pos, dist))
                elif element['Alias'].startswith('Ego2'):
                    if waypoint_pos not in SimulationContext.vehicleStatus['Ego_2'].waypoint_list:
                        SimulationContext.vehicleStatus['Ego_2'].waypoint_list.append((waypoint_pos, dist))

            # Sort the waypoints based on distance
            SimulationContext.vehicleStatus['Ego_1'].waypoint_list.sort(key=lambda x: x[1])
            SimulationContext.vehicleStatus['Ego_2'].waypoint_list.sort(key=lambda x: x[1])

            # Extract the sorted positions
            SimulationContext.vehicleStatus['Ego_1'].sorted_waypoint_list = [wp[0] for wp in SimulationContext.vehicleStatus['Ego_1'].waypoint_list]
            SimulationContext.vehicleStatus['Ego_2'].sorted_waypoint_list = [wp[0] for wp in SimulationContext.vehicleStatus['Ego_2'].waypoint_list]

            # Update the waypoint lists in SimulationContext
            SimulationContext.vehicleStatus['Ego_1'].waypoint_list = SimulationContext.vehicleStatus['Ego_1'].sorted_waypoint_list
            SimulationContext.vehicleStatus['Ego_2'].waypoint_list = SimulationContext.vehicleStatus['Ego_2'].sorted_waypoint_list

            SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed = len(SimulationContext.vehicleStatus['Ego_1'].sorted_waypoint_list)
            SimulationContext.vehicleStatus['Ego_2'].num_waypoints_sensed = len(SimulationContext.vehicleStatus['Ego_2'].sorted_waypoint_list)

            # Set the first destination if not already set
            if not SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint and SimulationContext.vehicleStatus['Ego_1'].sorted_waypoint_list:
                SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint = 0
                chosenPos1 = SimulationContext.vehicleStatus['Ego_1'].sorted_waypoint_list[0]
                SimulationContext.vehicleStatus['Ego_1'].CurrentDestination = chosenPos1
                #print("First destination for Ego1: ", chosenPos1)

            if not SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint and SimulationContext.vehicleStatus['Ego_2'].sorted_waypoint_list:
                SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint = 0
                chosenPos2 = SimulationContext.vehicleStatus['Ego_2'].sorted_waypoint_list[0]
                SimulationContext.vehicleStatus['Ego_2'].CurrentDestination = chosenPos2
                #print("First destination for Ego2: ", chosenPos2)
            SimulationContext.vehicleStatus['Ego_1'].waypoints_needs_sorting = False
            SimulationContext.vehicleStatus['Ego_2'].waypoints_needs_sorting = False
        SimulationContext.client.request(
            'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(
                pos1=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[0],
                pos2=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[1],
                pos3=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[2] + 250,
                color="bluesphere"))

        SimulationContext.client.request(
            'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(
                pos1=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[0],
                pos2=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[1],
                pos3=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[2] + 250,
                color="redsphere"))
        time.sleep(0.05)
        if SimulationContext.should_end_simulation:
            break


###############################################################################
def control_boat(vehicle_id, control_socket, sync_event):
    vehicleStatus = SimulationContext.vehicleStatus[vehicle_id]
    last_status = vehicleStatus.SimulationTime
    while True:
        if SimulationContext.should_end_simulation:
            print(f'end of test received for {vehicle_id}')
            break

        if last_status != vehicleStatus.SimulationTime:
            last_status = vehicleStatus.SimulationTime
            vehicleStatus.CurrentPosition = vehicleStatus.JsonData['Position']
            vehicleStatus.forward = vehicleStatus.JsonData['Forward']
            vehicleStatus.right = vehicleStatus.JsonData['Right']

        dist_to_target = vehicleStatus.get_distance_to_target()
        data_complete = (vehicleStatus.CurrentPosition and vehicleStatus.forward and vehicleStatus.right)

        #print(f"{vehicle_id} distance to first target= {dist_to_target}")
        #print(f"{vehicle_id} Visited targets {SimulationContext.vehicleStatus[vehicle_id].num_waypoints_reached}")

        if dist_to_target < 2000.0:
            if vehicleStatus.num_waypoints_reached < len(
                    vehicleStatus.waypoint_list):
                vehicleStatus.num_waypoints_reached += 1
                if vehicleStatus.num_waypoints_reached < len(
                        vehicleStatus.waypoint_list):
                    vehicleStatus.CurrentDestination = vehicleStatus.waypoint_list[vehicleStatus.num_waypoints_reached]
                    #dist_to_target = SimulationContext.vehicleStatus[vehicle_id].get_distance_to_target()
                    #print(f"Updated distance to target for {vehicle_id} = {dist_to_target}")

        if data_complete and vehicleStatus.num_waypoints_reached < 4:
            (throttle, steering) = SteerTowardsGoal(vehicleStatus.CurrentPosition, vehicleStatus.forward, vehicleStatus.right,
                                                    vehicleStatus.CurrentDestination,vehicle_id)
            command_string = 'SetControl t:%f s:%f' % (throttle, steering)
            print(f"Sending data to the vehicle = {vehicle_id} using=  {command_string}\n")
            control_socket.write(command_string.encode('utf-8'))

        sync_event.set()
        time.sleep(0.05)


import random

#
# def generate_random_coordinates(base_pos, deviation=50):
#     """
#     Generate random coordinates close to a given base position.
#
#     :param base_pos: A tuple of base coordinates (x, y, z).
#     :param deviation: The maximum deviation for each coordinate.
#     :return: A tuple of new random coordinates.
#     """
#     new_x = base_pos[0] + random.uniform(-deviation, deviation)
#     new_y = base_pos[1] + random.uniform(-deviation, deviation)
#     new_z = base_pos[2] + random.uniform(-deviation, deviation)
#     return (new_x, new_y, new_z)
#
# def show_balls(sync_event):
#     new_pos = [-131835.968750, 86185.601562, 549.999939]
#     while True:
#         # Increment each coordinate by 10
#         new_pos[0] += 10.0
#         new_pos[1] += 10.0
#         new_pos[2] += 10.0
#
#         # Assign the incremented values to new_x, new_y, new_z
#         new_x, new_y, new_z = new_pos[0], new_pos[1], new_pos[2]
#
#         SimulationContext.client.request(
#             'SpawnDebugSphere {pos1} {pos2} {pos3} 5 5 5 {color} 0.5'.format(
#                 pos1=new_x,
#                 pos2=new_y,
#                 pos3=new_z + 250,
#                 color="greensphere"))
#     sync_event.set()

###############################################################################
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

    datapath = get_sensordata_path('/Workshop/')

    sync_event = threading.Event()

    boat1_thread = threading.Thread(target=control_boat, args=('Ego_1', remoteControlSocket_1, sync_event))
    boat2_thread = threading.Thread(target=control_boat, args=('Ego_2', remoteControlSocket_2, sync_event))
    #show_balls_thread = threading.Thread(target=show_balls, args=(sync_event,))
    boat1_thread.start()
    boat2_thread.start()
    #show_balls_thread.start()

    boat1_thread.join()
    boat2_thread.join()
    #show_balls_thread.join()

    SimulationContext.client.execute('DestroySituation')
    sensorThread1.waitAllThreads()
    sensorThread2.waitAllThreads()

print("Start the demo ")

LaunchWorkshop()

print("--- end of script ---")
