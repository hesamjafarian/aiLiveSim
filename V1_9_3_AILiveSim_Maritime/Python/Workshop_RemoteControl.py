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
    def __init__(self,vehicle_id):
        self.vehicle_id = vehicle_id
        self.SimulationTime = 0.0
        self.StatusString = ''
        self.CurrentDestination = [0.0, 0.0, 0.0]
        self.CurrentPosition = [0.0, 0.0, 0.0]
        self.num_waypoints_reached = 0
        self.num_waypoints_sensed = 0
        self.current_target_waypoint = []
        # Hesam Modifications
        self.waypoint_list = []
        self.last_waypoint = ''

    def Copy(self, other):
        self.SimulationTime = other.SimulationTime
        self.StatusString = other.StatusString
        self.CurrentDestination = other.CurrentDestination
        self.CurrentPosition = other.CurrentPosition
        self.num_waypoints_reached = other.num_waypoints_reached
        self.num_waypoints_sensed = other.num_waypoints_sensed
        self.current_target_waypoint = other.current_target_waypoint
        # Hesam Modifications
        self.waypoint_list = other.waypoint_list
        self.last_waypoint = other.last_waypoint

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
        # if commandList[1].startswith("Waypoint"):
        #     #print("Waypoint reached: ", commandList[1])
        #     if SimulationContext.vehicleStatus['Ego_1'] != commandList[1]:
        #         SimulationContext.vehicleStatus['Ego_1'].last_waypoint = commandList[1]
        #         SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached += 1
        #         SimulationContext.client.execute(
        #             "SetObjectProperty StatusText TextToDisplay Reached_{0}_waypoints".format(
        #                 SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached))
        #         #SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint = (SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint + 1) % SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed
        #         #print(f"Number of elements is = {SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed}")
        #         if SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached <= SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed:
        #             # SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint = (SimulationContext.vehicleStatus[
        #             #                                                                         'Ego_1'].current_target_waypoint + 1) % \
        #             #                                                                    SimulationContext.vehicleStatus[
        #             #                                                                        'Ego_1'].num_waypoints_sensed
        #             NxtPos = SimulationContext.vehicleStatus['Ego_1'].waypoint_list[SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached]
        #             SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint = NxtPos
        #             print("New Destination chosen: Waypoint", SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint,
        #                   ", pos: ", SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint)
        #     elif SimulationContext.vehicleStatus['Ego_2'] != commandList[1]:
        #         SimulationContext.vehicleStatus['Ego_2'].last_waypoint = commandList[1]
        #         SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached += 1
        #         SimulationContext.client.execute(
        #             "SetObjectProperty StatusText TextToDisplay Reached_{0}_waypoints".format(
        #                 SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached))
        #         #print(f"Number of elements is = {SimulationContext.vehicleStatus['Ego_2'].num_waypoints_sensed}")
        #         if SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached <= SimulationContext.vehicleStatus[
        #             'Ego_2'].num_waypoints_sensed:
        #                 # SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint = (
        #                 #                                                                            SimulationContext.vehicleStatus[
        #                 #                                                                                'Ego_2'].current_target_waypoint + 1) % \
        #                 #                                                                    SimulationContext.vehicleStatus[
        #                 #                                                                        'Ego_2'].num_waypoints_sensed
        #             NxtPos = SimulationContext.vehicleStatus['Ego_2'].waypoint_list[
        #                 SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached]
        #             SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint = NxtPos
        #             print("New Destination chosen: Waypoint",
        #                   SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint,
        #                   ", pos: ", SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint)
    if commandList[0].startswith('Status'):
        SimulationContext.lock.acquire()
        SimulationContext.vehicleStatus['Ego_1'].SimulationTime = float(commandList[1])
        #print(f"Simulation Time Ego1: {SimulationContext.vehicleStatus['Ego_1'].SimulationTime}")
        SimulationContext.vehicleStatus['Ego_1'].StatusString = commandList[2]
        SimulationContext.vehicleStatus['Ego_2'].SimulationTime = float(commandList[1])
        #print(f"Simulation Time Ego2: {SimulationContext.vehicleStatus['Ego_2'].SimulationTime}")
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
ego1_waypoints = []
ego2_waypoints = []
spwanTest = 0
waypoints_needs_sorting = True
sorted_ego1_positions = None
sorted_ego2_positions = None

def compute_distance_to_target(self):
    if self.CurrentDestination and self.CurrentPosition:
        dx = self.CurrentDestination[0] - self.CurrentPosition[0]
        dy = self.CurrentDestination[1] - self.CurrentPosition[1]
        distance = np.sqrt(dx ** 2 + dy ** 2)
        return distance
    else:
        return None
def recieveObjectList(FilteredObject_Socket):
    def decodeJson(jsonstring):
        msg = json.loads(jsonstring)
        return msg

    global ego1_waypoints, ego2_waypoints
    global spwanTest,waypoints_needs_sorting,sorted_ego1_positions,sorted_ego2_positions

    while True:
        try:
            data = FilteredObject_Socket.read()
        except Exception as e:  # Be specific with the exception if possible
            SimulationContext.should_end_simulation = True
            print("Connection closed: declaring end of test, reason:", str(e))
            break

        message = data.decode('utf-8')
        decoded = decodeJson(message)

        if waypoints_needs_sorting:
            # Process each element in the message
            for num, element in enumerate(decoded["MSG"]):
                waypoint_pos = element["Pos"]
                dist = element["Dist"]
                if element['Alias'].startswith('Ego1'):
                    if waypoint_pos not in SimulationContext.vehicleStatus['Ego_1'].waypoint_list:
                        ego1_waypoints.append((waypoint_pos, dist))
                elif element['Alias'].startswith('Ego2'):
                    if waypoint_pos not in SimulationContext.vehicleStatus['Ego_2'].waypoint_list:
                        ego2_waypoints.append((waypoint_pos, dist))

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
            if  not SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint and sorted_ego1_positions:
                SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint = 0
                chosenPos1 = sorted_ego1_positions[0]
                SimulationContext.vehicleStatus['Ego_1'].CurrentDestination = chosenPos1
                print("First destination for Ego1: ", chosenPos1)

            if  not SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint and sorted_ego2_positions:
                SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint = 0
                chosenPos2 = sorted_ego2_positions[0]
                SimulationContext.vehicleStatus['Ego_2'].CurrentDestination = chosenPos2
                print("First destination for Ego2: ", chosenPos2)
            waypoints_needs_sorting = False

        # Always spawn spheres for current destinations
        if not waypoints_needs_sorting and sorted_ego1_positions is not None:
            SimulationContext.client.request(
                'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(
                    pos1=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[0],
                    pos2=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[1],
                    pos3=SimulationContext.vehicleStatus['Ego_1'].CurrentDestination[2] + 250,
                    color="bluesphere"))

        if not waypoints_needs_sorting and sorted_ego2_positions is not None:
            SimulationContext.client.request(
                'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(
                    pos1=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[0],
                    pos2=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[1],
                    pos3=SimulationContext.vehicleStatus['Ego_2'].CurrentDestination[2] + 250,
                    color="redsphere"))

        if SimulationContext.should_end_simulation:
            break


# def recieveObjectList(FilteredObject_Socket):
#     def decodeJson(jsonstring):
#         msg = json.loads(jsonstring)
#         return msg
#     while (True):
#         try:
#             data = FilteredObject_Socket.read()
#         except:
#             SimulationContext.lock.acquire()
#             SimulationContext.should_end_simulation = True
#             print("Connection closed: declaring end of test")
#             SimulationContext.lock.release()
#             break
#
#         message = data.decode('utf-8')
#         decoded = decodeJson(message)
#
#         #SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed = len(decoded["MSG"])
#         # NOTE: we add the element in the order they arrive, but we could instead sort them by distance
#         # when we add them.
#         for num, element in enumerate(decoded["MSG"]):
#             waypoint_pos = element["Pos"]
#             dist = element["Dist"]
#             if element['Alias'].startswith('Ego1'):
#                 if not waypoint_pos in SimulationContext.vehicleStatus['Ego_1'].waypoint_list:
#                     #SimulationContext.vehicleStatus['Ego_1'].waypoint_list.append(waypoint_pos)
#                     ego1_waypoints.append((waypoint_pos, dist))
#                     #SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed += 1
#             elif element['Alias'].startswith('Ego2'):
#                 if not waypoint_pos in SimulationContext.vehicleStatus['Ego_2'].waypoint_list:
#                     ego2_waypoints.append((waypoint_pos, dist))
#                     #SimulationContext.vehicleStatus['Ego_2'].waypoint_list.append(waypoint_pos)
#                     #SimulationContext.vehicleStatus['Ego_2'].num_waypoints_sensed += 1
#         # Sort the waypoints based on distance
#         ego1_waypoints.sort(key=lambda x: x[1])
#         ego2_waypoints.sort(key=lambda x: x[1])
#
#         # Extract the sorted positions
#         sorted_ego1_positions = [wp[0] for wp in ego1_waypoints]
#         sorted_ego2_positions = [wp[0] for wp in ego2_waypoints]
#
#         # Update the waypoint lists in SimulationContext
#         SimulationContext.vehicleStatus['Ego_1'].waypoint_list.extend(sorted_ego1_positions)
#         SimulationContext.vehicleStatus['Ego_2'].waypoint_list.extend(sorted_ego2_positions)
#         SimulationContext.vehicleStatus['Ego_1'].num_waypoints_sensed = len(ego1_waypoints)
#         SimulationContext.vehicleStatus['Ego_2'].num_waypoints_sensed = len(ego2_waypoints)
#
#         # when starting up the first time we didn't pick a target yet
#         if SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint < 0:
#             SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint = 0
#             chosenPos1 = SimulationContext.vehicleStatus['Ego_1'].waypoint_list[SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint]
#             SimulationContext.vehicleStatus['Ego_1'].CurrentDestination = chosenPos1
#             SimulationContext.client.request(
#                 'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(pos1=chosenPos1[0],
#                                                                                      pos2=chosenPos1[1],
#                                                                                      pos3=chosenPos1[2] + 250,
#                                                                                      color="bluesphere"))
#             print("First destination for Ego1: ", chosenPos1)
#
#         if SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint < 0:
#             SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint = 0
#             chosenPos2 = SimulationContext.vehicleStatus['Ego_2'].waypoint_list[SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint]
#             SimulationContext.vehicleStatus['Ego_2'].CurrentDestination = chosenPos2
#             SimulationContext.client.request(
#                 'SpawnDebugSphere {pos1} {pos2} {pos3} 2.5 2.5 2.5 {color} 1'.format(pos1=chosenPos2[0],
#                                                                                      pos2=chosenPos2[1],
#                                                                                      pos3=chosenPos2[2] + 250,
#                                                                                      color="redsphere"))
#             print("First destination for Ego2: ", chosenPos2)
#
#         #chosenPos1 = SimulationContext.vehicleStatus['Ego_1'].waypoint_list[SimulationContext.vehicleStatus['Ego_1'].current_target_waypoint]
#         #chosenPos2 = SimulationContext.vehicleStatus['Ego_2'].waypoint_list[SimulationContext.vehicleStatus['Ego_2'].current_target_waypoint]
#
#         if SimulationContext.should_end_simulation:
#             break


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

    lastStatus1 = SimulationContext.vehicleStatus['Ego_1'].SimulationTime
    lastStatus2 = SimulationContext.vehicleStatus['Ego_2'].SimulationTime

    datapath = get_sensordata_path('/Workshop/')
    # if (os.path.exists(datapath)):
    #     shutil.rmtree(datapath, ignore_errors=True)
    #     time.sleep(1)
    # os.mkdir(datapath)
    # PositionLog = [['time', 'posX', 'posY', 'posZ', 'forwardX', 'forwardY', 'forwardZ']]
    # with open(os.path.join(datapath, 'Positions.csv'), 'a') as csvfile:
    #     writer = csv.writer(csvfile)
    #     writer.writerow(PositionLog)

    while True:
        if SimulationContext.should_end_simulation == True:
            print('end of test received')
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

            # currentEntry = [SimulationContext.vehicleStatus['Ego_1'].SimulationTime, position_1[0], position_1[1], position_1[2],
            #                 forward_1[0], forward_1[1], forward_1[2]]
            # PositionLog.append(currentEntry)
            # with open(get_sensordata_path('/Workshop/Positions.csv'), 'a') as csvfile:
            #     writer = csv.writer(csvfile)
            #     writer.writerow(currentEntry)
        ego_1_dist_2_target = SimulationContext.vehicleStatus['Ego_1'].get_distance_to_target()
        print(f"Ego_1 distance to first target= {ego_1_dist_2_target}")
        print(f"Visited targets {SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached }")
        if ego_1_dist_2_target > 50.0 and ego_1_dist_2_target < 3500.0 :
            SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached +=1
            if SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached > len(SimulationContext.vehicleStatus['Ego_1'].waypoint_list):
                SimulationContext.vehicleStatus['Ego_1'].CurrentDestination = SimulationContext.vehicleStatus['Ego_1'].waypoint_list[SimulationContext.vehicleStatus['Ego_1'].num_waypoints_reached]
                ego_1_dist_2_target = SimulationContext.vehicleStatus['Ego_1'].get_distance_to_target()

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

            # currentEntry = [SimulationContext.vehicleStatus['Ego_2'].SimulationTime, position_2[0], position_2[1], position_2[2],
            #                 forward_2[0], forward_2[1], forward_2[2]]
            # PositionLog.append(currentEntry)
            # with open(get_sensordata_path('/Workshop/Positions.csv'), 'a') as csvfile:
            #     writer = csv.writer(csvfile)
            #     writer.writerow(currentEntry)
        ego_2_dist_2_target = SimulationContext.vehicleStatus['Ego_2'].get_distance_to_target()
        print(f"Ego_2 distance to first target= {ego_2_dist_2_target}")
        if  ego_2_dist_2_target> 50.0 and ego_2_dist_2_target < 3500.0 :
            SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached +=1
            if SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached > len(SimulationContext.vehicleStatus['Ego_2'].waypoint_list):
                SimulationContext.vehicleStatus['Ego_2'].CurrentDestination = SimulationContext.vehicleStatus['Ego_2'].waypoint_list[SimulationContext.vehicleStatus['Ego_2'].num_waypoints_reached]
                ego_2_dist_2_target = SimulationContext.vehicleStatus['Ego_2'].get_distance_to_target()

        if position_2 and forward_2 and right_2:
            (throttle_2, steering_2) = SteerTowardsGoal(position_2, forward_2, right_2,
                                                        SimulationContext.vehicleStatus['Ego_2'].CurrentDestination)
            command_string = 'SetControl t:%f s:%f' % (throttle_2, steering_2)
            remoteControlSocket_2.write(command_string.encode('utf-8'))

    SimulationContext.client.execute('DestroySituation')
    sensorThread1.waitAllThreads()
    sensorThread2.waitAllThreads()

# def LaunchWorkshop():
#     SimulationContext.client.connect()
#
#     print("Loading Turku, a boat and waypoints")
#     SimulationContext.client.request('LoadScenario Workshop_BoatControl')
#     SimulationContext.client.wait_for_task_complete()
#
#     print("Connecting to the control socket")
#     remoteControlSocket_1 = ALSLib.TCPClient.TCPClient(ALS_SERVER_IP_ADDRESS, 7700, 5)
#     remoteControlSocket_1.connect(5)
#
#     remoteControlSocket_2 = ALSLib.TCPClient.TCPClient(ALS_SERVER_IP_ADDRESS, 7701, 5)
#     remoteControlSocket_2.connect(5)
#
#     print("Starting the sensors collection Threads")
#     sensorDefinintion_1 = [
#       [ALS_SERVER_IP_ADDRESS, 8880, recieveObjectList]
#     ]
#     sensorDefinintion_2 = [
#         [ALS_SERVER_IP_ADDRESS, 8881, recieveObjectList]
#     ]
#
#     sensorThread1 = ThreadedSensorRecieve(sensorDefinintion_1)
#     sensorThread2 = ThreadedSensorRecieve(sensorDefinintion_2)
#
#     lastStatus1 = SimulationContext.vehicleStatus['Ego_1'].SimulationTime
#     lastStatus2 = SimulationContext.vehicleStatus['Ego_2'].SimulationTime
#
#     datapath = get_sensordata_path('/Workshop/')
#     #if (os.path.exists(datapath)):
#     #    shutil.rmtree(datapath, ignore_errors=True)
#     #    time.sleep(1)
#     #os.mkdir(datapath)
#     #PositionLog = [['time', 'posX', 'posY', 'posZ', 'forwardX', 'forwardY', 'forwardZ']]
#     #with open(os.path.join(datapath, 'Positions.csv'), 'a') as csvfile:
#     #    writer = csv.writer(csvfile)
#     #    writer.writerow(PositionLog)
#
#     while True:
#         if SimulationContext.should_end_simulation == True:
#             print('end of test recieved')
#             break
#         time.sleep(0.05)
#
#         if lastStatus1 != SimulationContext.vehicleStatus['Ego_1'].SimulationTime:
#             lastStatus1 = SimulationContext.vehicleStatus['Ego_1'].SimulationTime
#             parsedJson = json.loads(SimulationContext.vehicleStatus['Ego_1'].StatusString)
#             position = parsedJson['Position']
#             forward = parsedJson['Forward']
#             right = parsedJson['Right']
#             SimulationContext.vehicleStatus['Ego_1'].CurrentPosition = position
#
#             currentEntry = [SimulationContext.vehicleStatus['Ego_1'].SimulationTime, position[0], position[1], position[2],
#                             forward[0], forward[1], forward[2]]
#            # PositionLog.append(currentEntry)
#            # with open(get_sensordata_path('/Workshop/Positions.csv'), 'a') as csvfile:
#            #     writer = csv.writer(csvfile)
#            #     writer.writerow(currentEntry)
#
#         (throttle, steering) = SteerTowardsGoal(position, forward, right,
#                                                 SimulationContext.vehicleStatus['Ego_1'].CurrentDestination)
#         command_string = 'SetControl t:%f s:%f' % (throttle, steering)
#         remoteControlSocket_1.write(command_string.encode('utf-8'))
#
#         if lastStatus2 != SimulationContext.vehicleStatus['Ego_2'].SimulationTime:
#             lastStatus2 = SimulationContext.vehicleStatus['Ego_2'].SimulationTime
#             parsedJson = json.loads(SimulationContext.vehicleStatus['Ego_2'].StatusString)
#             position = parsedJson['Position']
#             forward = parsedJson['Forward']
#             right = parsedJson['Right']
#             SimulationContext.vehicleStatus['Ego_2'].CurrentPosition = position
#
#             currentEntry = [SimulationContext.vehicleStatus['Ego_2'].SimulationTime, position[0], position[1], position[2],
#                             forward[0], forward[1], forward[2]]
#            # PositionLog.append(currentEntry)
#            # with open(get_sensordata_path('/Workshop/Positions.csv'), 'a') as csvfile:
#            #     writer = csv.writer(csvfile)
#            #     writer.writerow(currentEntry)
#
#         (throttle, steering) = SteerTowardsGoal(position, forward, right,
#                                                 SimulationContext.vehicleStatus['Ego_2'].CurrentDestination)
#         command_string = 'SetControl t:%f s:%f' % (throttle, steering)
#         remoteControlSocket_1.write(command_string.encode('utf-8'))
#         remoteControlSocket_2.write(command_string.encode('utf-8'))
#
#     SimulationContext.client.execute('DestroySituation')
#     sensorThread1.waitAllThreads()
#     sensorThread2.waitAllThreads()


print("Start the demo ")

LaunchWorkshop()

print("--- end of script ---")
