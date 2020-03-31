import os
import sys
import random
import math
# call utils module ,including the middle implementation function of the platooning
from utils import add_platooning_vehicle, start_sumo, running, communicate, get_distance

# ensure the correct PATH
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import traci

# call plexe module ,including the middle implementation function of the platooning
from plexe import Plexe, ACC, CACC, RPM, GEAR, RADAR_DISTANCE


# range of traffic light broadcast
RANGE = 51
# vehicle length
LENGTH = 8
# distance between multiple platoons
PLATOON_DISTANCE = 52
# vehicle length
LENGTH = 4
# inter-vehicle distance
DISTANCE = 5

FAKED_CACC = 3

# state at trafficlights
ALL_STOP = 0
ALL_PASS = 1
PART_PASS = 2






#structure platoon ,return the communication
def add_vehicles(plexe, n, n_platoons, vtype, route, position, speed, distance, real_engine=False):
    """
    Adds a set of platoons of n vehicles each to the simulation
    :param plexe: API instance
    :param n: number of vehicles of the platoon
    :param n_platoons: number of platoons
    :param vtype: the type of car in rou.xml
    :param route: the route in rou.xml
    :param position: the position of begining to start
    :param real_engine: set to true to use the realistic engine model,
    false to use a first order lag model
    :return: returns the topology of the platoon, i.e., a dictionary which
    indicates, for each vehicle, who is its leader and who is its front
    vehicle. The topology can the be used by the data exchange logic to
    automatically fetch data from leading and front vehicle to feed the CACC
    """
    # add a platoon of n vehicles
    topology = {}
    p_length = n * LENGTH + (n - 1) * distance
    for p in range(n_platoons):
        for i in range(n):
            vid = "v.%d.%d" % (p , i)
            add_platooning_vehicle(plexe, vid, position + (n_platoons - p - 1) * (p_length + PLATOON_DISTANCE) + (n - i - 1) * (distance+LENGTH) + 1,
                                   0, speed, distance, vtype, route, real_engine)
            plexe.set_fixed_lane(vid, 0, False)
            traci.vehicle.setSpeedMode(vid, 0)
            plexe.use_controller_acceleration(vid, False)
            if i == 0:
                plexe.set_active_controller(vid, ACC)
            else:
                plexe.set_active_controller(vid, CACC)
            if i > 0:
                topology[vid] = {"front": "v.%d.%d" % (p, i - 1),
                                 "leader": "v.%d.0" % p}
            else:
                topology[vid] = {}
            # RGBA
            traci.vehicle.setColor(vid, (255, 0, 255,255))
            plexe.set_path_cacc_parameters(vid, distance=DISTANCE)
    return topology



def getnext_trafficlightandphace(vehicle):
    vehicle_NextTLS = traci.vehicle.getNextTLS(vehicle)
    # print (vehicle_NextTLS)
    if vehicle_NextTLS == ():
        return "none", "none", "none","none",
    else:
        absolute_time = traci.trafficlight.getNextSwitch(vehicle_NextTLS[0][0])
        plase_temp = vehicle_NextTLS[0][3]
        if vehicle_NextTLS[0][3] == 'G':
            plase_temp = 'g'
        # id ,current_phase ,distance ,absolute_time
        return vehicle_NextTLS[0][0], plase_temp, vehicle_NextTLS[0][2], absolute_time


def judge_state(plexe, car_num, pass_num):
    state_flag = None
    if pass_num >= car_num:
        state_flag = 1
    elif pass_num < car_num and pass_num > 0:
        state_flag = 2
    elif pass_num <= 0:
        state_flag = 0
    return state_flag





def main(demo_mode, real_engine, setter=None):
    # used to randomly color the vehicles
    random.seed(1)
    start_sumo("cfg/jiangxinzhou.sumo.cfg", False)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0
    redlight_stop = 0
    topology_0 = dict()
    platoons_num = 1
    car_num = [26]
    leader = [0]
    state_2_flag = [0]
    redlight_stop = [0]
    judge_flag = [0]
    special_stop_flag = [0]
    color = [(255, 255, 255, 255), (255, 255, 0, 255), (0, 255, 255, 255),(255, 0, 255, 255), (255, 255, 255, 255), (255, 255, 0, 255), (0, 255, 255, 255),(255, 0, 255, 255), (255, 255, 255, 255), (255, 255, 0, 255), (0, 255, 255, 255),(255, 0, 255, 255)]
    special_judge_flag = [0]



    while running(demo_mode, step, 500000):

        # when reaching 60 seconds, reset the simulation when in demo_mode
        if demo_mode and step == 500000:
            start_sumo("cfg/jiangxinzhou.sumo.cfg", True)
            step = 0
            random.seed(1)

        traci.simulationStep()

        if step == 0:

            # create vehicles and track the braking vehicle
            distance_0 = 5
            speed_0 = 25
            car_num_0 = car_num[0]
            platoons_num_0 = 1
            position_0 = 1000
            topology_0 = add_vehicles(plexe, car_num_0, platoons_num_0, "vtypeauto", "platoon_route_0",
                                      position_0, speed_0, distance_0, real_engine)


            traci.gui.trackVehicle("View #0", "v.0.0")
            # traci.vehicle.highlight("v.0.0", color=(255,255,0,255), size=-1, alphaMax=-1, duration=-1, type=0)
            # traci.gui.trackVehicle("View #1", "v.0.1")
            traci.gui.setZoom("View #0", 700)
            # traci.gui.setOffset("View #0",5078.44,6332.91)
            # acf = traci.gui.getZoom(viewID='View #0')
            # print (acf)

        if step % 10 == 1:
            # simulate vehicle communication every 100 ms
            communicate(plexe, topology_0)
            # print (topology_0)

        if step > 10:
            # print(platoons_num)

            for i in range(platoons_num):

                # print(i)

                num_temp = int(leader[i])
                leader_id = "v.0.%d" % num_temp
                # for size_cycle in range (0,100):
                traci.vehicle.highlight(leader_id, color[i], size=50, alphaMax=-1, duration=-1, type=0)

                # print (leader_id)
                nextTLS, current_phase, left_distance, absolute_time = getnext_trafficlightandphace(leader_id)

                if ((left_distance >= RANGE - 1) and (left_distance < RANGE)) :
                    judge_flag[i] = 1
                else:
                    judge_flag[i] = 0

                if left_distance <= 0.5 :
                    state_2_flag[i] = 0
                    special_stop_flag[i] = 0
                # print (left_distance,state_2_flag[i],special_stop_flag[i])

                leader_data = plexe.get_vehicle_data(leader_id)
                if (nextTLS != "none" and judge_flag[i] == 1):
                    time_left = absolute_time - traci.simulation.getTime()
                    pass_num = int(math.floor((leader_data.speed * time_left - RANGE) / (LENGTH + DISTANCE)))
                    state = judge_state(plexe, car_num[i], pass_num)
                    # if i==0:
                    #     print (pass_num,state)
                    if ((state == 0 and current_phase == 'g') or (current_phase == 'r')) and special_stop_flag[i] == 0:
                        # plexe.set_cc_desired_speed(leader_id, 0)
                        # plexe.set_active_controller(leader_id, 3)
                        decel = leader_data.speed ** 2 / (2 * (left_distance - 10))
                        plexe.set_fixed_acceleration("v.0.%d" % int(leader[i]), True, -decel)
                        redlight_stop[i] = 1
                    # ALL_PASS = 1
                    # elif state == 1 and current_phase == 'g':
                    #     continue
                    # PART_PASS = 2
                    elif state == 2 and current_phase == 'g' and state_2_flag[i] == 0:
                        new_platoon_leader = "v.0.%d" % (int(leader[i]) + pass_num)
                        for j in range((int(leader[i]) + pass_num) + 1, (int(leader[i]) + car_num[i])):
                            # temporarily change the leader
                            topology_0["v.0.%d" % j]["leader"] = new_platoon_leader
                            traci.vehicle.setColor("v.0.%d" % j, color[platoons_num])

                        # the front vehicle if the vehicle opening the gap is the joiner
                        plexe.set_active_controller(new_platoon_leader, ACC)
                        traci.vehicle.setColor(new_platoon_leader, color[platoons_num])
                        plexe.set_path_cacc_parameters(new_platoon_leader, distance = 100)
                        topology_0[new_platoon_leader] = {}


                        leader.append(int(leader[i]) + pass_num)
                        car_num.append(car_num[i] - pass_num)
                        platoons_num = platoons_num + 1
                        car_num[i] = pass_num

                        redlight_stop.append(1)
                        state_2_flag[i] = 1
                        state_2_flag.append(0)
                        judge_flag.append(0)
                        special_judge_flag[i] = 1
                        special_judge_flag.append(0)

                        # must stop after separated
                        special_stop_flag.append(1)
                        leader_data_temp = plexe.get_vehicle_data("v.0.%d" % int(leader[platoons_num-1]))
                        nextTLS, current_phase, left_distance, absolute_time = getnext_trafficlightandphace("v.0.%d" % int(leader[platoons_num-1]))
                        decel = leader_data_temp.speed ** 2 / (2 * (left_distance - 10))
                        plexe.set_fixed_acceleration("v.0.%d" % int(leader[platoons_num-1]), True, -decel)
                        # plexe.set_cc_desired_speed("v.0.%d" % int(leader[i+1]), 0)
                        # plexe.set_active_controller("v.0.%d" % int(leader[platoons_num-1]), FAKED_CACC)
                        # traci.gui.trackVehicle("View #0", "v.0.%d" % int(leader[i+1]))

                # if i>=1:
                #     print(i, leader_id, leader, car_num,special_stop_flag,state_2_flag)



                # print (redlight_stop,current_phase)
                if redlight_stop[i] == 1 and current_phase != 'r' and int(leader_data.speed) <= 1:
                    plexe.set_fixed_acceleration(leader_id, False ,0)
                    plexe.set_cc_desired_speed(leader_id, 25)
                    plexe.set_active_controller(leader_id, ACC)
                    plexe.set_path_cacc_parameters(leader_id, distance=DISTANCE)

                    redlight_stop[i] = 0













        if real_engine and setter is not None:
            # if we are running with the dashboard, update its values
            tracked_id = traci.gui.getTrackedVehicle("View #0")
            if tracked_id != "":
                ed = plexe.get_engine_data(tracked_id)
                vd = plexe.get_vehicle_data(tracked_id)
                setter(ed[RPM], ed[GEAR], vd.speed, vd.acceleration)

        step += 1
    traci.close()


if __name__ == "__main__":
    main(True, True)










# if step > 0:
#     """
#     leader_data_1 = plexe.get_vehicle_data("v.1.0")
#
#     trafficlight_a = traci.trafficlight.getIDList()
#     trafficlight_b = traci.trafficlight.getRedYellowGreenState("round_point_1")
#     trafficlight_c = traci.trafficlight.getIDCount()
#     trafficlight_d = traci.trafficlight.getPhaseDuration("round_point_1")
#     trafficlight_e = traci.trafficlight.getControlledLanes("round_point_1")
#     trafficlight_f = traci.trafficlight.getControlledLinks("round_point_1")
#     trafficlight_g = traci.trafficlight.getProgram("round_point_1")
#     trafficlight_h = traci.trafficlight.getCompleteRedYellowGreenDefinition("round_point_1")
#     trafficlight_i = traci.trafficlight.getNextSwitch("round_point_1")
#     """
#     print (1)
#     leader_data_0 = plexe.get_vehicle_data("v.0.0")
#     # print (leader_data_0.speed)
#     vehicle_NextTLS, current_phase, left_distance, absolute_time = getnext_trafficlightandphace("v.0.0")
#     vehicle_NextTLS_2, current_phase_2, left_distance_2, absolute_time_2 = getnext_trafficlightandphace(new_leader_id)
#
#     # the leader at the cross
#     if vehicle_NextTLS != "none":
#         print (2)
#         # print (vehicle_NextTLS)
#         time_left = absolute_time - traci.simulation.getTime()
#
#         if left_distance >= RANGE and cross_judge_flag == True:
#             passing_cross_flag = False
#
#         if left_distance <= RANGE and current_phase == 'g' and split == False and passing_cross_flag == False:
#             passing_cross_flag = True
#             cross_judge_flag = True
#             print (3)
#             new_leader = int(math.floor((leader_data_0.speed * time_left - RANGE) / (LENGTH + DISTANCE)))
#             # print (new_leader)
#
#             # one car connot pass the trafficlight at least
#             if new_leader < car_num_0:
#                 print (4)
#
#                 new_leader_id = "v.0.%d" % new_leader
#                 print (new_leader_id)
#                 # change topology: add new leader and decelerate.
#
#                 print(topology_0)
#                 for i in range(new_leader + 1, car_num_0):
#                     topology_0["v.0.%d" % i]["leader"] = new_leader_id
#                 topology_0[new_leader_id] = {}
#
#                 new_leader_data = plexe.get_vehicle_data(new_leader_id)
#                 # print(new_leader_data)
#                 # a = (v^2-0)/2*x
#                 decel = new_leader_data.speed ** 2 / (2 * (left_distance + 20 + (new_leader) * (LENGTH + DISTANCE)))
#                 # # print(decel)
#                 plexe.set_fixed_acceleration(new_leader_id, True, - decel)
#                 traci.gui.trackVehicle("View #0", new_leader_id)
#                 current_tflight_temp, current_phase, left_distance, absolute_time = getnext_trafficlightandphace(
#                     "v.0.0")
#                 speed = 10
#                 split = True
#
#         elif left_distance <= RANGE and current_phase == 'r' and split == False and passing_cross_flag == False:
#             decel = leader_data_0.speed ** 2 / (
#                     (2 * (RANGE + new_leader * (LENGTH + DISTANCE))) - cross_safe_stop_distance + 1)
#             # print(decel)
#             passing_cross_flag = True
#             cross_judge_flag = True
#             print (5)
#             plexe.set_fixed_acceleration("v.0.0", True, - decel)
#             red_stop_flag = True
#
#         elif (
#                 left_distance <= cross_safe_stop_distance + 20) and current_phase == 'g' and split == False and red_stop_flag == True:
#             # plexe.set_fixed_acceleration("v.0.0", True, 5)
#             print (6)
#             red_stop_flag = False
#
#
#         elif (left_distance_2 <= cross_safe_stop_distance) and current_phase_2 == 'g' and split == True:
#             # plexe.set_fixed_acceleration(new_leader_id, True, 2)
#             traci.vehicle.setSpeedMode(new_leader_id, 0)
#             plexe.set_active_controller(new_leader_id, ACC)
#             # FAKED_CACC = 3
#             # plexe.set_active_controller(new_leader_id, FAKED_CACC)
#             plexe.set_cc_desired_speed(new_leader_id, SPEED + 35)
#
#             print (7)
#
#         elif split == True and vehicle_NextTLS_2 == vehicle_NextTLS:
#             topology_0[new_leader_id]["leader"] = "v.0.0"
#             topology_0[new_leader_id]["front"] = "v.0.13"
#             # IN cacc mode , the safe distance of cars when in the same lane.
#             plexe.set_path_cacc_parameters(new_leader_id, distance=30)
#
#             # print (topology_0)
#             # print (8)
#             # new_leader_follow = new_leader - 1
#             # # print (new_leader_follow)
#             # new_leader_follow_id = "v.0.%d" % new_leader_follow
#             # new_leader_follow_data = plexe.get_vehicle_data(new_leader_follow_id)
#             # new_leader_data = plexe.get_vehicle_data(new_leader_id)
#             # distance_platoon_x = abs(new_leader_data.pos_x - new_leader_follow_data.pos_x)
#             # distance_platoon_y = abs(new_leader_data.pos_y - new_leader_follow_data.pos_y)
#             # # print(distance_platoon_x,distance_platoon_y)
#             # distance_platoon = pow(distance_platoon_x ** 2 + distance_platoon_y ** 2, 0.5)
#             # print (distance_platoon)
#             # if distance_platoon == 50:
#             #     print (9)
#             #     for i in range(new_leader + 1, car_num_0):
#             #         topology_0["v.0.%d" % i]["leader"] = "v.0.0"
#             #         plexe.set_path_cacc_parameters("v.0.%d" % i, distance=DISTANCE)
#             #     topology_0[new_leader_id]["leader"] = "v.0.0"
#             #     topology_0[new_leader_id]["front"] = new_leader_follow_id
#             #     # IN cacc mode , the safe distance of cars when in the same lane.
#             #     plexe.set_path_cacc_parameters(new_leader_id, distance=DISTANCE)
#
#
#     else:
#         continue
#
#     if passing_cross_flag == False and split == False:
#         if leader_data_0.speed > SPEED:
#             plexe.set_fixed_acceleration("v.0.0", True, -1)
#         else:
#             plexe.set_fixed_acceleration("v.0.0", True, 1)
#
#     if passing_cross_flag == False and split == True:
#         if leader_data_0.speed > speed:
#             plexe.set_fixed_acceleration("v.0.0", True, -1)
#         else:
#             plexe.set_fixed_acceleration("v.0.0", True, 1)
