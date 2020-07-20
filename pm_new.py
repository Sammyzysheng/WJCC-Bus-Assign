# -*- coding: utf-8 -*-
"""
Created on Thu May  7 15:16:24 2020

@author: zsheng
"""

# @author: Wade Strain
# MSBA - Capstone - PM bus routes
# April 29, 2020

# NEED to have these installed before running code
# python -m pip install --upgrade --user ortools
# pip install geopy

from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2, pywrapcp
from geopy.distance import distance
import pandas as pd
import datetime

'''
Reads in PM data for each school by tier. Optimizes the routes to each school
seperately. Assigns buses to those routes, based on the distance between the
buses' last position and the next school in the next tier. Does not assign depots.

Files:
Tier*\\Tier*_pm.csv           -> latitude, longitude, stop's school, stop, students, distance from school
Tier*\\SCHOOL_pmDistance.csv  -> distance matrix from each stop to another stop for the school
Tier*\\SCHOOL_pmRouteData.csv -> output of the optimized routes per school
bus_assignments.csv           -> table of buses and their assigned routes, duration and load of each route
school_locations.csv          -> latitude and longitude of each school and depot capacity

Need to have the above imports imported (refer to commented section above imports)
Need to have this file in the same location as the above files
'''

# load in the data for the schools in the same Tier
def load_tier_data(tier):
    # load capacity per stop (number of students) and distance and time to schools from stop
    stops_data = pd.read_csv('Tier'+str(tier)+'\\'+'Tier'+str(tier)+'_pm.csv', delimiter=',')
    stops_data = stops_data.values
    return stops_data

# create data dictionary for the ortools to solve the CVRP
def create_data_model(school_code, filename, stops_data):
    # load in time matrix data
    time_data = pd.read_csv(filename, delimiter=',', header=None)
    time_data = time_data.values

    # create data matrix; add the school as depot to the time matrix in 0th row and col
    numStops = len(time_data)
    time_matrix = [[int(row[11]) for row in stops_data if row[4]==school_code]]
    time_matrix[0].append(0)
    for i in range(numStops):
        row = [time_matrix[0][i+1]]
        for j in range(numStops):
            temp = time_data[i][j].lstrip('(').rstrip(')').split(',')
            row.append(int(temp[1]))
        row.append(0)  # for the dummy stop
        time_matrix.append(row)

    # add dummy stop to the time matrix
    dummy_stop = [0 for i in range(len(stops_data))]
    time_matrix.append(dummy_stop.append(0))

    data = {}
    data['students'] = [-1*int(row[7]) for row in stops_data if row[4]==school_code]
    # adjust time at stops based on number of students to pick up
    # stops with 5 or less take a minute, after that its 10 seconds a kid
    for i in range(1,len(time_matrix)-1):
        for j in range(1,len(time_matrix[i])-1):
            if i == j:
                pass
            elif data['students'][j] <= 5:
                time_matrix[i][j] += 60
            else:
                time_matrix[i][j] = time_matrix[i][j] + 60 + (data['students'][j]-5 * 10)

    data['time_matrix'] = time_matrix
    data['num_vehicles'] = len(buses_avail)
    data['vehicle_capacities'] = buses_avail
    data['starts'] = [0 for i in range(data['num_vehicles'])]
    data['ends'] = [len(time_matrix)-1 for i in range(data['num_vehicles'])]
    return data

# find local optimum for CVRP problem
# adapted from https://developers.google.com/optimization/routing/cvrp
def main(data):
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']), data['num_vehicles'],
                                           data['starts'], data['ends'])
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        # returns the time between two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['time_matrix'][from_node][to_node]

    def student_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['students'][from_node]

    # time callback, define cost of each arc
    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # add student capacity constraint
    routing.AddDimension(
        transit_callback_index,
        0,        # no waiting time necessary
        max_route_time,
        True,     # start cumul at 0
        'Time')

    student_callback_index = routing.RegisterUnaryTransitCallback(student_callback)
    routing.AddDimensionWithVehicleCapacity(
        student_callback_index,
        0,                            # null capacity slack
        data['vehicle_capacities'],   # vehicle maximum capacities
        True,                         # start cumul to zero
        'Capacity')

    # setting first solution heuristics
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds = 100
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    # solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        analyze_solution(data, manager, routing, solution)
    else:
        print(routing.status())

# print and keep track of route from local solution
def analyze_solution(data, manager, routing, solution):
    total_time = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_time = 0
        route_load = 0
        route = []
        while not routing.IsEnd(solution.Value(routing.NextVar(index))):
            node_index = manager.IndexToNode(index)
            route_load += data['students'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(node_index, route_load)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_time += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
            route.append((node_index, data['students'][node_index]))

        plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index), route_load)
        plan_output += 'Time of the route: {} sec\n'.format(route_time)
        plan_output += 'Load of the route: {}\n'.format(route_load)
        total_time += route_time
        route_load += data['students'][manager.IndexToNode(index)]
        total_load += route_load
        route.append((manager.IndexToNode(index), data['students'][manager.IndexToNode(index)]))

        if route_time != 0:
            # print(plan_output)
            # print(route)
            buses_avail[vehicle_id] = 0
            buses_used_counter.append(1)
            opt_routes.append([route, route_time, route_load])
    print('Total time of all routes: {}sec'.format(total_time))
    print('Total load of all routes: {}'.format(total_load))

# save routes to external csv file
def output_routes(opt_routes, tier, school, stops_data):
    routeDict = {'route': [],
                 'latitude': [],
                 'longitude': [],
                 'stop name': [],
                 'count': [],
                 'order': []}
    stops_data = [row for row in stops_data if row[4] == school]

    for i in range(len(opt_routes)):
        for j in range(len(opt_routes[i][0])):
            stop = opt_routes[i][0][j][0]
            routeDict['route'].append(i)
            routeDict['latitude'].append(stops_data[stop][1])
            routeDict['longitude'].append(stops_data[stop][0])
            routeDict['stop name'].append(stops_data[stop][5])
            routeDict['count'].append(-1*opt_routes[i][0][j][1])
            routeDict['order'].append(j+1)

    routeData = pd.DataFrame(routeDict)
    routeData.to_csv('Tier'+str(tier)+'\\'+school+'_pmRouteData.csv', index=False)

# calculate distances between buses' last stops and next tier schools
def calc_distances(x_coord, tier):
    school_locations = pd.read_csv('school_locations.csv', delimiter=',')
    school_locations = school_locations.values

    d = []
    for i in range(len(school_locations)):
         if school_locations[i][3] >= tier:
             y_coord = (school_locations[i][1],school_locations[i][0])
             dist = distance(x_coord, y_coord).m
             d.append((school_locations[i][2], dist, school_locations[i][3]))

    d.sort(key=lambda x: x[1])
    return d

def which_tier(school):
    temp = 0
    for t, s in enumerate(school_codes):
        if school in s:
            temp = t+1

    return temp

if __name__ == '__main__':
    tiers = 3
    routes = {}
    numBuses = 97
    bus_capacity = 54
    max_route_time = 2700    # 45 minutes in seconds for max time per bus for their routes
    school_codes = [['JHS', 'LHS', 'WHS','TMS'],
                    ['HMS', 'JBM', 'BMS', 'DJM', 'JR', 'SH'],
                    ['MAT', 'MW', 'CBB', 'LL', 'NES', 'JBB']]
    buses_used = {}

    ################### FInd Optimized Routes per School ########################
    # iterate through each tier to find optimum routes for each school
    start_time = datetime.datetime.now()
    for tier in range(1,tiers+1):
        print('\nStarting Tier '+str(tier)+'...')
        stops_data = load_tier_data(tier)
        buses_avail = [bus_capacity for i in range(numBuses)]
        for school in school_codes[tier-1]:
            print('\nOptimizing routes for ' + school + '...')
            opt_routes = []
            buses_used_counter = []
            data = create_data_model(school, 'Tier'+str(tier)+'\\'+school+'_pmDistance.csv', stops_data)
            main(data)
            output_routes(opt_routes, tier, school, stops_data)
            routes[school] = opt_routes
            buses_used[school] = sum(buses_used_counter)

    end_time = datetime.datetime.now()
    print('\nTime to Compute:', end_time-start_time)
    for school, count in buses_used.items():
        print('{} used {} buses'.format(school, count))

    ################### Assign Buses to Routes ################################
    bus_routes = {i:[] for i in range(numBuses)}
    # assign buses to tier 1 school routes
    bus_counter = 0
    for school in school_codes[0]:
        for i in range(buses_used[school]):
            buses_used[school] -= 1
            route =  buses_used[school]
            bus_routes[bus_counter].append([school, route, routes[school][route][1]])
            bus_counter += 1

    # calculate distance dictionary, [tier, school, route]: tuples of (next school, distances)
    distances = {}
    for tier in range(1,tiers):
        for school in school_codes[tier-1]:
            route_data = pd.read_csv('Tier'+str(tier)+'\\'+school+'_pmRouteData.csv', delimiter=',')
            route_data = route_data.values
            # gets the row of the last stop per route
            last_stops = [route_data[row] for row in range(len(route_data)) if row+1 == len(route_data) or route_data[row+1][0] != route_data[row][0]]
            # iterate through the buses needed for this school and assign routes
            for stop in last_stops:
                # distances is a list of tuples (school, distance, next_tier)
                distances[(tier, school, stop[0])] = calc_distances((stop[1], stop[2]), tier+1)

    # assign buses to tier 2 and 3 school routes based on distances from last stops of tier 1 and 2
    bus_counter = 0
    for tier in range(2,tiers+1):
        for bus, assignments in bus_routes.items():
            if len(assignments) == 0:
                bus_counter = bus
                break

            school = assignments[len(assignments)-1][0]
            route = assignments[len(assignments)-1][1]

            if len(assignments)==1 and tier == 2:
                dists = [dist for dist in distances[(1, school, route)] if dist[2]==tier]
            elif len(assignments)==1 and tier == 3:
                dists = [dist for dist in distances[(which_tier(school), school, route)] if dist[2]==tier]
            else:
                dists = distances[(2, school, route)]

            for i in range(len(dists)):
                if buses_used[dists[i][0]] > 0:
                    next_school = dists[i][0]
                    duration = (dists[i][1]*1.536)//13    # 1.536 is avg factor to convert from euclidean to google maps distance
                                                              # 13 is about the avg velocity of the bus when doing its route
                    # check to make sure that the bus can make it to the next school on time from its last stop
                    if bus_routes[bus][len(assignments)-1][2] + duration <= max_route_time or (which_tier(next_school)==3 and which_tier(school)==1):
                        buses_used[next_school] -= 1
                        next_route = buses_used[dists[i][0]]
                        next_route_duration = routes[next_school][next_route][1]
                        bus_routes[bus][len(assignments)-1][2] += duration
                        bus_routes[bus].append([next_school, next_route, next_route_duration])
                        break
            # keeps track of how many buses have been assigned for each tier

        # checks for unassigned buses and assigns them
        for school in school_codes[tier-1]:
            while buses_used[school] > 0:
                buses_used[school] -= 1
                bus_routes[bus_counter].append([school, buses_used[school], routes[school][buses_used[school]][1]])
                bus_counter += 1

    # print(bus_routes)
    # print(buses_used)

    # final output to file
    output = []
    for bus, assignments in bus_routes.items():
        for i in range(len(assignments)):
            school = assignments[i][0]
            route = assignments[i][1]
            duration = assignments[i][2]
            output.append([bus, school, route, duration, routes[school][route][2]])

    output_df = pd.DataFrame(output, columns=['bus', 'school', 'route', 'duration', 'load'])
    output_df.to_csv('bus_assignments.csv', index=False)