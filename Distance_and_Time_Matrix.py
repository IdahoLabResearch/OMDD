import pandas as pd
import math
from math import sin, cos, sqrt, atan2, radians
from machineLearningModel import ascend, descend, hover, forwardFlight
# import folium
# from scipy.optimize import fsolve
# import matplotlib.pyplot as plt
import requests
import json
# import itertools
from datetime import datetime, timedelta
from docplex.mp.model import Model
import time
# import networkx as nx
import itertools

# Data initialization
providerLocation = "data/ProviderLocationScenario_1.csv"
# providerLocation = "data/ProviderLocation_Hourly.csv"
# droneData = "data/DroneDataScenario_2_DJI.csv"
# deliveryData = "data/Courier1_deliveryData_AvgPower.csv"
# deliveryData = "data/VehicleOnlyDataFile.csv"
deliveryData = "data/VehicleSmallVTOL.csv"

minNumberOfDrones = True
debug = True
batteryReplacement = True
visualization = False
packageDropping = False
gap = 0.15

# Flight components
hoveringBolLoaded = True
hoveringBolUnloaded = True

# maxPermissibleDelay = 1800.0  # in seconds **This is the time window for delivery

packageUnloadingTimeDropping = 30.0 # in seconds. **Need to delare for different drone types

if packageDropping == True:
    packageUnloadingTimeLanding = 0.0 # in seconds. **Need to delare for different drone types
else:
    packageUnloadingTimeLanding = 30.0  # in seconds. **Need to delare for different drone types

packageLoadingTime = 300.0 # in seconds. **Need to delare for different drone types

# For Vehicle
packageLoadingTimeVehicle = 300
packageUnloadingTimeVehicle = 300

air_density = 1.2754  # in kg/m^3
t_0 = datetime.strptime('2/3/2015 1:00', "%m/%d/%Y %H:%M").timestamp()  #converts time into a number. Need to understand the use**
# t_0 = datetime.strptime('2/2/2015 6:00:00 PM', "%m/%d/%Y %I:%M:%S %p").timestamp()
# print("initial time: ", t_0) # 1646665200.0

# Drone Type Used
# vehicle = 'Toyota Prius'
# C_L = 60.0 # hourly labor cost in dollar
# n_d = 5.0 # number of drones per operator
# C_L_d = math.ceil(C_L/n_d) # hourly operator cost per drone

C_E = 0.13 # energy cost per kwh in dollar

# For Ground Vehicle
vehicle = 'Hyundai Accent 2022'

# Model: Toyota Prius (2021)
# Price of New Car: $25735 (Source: fueleconomy.gov)
# Assuming 200000 miles expected lifetime (Source: Consumer Survey)
# Maintenance cost: $528 per year assuming 2400 hours of annual usage (Source: edmunds.com)
# Car driver wage: $16.83 (Source: Bureau of Labor Statics, USA)
# Average vehicle speed: 20 mph (Source: San Francisco Municipal Transportation Agency)

# Model: KIA Niro (2022)
# Price of New Car: $24690 (Source: fueleconomy.gov)
# Assuming 200000 miles expected lifetime (Source: Consumer Survey)
# Maintenance cost: $600 per year assuming 2400 hours of annual usage (Source: edmunds.com)
# Car driver wage: $16.83 (Source: Bureau of Labor Statics, USA)
# Average vehicle speed: 20 mph (Source: San Francisco Municipal Transportation Agency)

# Model: Honda Insight (2022)
# Price of New Car: $25760 (Source: fueleconomy.gov)
# Assuming 200000 miles expected lifetime (Source: Consumer Survey)
# Maintenance cost: $528 per year assuming 2400 hours of annual usage (Source: edmunds.com)
# Car driver wage: $16.83 (Source: Bureau of Labor Statics, USA)
# Average vehicle speed: 20 mph (Source: San Francisco Municipal Transportation Agency)

# Model: Hyundai Accent (2022)
# Price of New Car: $19600 (Source: fueleconomy.gov)
# Assuming 200000 miles expected lifetime (Source: Consumer Survey)
# Maintenance cost: $528 per year assuming 2400 hours of annual usage (Source: edmunds.com)
# Car driver wage: $16.83 (Source: Bureau of Labor Statics, USA)
# Average vehicle speed: 20 mph (Source: San Francisco Municipal Transportation Agency)

# Model: Honda Civic (2015)
# Price of New Car: $20040 (Source: fueleconomy.gov)
# Assuming 200000 miles expected lifetime (Source: Consumer Survey)
# Maintenance cost: $528 per year assuming 2400 hours of annual usage (Source: edmunds.com)
# Car driver wage: $16.83 (Source: Bureau of Labor Statics, USA)
# Average vehicle speed: 20 mph (Source: San Francisco Municipal Transportation Agency)

# Hourly Amortized costs for ground vehicle
C_v = 0.0064
C_mv = 0.22
C_lv = 16.83

if vehicle == 'Toyota Prius 2021':
    MPG = 54  # Considering city driving
elif vehicle == 'Hyundai Accent 2022':
    MPG = 33  # Considering city driving
else:
    MPG = 31  # Considering city driving


# if vehicle == 'Toyota Prius 2021':
#     C_v = 0.0064
#     C_mv = 0.22
#     C_lv = 16.83
#     MPG = 54  # Considering city driving
# elif vehicle == 'KIA Niro 2022':
#     C_v = 0.0062
#     C_mv = 0.25
#     C_lv = 16.83
#     MPG = 51  # Considering city driving
# elif vehicle == 'Honda Insight 2022':
#     C_v = 0.00644
#     C_mv = 0.22
#     C_lv = 16.83
#     MPG = 55  # Considering city driving
# elif vehicle == 'Hyundai Accent 2022':
#     C_v = 0.0049
#     C_mv = 0.25
#     C_lv = 16.83
#     MPG = 33  # Considering city driving
# else:
#     C_v = 0.00501
#     C_mv = 0.22
#     C_lv = 16.83
#     MPG = 31  # Considering city driving

conversionFactor = 33.7
vehicleEnergyConsumptionPerMile = conversionFactor / MPG  # kwh/mile

# Defining the Big M's for the constraints**
M = 1000000000000.0  # a large number (for time constraints)
gta = 0.2  # drone parameter in the Dorling model
grvitational_constant = 9.81  # in meter/s^2
whTokwhConvert = 0.001
wsecToWattHourConvert = 1/3600
hourToSecConvert = 3600.0

# '''Energy data with Package****(May need to declare for different types of drones)

# '''Data for ascending**

AscendHeight = 200 # in ft
durationAscendLoaded = 25.002 # in seconds

# Data for ascending in angle
avgWattAngleAscendLoaded = 0.0 # watts
durationAngleAscendLoaded = 0.0 # in seconds

# End of Data for ascending'''


# '''Data for descending
if packageDropping == True:
    durationDescendLoaded = 35.209  # in seconds.
else:
    durationDescendLoaded = 45.209 # in seconds

# Data for descending in angle
avgWattAngleDescendLoaded = 0.0   # watts
durationAngleDescendLoaded = 0.0  # in seconds

# End of Data for descending'''


# Data for Hovering
if hoveringBolLoaded == True:
    if packageDropping == True:
        durationHoverLoaded = 30.0 # in seconds
    else:
        durationHoverLoaded = 5.0 # in seconds

# End Energy data with Package****(May need to declare for different types of drones)'''


# '''Energy data for Empty Drone. ***(May need to declare for different types of drones)'''

# '''Data for ascending**
if packageDropping == True:
    durationAscendUnloaded = 20.002 # in seconds ToDo: make sure to have this ascending time to be 5 seconds less than actual ascending time when package dropping
else:
    durationAscendUnloaded = 25.002  # in seconds
avgWattAngleAscendUnloaded = 0.0  # watts
durationAngleAscendUnloaded = 0.0 # in seconds

# End of Data for ascending'''


# '''Data for descending
durationDescendUnloaded = 45.209 # in seconds
avgWattAngleDescendUnloaded = 0.0 # watts
durationAngleDescendUnloaded = 0.0 # in seconds
# End of Data for descending'''

# Data for Hovering
if hoveringBolUnloaded == True:
    avgWattHoverUnloaded = 1039.254162 # watts
    durationHoverUnloaded = 5.0 # in seconds

# End of Energy data for Empty Drone. ***(May need to declare for different types of drones)'''


# Class to define the delivery locations
class Node:
    def __init__(self, deliveryID, readyTime, lat, lon, payload, distanceFromDepot, timeFromDepot, timeReturnToDepot):
        self.deliveryID = deliveryID
        self.readyTime = readyTime
        self.lat = lat
        self.lon = lon
        self.packageWeight = payload
        # self.avgWattAngleAscendLoaded = 0.0
        # self.avgWattAngleDescendLoaded = 0.0
        self.distanceFromDepot = distanceFromDepot
        self.timeFromDepot = timeFromDepot  # store for each drone
        self.timeReturnToDepot = timeReturnToDepot  # store for each drone
        self.earliestServiceTime = 0.0
        self.maxDelayedServiceTime = 0.0
        self.energyconsumptionDronesArrival = []  # store for each drone, indexed by drone
        self.energyconsumptionDronesReturn = []  # store for each drone, indexed by drone

# Class to define type of drones (Need to modify for type of drones)


class Drone:
    def __init__(self, dType, numRotors, droneSpeedLoaded, droneSpeedUnloaded, payloadCap, bodyMass, batteryMass, initCharge,
                 minChargeReq, flyingTimePerMile, batReplaceTime, c_d, c_m, c_bat, avgWattAscendLoaded, avgWattDescendLoaded,
                 avgWattHoverLoaded, avgWattFlightLoaded, avgWattAscendUnLoaded, avgWattDescendUnLoaded, avgWattHoverUnLoaded,
                 avgWattFlightUnLoaded):
        self.dType = dType
        self.numRotors = numRotors
        self.optimalSpeedLoaded = droneSpeedLoaded
        self.optimalSpeedUnloaded = droneSpeedUnloaded
        self.capacity = payloadCap
        self.bodyMass = bodyMass
        self.batteryMass = batteryMass
        self.initCharge = initCharge
        self.minChargeReq = minChargeReq
        self.allowableEnergy = initCharge - minChargeReq
        self.batReplaceTime = batReplaceTime
        self.flyingTimePerMile = flyingTimePerMile
        self.avgWattAscendLoaded = avgWattAscendLoaded
        self.avgWattDescendLoaded = avgWattDescendLoaded
        self.avgWattHoverLoaded = avgWattHoverLoaded
        self.avgWattFlightLoaded = avgWattFlightLoaded
        self.avgWattAscendUnLoaded = avgWattAscendUnLoaded
        self.avgWattDescendUnLoaded = avgWattDescendUnLoaded
        self.avgWattHoverUnLoaded = avgWattHoverUnLoaded
        self.avgWattFlightUnLoaded = avgWattFlightUnLoaded
        self.C_d = c_d
        self.C_d_M = c_m
        self.F_c = c_m + c_d + 480
        self.C_bat = c_bat
        self.M_g = 2.5*initCharge  #(for battery replacement constraints)**
        self.M_l = -initCharge
        self.M_u = initCharge
        self.Q_l = -initCharge
        self.Q_u = initCharge

# Ideally the optimal speed will be a dictionary where the keys are delivery locations and values are the optimal speed for that location.
# for now just using a single optimal value for all payloads


class NestedDict(dict):
    def __missing__(self, x):
        self[x] = NestedDict()
        return self[x]

# Function to create a list of objects (Drone objects for number of type of drones,
#  Node object to create each delivery location,
#  and the adress for source location. Here coding for provider location can be done outside )


def CreateListOfObjects(providerLocation, deliveryData):
    # listOfDrones = []
    listOfDeliveryLocs = []

    provider_df = pd.read_csv(providerLocation)
    providerLat = provider_df['lat'][0]
    providerLon = provider_df['long'][0]

    # drone_df = pd.read_csv(droneData)
    #
    # for row in range(len(drone_df)):
    #     listOfDrones.append(
    #         Drone(drone_df['type'][row], drone_df['numRotors'][row], drone_df['droneSpeedLoaded'][row],
    #               drone_df['droneSpeedUnloaded'][row], drone_df['payloadCap'][row], drone_df['bodyMass'][row],
    #               drone_df['batteryMass'][row], drone_df['initBatCharge'][row], drone_df['minChargeReq'][row],
    #               drone_df['flyingTimePerMile'][row], drone_df['batReplaceTime'][row], drone_df['C_d'][row],
    #               drone_df['C_M'][row],
    #               drone_df['C_bat'][row], drone_df['AvgWattAscendLoaded'][row], drone_df['AvgWattDescendLoaded'][row],
    #               drone_df['AvgWattHoverLoaded'][row], drone_df['AvgWattFlightLoaded'][row],
    #               drone_df['AvgWattAscendUnLoaded'][row],
    #               drone_df['AvgWattDescendUnLoaded'][row], drone_df['AvgWattHoverUnLoaded'][row],
    #               drone_df['AvgWattFlightUnLoaded'][row]))

    delivery_df = pd.read_csv(deliveryData)

    for row in range(len(delivery_df)):
        foodReadyTime = datetime.strptime(delivery_df['food_ready_time'][row],
                                          "%m/%d/%Y %H:%M").timestamp()  # "%m/%d/%Y %I:%M:%S %p"
        listOfDeliveryLocs.append(Node(delivery_df['delivery_id'][row], foodReadyTime, delivery_df['dropoff_lat'][row],
                                       delivery_df['dropoff_long'][row], delivery_df['PackageWeight'][row], delivery_df['Road_Distance'][row], delivery_df['Time_From_Depot'][row], delivery_df['Time_ReturnTo_Depot'][row]))

    return listOfDeliveryLocs, providerLat, providerLon


# Function for calculating the distance for each location pair
def computeDistOfEachPair(srclat, srclon, destlat, destlon):
    R = 3958.8  # in mile
    rad_srcLat = radians(abs(srclat))
    rad_srcLon = radians(abs(srclon))
    rad_destLat = radians(abs(destlat))
    rad_destLon = radians(abs(destlon))
    dLon = rad_destLon - rad_srcLon
    dLat = rad_destLat - rad_srcLat
    a = sin(dLat / 2) ** 2 + cos(rad_srcLat) * cos(rad_destLat) * sin(dLon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distOfEachPair = R * c
    return distOfEachPair  # in mile


def computeRoadDistanceFromDepot(srclat, srclon, destlat, destlon):
    r = requests.get(
        f"""http://router.project-osrm.org/route/v1/car/{srclon},{srclat};{destlon},{destlat}?overview=false""")
    route = json.loads(r.content)["routes"][0]
    drivingDistance = route["distance"] * 0.000621371  # one-way distance in mile
    travelTimeAPI = (route["duration"])  # single-trip travel time in seconds
    return drivingDistance, travelTimeAPI

def createListOfLocsWithDummy(listOfDeliveryLocs, providerLat, providerLon):
    listOfLocsDummySink = []
    if listOfDeliveryLocs:
        listOfLocsDummySink.append(Node(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        # listOfLocsDummySink[-1].distanceFromDepot = 0.0
        listOfLocsDummySink[-1].lat = providerLat
        listOfLocsDummySink[-1].lon = providerLon
        listOfLocsDummySink[-1].earliestServiceTime = 0.0
        listOfLocsDummySink[-1].maxDelayedServiceTime = 0.0
        # listOfLocsDummySink[-1].timeFromDepot = 0
        # listOfLocsDummySink[-1].timeReturnToDepot = 0
        listOfLocsDummySink[-1].energyconsumptionDronesArrival = 0
        listOfLocsDummySink[-1].energyconsumptionDronesReturn = 0

        listOfLocsDummySink.extend(listOfDeliveryLocs)

        listOfLocsDummySink.append(Node(-99, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        # listOfLocsDummySink[-1].distanceFromDepot = 0.0
        listOfLocsDummySink[-1].lat = providerLat
        listOfLocsDummySink[-1].lon = providerLon
        listOfLocsDummySink[-1].earliestServiceTime = 0.0
        listOfLocsDummySink[-1].maxDelayedServiceTime = 0.0
        # listOfLocsDummySink[-1].timeFromDepot = 0
        # listOfLocsDummySink[-1].timeReturnToDepot = 0
        listOfLocsDummySink[-1].energyconsumptionDronesArrival = 0
        listOfLocsDummySink[-1].energyconsumptionDronesReturn = 0
    return listOfLocsDummySink


# Calculating distance and Time matrix for circular delivery of ground vehicles


def distanceAndTimeMatrix(listOfLocsDummySink):
    distanceMatrix = {}
    timeMatrix = {}
    energyMatrix = {}
    for i in listOfLocsDummySink[:-1]:
        sourceLat = i.lat
        sourceLon = i.lon
        for j in listOfLocsDummySink[170:]:
            if ((i.deliveryID, j.deliveryID) or (j.deliveryID, i.deliveryID)) in distanceMatrix:
                continue
            destLat = j.lat
            destLon = j.lon
            if i == j:
                distanceMatrix[(i.deliveryID, j.deliveryID)], timeMatrix[(i.deliveryID, j.deliveryID)] = 0, 0
                energyMatrix[(i.deliveryID, j.deliveryID)] = 0
            else:
                distanceMatrix[(i.deliveryID, j.deliveryID)], timeMatrix[
                    (i.deliveryID, j.deliveryID)] = computeRoadDistanceFromDepot(sourceLat, sourceLon, destLat, destLon)
                energyMatrix[(i.deliveryID, j.deliveryID)] = distanceMatrix[(
                i.deliveryID, j.deliveryID)] * vehicleEnergyConsumptionPerMile
                distanceMatrix[(j.deliveryID, i.deliveryID)] = distanceMatrix[(i.deliveryID, j.deliveryID)]
                timeMatrix[(j.deliveryID, i.deliveryID)] = timeMatrix[(i.deliveryID, j.deliveryID)]
                energyMatrix[(j.deliveryID, i.deliveryID)] = energyMatrix[(i.deliveryID, j.deliveryID)]
    return distanceMatrix, energyMatrix, timeMatrix


listOfDeliveryLocs, providerLat, providerLon = CreateListOfObjects(providerLocation, deliveryData)
listOfLocsDummySink = createListOfLocsWithDummy(listOfDeliveryLocs, providerLat, providerLon)
distanceMatrix, energyMatrix, timeMatrix = distanceAndTimeMatrix(listOfLocsDummySink)
# Create a DataFrame from the dictionary
# distancedf = pd.DataFrame(list(distanceMatrix.items()), columns=['Pair', 'Distance'])
# timedf = pd.DataFrame(list(timeMatrix.items()), columns=['Pair', 'Time'])
# energydf = pd.DataFrame(list(energyMatrix.items()), columns=['Pair', 'Energy'])
# distancedf.to_csv('Distance_Matrix.csv')
# timedf.to_csv('Time_Matrix.csv')
# energydf.to_csv('Energy_Matrix.csv')
print(distanceMatrix)