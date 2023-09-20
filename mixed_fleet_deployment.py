'''
Author: Tanveer Hossain Bhuiyan, Ph.D. Contact: tanveer.bhuiyan@utsa.edu
Assistant Professor, The University of Texas at San Antonio. This code implements the mixed drone ground vehicle fleet routing
'''

import pandas as pd
import math
from math import sin, cos, sqrt, atan2, radians
from machineLearningModel import ascend, descend, hover, forwardFlight

import requests
import json
# import itertools
from datetime import datetime, timedelta
from docplex.mp.model import Model
import time

import itertools

# Data initialization
providerLocation = "ProviderLocationScenario_1.csv"

droneData = "DroneDataScenario_1.csv"


deliveryData = "Input_data.csv"

OnlyDrone = False
minNumberOfDrones = True
debug = True
batteryReplacement = True
visualization = False
packageDropping = False
gap = 0.15


hoveringBolLoaded = True
hoveringBolUnloaded = True

maxPermissibleDelay = 1200.0

packageUnloadingTimeDropping = 30.0

if packageDropping == True:
    packageUnloadingTimeLanding = 0.0
else:
    packageUnloadingTimeLanding = 30.0

packageLoadingTime = 300.0

packageLoadingTimeVehicle = 300
packageUnloadingTimeVehicle = 120

air_density = 1.2754
t_0 = datetime.strptime('2/3/2015 1:00', "%m/%d/%Y %H:%M").timestamp()


droneType = 'Small VTOL'
C_L = 60.0
n_d = 5.0
C_L_d = math.ceil(C_L/n_d)

C_E = 0.13


vehicle = 'Hyundai Accent 2022'


if vehicle == 'Toyota Prius 2021':
    C_v = 0.0064
    C_mv = 0.22
    C_lv = 16.83
    MPG = 54
elif vehicle == 'KIA Niro 2022':
    C_v = 0.0062
    C_mv = 0.25
    C_lv = 16.83
    MPG = 51
elif vehicle == 'Honda Insight 2022':
    C_v = 0.00644
    C_mv = 0.22
    C_lv = 16.83
    MPG = 55
elif vehicle == 'Hyundai Accent 2022':
    C_v = 0.0049
    C_mv = 0.25
    C_lv = 16.83
    MPG = 33
else:
    C_v = 0.00501
    C_mv = 0.22
    C_lv = 16.83
    MPG = 31

conversionFactor = 33.7
vehicleEnergyConsumptionPerMile = conversionFactor / MPG

M = 10000.0
gta = 0.2
grvitational_constant = 9.81
whTokwhConvert = 0.001
wsecToWattHourConvert = 1/3600
hourToSecConvert = 3600.0



AscendHeight = 200
durationAscendLoaded = 25.002

avgWattAngleAscendLoaded = 0.0
durationAngleAscendLoaded = 0.0


if packageDropping == True:
    durationDescendLoaded = 35.209
else:
    durationDescendLoaded = 45.209


avgWattAngleDescendLoaded = 0.0
durationAngleDescendLoaded = 0.0


if hoveringBolLoaded == True:
    if packageDropping == True:
        durationHoverLoaded = 30.0
    else:
        durationHoverLoaded = 5.0

if packageDropping == True:
    durationAscendUnloaded = 20.002
else:
    durationAscendUnloaded = 25.002
avgWattAngleAscendUnloaded = 0.0
durationAngleAscendUnloaded = 0.0


durationDescendUnloaded = 45.209
avgWattAngleDescendUnloaded = 0.0
durationAngleDescendUnloaded = 0.0


if hoveringBolUnloaded == True:
    avgWattHoverUnloaded = 1039.254162
    durationHoverUnloaded = 5.0

class Node:
    def __init__(self, deliveryID, readyTime, lat, lon, payload):
        self.deliveryID = deliveryID
        self.readyTime = readyTime
        self.lat = lat
        self.lon = lon
        self.packageWeight = payload
        self.avgWattAngleAscendLoaded = 0.0
        self.avgWattAngleDescendLoaded = 0.0
        self.distanceFromDepot = 0.0
        self.timeFromDepot = []
        self.timeReturnToDepot = []
        self.earliestServiceTime = 0.0
        self.maxDelayedServiceTime = 0.0
        self.energyconsumptionDronesArrival = []
        self.energyconsumptionDronesReturn = []



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
        self.M_g = 2.5*initCharge
        self.M_l = -initCharge
        self.M_u = initCharge
        self.Q_l = -initCharge
        self.Q_u = initCharge


def CreateListOfObjects(providerLocation, droneData, deliveryData):
    listOfDrones = []
    listOfDeliveryLocs = []

    provider_df = pd.read_csv(providerLocation)
    providerLat = provider_df['lat'][0]
    providerLon = provider_df['long'][0]

    drone_df = pd.read_csv(droneData)

    for row in range(len(drone_df)):
        listOfDrones.append(
            Drone(drone_df['type'][row], drone_df['numRotors'][row], drone_df['droneSpeedLoaded'][row],
                  drone_df['droneSpeedUnloaded'][row], drone_df['payloadCap'][row], drone_df['bodyMass'][row],
                  drone_df['batteryMass'][row], drone_df['initBatCharge'][row], drone_df['minChargeReq'][row],
                  drone_df['flyingTimePerMile'][row], drone_df['batReplaceTime'][row], drone_df['C_d'][row],
                  drone_df['C_M'][row],
                  drone_df['C_bat'][row], drone_df['AvgWattAscendLoaded'][row], drone_df['AvgWattDescendLoaded'][row],
                  drone_df['AvgWattHoverLoaded'][row], drone_df['AvgWattFlightLoaded'][row],
                  drone_df['AvgWattAscendUnLoaded'][row],
                  drone_df['AvgWattDescendUnLoaded'][row], drone_df['AvgWattHoverUnLoaded'][row],
                  drone_df['AvgWattFlightUnLoaded'][row]))

    delivery_df = pd.read_csv(deliveryData)

    for row in range(len(delivery_df)):
        foodReadyTime = datetime.strptime(delivery_df['food_ready_time'][row],
                                          "%m/%d/%Y %H:%M").timestamp()
        listOfDeliveryLocs.append(Node(delivery_df['delivery_id'][row], foodReadyTime, delivery_df['dropoff_lat'][row],
                                       delivery_df['dropoff_long'][row], delivery_df['PackageWeight'][row]))

    return listOfDrones, listOfDeliveryLocs, providerLat, providerLon


def computeDistOfEachPair(srclat, srclon, destlat, destlon):
    R = 3958.8
    rad_srcLat = radians(abs(srclat))
    rad_srcLon = radians(abs(srclon))
    rad_destLat = radians(abs(destlat))
    rad_destLon = radians(abs(destlon))
    dLon = rad_destLon - rad_srcLon
    dLat = rad_destLat - rad_srcLat
    a = sin(dLat / 2) ** 2 + cos(rad_srcLat) * cos(rad_destLat) * sin(dLon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    distOfEachPair = R * c
    return distOfEachPair


def computeRoadDistanceFromDepot(srclat, srclon, destlat, destlon):
    r = requests.get(
        f"""http://router.project-osrm.org/route/v1/car/{srclon},{srclat};{destlon},{destlat}?overview=false""")
    route = json.loads(r.content)["routes"][0]
    drivingDistance = route["distance"] * 0.000621371
    travelTimeAPI = (route["duration"])
    return drivingDistance, travelTimeAPI

def dataInterpolation(maxLoad, energyLoeaded, energyUnloaded, packageWeight):
    calculatedEnergy = energyUnloaded + ((energyLoeaded - energyUnloaded)/maxLoad)*packageWeight
    return calculatedEnergy


def computeAscendDescendHoverEnergyLoaded(avgWattAscendLoaded, avgWattAngleAscendLoaded, avgWattDescendLoaded, avgWattAngleDescendLoaded, avgWattHoverLoaded):
    ascendEnergyLoaded = avgWattAscendLoaded*durationAscendLoaded*wsecToWattHourConvert*whTokwhConvert
    angleAscendEnergyLoaded = avgWattAngleAscendLoaded * durationAngleAscendLoaded * wsecToWattHourConvert * whTokwhConvert
    descendEnergyLoaded = avgWattDescendLoaded * durationDescendLoaded * wsecToWattHourConvert * whTokwhConvert
    angleDescendEnergyLoaded = avgWattAngleDescendLoaded * durationAngleDescendLoaded * wsecToWattHourConvert * whTokwhConvert
    if hoveringBolLoaded == True:
        hoverEnergyLoaded = avgWattHoverLoaded * durationHoverLoaded * wsecToWattHourConvert * whTokwhConvert
        totalAscendDescendHoverLoaded = ascendEnergyLoaded + angleAscendEnergyLoaded + descendEnergyLoaded + angleDescendEnergyLoaded + hoverEnergyLoaded
    else:
        totalAscendDescendHoverLoaded = ascendEnergyLoaded + angleAscendEnergyLoaded + descendEnergyLoaded + angleDescendEnergyLoaded
    return totalAscendDescendHoverLoaded



def computeAscendDescendHoverEnergyUnloaded(avgWattAscendUnloaded, avgWattAngleAscendUnloaded, avgWattDescendUnloaded, avgWattAngleDescendUnloaded, avgWattHoverUnloaded):
    ascendEnergyUnloaded = avgWattAscendUnloaded*durationAscendUnloaded*wsecToWattHourConvert*whTokwhConvert
    angleAscendEnergyUnloaded = avgWattAngleAscendUnloaded*durationAngleAscendUnloaded*wsecToWattHourConvert*whTokwhConvert
    descendEnergyUnloaded = avgWattDescendUnloaded*durationDescendUnloaded*wsecToWattHourConvert * whTokwhConvert
    angleDescendEnergyUnloaded = avgWattAngleDescendUnloaded*durationAngleDescendUnloaded*wsecToWattHourConvert*whTokwhConvert
    if hoveringBolUnloaded == True:
        hoverEnergyUnloaded = avgWattHoverUnloaded * durationHoverUnloaded * wsecToWattHourConvert * whTokwhConvert
        totalAscendDescendHoverUnloaded = ascendEnergyUnloaded + angleAscendEnergyUnloaded + descendEnergyUnloaded + angleDescendEnergyUnloaded + hoverEnergyUnloaded
    else:
        totalAscendDescendHoverUnloaded = ascendEnergyUnloaded + angleAscendEnergyUnloaded + descendEnergyUnloaded + angleDescendEnergyUnloaded
    return totalAscendDescendHoverUnloaded


def computeParams(listOfDrones, listOfDeliveryLocs, providerLat, providerLon, maxPermissibleDelay):

    for i in range(len(listOfDeliveryLocs)):
        packageWeight = listOfDeliveryLocs[i].packageWeight
        listOfDeliveryLocs[i].earliestServiceTime = listOfDeliveryLocs[i].readyTime
        listOfDeliveryLocs[i].maxDelayedServiceTime = listOfDeliveryLocs[i].earliestServiceTime + maxPermissibleDelay
        destLat = listOfDeliveryLocs[i].lat
        destLon = listOfDeliveryLocs[i].lon
        distanceFromDepot = computeDistOfEachPair(providerLat, providerLon, destLat, destLon)
        listOfDeliveryLocs[i].distanceFromDepot = distanceFromDepot


        for d in range(len(listOfDrones)):

            flyingTimePerMile = listOfDrones[d].flyingTimePerMile
            maxLoad = listOfDrones[d].capacity
            if listOfDrones[d].dType == 'DJI':
                avgWattAscendLoaded = ascend(packageWeight)
                avgWattDescendLoaded = descend(packageWeight)
                avgWattHoverLoaded = hover(packageWeight)
                avgWattFlightLoaded = forwardFlight(packageWeight)
                avgWattAscendUnLoaded = ascend(0)
                avgWattDescendUnLoaded = descend(0)
                avgWattHoverUnLoaded = hover(0)
                avgWattFlightUnLoaded = forwardFlight(0)
            else:
                avgWattAscendLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattAscendLoaded,
                                                        listOfDrones[d].avgWattAscendUnLoaded, packageWeight)
                avgWattDescendLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattDescendLoaded,
                                                         listOfDrones[d].avgWattDescendUnLoaded, packageWeight)
                avgWattHoverLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattHoverLoaded,
                                                       listOfDrones[d].avgWattHoverUnLoaded, packageWeight)
                avgWattFlightLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattFlightLoaded,
                                                        listOfDrones[d].avgWattFlightUnLoaded, packageWeight)
                avgWattAscendUnLoaded = listOfDrones[d].avgWattAscendUnLoaded
                avgWattDescendUnLoaded = listOfDrones[d].avgWattDescendUnLoaded
                avgWattHoverUnLoaded = listOfDrones[d].avgWattHoverUnLoaded
                avgWattFlightUnLoaded = listOfDrones[d].avgWattFlightUnLoaded

            flyingTimeFromDepot = (distanceFromDepot / listOfDrones[
                d].optimalSpeedLoaded) * hourToSecConvert
            timeFromDepot = flyingTimeFromDepot + durationAscendLoaded + durationAngleAscendLoaded + durationDescendLoaded + durationAngleDescendLoaded + durationHoverLoaded
            listOfDeliveryLocs[i].timeFromDepot.append(timeFromDepot)

            flyingTimeReturnToDepot = (distanceFromDepot / listOfDrones[
                d].optimalSpeedUnloaded) * hourToSecConvert
            timeReturnToDepot = flyingTimeReturnToDepot + durationAscendUnloaded + durationAngleAscendUnloaded + durationDescendUnloaded + durationAngleDescendUnloaded + durationHoverUnloaded
            listOfDeliveryLocs[i].timeReturnToDepot.append(timeReturnToDepot)

            forwardFlightWhPerMileLoaded = (avgWattFlightLoaded * flyingTimePerMile) * wsecToWattHourConvert
            energyPerMileLoaded = forwardFlightWhPerMileLoaded * whTokwhConvert
            energyConsumptionFlightLoaded = energyPerMileLoaded * distanceFromDepot

            forwardFlightWhPerMileUnloaded = (avgWattFlightUnLoaded * flyingTimePerMile) * wsecToWattHourConvert
            energyPerMileUnLoaded = forwardFlightWhPerMileUnloaded * whTokwhConvert
            energyConsumptionFlightUnLoaded = energyPerMileUnLoaded * distanceFromDepot


            ascendDescendHoverEnergyLoaded = computeAscendDescendHoverEnergyLoaded(avgWattAscendLoaded, listOfDeliveryLocs[i].avgWattAngleAscendLoaded, avgWattDescendLoaded, listOfDeliveryLocs[i].avgWattAngleDescendLoaded, avgWattHoverLoaded)


            ascendDescendHoverEnergyUnloaded = computeAscendDescendHoverEnergyUnloaded(avgWattAscendUnLoaded, avgWattAngleAscendUnloaded, avgWattDescendUnLoaded, avgWattAngleDescendUnloaded, avgWattHoverUnLoaded)

            totalEnergyConsumptionLoaded = energyConsumptionFlightLoaded + ascendDescendHoverEnergyLoaded
            totalEnergyConsumptionUnloaded = energyConsumptionFlightUnLoaded + ascendDescendHoverEnergyUnloaded
            listOfDeliveryLocs[i].energyconsumptionDronesArrival.append(totalEnergyConsumptionLoaded)
            listOfDeliveryLocs[i].energyconsumptionDronesReturn.append(totalEnergyConsumptionUnloaded)





def lisOfObjects(listOfDrones, listOfDeliveryLocs):
    eligible = {}
    for d in listOfDrones:
        eligible[d] = []

    index = 0
    for d in listOfDrones:
        for i in listOfDeliveryLocs:
            if (i.packageWeight <= d.capacity) and ((i.energyconsumptionDronesArrival[index] + i.energyconsumptionDronesReturn[index]) <= d.allowableEnergy):
                eligible[d].append(i)
        index += 1

    if droneType == 'Tarot':
        listOfDeliveryLocsForTarot = eligible[listOfDrones[0]]
        listOfDeliveryLocsForVehicle = [i for i in listOfDeliveryLocs if i not in listOfDeliveryLocsForTarot]
        listOfDeliveryLocs = listOfDeliveryLocsForTarot
        listOfDrones = [listOfDrones[0]]
    elif droneType == 'DJI':
        listOfDeliveryLocsForDJI = eligible[listOfDrones[1]]
        listOfDeliveryLocsForVehicle = [i for i in listOfDeliveryLocs if i not in listOfDeliveryLocsForDJI]
        listOfDeliveryLocs = listOfDeliveryLocsForDJI
        listOfDrones = [listOfDrones[1]]
    elif droneType == 'Small VTOL':
        listOfDeliveryLocsForSmallVTOL = eligible[listOfDrones[2]]
        listOfDeliveryLocsForVehicle = [i for i in listOfDeliveryLocs if i not in listOfDeliveryLocsForSmallVTOL]
        listOfDeliveryLocs = listOfDeliveryLocsForSmallVTOL
        listOfDrones = [listOfDrones[2]]
    else:
        listOfDeliveryLocsForVTOL = eligible[listOfDrones[3]]
        listOfDeliveryLocsForVehicle = [i for i in listOfDeliveryLocs if i not in listOfDeliveryLocsForVTOL]
        listOfDeliveryLocs = listOfDeliveryLocsForVTOL
        listOfDrones = [listOfDrones[3]]
    return listOfDrones, listOfDeliveryLocs, listOfDeliveryLocsForVehicle




def computeParamsForDrone(listOfDrones, listOfDeliveryLocs, providerLat, providerLon, maxPermissibleDelay):
    for i in range(len(listOfDeliveryLocs)):
        packageWeight = listOfDeliveryLocs[i].packageWeight
        listOfDeliveryLocs[i].earliestServiceTime = listOfDeliveryLocs[i].readyTime
        listOfDeliveryLocs[i].maxDelayedServiceTime = listOfDeliveryLocs[i].earliestServiceTime + maxPermissibleDelay
        destLat = listOfDeliveryLocs[i].lat
        destLon = listOfDeliveryLocs[i].lon
        distanceFromDepot = computeDistOfEachPair(providerLat, providerLon, destLat, destLon)
        listOfDeliveryLocs[i].distanceFromDepot = distanceFromDepot


        for d in range(len(listOfDrones)):
            flyingTimePerMile = listOfDrones[d].flyingTimePerMile
            maxLoad = listOfDrones[d].capacity
            if listOfDrones[d].dType == 'DJI':
                avgWattAscendLoaded = ascend(packageWeight)
                avgWattDescendLoaded = descend(packageWeight)
                avgWattHoverLoaded = hover(packageWeight)
                avgWattFlightLoaded = forwardFlight(packageWeight)
                avgWattAscendUnLoaded = ascend(0)
                avgWattDescendUnLoaded = descend(0)
                avgWattHoverUnLoaded = hover(0)
                avgWattFlightUnLoaded = forwardFlight(0)
            else:
                avgWattAscendLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattAscendLoaded,
                                                        listOfDrones[d].avgWattAscendUnLoaded, packageWeight)
                avgWattDescendLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattDescendLoaded,
                                                         listOfDrones[d].avgWattDescendUnLoaded, packageWeight)
                avgWattHoverLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattHoverLoaded,
                                                       listOfDrones[d].avgWattHoverUnLoaded, packageWeight)
                avgWattFlightLoaded = dataInterpolation(maxLoad, listOfDrones[d].avgWattFlightLoaded,
                                                        listOfDrones[d].avgWattFlightUnLoaded, packageWeight)
                avgWattAscendUnLoaded = listOfDrones[d].avgWattAscendUnLoaded
                avgWattDescendUnLoaded = listOfDrones[d].avgWattDescendUnLoaded
                avgWattHoverUnLoaded = listOfDrones[d].avgWattHoverUnLoaded
                avgWattFlightUnLoaded = listOfDrones[d].avgWattFlightUnLoaded

            flyingTimeFromDepot = (distanceFromDepot / listOfDrones[
                d].optimalSpeedLoaded) * hourToSecConvert
            timeFromDepot = flyingTimeFromDepot + durationAscendLoaded + durationAngleAscendLoaded + durationDescendLoaded + durationAngleDescendLoaded + durationHoverLoaded
            listOfDeliveryLocs[i].timeFromDepot = timeFromDepot

            flyingTimeReturnToDepot = (distanceFromDepot / listOfDrones[
                d].optimalSpeedUnloaded) * hourToSecConvert
            timeReturnToDepot = flyingTimeReturnToDepot + durationAscendUnloaded + durationAngleAscendUnloaded + durationDescendUnloaded + durationAngleDescendUnloaded + durationHoverUnloaded
            listOfDeliveryLocs[i].timeReturnToDepot = timeReturnToDepot

            forwardFlightWhPerMileLoaded = (avgWattFlightLoaded * flyingTimePerMile) * wsecToWattHourConvert
            energyPerMileLoaded = forwardFlightWhPerMileLoaded * whTokwhConvert
            energyConsumptionFlightLoaded = energyPerMileLoaded * distanceFromDepot

            forwardFlightWhPerMileUnloaded = (avgWattFlightUnLoaded * flyingTimePerMile) * wsecToWattHourConvert
            energyPerMileUnLoaded = forwardFlightWhPerMileUnloaded * whTokwhConvert
            energyConsumptionFlightUnLoaded = energyPerMileUnLoaded * distanceFromDepot


            ascendDescendHoverEnergyLoaded = computeAscendDescendHoverEnergyLoaded(avgWattAscendLoaded, listOfDeliveryLocs[i].avgWattAngleAscendLoaded, avgWattDescendLoaded, listOfDeliveryLocs[i].avgWattAngleDescendLoaded, avgWattHoverLoaded)


            ascendDescendHoverEnergyUnloaded = computeAscendDescendHoverEnergyUnloaded(avgWattAscendUnLoaded, avgWattAngleAscendUnloaded, avgWattDescendUnLoaded, avgWattAngleDescendUnloaded, avgWattHoverUnLoaded)

            totalEnergyConsumptionLoaded = energyConsumptionFlightLoaded + ascendDescendHoverEnergyLoaded
            totalEnergyConsumptionUnloaded = energyConsumptionFlightUnLoaded + ascendDescendHoverEnergyUnloaded
            listOfDeliveryLocs[i].energyconsumptionDronesArrival = totalEnergyConsumptionLoaded
            listOfDeliveryLocs[i].energyconsumptionDronesReturn = totalEnergyConsumptionUnloaded


def computeParamsForVehicle(listOfDeliveryLocs, providerLat, providerLon, maxPermissibleDelay):

    for i in range(len(listOfDeliveryLocs)):
        listOfDeliveryLocs[i].earliestServiceTime = listOfDeliveryLocs[i].readyTime
        listOfDeliveryLocs[i].maxDelayedServiceTime = listOfDeliveryLocs[i].earliestServiceTime + maxPermissibleDelay
        destLat = listOfDeliveryLocs[i].lat
        destLon = listOfDeliveryLocs[i].lon
        distanceFromDepot, timeFromDepot = computeRoadDistanceFromDepot(providerLat, providerLon, destLat, destLon)
        listOfDeliveryLocs[i].distanceFromDepot = distanceFromDepot
        listOfDeliveryLocs[i].timeFromDepot = timeFromDepot
        listOfDeliveryLocs[i].timeReturnToDepot = timeFromDepot
        totalEnergyConsumptionLoaded = vehicleEnergyConsumptionPerMile*distanceFromDepot
        totalEnergyConsumptionUnloaded = vehicleEnergyConsumptionPerMile*distanceFromDepot
        listOfDeliveryLocs[i].energyconsumptionDronesArrival = totalEnergyConsumptionLoaded
        listOfDeliveryLocs[i].energyconsumptionDronesReturn = totalEnergyConsumptionUnloaded


def createListOfLocsWithDummy(listOfDeliveryLocs):
    listOfLocsDummySink = []
    if listOfDeliveryLocs:
        listOfLocsDummySink.append(Node(0, 0.0, 0.0, 0.0, 0.0))
        listOfLocsDummySink[-1].distanceFromDepot = 0.0
        listOfLocsDummySink[-1].earliestServiceTime = 0.0
        listOfLocsDummySink[-1].maxDelayedServiceTime = 0.0
        listOfLocsDummySink[-1].timeFromDepot = 0
        listOfLocsDummySink[-1].timeReturnToDepot = 0
        listOfLocsDummySink[-1].energyconsumptionDronesArrival = 0
        listOfLocsDummySink[-1].energyconsumptionDronesReturn = 0

        listOfLocsDummySink.extend(listOfDeliveryLocs)

        listOfLocsDummySink.append(Node(-99, 0.0, 0.0, 0.0, 0.0))
        listOfLocsDummySink[-1].distanceFromDepot = 0.0
        listOfLocsDummySink[-1].earliestServiceTime = 0.0
        listOfLocsDummySink[-1].maxDelayedServiceTime = 0.0
        listOfLocsDummySink[-1].timeFromDepot = 0
        listOfLocsDummySink[-1].timeReturnToDepot = 0
        listOfLocsDummySink[-1].energyconsumptionDronesArrival = 0
        listOfLocsDummySink[-1].energyconsumptionDronesReturn = 0
    return listOfLocsDummySink


def computeRoutingCost(listOfLocsDummySinkForDrone, listOfLocsDummySinkForVehicle, minNumberOfDrones, penaltyOutgoingArcs):
    routingCost = {}
    for i in range(len(listOfLocsDummySinkForDrone)):
        for j in range(len(listOfLocsDummySinkForDrone)):
            if listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID:
                routingCost[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID] = listOfLocsDummySinkForDrone[i].energyconsumptionDronesReturn+ listOfLocsDummySinkForDrone[j].energyconsumptionDronesArrival

    if listOfLocsDummySinkForVehicle:
        for i in range(len(listOfLocsDummySinkForVehicle)):
            for j in range(len(listOfLocsDummySinkForVehicle)):
                if listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[j].deliveryID:
                    routingCost[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID] = listOfLocsDummySinkForVehicle[i].energyconsumptionDronesReturn + listOfLocsDummySinkForVehicle[j].energyconsumptionDronesArrival
    if minNumberOfDrones == True:
        for key, val in routingCost.items():
            if key[0] == 0:
                routingCost[key] = val + penaltyOutgoingArcs

    return routingCost


def computeTotalEnergyConsumed(zVal, numberOfDronesUsed, routingCost, penaltyOutgoingArcs):
    totalEnergy = 0.0
    for k, v in zVal.items():
        if v > 0.5:
            totalEnergy = totalEnergy + routingCost[k]
    totalEnergy = totalEnergy - numberOfDronesUsed * penaltyOutgoingArcs
    return totalEnergy


def computeMaxEnergyToServeCustomerLoc(listOfDeliveryLocs):
    maxEnergyArrivalAllLocs = []
    maxEnergyReturnAllLocs = []
    for i in range(len(listOfDeliveryLocs)):
        maxEnergyArrivalAllLocs.append(max(listOfDeliveryLocs[i].energyconsumptionDronesArrival))
        maxEnergyReturnAllLocs.append(max(listOfDeliveryLocs[i].energyconsumptionDronesReturn))
    maxTotalEnergyRequired = max(maxEnergyArrivalAllLocs) + max(maxEnergyReturnAllLocs)
    return maxTotalEnergyRequired


def computeAvgDistance(listOfDeliveryLocs):
    distanceFromDepotInmile = {}
    for i in range(len(listOfDeliveryLocs)):
        distanceFromDepotInmile[listOfDeliveryLocs[i].deliveryID] = listOfDeliveryLocs[i].distanceFromDepot
    sumDistanceAllLocations = sum(distanceFromDepotInmile.values())
    avgDistanceAllLocations = sumDistanceAllLocations / len(listOfDeliveryLocs)
    return avgDistanceAllLocations


def computePenaltyFindingRedundantPairs(listOfDeliveryLocs, listOfDeliveryLocsForVehicle):
    if not listOfDeliveryLocs:
        maxTimeFromDepot = max([i.timeFromDepot for i in listOfDeliveryLocsForVehicle])
    elif not listOfDeliveryLocsForVehicle:
        maxTimeFromDepot = max([i.timeFromDepot for i in listOfDeliveryLocs])
    else:
        maxTimeFromDepot = max(max([i.timeFromDepot for i in listOfDeliveryLocs]), max([i.timeFromDepot for i in listOfDeliveryLocsForVehicle]))


    return maxTimeFromDepot


def computeUnnecessaryPairs(listOfDeliveryLocs, listOfDeliveryLocsForVehicle, maxPermissibleDelay, penaltyTime):
    listOfUnnecessaryPairs = []
    for i in range(len(listOfDeliveryLocs)):
        for j in range(len(listOfDeliveryLocs)):
            if ((listOfDeliveryLocs[i].earliestServiceTime + packageLoadingTime + listOfDeliveryLocs[i].timeFromDepot
                 + listOfDeliveryLocs[i].timeReturnToDepot + packageUnloadingTimeLanding) >= (
                        listOfDeliveryLocs[j].earliestServiceTime + maxPermissibleDelay)) or (
                    listOfDeliveryLocs[j].earliestServiceTime > (
                    listOfDeliveryLocs[i].earliestServiceTime + maxPermissibleDelay +
                    listOfDeliveryLocs[i].timeFromDepot + listOfDeliveryLocs[i].timeReturnToDepot
                         + packageLoadingTime + packageUnloadingTimeLanding + penaltyTime)):
                listOfUnnecessaryPairs.append((listOfDeliveryLocs[i].deliveryID, listOfDeliveryLocs[j].deliveryID))
    for i in range(len(listOfDeliveryLocsForVehicle)):
        for j in range(len(listOfDeliveryLocsForVehicle)):
            if ((listOfDeliveryLocsForVehicle[i].earliestServiceTime + packageLoadingTime + listOfDeliveryLocsForVehicle[i].timeFromDepot
                 + listOfDeliveryLocsForVehicle[i].timeReturnToDepot + packageUnloadingTimeLanding) >= (
                        listOfDeliveryLocsForVehicle[j].earliestServiceTime + maxPermissibleDelay)) or (
                    listOfDeliveryLocsForVehicle[j].earliestServiceTime > (
                    listOfDeliveryLocsForVehicle[i].earliestServiceTime + maxPermissibleDelay +
                    listOfDeliveryLocsForVehicle[i].timeFromDepot + listOfDeliveryLocsForVehicle[i].timeReturnToDepot
                         + packageLoadingTime + packageUnloadingTimeLanding + penaltyTime)):
                listOfUnnecessaryPairs.append((listOfDeliveryLocsForVehicle[i].deliveryID, listOfDeliveryLocsForVehicle[j].deliveryID))
    return listOfUnnecessaryPairs

def computeNumberOfDronesUsed(zVal):
    outgoingArcsFromDepotDict = {}
    for key, value in zVal.items():
        if key[0] == 0 and value > 0.5:
            outgoingArcsFromDepotDict[key] = value
    numberOfDronesUsed = len(outgoingArcsFromDepotDict)
    return numberOfDronesUsed, outgoingArcsFromDepotDict

def computeNumberOfBatteriesReplaced(yVal):
    numOfBatteryReplaced = 0.0
    for key, value in yVal.items():
        if value > 0.5:
            numOfBatteryReplaced = numOfBatteryReplaced + 1.0
    return numOfBatteryReplaced

def findOptimalRoutes(zVal):
    dictSrcDests = dict()
    for k, v in zVal.items():
        if v > 0.5:
            if k[0] in dictSrcDests:
                dictSrcDests[k[0]].append(k[1])
            else:
                dictSrcDests[k[0]] = [k[1]]
    routes = list()
    nextNodesFrom0 = dictSrcDests[0]
    for i in nextNodesFrom0:
        tempRoute = [0, i]
        while True:
            nextNode = dictSrcDests[i][0]
            if nextNode != -99:
                tempRoute.append(nextNode)
                i = nextNode
            else:
                tempRoute.append(0)
                routes.append(tempRoute)
                break
    return routes


def createMIPmodel(listOfLocsDummySinkForDrone, listOfLocsDummySinkForVehicle, listOfDrones, routingCost, listOfUnnecessaryPairs):
    initBatCharge = listOfDrones[0].initCharge
    minChargeReq = listOfDrones[0].minChargeReq
    C_d = listOfDrones[0].C_d
    C_d_M = listOfDrones[0].C_d_M
    M_g = listOfDrones[0].M_g
    M_l = listOfDrones[0].M_l
    M_u = listOfDrones[0].M_u
    Q_l = listOfDrones[0].Q_l
    Q_u = listOfDrones[0].Q_u
    C_bat = listOfDrones[0].C_bat
    batReplaceTime = listOfDrones[0].batReplaceTime
    tm = Model(name='minTransCost_IdenticalDrones_CompactModel')
    tm.parameters.mip.tolerances.mipgap.set(gap)

    y = {}
    for i in range(len(listOfLocsDummySinkForDrone)):
        y[listOfLocsDummySinkForDrone[i].deliveryID] = tm.binary_var(
            name='y_' + str(listOfLocsDummySinkForDrone[i].deliveryID))

    g = {}
    for i in range(len(listOfLocsDummySinkForDrone)):
        g[listOfLocsDummySinkForDrone[i].deliveryID] = tm.continuous_var(lb=0, ub=initBatCharge,
                                                                         name='g_' + str(
                                                                             listOfLocsDummySinkForDrone[i].deliveryID))

    g_prime = {}
    for i in range(len(listOfLocsDummySinkForDrone)):
        g_prime[listOfLocsDummySinkForDrone[i].deliveryID] = tm.continuous_var(lb=-initBatCharge, ub=initBatCharge,
                                                                               name='g_prime_' + str(
                                                                                   listOfLocsDummySinkForDrone[
                                                                                       i].deliveryID))
    f = {}
    for i in range(len(listOfLocsDummySinkForDrone)):
        f[listOfLocsDummySinkForDrone[i].deliveryID] = tm.continuous_var(lb=0,
                                                                         name='f_' + str(
                                                                             listOfLocsDummySinkForDrone[i].deliveryID))

    z = {}
    for i in range(len(listOfLocsDummySinkForDrone)):
        for j in range(len(listOfLocsDummySinkForDrone)):
            if (listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID) not in listOfUnnecessaryPairs:
                if listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID:
                    z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID] = tm.binary_var(
                        name='z_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + '_' + str(
                            listOfLocsDummySinkForDrone[j].deliveryID))

    if listOfLocsDummySinkForVehicle:
        x = {}
        for i in range(len(listOfLocsDummySinkForVehicle)):
            for j in range(len(listOfLocsDummySinkForVehicle)):
                if (listOfLocsDummySinkForVehicle[i].deliveryID,
                    listOfLocsDummySinkForVehicle[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[j].deliveryID:
                        x[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[
                            j].deliveryID] = tm.binary_var(
                            name='x_' + str(listOfLocsDummySinkForVehicle[i].deliveryID) + '_' + str(
                                listOfLocsDummySinkForVehicle[j].deliveryID))
        f_v = {}
        for i in range(len(listOfLocsDummySinkForVehicle)):
            f_v[listOfLocsDummySinkForVehicle[i].deliveryID] = tm.continuous_var(lb=0,
                                                                             name='f_v_' + str(
                                                                                 listOfLocsDummySinkForVehicle[
                                                                                     i].deliveryID))

        if batteryReplacement == True:
            tm.minimize(C_E * tm.sum(routingCost[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID] * z[
                listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]
                                     for i in range(0, len(listOfLocsDummySinkForDrone) - 1) for j in
                                     range(1, len(listOfLocsDummySinkForDrone)) if ((listOfLocsDummySinkForDrone[i].deliveryID,
                                                                             listOfLocsDummySinkForDrone[
                                                                                 j].deliveryID) not in listOfUnnecessaryPairs) and (
                                                 listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID))
                        + C_E * tm.sum(routingCost[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID] * x[
                listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID]
                                     for i in range(0, len(listOfLocsDummySinkForVehicle) - 1) for j in
                                     range(1, len(listOfLocsDummySinkForVehicle)) if ((listOfLocsDummySinkForVehicle[i].deliveryID,
                                                                             listOfLocsDummySinkForVehicle[
                                                                                 j].deliveryID) not in listOfUnnecessaryPairs) and (
                                                 listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[j].deliveryID))
                        + (C_d + C_L_d + C_d_M) * tm.sum(
                z[0, listOfLocsDummySinkForDrone[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForDrone)))
                        + (C_v + C_lv + C_mv) * tm.sum(
                x[0, listOfLocsDummySinkForVehicle[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForVehicle)))
                        + C_bat * tm.sum(y[listOfLocsDummySinkForDrone[i].deliveryID] for i in range(len(listOfLocsDummySinkForDrone))))
        else:
            tm.minimize(C_E * tm.sum(
                routingCost[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID] * z[
                    listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]
                for i in range(0, len(listOfLocsDummySinkForDrone) - 1) for j in
                range(1, len(listOfLocsDummySinkForDrone)) if ((listOfLocsDummySinkForDrone[i].deliveryID,
                                                                listOfLocsDummySinkForDrone[
                                                                    j].deliveryID) not in listOfUnnecessaryPairs) and (
                        listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID))
                        + C_E * tm.sum(
                routingCost[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID] *
                x[
                    listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID]
                for i in range(0, len(listOfLocsDummySinkForVehicle) - 1) for j in
                range(1, len(listOfLocsDummySinkForVehicle)) if ((listOfLocsDummySinkForVehicle[i].deliveryID,
                                                                  listOfLocsDummySinkForVehicle[
                                                                      j].deliveryID) not in listOfUnnecessaryPairs) and (
                        listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[j].deliveryID))
                        + (C_d + C_L_d + C_d_M) * tm.sum(
                z[0, listOfLocsDummySinkForDrone[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForDrone)))
                        + (C_v + C_lv + C_mv) * tm.sum(
                x[0, listOfLocsDummySinkForVehicle[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForVehicle))))
    else:
        if batteryReplacement == True:
            tm.minimize(C_E * tm.sum(routingCost[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID] * z[
                listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]
                                     for i in range(0, len(listOfLocsDummySinkForDrone) - 1) for j in
                                     range(1, len(listOfLocsDummySinkForDrone)) if ((listOfLocsDummySinkForDrone[i].deliveryID,
                                                                             listOfLocsDummySinkForDrone[
                                                                                 j].deliveryID) not in listOfUnnecessaryPairs) and (
                                                 listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID))
                        + (C_d + C_L_d + C_d_M) * tm.sum(
                z[0, listOfLocsDummySinkForDrone[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForDrone)))
                        + C_bat * tm.sum(y[listOfLocsDummySinkForDrone[i].deliveryID] for i in range(len(listOfLocsDummySinkForDrone))))
        else:
            tm.minimize(C_E * tm.sum(routingCost[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID] * z[
                listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]
                                     for i in range(0, len(listOfLocsDummySinkForDrone) - 1) for j in
                                     range(1, len(listOfLocsDummySinkForDrone)) if ((listOfLocsDummySinkForDrone[i].deliveryID,
                                                                             listOfLocsDummySinkForDrone[
                                                                                 j].deliveryID) not in listOfUnnecessaryPairs) and (
                                                 listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID))
                        + (C_d + C_L_d + C_d_M) * tm.sum(
                z[0, listOfLocsDummySinkForDrone[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForDrone))))


    for j in range(1, len(listOfLocsDummySinkForDrone)):
        tm.add_constraint(f[listOfLocsDummySinkForDrone[j].deliveryID] >= (t_0 + listOfLocsDummySinkForDrone[j].timeFromDepot
                                                                   - M * (1 - z[0, listOfLocsDummySinkForDrone[j].deliveryID])),
                          ctname='cnstr10_' + str(listOfLocsDummySinkForDrone[j].deliveryID))

    for i in range(1, len(listOfLocsDummySinkForDrone)):
        for j in range(1, len(listOfLocsDummySinkForDrone)):
            if (listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID) not in listOfUnnecessaryPairs:
                if listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID:
                    if packageDropping == True:
                        tm.add_constraint(f[listOfLocsDummySinkForDrone[j].deliveryID] >= (
                                    f[listOfLocsDummySinkForDrone[i].deliveryID] + packageUnloadingTimeLanding +
                                    listOfLocsDummySinkForDrone[i].timeReturnToDepot + packageLoadingTime +
                                    listOfLocsDummySinkForDrone[j].timeFromDepot + batReplaceTime * y[
                                        listOfLocsDummySinkForDrone[i].deliveryID] -
                                    M * (1 - z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID])),
                                          ctname='cnstr11_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + str(
                                              listOfLocsDummySinkForDrone[j].deliveryID))
                    else:
                        tm.add_constraint(f[listOfLocsDummySinkForDrone[j].deliveryID] >= (
                                    f[listOfLocsDummySinkForDrone[i].deliveryID] + packageUnloadingTimeLanding +
                                    listOfLocsDummySinkForDrone[i].timeReturnToDepot
                                         + packageLoadingTime +
                                    listOfLocsDummySinkForDrone[j].timeFromDepot + batReplaceTime * y[
                                        listOfLocsDummySinkForDrone[i].deliveryID] -
                                    M * (1 - z[
                                listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID])),
                                          ctname='cnstr11_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + str(
                                              listOfLocsDummySinkForDrone[j].deliveryID))

    for i in range(1, len(listOfLocsDummySinkForDrone) - 1):
        tm.add_constraint(f[listOfLocsDummySinkForDrone[i].deliveryID] >= listOfLocsDummySinkForDrone[i].earliestServiceTime,
                          ctname='cnstr12_' + str(listOfLocsDummySinkForDrone[i].deliveryID))
        tm.add_constraint(f[listOfLocsDummySinkForDrone[i].deliveryID] <= listOfLocsDummySinkForDrone[i].maxDelayedServiceTime,
                          ctname='cnstr13_' + str(listOfLocsDummySinkForDrone[i].deliveryID))

    for j in range(1, len(listOfLocsDummySinkForDrone) - 1):
        tm.add_constraint(tm.sum(z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]
                                 for i in range(0, len(listOfLocsDummySinkForDrone) - 1) if ((listOfLocsDummySinkForDrone[i].deliveryID,
                                                                                      listOfLocsDummySinkForDrone[
                                                                                          j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID))
                          == tm.sum(z[listOfLocsDummySinkForDrone[j].deliveryID, listOfLocsDummySinkForDrone[i].deliveryID]
                                    for i in range(1, len(listOfLocsDummySinkForDrone)) if ((listOfLocsDummySinkForDrone[j].deliveryID,
                                                                                     listOfLocsDummySinkForDrone[
                                                                                         i].deliveryID) not in listOfUnnecessaryPairs) and (
                                                listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[
                                            j].deliveryID)))

    tm.add_constraint(tm.sum(z[0, listOfLocsDummySinkForDrone[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForDrone) - 1)) ==
                      tm.sum(z[listOfLocsDummySinkForDrone[j].deliveryID, -99] for j in range(1, len(listOfLocsDummySinkForDrone) - 1)))

    for j in range(1, len(listOfLocsDummySinkForDrone) - 1):
        tm.add_constraint(tm.sum(z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]
                                 for i in range(0, len(listOfLocsDummySinkForDrone) - 1) if ((listOfLocsDummySinkForDrone[i].deliveryID,
                                                                                      listOfLocsDummySinkForDrone[
                                                                                          j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[
                                         j].deliveryID)) == 1)

    for i in range(1, len(listOfLocsDummySinkForDrone) - 1):
        tm.add_constraint(tm.sum(z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]
                                 for j in range(1, len(listOfLocsDummySinkForDrone)) if ((listOfLocsDummySinkForDrone[i].deliveryID,
                                                                                  listOfLocsDummySinkForDrone[
                                                                                      j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[
                                         j].deliveryID)) == 1)

    if listOfLocsDummySinkForVehicle:
        for j in range(1, len(listOfLocsDummySinkForVehicle)):
            tm.add_constraint(
                f_v[listOfLocsDummySinkForVehicle[j].deliveryID] >= (t_0 + listOfLocsDummySinkForVehicle[j].timeFromDepot
                                                                 - M * (1 - x[
                            0, listOfLocsDummySinkForVehicle[j].deliveryID])),
                ctname='cnstr10_' + str(listOfLocsDummySinkForVehicle[j].deliveryID))

        for i in range(1, len(listOfLocsDummySinkForVehicle)):
            for j in range(1, len(listOfLocsDummySinkForVehicle)):
                if (listOfLocsDummySinkForVehicle[i].deliveryID,
                    listOfLocsDummySinkForVehicle[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[j].deliveryID:
                        tm.add_constraint(f_v[listOfLocsDummySinkForVehicle[j].deliveryID] >= (
                                f_v[listOfLocsDummySinkForVehicle[i].deliveryID] + packageUnloadingTimeVehicle +
                                listOfLocsDummySinkForVehicle[i].timeReturnToDepot + packageLoadingTimeVehicle +
                                listOfLocsDummySinkForVehicle[j].timeFromDepot -
                                M * (1 - x[
                            listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID])),
                                          ctname='cnstr11_' + str(listOfLocsDummySinkForVehicle[i].deliveryID) + str(
                                              listOfLocsDummySinkForVehicle[j].deliveryID))

        for i in range(1, len(listOfLocsDummySinkForVehicle) - 1):
            tm.add_constraint(
                f_v[listOfLocsDummySinkForVehicle[i].deliveryID] >= listOfLocsDummySinkForVehicle[i].earliestServiceTime,
                ctname='cnstr12_' + str(listOfLocsDummySinkForVehicle[i].deliveryID))
            tm.add_constraint(
                f_v[listOfLocsDummySinkForVehicle[i].deliveryID] <= listOfLocsDummySinkForVehicle[i].maxDelayedServiceTime,
                ctname='cnstr13_' + str(listOfLocsDummySinkForVehicle[i].deliveryID))

        for j in range(1, len(listOfLocsDummySinkForVehicle) - 1):
            tm.add_constraint(
                tm.sum(x[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID]
                       for i in range(0, len(listOfLocsDummySinkForVehicle) - 1) if
                       ((listOfLocsDummySinkForVehicle[i].deliveryID,
                         listOfLocsDummySinkForVehicle[
                             j].deliveryID) not in listOfUnnecessaryPairs) and (
                               listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[j].deliveryID))
                == tm.sum(x[listOfLocsDummySinkForVehicle[j].deliveryID, listOfLocsDummySinkForVehicle[i].deliveryID]
                          for i in range(1, len(listOfLocsDummySinkForVehicle)) if
                          ((listOfLocsDummySinkForVehicle[j].deliveryID,
                            listOfLocsDummySinkForVehicle[
                                i].deliveryID) not in listOfUnnecessaryPairs) and (
                                  listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[
                              j].deliveryID)))

        tm.add_constraint(tm.sum(
            x[0, listOfLocsDummySinkForVehicle[j].deliveryID] for j in range(1, len(listOfLocsDummySinkForVehicle) - 1)) ==
                          tm.sum(x[listOfLocsDummySinkForVehicle[j].deliveryID, -99] for j in
                                 range(1, len(listOfLocsDummySinkForVehicle) - 1)))

        for j in range(1, len(listOfLocsDummySinkForVehicle) - 1):
            tm.add_constraint(
                tm.sum(x[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID]
                       for i in range(0, len(listOfLocsDummySinkForVehicle) - 1) if
                       ((listOfLocsDummySinkForVehicle[i].deliveryID,
                         listOfLocsDummySinkForVehicle[
                             j].deliveryID) not in listOfUnnecessaryPairs) and (
                               listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[
                           j].deliveryID)) == 1)

        for i in range(1, len(listOfLocsDummySinkForVehicle) - 1):
            tm.add_constraint(
                tm.sum(x[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID]
                       for j in range(1, len(listOfLocsDummySinkForVehicle)) if
                       ((listOfLocsDummySinkForVehicle[i].deliveryID,
                         listOfLocsDummySinkForVehicle[
                             j].deliveryID) not in listOfUnnecessaryPairs) and (
                               listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[
                           j].deliveryID)) == 1)

    if batteryReplacement == True:
        for j in range(1, len(listOfLocsDummySinkForDrone)):
            tm.add_constraint(g_prime[listOfLocsDummySinkForDrone[j].deliveryID] <= initBatCharge -
                              (listOfLocsDummySinkForDrone[j].energyconsumptionDronesArrival +
                               listOfLocsDummySinkForDrone[j].energyconsumptionDronesReturn)
                              + M_g * (1 - z[0, listOfLocsDummySinkForDrone[j].deliveryID]),
                              ctname='cnstr14_' + str(listOfLocsDummySinkForDrone[j].deliveryID))

        for i in range(1, len(listOfLocsDummySinkForDrone) - 1):
            for j in range(1, len(listOfLocsDummySinkForDrone)):
                if (listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID:
                        tm.add_constraint(
                            g_prime[listOfLocsDummySinkForDrone[j].deliveryID] <= g[listOfLocsDummySinkForDrone[i].deliveryID] -
                            (listOfLocsDummySinkForDrone[j].energyconsumptionDronesArrival +
                             listOfLocsDummySinkForDrone[j].energyconsumptionDronesReturn) +
                            M_g * (1 - z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]),
                            ctname='cnstr15_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + '_' + str(
                                listOfLocsDummySinkForDrone[j].deliveryID))

        for i in range(0, len(listOfLocsDummySinkForDrone) - 1):
            for j in range(1, len(listOfLocsDummySinkForDrone)):
                if (listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID:
                        tm.add_constraint(g_prime[listOfLocsDummySinkForDrone[j].deliveryID] - minChargeReq >=
                                          M_l * y[listOfLocsDummySinkForDrone[i].deliveryID] +
                                          M_g * (z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[
                            j].deliveryID] - 1),
                                          ctname='cnstr16_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + '_' + str(
                                              listOfLocsDummySinkForDrone[j].deliveryID))
                        tm.add_constraint(g_prime[listOfLocsDummySinkForDrone[j].deliveryID] - minChargeReq <=
                                          M_u * (1 - y[listOfLocsDummySinkForDrone[i].deliveryID]) +
                                          M_g * (1 - z[
                            listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]),
                                          ctname='cnstr17_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + '_' + str(
                                              listOfLocsDummySinkForDrone[j].deliveryID))

                        tm.add_constraint(g[listOfLocsDummySinkForDrone[j].deliveryID] <= initBatCharge
                                          - (listOfLocsDummySinkForDrone[j].energyconsumptionDronesArrival +
                                             listOfLocsDummySinkForDrone[j].energyconsumptionDronesReturn)
                                          + Q_u * (1 - y[listOfLocsDummySinkForDrone[i].deliveryID]) +
                                          M_g * (1 - z[
                            listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID])
                                          , ctname='cnstr18_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + '_' + str(
                                listOfLocsDummySinkForDrone[j].deliveryID))

        for i in range(1, len(listOfLocsDummySinkForDrone) - 1):
            for j in range(1, len(listOfLocsDummySinkForDrone)):
                if (listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID:
                        tm.add_constraint(g[listOfLocsDummySinkForDrone[j].deliveryID] <= g[listOfLocsDummySinkForDrone[i].deliveryID] -
                                          (listOfLocsDummySinkForDrone[j].energyconsumptionDronesArrival +
                                           listOfLocsDummySinkForDrone[j].energyconsumptionDronesReturn)
                                          + Q_u * y[listOfLocsDummySinkForDrone[i].deliveryID] +
                                          M_g * (1 - z[
                            listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]),
                                          ctname='cnstr19_' + str(listOfLocsDummySinkForDrone[i].deliveryID) + '_' + str(
                                              listOfLocsDummySinkForDrone[j].deliveryID))

        for j in range(1, len(listOfLocsDummySinkForDrone)):
            tm.add_constraint(g[listOfLocsDummySinkForDrone[j].deliveryID] <= initBatCharge
                              - (listOfLocsDummySinkForDrone[j].energyconsumptionDronesArrival +
                                 listOfLocsDummySinkForDrone[j].energyconsumptionDronesReturn)
                              + Q_u * y[0] +
                              M_g * (1 - z[0, listOfLocsDummySinkForDrone[j].deliveryID])
                              , ctname='cnstr18_' + str(listOfLocsDummySinkForDrone[j].deliveryID))

    tm.print_information()
    with open('Model(New).txt', 'w') as file:
        file.write(tm.lp_string)
        file.write('\n----------------------------------------------------------------------------------------------')
        file.write('\n')
        file.write("\nList of unnecessary pairs: %s" % listOfUnnecessaryPairs)
    start_time = time.time()
    tms = tm.solve()
    runtime = (time.time() - start_time)
    mipGap = tm.parameters.mip.tolerances.mipgap.get()


    objVal = tms.objective_value
    fVal = {}
    for i in range(1, len(listOfLocsDummySinkForDrone) - 1):
        dt1 = datetime.fromtimestamp(tms[f[listOfLocsDummySinkForDrone[i].deliveryID]])
        dtVar = dt1.strftime("%m/%d/%Y %I:%M:%S %p")
        fVal[listOfLocsDummySinkForDrone[i].deliveryID] = dtVar

    zVal = {}
    for i in range(len(listOfLocsDummySinkForDrone)):
        for j in range(len(listOfLocsDummySinkForDrone)):
            if (listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID) not in listOfUnnecessaryPairs:
                if listOfLocsDummySinkForDrone[i].deliveryID != listOfLocsDummySinkForDrone[j].deliveryID:
                    if tms[z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]] > 0.0:
                        zVal[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID] = tms[
                            z[listOfLocsDummySinkForDrone[i].deliveryID, listOfLocsDummySinkForDrone[j].deliveryID]]

    if listOfLocsDummySinkForVehicle:
        f_vVal = {}
        for i in range(1, len(listOfLocsDummySinkForVehicle) - 1):
            dt1 = datetime.fromtimestamp(tms[f_v[listOfLocsDummySinkForVehicle[i].deliveryID]])
            dtVar = dt1.strftime("%m/%d/%Y %I:%M:%S %p")
            f_vVal[listOfLocsDummySinkForVehicle[i].deliveryID] = dtVar

        xVal = {}
        for i in range(len(listOfLocsDummySinkForVehicle)):
            for j in range(len(listOfLocsDummySinkForVehicle)):
                if (listOfLocsDummySinkForVehicle[i].deliveryID,
                    listOfLocsDummySinkForVehicle[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySinkForVehicle[i].deliveryID != listOfLocsDummySinkForVehicle[j].deliveryID:
                        if tms[x[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[
                            j].deliveryID]] > 0.0:
                            xVal[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID] = \
                            tms[
                                x[listOfLocsDummySinkForVehicle[i].deliveryID, listOfLocsDummySinkForVehicle[j].deliveryID]]
    else:
        f_vVal = {}
        xVal = {}

    if batteryReplacement == True:
        gVal = {}
        g_primeVal = {}
        yVal = {}
        for i in range(len(listOfLocsDummySinkForDrone)):
            yVal[listOfLocsDummySinkForDrone[i].deliveryID] = tms[y[listOfLocsDummySinkForDrone[i].deliveryID]]
            gVal[listOfLocsDummySinkForDrone[i].deliveryID] = tms[g[listOfLocsDummySinkForDrone[i].deliveryID]]
            g_primeVal[listOfLocsDummySinkForDrone[i].deliveryID] = tms[g_prime[listOfLocsDummySinkForDrone[i].deliveryID]]

    return objVal, fVal, f_vVal, zVal, xVal, gVal, g_primeVal, yVal, mipGap, runtime


def runDroneRoutingOPT():
    listOfDrone, listOfDeliveryLoc, providerLat, providerLon = CreateListOfObjects(providerLocation, droneData,
                                                                                   deliveryData)
    computeParams(listOfDrone, listOfDeliveryLoc, providerLat, providerLon, maxPermissibleDelay)
    listOfDrones, listOfDeliveryLocs, listOfDeliveryLocsForVehicle = lisOfObjects(listOfDrone, listOfDeliveryLoc) # creating the list of deliveries for each drone type, and return the specific drone we are using, deliveries can be served by this drone, and deliveries to be served by vehicles 
    computeParamsForDrone(listOfDrones, listOfDeliveryLocs, providerLat, providerLon, maxPermissibleDelay)
    computeParamsForVehicle(listOfDeliveryLocsForVehicle, providerLat, providerLon, maxPermissibleDelay)
    maxTimeFromDepot = computePenaltyFindingRedundantPairs(listOfDeliveryLocs, listOfDeliveryLocsForVehicle) #
    penaltyTime = 2.5 * maxTimeFromDepot
    listOfUnnecessaryPairs = computeUnnecessaryPairs(listOfDeliveryLocs, listOfDeliveryLocsForVehicle,
                                                     maxPermissibleDelay, penaltyTime)

    listOfLocsDummySinkForDrone = createListOfLocsWithDummy(listOfDeliveryLocs)
    listOfLocsDummySinkForVehicle = createListOfLocsWithDummy(listOfDeliveryLocsForVehicle)
    if OnlyDrone == True:
        listOfLocsDummySinkForVehicle = []

    penaltyList = [0.1, 1, 10, 1000, 1000000]
    for item in penaltyList:
        penaltyMultiplier = item
        initBatCharge = listOfDrones[0].initCharge
        penaltyOutgoingArcs = penaltyMultiplier * initBatCharge
        routingCost = computeRoutingCost(listOfLocsDummySinkForDrone, listOfLocsDummySinkForVehicle, minNumberOfDrones, penaltyOutgoingArcs)
        objVal, fVal, f_vVal, zVal, xVal, gVal, g_primeVal, yVal, mipGap, runtime = createMIPmodel(listOfLocsDummySinkForDrone, listOfLocsDummySinkForVehicle, listOfDrones, routingCost,
                                                                                     listOfUnnecessaryPairs)
        numberOfBatteriesReplaced = computeNumberOfBatteriesReplaced(yVal)

        numberOfDronesUsed, outgoingDroneArcsFromDepotDict = computeNumberOfDronesUsed(zVal)
        droneRoutes = findOptimalRoutes(zVal)
        totalEnergyConsumedByDrone = computeTotalEnergyConsumed(zVal, numberOfDronesUsed, routingCost,
                                                                penaltyOutgoingArcs)
        if listOfLocsDummySinkForVehicle:
            numberOfVehiclesUsed, outgoingVehicleArcsFromDepotDict = computeNumberOfDronesUsed(xVal)

            vehicleRoutes = findOptimalRoutes(xVal)

            totalEnergyConsumedByVehicle = computeTotalEnergyConsumed(xVal, numberOfVehiclesUsed, routingCost,
                                                                    penaltyOutgoingArcs)
            totalEnergyConsumed = totalEnergyConsumedByDrone + totalEnergyConsumedByVehicle
        else:
            numberOfVehiclesUsed, outgoingVehicleArcsFromDepotDict = 0, 0

            vehicleRoutes = 0

            totalEnergyConsumedByVehicle = 0
            totalEnergyConsumed = totalEnergyConsumedByDrone

        totalDelideryTimebyDrone = sum(
            [(packageLoadingTime + i.timeFromDepot + packageUnloadingTimeLanding + i.timeReturnToDepot) for i in
             listOfDeliveryLocs])
        totalDelideryTimebyVehicle = sum(
            [(packageLoadingTimeVehicle + i.timeFromDepot + packageUnloadingTimeVehicle + i.timeReturnToDepot) for i in
             listOfDeliveryLocsForVehicle])

        ofile = open("Output of Drone and Vehicle Mixed Model(Project).txt", "a")
        ofile.write("\nUsed Drone Type: %s" % droneType)
        ofile.write("\nUsed Vehicle Type: %s" % vehicle)
        ofile.write("\nTotal Number of delivery locations: %s" % (len(listOfDeliveryLocs) + len(listOfDeliveryLocsForVehicle)))
        ofile.write("\nNumber of delivery locations served by drone: %s" % (len(listOfLocsDummySinkForDrone) - 2.0))
        ofile.write("\nNumber of delivery locations served by vehicle: %s" % (len(listOfLocsDummySinkForVehicle) - 2.0))
        ofile.write("\nMaximum permissible delay (Seconds): %s" % maxPermissibleDelay)
        ofile.write("\nPackage loading time for drones (Seconds): %s" % packageLoadingTime)
        if packageDropping == True:
            ofile.write("\nPackage unloading time for drones while dropping (Seconds): %s" % packageUnloadingTimeDropping)
        else:
            ofile.write("\nPackage unloading time for drones while landing (Seconds): %s" % packageUnloadingTimeLanding)
        ofile.write("\nRuntime (Seconds): %s" % runtime)
        ofile.write("\nPackage loading time for ground vehicles (Seconds): %s" % packageLoadingTimeVehicle)
        ofile.write("\nPackage unloading time for ground vehicles (Seconds): %s" % packageUnloadingTimeVehicle)
        ofile.write("\nObjective value: %s" % objVal)
        ofile.write("\nMIP Gap: %s" % mipGap)
        ofile.write("\nNumber of Drones used : %s" % numberOfDronesUsed)
        ofile.write("\nNumber of Vehicle used : %s" % numberOfVehiclesUsed)
        ofile.write("\nTotal Delivery Time by Drone (Minute) : %s" % ((totalDelideryTimebyDrone/60) + (300*numberOfBatteriesReplaced/60)))
        ofile.write("\nTotal Delivery Time by Vehicle (Minute) : %s" % (totalDelideryTimebyVehicle/60))
        ofile.write("\nTotal Battery Replacement Time (Minute) : %s" % (300*numberOfBatteriesReplaced/60))
        ofile.write("\nPenalty Multiplier : %s" % penaltyMultiplier)
        ofile.write("\nTotal amount of energy consumed by Drone: %s" % totalEnergyConsumedByDrone)
        ofile.write("\nTotal amount of energy consumed by Vehicle: %s" % totalEnergyConsumedByVehicle)
        ofile.write("\nTotal amount of energy consumed: %s" % totalEnergyConsumed)
        ofile.write("\nAscending height (feet): %s" % AscendHeight)
        ofile.write("\nHovering while package delivery: %s" % hoveringBolLoaded)
        ofile.write("\nHovering duration while delivering package: %s" % durationHoverLoaded)
        ofile.write("\nHovering while returning empty: %s" % hoveringBolUnloaded)
        ofile.write("\nHovering duration while returning to depot: %s" % durationHoverUnloaded)
        ofile.write("\nOutgoing arcs from the provider location for Drone: %s" % outgoingDroneArcsFromDepotDict)
        ofile.write("\nOutgoing arcs from the provider location for Vehicle: %s" % outgoingVehicleArcsFromDepotDict)
        ofile.write("\nSequences in which deliveries are made by Drone: %s" % zVal)
        ofile.write("\nSequences in which deliveries are made by Vehicle: %s" % xVal)
        ofile.write("\nTiming of when the delivery locations are served by Drone: %s" % fVal)
        ofile.write("\nTiming of when the delivery locations are served by Vehicle: %s" % f_vVal)
        ofile.write("\nList of routes for Drone: %s" % droneRoutes)
        ofile.write("\nList of routes for Vehicle: %s" % vehicleRoutes)
        ofile.write("\nBattery replacement : %s" % batteryReplacement)
        if batteryReplacement == True:
            ofile.write("\nBattery replacement decisions : %s" % yVal)
            ofile.write("\nNumber of Batteries Replaced : %s" % numberOfBatteriesReplaced)
            ofile.write("\nRemaining battery charge after serving each location at the depot: %s" % gVal)
            ofile.write(
                "\nTemporary Remaining battery charge after serving each location at the depot: %s" % g_primeVal)
        ofile.write("\n----------------------------------------------------------------------------------------------")
        ofile.write("\n----------------------------------------------------------------------------------------------")
        ofile.close()


if __name__ == "__main__":
    runDroneRoutingOPT()
