'''
@author: Tanveer Hossain Bhuiyan (tanveerhossain.bhuiyan@inl.gov)
This code implements the drone deployment optimization model
Energy is measured in U.S. units, i.e., watt-hour, and kwh. Drone flight Time is measured in seconds.
'''
import docplex.mp.sdetails
import numpy as np
import pandas as pd
import math
from math import sin, cos, sqrt, atan2, radians
from datetime import datetime, timedelta
from docplex.mp.model import Model
import time

#### Data initialization ####

minNumberOfDrones = True
batteryReplacement = True
packageDropping = False
gap = 0.15
hoveringBolLoaded = True
hoveringBolUnloaded = True
initBatCharge = 0.6
minChargeReq = 0.09
maxPermissibleDelay = 900.0
payload = 2.5
droneSpeedLoaded = 30.0
droneSpeedUnloaded = 30.0
packageUnloadingTimeDropping = 30.0
if packageDropping == True:
    packageUnloadingTimeLanding = 0.0
else:
    packageUnloadingTimeLanding = 30.0
packageLoadingTime = 300.0
batReplaceTime = 300.0
t_0 = datetime.strptime('2/2/2015 6:00:00 PM', "%m/%d/%Y %I:%M:%S %p").timestamp()
C_d = 1.82
C_bat = 0.42
C_L = 60.0
n_d = 5.0
C_L_d = math.ceil(C_L / n_d)
C_d_M = 0.35
C_E = 0.13

M = 1000000000000.0
M_g = 2.5 * initBatCharge
M_l = -initBatCharge
M_u = initBatCharge
Q_l = -initBatCharge
Q_u = initBatCharge
whTokwhConvert = 0.001
wsecToWattHourConvert = 1 / 3600
hourToSecConvert = 3600.0

# Energy data with Package
AscendHeight = 200  # in ft
avgWattAscendLoaded = 1487.300573
durationAscendLoaded = 25.002
avgWattAngleAscendLoaded = 0.0
durationAngleAscendLoaded = 0.0
avgWattDescendLoaded = 1104.471857
if packageDropping == True:
    durationDescendLoaded = 35.209
else:
    durationDescendLoaded = 45.209
avgWattAngleDescendLoaded = 0.0
durationAngleDescendLoaded = 0.0
forwardFlightWhPerMileLoaded = 51.3620
if hoveringBolLoaded == True:
    avgWattHoverLoaded = 1211.63075
    if packageDropping == True:
        durationHoverLoaded = 30.0
    else:
        durationHoverLoaded = 5.0

# Energy data for Empty Drone
avgWattAscendUnloaded = 1351.445644
if packageDropping == True:
    durationAscendUnloaded = 19.6
else:
    durationAscendUnloaded = 24.6
avgWattAngleAscendUnloaded = 0.0
durationAngleAscendUnloaded = 0.0
avgWattDescendUnloaded = 1023.868042
durationDescendUnloaded = 41.803
avgWattAngleDescendUnloaded = 0.0
durationAngleDescendUnloaded = 0.0
forwardFlightWhPerMileUnloaded = 41.20
if hoveringBolUnloaded == True:
    avgWattHoverUnloaded = 1039.254162
    durationHoverUnloaded = 5.0
providerLocation = "ProviderLocation_Hourly.csv"
droneData = "DroneDataScenario_1_Hourly.csv"
deliveryData = "DeliveryLocsScenario_1_Hourly.csv"

class Node:
    def __init__(self, deliveryID, readyTime, lat, lon, payload):
        self.deliveryID = deliveryID
        self.readyTime = readyTime
        self.lat = lat
        self.lon = lon
        self.packageWeight = payload
        self.distanceFromDepot = 0.0
        self.timeFromDepot = []
        self.timeReturnToDepot = []
        self.earliestServiceTime = 0.0
        self.maxDelayedServiceTime = 0.0
        self.energyconsumptionDronesArrival = []
        self.energyconsumptionDronesReturn = []

class Drone:
    def __init__(self, type, numRotors, droneSpeedLoaded, droneSpeedUnloaded, payloadCap, bodyMass, batteryMass):
        self.dType = type
        self.numRotors = numRotors
        self.optimalSpeedLoaded = droneSpeedLoaded
        self.optimalSpeedUnloaded = droneSpeedUnloaded
        self.capacity = payloadCap
        self.bodyMass = bodyMass
        self.batteryMass = batteryMass
        self.initCharge = 0.0
        self.minChargeReq = 0.0
        self.batReplaceTime = 0.0

def runDroneRoutingOPT():
    penaltyList = [10000.0, 1000000.0, 100000000.0]
    for item in penaltyList:
        penaltyMultiplier = item
        penaltyOutgoingArcs = penaltyMultiplier * initBatCharge

        listOfDrones, listOfDeliveryLocs, providerLat, providerLon = CreateListOfObjects(payload, providerLocation,
                                                                                         droneData, deliveryData)

        computeParams(listOfDrones, listOfDeliveryLocs, providerLat, providerLon, initBatCharge,
                      minChargeReq, maxPermissibleDelay, batReplaceTime)

        maxTotalEnergyRequired = computeMaxEnergyToServeCustomerLoc(listOfDeliveryLocs)

        avgDistanceOfLocationsFromDepot = computeAvgDistance(listOfDeliveryLocs)

        listOfLocationsOutOFRange = findLocationsOutOfRange(listOfDeliveryLocs)

        maxTimeFromDepot, furthestLat, furthestLon = computePenaltyFindingRedundantPairs(listOfDeliveryLocs)
        penaltyTime = 2.5 * maxTimeFromDepot
        listOfUnnecessaryPairs = computeUnnecessaryPairs(listOfDeliveryLocs, maxPermissibleDelay, penaltyTime)
        listOfLocsDummySink = createListOfLocsWithDummy(listOfDeliveryLocs, listOfDrones)
        routingCost = computeRoutingCost(listOfLocsDummySink, minNumberOfDrones, penaltyOutgoingArcs)
        objVal, fVal, zVal, gVal, g_primeVal, yVal, mipGap, runtime = createMIPmodel(listOfLocsDummySink, routingCost,
                                                                                     listOfUnnecessaryPairs)
        numberOfBatteriesReplaced = computeNumberOfBatteriesReplaced(yVal)

        numberOfDronesUsed, outgoingArcsFromDepotDict = computeNumberOfDronesUsed(zVal)
        routes = findOptimalRoutes(zVal)
        totalEnergyConsumed = computeTotalEnergyConsumed(zVal, numberOfDronesUsed, routingCost, penaltyOutgoingArcs)

        ofile = open("Output model DJI drone flying straight.txt", "a")
        ofile.write("\nNumber of delivery locations: %s" % (len(listOfLocsDummySink) - 2.0))
        ofile.write("\nAverage distance of the delivery locations from depot: %s" % avgDistanceOfLocationsFromDepot)
        ofile.write("\nInitial battery charge (KWH): %s" % initBatCharge)
        ofile.write("\nMinimum required battery charge (KWH): %s" % minChargeReq)
        ofile.write("\nMaximum permissible delay (Seconds): %s" % maxPermissibleDelay)
        ofile.write("\nPackage loading time (Seconds): %s" % packageLoadingTime)
        if packageDropping == True:
            ofile.write("\nPackage unloading time dropping (Seconds): %s" % packageUnloadingTimeDropping)
        else:
            ofile.write("\nPackage unloading time landing (Seconds): %s" % packageUnloadingTimeLanding)
        ofile.write("\nRuntime (Seconds): %s" % runtime)
        ofile.write("\nObjective value: %s" % objVal)
        ofile.write("\nMIP Gap: %s" % mipGap)
        ofile.write("\nNumber of Drones used : %s" % numberOfDronesUsed)
        ofile.write("\nPenalty Multiplier : %s" % penaltyMultiplier)
        ofile.write("\nTotal amount of energy consumed: %s" % totalEnergyConsumed)
        ofile.write("\nPackage weight: %s" % payload)
        ofile.write("\nDrone speed: %s" % droneSpeedLoaded)
        ofile.write("\nAscending height (feet): %s" % AscendHeight)
        ofile.write("\nHovering while package delivery: %s" % hoveringBolLoaded)
        ofile.write("\nHovering duration while delivering package: %s" % durationHoverLoaded)
        ofile.write("\nHovering while returning empty: %s" % hoveringBolUnloaded)
        ofile.write("\nHovering duration while returning to depot: %s" % durationHoverUnloaded)
        ofile.write("\nList of locations out of range: %s" % listOfLocationsOutOFRange)
        ofile.write("\nOutgoing arcs from the provider location: %s" % outgoingArcsFromDepotDict)
        ofile.write("\nSequences in which deliveries are made: %s" % zVal)
        ofile.write("\nTiming of when the delivery locations are served: %s" % fVal)
        ofile.write("\nList of routes: %s" % routes)
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

def CreateListOfObjects(payload, providerLocation, droneData, deliveryData):
    listOfDrones = []
    listOfDeliveryLocs = []
    listOfDeliveryLocsWithDummy = []
    provider_df = pd.read_csv(providerLocation, delimiter=',', usecols=[0, 1, 2])
    providerLat = provider_df['lat'][0]
    providerLon = provider_df['long'][0]
    drone_df = pd.read_csv(droneData, delimiter=',', usecols=[0, 1, 2, 3, 4, 5, 6])
    for row in range(len(drone_df)):
        listOfDrones.append(
            Drone(drone_df['type'][row], drone_df['numRotors'][row], droneSpeedLoaded,
                  droneSpeedUnloaded, drone_df['payloadCap'][row], drone_df['bodyMass'][row],
                  drone_df['batteryMass'][row]))
    delivery_df = pd.read_csv(deliveryData, delimiter=',', usecols=[0, 1, 2, 3, 4])
    for row in range(len(delivery_df)):
        foodReadyTime = datetime.strptime(delivery_df['food_ready_time'][row],
                                          "%m/%d/%Y %H:%M").timestamp()
        listOfDeliveryLocs.append(Node(delivery_df['delivery_id'][row], foodReadyTime, delivery_df['dropoff_lat'][row],
                                       delivery_df['dropoff_long'][row], payload))
    return listOfDrones, listOfDeliveryLocs, providerLat, providerLon

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
    return distOfEachPair

def computeAscendDescendHoverEnergyLoaded():
    ascendEnergyLoaded = avgWattAscendLoaded * durationAscendLoaded * wsecToWattHourConvert * whTokwhConvert
    angleAscendEnergyLoaded = avgWattAngleAscendLoaded * durationAngleAscendLoaded * wsecToWattHourConvert * whTokwhConvert
    descendEnergyLoaded = avgWattDescendLoaded * durationDescendLoaded * wsecToWattHourConvert * whTokwhConvert
    angleDescendEnergyLoaded = avgWattAngleDescendLoaded * durationAngleDescendLoaded * wsecToWattHourConvert * whTokwhConvert
    if hoveringBolLoaded == True:
        hoverEnergyLoaded = avgWattHoverLoaded * durationHoverLoaded * wsecToWattHourConvert * whTokwhConvert
        totalAscendDescendHoverLoaded = ascendEnergyLoaded + angleAscendEnergyLoaded + descendEnergyLoaded + angleDescendEnergyLoaded + hoverEnergyLoaded
    else:
        totalAscendDescendHoverLoaded = ascendEnergyLoaded + angleAscendEnergyLoaded + descendEnergyLoaded + angleDescendEnergyLoaded
    return totalAscendDescendHoverLoaded

def computeAscendDescendHoverEnergyUnloaded():
    ascendEnergyUnloaded = avgWattAscendUnloaded * durationAscendUnloaded * wsecToWattHourConvert * whTokwhConvert
    angleAscendEnergyUnloaded = avgWattAngleAscendUnloaded * durationAngleAscendUnloaded * wsecToWattHourConvert * whTokwhConvert
    descendEnergyUnloaded = avgWattDescendUnloaded * durationDescendUnloaded * wsecToWattHourConvert * whTokwhConvert
    angleDescendEnergyUnloaded = avgWattAngleDescendUnloaded * durationAngleDescendUnloaded * wsecToWattHourConvert * whTokwhConvert
    if hoveringBolUnloaded == True:
        hoverEnergyUnloaded = avgWattHoverUnloaded * durationHoverUnloaded * wsecToWattHourConvert * whTokwhConvert
        totalAscendDescendHoverUnloaded = ascendEnergyUnloaded + angleAscendEnergyUnloaded + descendEnergyUnloaded + angleDescendEnergyUnloaded + hoverEnergyUnloaded
    else:
        totalAscendDescendHoverUnloaded = ascendEnergyUnloaded + angleAscendEnergyUnloaded + descendEnergyUnloaded + angleDescendEnergyUnloaded
    return totalAscendDescendHoverUnloaded

def computeParams(listOfDrones, listOfDeliveryLocs, providerLat, providerLon, initBatCharge,
                  minChargeReq, maxPermissibleDelay, batReplaceTime):
    for i in range(len(listOfDeliveryLocs)):
        packageWeight = listOfDeliveryLocs[i].packageWeight
        listOfDeliveryLocs[i].earliestServiceTime = listOfDeliveryLocs[i].readyTime
        listOfDeliveryLocs[i].maxDelayedServiceTime = listOfDeliveryLocs[i].earliestServiceTime + maxPermissibleDelay
        destLat = listOfDeliveryLocs[i].lat
        destLon = listOfDeliveryLocs[i].lon
        distanceFromDepot = computeDistOfEachPair(providerLat, providerLon, destLat, destLon)
        listOfDeliveryLocs[i].distanceFromDepot = distanceFromDepot
        for d in range(len(listOfDrones)):
            listOfDrones[d].initCharge = initBatCharge
            listOfDrones[d].minChargeReq = minChargeReq
            listOfDrones[d].batReplaceTime = batReplaceTime
            flyingTimeFromDepot = (distanceFromDepot / listOfDrones[d].optimalSpeedLoaded) * hourToSecConvert
            timeFromDepot = flyingTimeFromDepot + durationAscendLoaded + durationAngleAscendLoaded + durationDescendLoaded + durationAngleDescendLoaded + durationHoverLoaded
            listOfDeliveryLocs[i].timeFromDepot.append(timeFromDepot)
            flyingTimeReturnToDepot = (distanceFromDepot / listOfDrones[d].optimalSpeedUnloaded) * hourToSecConvert
            timeReturnToDepot = flyingTimeReturnToDepot + durationAscendUnloaded + durationAngleAscendUnloaded + durationDescendUnloaded + durationAngleDescendUnloaded + durationHoverUnloaded
            listOfDeliveryLocs[i].timeReturnToDepot.append(timeReturnToDepot)
            energyPerMileLoaded = forwardFlightWhPerMileLoaded * whTokwhConvert
            energyPerMileUnLoaded = forwardFlightWhPerMileUnloaded * whTokwhConvert
            energyConsumptionFlightLoaded = energyPerMileLoaded * distanceFromDepot
            energyConsumptionFlightUnLoaded = energyPerMileUnLoaded * distanceFromDepot
            ascendDescendHoverEnergyLoaded = computeAscendDescendHoverEnergyLoaded()
            ascendDescendHoverEnergyUnloaded = computeAscendDescendHoverEnergyUnloaded()
            totalEnergyConsumptionLoaded = energyConsumptionFlightLoaded + ascendDescendHoverEnergyLoaded
            totalEnergyConsumptionUnloaded = energyConsumptionFlightUnLoaded + ascendDescendHoverEnergyUnloaded
            listOfDeliveryLocs[i].energyconsumptionDronesArrival.append(totalEnergyConsumptionLoaded)
            listOfDeliveryLocs[i].energyconsumptionDronesReturn.append(totalEnergyConsumptionUnloaded)

def computeMaxEnergyToServeCustomerLoc(listOfDeliveryLocs):
    maxEnergyArrivalAllLocs = []
    maxEnergyReturnAllLocs = []
    for i in range(len(listOfDeliveryLocs)):
        maxEnergyArrivalAllLocs.append(max(listOfDeliveryLocs[i].energyconsumptionDronesArrival))
        maxEnergyReturnAllLocs.append(max(listOfDeliveryLocs[i].energyconsumptionDronesReturn))
    maxTotalEnergyRequired = max(maxEnergyArrivalAllLocs) + max(maxEnergyReturnAllLocs)
    return maxTotalEnergyRequired

def computePenaltyFindingRedundantPairs(listOfDeliveryLocs):
    distanceFromDepot = []
    for i in range(len(listOfDeliveryLocs)):
        distanceFromDepot.append(listOfDeliveryLocs[i].distanceFromDepot)
    maxDistance = max(distanceFromDepot)
    maxDistIndex = distanceFromDepot.index(maxDistance)
    destLat = listOfDeliveryLocs[maxDistIndex].lat
    destLon = listOfDeliveryLocs[maxDistIndex].lon
    distanceFromDepotInMeter = maxDistance * 1609.34
    maxTimeFromDepot = max(listOfDeliveryLocs[maxDistIndex].timeReturnToDepot)

    return maxTimeFromDepot, destLat, destLon

def computeUnnecessaryPairs(listOfDeliveryLocs, maxPermissibleDelay, penaltyTime):
    listOfUnnecessaryPairs = []
    for i in range(len(listOfDeliveryLocs)):
        minTimeToReturnDepot = min(listOfDeliveryLocs[i].timeReturnToDepot)
        maxTimeToReturnDepot = max(listOfDeliveryLocs[i].timeReturnToDepot)
        for j in range(len(listOfDeliveryLocs)):
            minTimeFromDepot = min(listOfDeliveryLocs[j].timeFromDepot)
            maxTimeFromDepot = max(listOfDeliveryLocs[j].timeFromDepot)
            if ((listOfDeliveryLocs[i].earliestServiceTime + packageLoadingTime + listOfDeliveryLocs[i].timeFromDepot[
                0] + listOfDeliveryLocs[i].timeReturnToDepot[0] + packageUnloadingTimeLanding) >= (
                        listOfDeliveryLocs[j].earliestServiceTime + maxPermissibleDelay)) or (
                    (listOfDeliveryLocs[j].earliestServiceTime) > (
                    listOfDeliveryLocs[i].earliestServiceTime + maxPermissibleDelay +
                    listOfDeliveryLocs[i].timeFromDepot[0] + listOfDeliveryLocs[i].timeReturnToDepot[
                        0] + packageLoadingTime + packageUnloadingTimeLanding + penaltyTime)):
                listOfUnnecessaryPairs.append((listOfDeliveryLocs[i].deliveryID, listOfDeliveryLocs[j].deliveryID))
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

def computeTotalEnergyConsumed(zVal, numberOfDronesUsed, routingCost, penaltyOutgoingArcs):
    totalEnergy = 0.0
    for k, v in zVal.items():
        if v > 0.5:
            totalEnergy = totalEnergy + routingCost[k]
    totalEnergy = totalEnergy - numberOfDronesUsed * penaltyOutgoingArcs
    return totalEnergy

def computeAvgDistance(listOfDeliveryLocs):
    distanceFromDepotInmile = {}
    for i in range(len(listOfDeliveryLocs)):
        distanceFromDepotInmile[listOfDeliveryLocs[i].deliveryID] = listOfDeliveryLocs[i].distanceFromDepot
    sumDistanceAllLocations = sum(distanceFromDepotInmile.values())
    avgDistanceAllLocations = sumDistanceAllLocations / len(listOfDeliveryLocs)
    return avgDistanceAllLocations

def findLocationsOutOfRange(listOfDeliveryLocs):
    locationsOutOfRange = []
    for i in range(len(listOfDeliveryLocs)):
        totalEnergyRequired = 0.0
        energyForDroneArrival = listOfDeliveryLocs[i].energyconsumptionDronesArrival[0]
        energyForDroneReturn = listOfDeliveryLocs[i].energyconsumptionDronesReturn[0]
        totalEnergyRequired = energyForDroneArrival + energyForDroneReturn
        if totalEnergyRequired > (initBatCharge - minChargeReq):
            locationsOutOfRange.append(listOfDeliveryLocs[i].deliveryID)

    print("locationsOutOfRange: ", locationsOutOfRange)
    return locationsOutOfRange

def createListOfLocsWithDummy(listOfDeliveryLocs, listOfDrones):
    droneDataForDummyAll_0 = []
    for d in range(len(listOfDrones)):
        droneDataForDummyAll_0.append(0.0)
    listOfLocsDummySink = []
    listOfLocsDummySink.append(Node(0, 0.0, 0.0, 0.0, 0.0))
    listOfLocsDummySink[-1].distanceFromDepot = 0.0
    listOfLocsDummySink[-1].earliestServiceTime = 0.0
    listOfLocsDummySink[-1].maxDelayedServiceTime = 0.0
    for d in range(len(listOfDrones)):
        listOfLocsDummySink[-1].timeFromDepot.append(0.0)
        listOfLocsDummySink[-1].timeReturnToDepot.append(0.0)
        listOfLocsDummySink[-1].energyconsumptionDronesArrival.append(0.0)
        listOfLocsDummySink[-1].energyconsumptionDronesReturn.append(0.0)
    listOfLocsDummySink.extend(listOfDeliveryLocs)
    listOfLocsDummySink.append(Node(-99, 0.0, 0.0, 0.0, 0.0))
    listOfLocsDummySink[-1].distanceFromDepot = 0.0
    listOfLocsDummySink[-1].earliestServiceTime = 0.0
    listOfLocsDummySink[-1].maxDelayedServiceTime = 0.0
    for d in range(len(listOfDrones)):
        listOfLocsDummySink[-1].timeFromDepot.append(0.0)
        listOfLocsDummySink[-1].timeReturnToDepot.append(0.0)
        listOfLocsDummySink[-1].energyconsumptionDronesArrival.append(0.0)
        listOfLocsDummySink[-1].energyconsumptionDronesReturn.append(0.0)

    return listOfLocsDummySink

def computeRoutingCost(listOfLocsDummySink, minNumberOfDrones, penaltyOutgoingArcs):
    routingCost = {}
    for i in range(len(listOfLocsDummySink)):
        for j in range(len(listOfLocsDummySink)):
            if listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID:
                routingCost[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID] = listOfLocsDummySink[
                                                                                                        i].energyconsumptionDronesReturn[
                                                                                                        0] + \
                                                                                                    listOfLocsDummySink[
                                                                                                        j].energyconsumptionDronesArrival[
                                                                                                        0]
    if minNumberOfDrones == True:
        for key, val in routingCost.items():
            if key[0] == 0:
                routingCost[key] = val + penaltyOutgoingArcs

    return routingCost

def createMIPmodel(listOfLocsDummySink, routingCost, listOfUnnecessaryPairs):
    tm = Model(name='minTransCost_IdenticalDrones_CompactModel')
    tm.parameters.mip.tolerances.mipgap.set(gap)

    y = {}
    for i in range(len(listOfLocsDummySink)):
        y[listOfLocsDummySink[i].deliveryID] = tm.binary_var(
            name='y_' + str(listOfLocsDummySink[i].deliveryID))

    z = {}
    for i in range(len(listOfLocsDummySink)):
        for j in range(len(listOfLocsDummySink)):
            if (listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID) not in listOfUnnecessaryPairs:
                if listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID:
                    z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID] = tm.binary_var(
                        name='z_' + str(listOfLocsDummySink[i].deliveryID) + '_' + str(
                            listOfLocsDummySink[j].deliveryID))

    g = {}
    for i in range(len(listOfLocsDummySink)):
        g[listOfLocsDummySink[i].deliveryID] = tm.continuous_var(lb=0, ub=initBatCharge,
                                                                 name='g_' + str(
                                                                     listOfLocsDummySink[i].deliveryID))

    g_prime = {}
    for i in range(len(listOfLocsDummySink)):
        g_prime[listOfLocsDummySink[i].deliveryID] = tm.continuous_var(lb=-initBatCharge, ub=initBatCharge,
                                                                       name='g_prime_' + str(
                                                                           listOfLocsDummySink[i].deliveryID))
    f = {}
    for i in range(len(listOfLocsDummySink)):
        f[listOfLocsDummySink[i].deliveryID] = tm.continuous_var(lb=0,
                                                                 name='f_' + str(listOfLocsDummySink[i].deliveryID))

    if batteryReplacement == True:
        tm.minimize(C_E * tm.sum(routingCost[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID] * z[
            listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]
                                 for i in range(0, len(listOfLocsDummySink) - 1) for j in
                                 range(1, len(listOfLocsDummySink)) if ((listOfLocsDummySink[i].deliveryID,
                                                                         listOfLocsDummySink[
                                                                             j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID))
                    + (C_d + C_L_d + C_d_M) * tm.sum(
            z[0, listOfLocsDummySink[j].deliveryID] for j in range(1, len(listOfLocsDummySink)))
                    + C_bat * tm.sum(y[listOfLocsDummySink[i].deliveryID] for i in range(len(listOfLocsDummySink))))
    else:
        tm.minimize(C_E * tm.sum(routingCost[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID] * z[
            listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]
                                 for i in range(0, len(listOfLocsDummySink) - 1) for j in
                                 range(1, len(listOfLocsDummySink)) if ((listOfLocsDummySink[i].deliveryID,
                                                                         listOfLocsDummySink[
                                                                             j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID))
                    + (C_d + C_L_d + C_d_M) * tm.sum(
            z[0, listOfLocsDummySink[j].deliveryID] for j in range(1, len(listOfLocsDummySink))))

    for j in range(1, len(listOfLocsDummySink)):
        tm.add_constraint(f[listOfLocsDummySink[j].deliveryID] >= (t_0 + listOfLocsDummySink[j].timeFromDepot[0]
                                                                   - M * (1 - z[0, listOfLocsDummySink[j].deliveryID])),
                          ctname='cnstr10_' + str(listOfLocsDummySink[j].deliveryID))

    for i in range(1, len(listOfLocsDummySink)):
        for j in range(1, len(listOfLocsDummySink)):
            if (listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID) not in listOfUnnecessaryPairs:
                if listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID:
                    if packageDropping == True:
                        tm.add_constraint(f[listOfLocsDummySink[j].deliveryID] >= (
                                    f[listOfLocsDummySink[i].deliveryID] + packageUnloadingTimeLanding +
                                    listOfLocsDummySink[i].timeReturnToDepot[0] + packageLoadingTime +
                                    listOfLocsDummySink[j].timeFromDepot[0] + batReplaceTime * y[
                                        listOfLocsDummySink[i].deliveryID] -
                                    M * (1 - z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID])),
                                          ctname='cnstr11_' + str(listOfLocsDummySink[i].deliveryID) + str(
                                              listOfLocsDummySink[j].deliveryID))
                    else:
                        tm.add_constraint(f[listOfLocsDummySink[j].deliveryID] >= (
                                    f[listOfLocsDummySink[i].deliveryID] + packageUnloadingTimeLanding +
                                    listOfLocsDummySink[i].timeReturnToDepot[
                                        0] + packageLoadingTime +
                                    listOfLocsDummySink[j].timeFromDepot[0] + batReplaceTime * y[
                                        listOfLocsDummySink[i].deliveryID] -
                                    M * (1 - z[
                                listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID])),
                                          ctname='cnstr11_' + str(listOfLocsDummySink[i].deliveryID) + str(
                                              listOfLocsDummySink[j].deliveryID))

    for i in range(1, len(listOfLocsDummySink) - 1):
        tm.add_constraint(f[listOfLocsDummySink[i].deliveryID] >= listOfLocsDummySink[i].earliestServiceTime,
                          ctname='cnstr12_' + str(listOfLocsDummySink[i].deliveryID))
        tm.add_constraint(f[listOfLocsDummySink[i].deliveryID] <= listOfLocsDummySink[i].maxDelayedServiceTime,
                          ctname='cnstr13_' + str(listOfLocsDummySink[i].deliveryID))

    for j in range(1, len(listOfLocsDummySink) - 1):
        tm.add_constraint(tm.sum(z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]
                                 for i in range(0, len(listOfLocsDummySink) - 1) if ((listOfLocsDummySink[i].deliveryID,
                                                                                      listOfLocsDummySink[
                                                                                          j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID))
                          == tm.sum(z[listOfLocsDummySink[j].deliveryID, listOfLocsDummySink[i].deliveryID]
                                    for i in range(1, len(listOfLocsDummySink)) if ((listOfLocsDummySink[j].deliveryID,
                                                                                     listOfLocsDummySink[
                                                                                         i].deliveryID) not in listOfUnnecessaryPairs) and (
                                                listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[
                                            j].deliveryID)))

    tm.add_constraint(tm.sum(z[0, listOfLocsDummySink[j].deliveryID] for j in range(1, len(listOfLocsDummySink) - 1)) ==
                      tm.sum(z[listOfLocsDummySink[j].deliveryID, -99] for j in range(1, len(listOfLocsDummySink) - 1)))

    for j in range(1, len(listOfLocsDummySink) - 1):
        tm.add_constraint(tm.sum(z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]
                                 for i in range(0, len(listOfLocsDummySink) - 1) if ((listOfLocsDummySink[i].deliveryID,
                                                                                      listOfLocsDummySink[
                                                                                          j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[
                                         j].deliveryID)) == 1)

    for i in range(1, len(listOfLocsDummySink) - 1):
        tm.add_constraint(tm.sum(z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]
                                 for j in range(1, len(listOfLocsDummySink)) if ((listOfLocsDummySink[i].deliveryID,
                                                                                  listOfLocsDummySink[
                                                                                      j].deliveryID) not in listOfUnnecessaryPairs) and (
                                             listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[
                                         j].deliveryID)) == 1)

    if batteryReplacement == True:
        for j in range(1, len(listOfLocsDummySink)):
            tm.add_constraint(g_prime[listOfLocsDummySink[j].deliveryID] <= initBatCharge -
                              (listOfLocsDummySink[j].energyconsumptionDronesArrival[0] +
                               listOfLocsDummySink[j].energyconsumptionDronesReturn[0])
                              + M_g * (1 - z[0, listOfLocsDummySink[j].deliveryID]),
                              ctname='cnstr14_' + str(listOfLocsDummySink[j].deliveryID))

        for i in range(1, len(listOfLocsDummySink) - 1):
            for j in range(1, len(listOfLocsDummySink)):
                if (listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID:
                        tm.add_constraint(
                            g_prime[listOfLocsDummySink[j].deliveryID] <= g[listOfLocsDummySink[i].deliveryID] -
                            (listOfLocsDummySink[j].energyconsumptionDronesArrival[0] +
                             listOfLocsDummySink[j].energyconsumptionDronesReturn[0]) +
                            M_g * (1 - z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]),
                            ctname='cnstr15_' + str(listOfLocsDummySink[i].deliveryID) + '_' + str(
                                listOfLocsDummySink[j].deliveryID))

        for i in range(0, len(listOfLocsDummySink) - 1):
            for j in range(1, len(listOfLocsDummySink)):
                if (listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID:
                        tm.add_constraint(g_prime[listOfLocsDummySink[j].deliveryID] - minChargeReq >=
                                          M_l * y[listOfLocsDummySink[i].deliveryID] +
                                          M_g * (z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[
                            j].deliveryID] - 1),
                                          ctname='cnstr16_' + str(listOfLocsDummySink[i].deliveryID) + '_' + str(
                                              listOfLocsDummySink[j].deliveryID))
                        tm.add_constraint(g_prime[listOfLocsDummySink[j].deliveryID] - minChargeReq <=
                                          M_u * (1 - y[listOfLocsDummySink[i].deliveryID]) +
                                          M_g * (1 - z[
                            listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]),
                                          ctname='cnstr17_' + str(listOfLocsDummySink[i].deliveryID) + '_' + str(
                                              listOfLocsDummySink[j].deliveryID))

                        tm.add_constraint(g[listOfLocsDummySink[j].deliveryID] <= initBatCharge
                                          - (listOfLocsDummySink[j].energyconsumptionDronesArrival[0] +
                                             listOfLocsDummySink[j].energyconsumptionDronesReturn[0])
                                          + Q_u * (1 - y[listOfLocsDummySink[i].deliveryID]) +
                                          M_g * (1 - z[
                            listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID])
                                          , ctname='cnstr18_' + str(listOfLocsDummySink[i].deliveryID) + '_' + str(
                                listOfLocsDummySink[j].deliveryID))

        for i in range(1, len(listOfLocsDummySink) - 1):
            for j in range(1, len(listOfLocsDummySink)):
                if (listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID) not in listOfUnnecessaryPairs:
                    if listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID:
                        tm.add_constraint(g[listOfLocsDummySink[j].deliveryID] <= g[listOfLocsDummySink[i].deliveryID] -
                                          (listOfLocsDummySink[j].energyconsumptionDronesArrival[0] +
                                           listOfLocsDummySink[j].energyconsumptionDronesReturn[0])
                                          + Q_u * y[listOfLocsDummySink[i].deliveryID] +
                                          M_g * (1 - z[
                            listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]),
                                          ctname='cnstr19_' + str(listOfLocsDummySink[i].deliveryID) + '_' + str(
                                              listOfLocsDummySink[j].deliveryID))

        for j in range(1, len(listOfLocsDummySink)):
            tm.add_constraint(g[listOfLocsDummySink[j].deliveryID] <= initBatCharge
                              - (listOfLocsDummySink[j].energyconsumptionDronesArrival[0] +
                                 listOfLocsDummySink[j].energyconsumptionDronesReturn[0])
                              + Q_u * y[0] +
                              M_g * (1 - z[0, listOfLocsDummySink[j].deliveryID])
                              , ctname='cnstr18_' + str(listOfLocsDummySink[j].deliveryID))

    start_time = time.time()
    tms = tm.solve()
    runtime = (time.time() - start_time)
    mipGap = tm.parameters.mip.tolerances.mipgap.get()
    tms.display()

    objVal = tms.objective_value
    fVal = {}
    for i in range(1, len(listOfLocsDummySink) - 1):
        dt1 = datetime.fromtimestamp(tms[f[listOfLocsDummySink[i].deliveryID]])
        dtVar = dt1.strftime("%m/%d/%Y %I:%M:%S %p")
        fVal[listOfLocsDummySink[i].deliveryID] = dtVar

    zVal = {}
    for i in range(len(listOfLocsDummySink)):
        for j in range(len(listOfLocsDummySink)):
            if (listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID) not in listOfUnnecessaryPairs:
                if listOfLocsDummySink[i].deliveryID != listOfLocsDummySink[j].deliveryID:
                    if tms[z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]] > 0.0:
                        zVal[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID] = tms[
                            z[listOfLocsDummySink[i].deliveryID, listOfLocsDummySink[j].deliveryID]]

    if batteryReplacement == True:
        gVal = {}
        g_primeVal = {}
        yVal = {}
        for i in range(len(listOfLocsDummySink)):
            yVal[listOfLocsDummySink[i].deliveryID] = tms[y[listOfLocsDummySink[i].deliveryID]]
            gVal[listOfLocsDummySink[i].deliveryID] = tms[g[listOfLocsDummySink[i].deliveryID]]
            g_primeVal[listOfLocsDummySink[i].deliveryID] = tms[g_prime[listOfLocsDummySink[i].deliveryID]]

    return objVal, fVal, zVal, gVal, g_primeVal, yVal, mipGap, runtime

if __name__ == "__main__":
    runDroneRoutingOPT()