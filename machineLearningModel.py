import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.utils import shuffle
from sklearn.metrics import mean_squared_error

ascendBol = True
descendBol = False
hoverBol = False
forwardFlightBol = False

ascendData = "data/ascend_data_DJI.csv"
descendData = "data/descend_data_DJI.csv"
hoverData = "data/hover_data_DJI.csv"
forwardFlightData_30mph = "data/forwardFlightData_DJI_30mph.csv"

ascendData_df = shuffle(pd.read_csv(ascendData, delimiter=',', usecols=[0, 1, 2, 3, 4]))
descendData_df = shuffle(pd.read_csv(descendData, delimiter=',', usecols=[0, 1, 2, 3, 4]))
hoverData_df = shuffle(pd.read_csv(hoverData, delimiter=',', usecols=[0, 1, 2, 3, 4]))
forwardFlightData_df = shuffle(pd.read_csv(forwardFlightData_30mph, delimiter=',', usecols=[0, 1, 2, 3, 4]))


# Function to Calculate Ascend Energy
def ascend(load):
    train_Feature = ascendData_df[['weight']].values  # we only take one feature.
    train_Target = ascendData_df['avgWatts'].values
    reg = LinearRegression().fit(train_Feature, train_Target)
    estimated_values = reg.predict([[load]])
    return estimated_values[0]


# Function to Calculate Descend Energy
def descend(load):
    train_Feature = descendData_df[['weight']].values  # we only take one feature.
    train_Target = descendData_df['avgWatts'].values
    reg = LinearRegression().fit(train_Feature, train_Target)
    estimated_values = reg.predict([[load]])
    return estimated_values[0]


# Function to Calculate Hover Energy
def hover(load):
    train_Feature = hoverData_df[['weight']].values  # we only take one feature.
    train_Target = hoverData_df['avgWatts'].values
    reg = LinearRegression().fit(train_Feature, train_Target)
    estimated_values = reg.predict([[load]])
    return estimated_values[0]


# Function to Calculate Forward Flight Energy
def forwardFlight(load):
    train_Feature = forwardFlightData_df[['weight']].values  # we only take one feature.
    train_Target = forwardFlightData_df['avgWatts'].values
    reg = LinearRegression().fit(train_Feature, train_Target)
    estimated_values = reg.predict([[load]])
    return estimated_values[0]


# Function for Data Interpolation
def dataInterpolation(maxLoad, energyLoeaded, energyUnloaded, packageWeight):
    calculatedEnergy = energyUnloaded + ((energyLoeaded - energyUnloaded)/maxLoad)*packageWeight
    return calculatedEnergy