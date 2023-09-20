# Optimizing Mixed Fleet of Aerial Drones and Ground Vehicles for Time-Sensitive Products.
In this model, both types of vehicles (aerial dornes and ground vehicles) perform direct delivery (serve a single customer in each trip) 
This code implements the optimization method for routing a mixed-fleet of aerial drones and ground vehicles for time-sensitive product delivery.

* Data Files: 
	* ascend_data_DJI.csv, descend_data_DJI.csv, forwardFlightData_DJI_30mph.csv, hover_data_DJI.csv: These files are the data file to estimate power consumption of drones using machine learning
	* Distance_Matrix.csv, Time_Matrix.csv, Energy_Matrix.csv: These files contains the distance, time, and energy of each pair of location
	* DroneDataScenario_1.csv: This file contains drone type wise necessary data for cost and energy calculation
	* ProviderLocationScenario_1.csv: This file contains the lattitude and longitude of the depot location
	* Input_data.csv: This is the main delivery data file with all 207 deliveries
	* Road_Distance: This file contains the road distance for all deliveries
	
*Python Files:
	* machineLearningModel.py: This file contains the implementation of the machine learning algorithm. This file needs the ascend_data_DJI.csv, descend_data_DJI.csv, forwardFlightData_DJI_30mph.csv, hover_data_DJI.csv data files. Functions of this file are called from the main project Python files (all models need to call this file to estimate energy consumption of DJI drone) 
	* mixed_fleet_deployment.py: This file contains the code to run the model for Drone and Ground vehicle mixed model considering direct delivery. 
	* How to run:
		To run this file, 
			You need the machineLearningModel.py file to import its functions. 
			You need the DroneDataScenario_1.csv data file for information on drones, Input_data.csv data file for the delivery information, and ProviderLocationScenario_1.csv data file for the co-ordinate of the depot location
		* Important packages to be installed are pandas, math, requests, json, datetime, docplex.mp.model, and time
		* To run the model for different number of deliveries, change the required delivery data file in line 26
		* To run the model for different time window, change the value of maxPermissibleDelay variable to desired time window in seconds in line 40
		* To run the model for different drone type, change the value of droneType variable to desired drone type ('Tarot', 'DJI', 'Small VTOL', 'VTOL') in line 59
		* To run the model for different ground vehicle type, change the value of droneType variable to desired ground vehicle type ('Hyundai Accent 2022', 'Toyota Prius 2021', 'KIA Niro 2022', 'Honda Insight 2022', 'Honda Civic 2015') in line 65
		* To run the model for different penalty, change the penalties in line 1122
		* To run the model only for drones, change the value of OnlyDrone in line 25 to "True"