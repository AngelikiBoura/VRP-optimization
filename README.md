# VRP-optimization
For the course , Optimization Methods in Management Science, we were called to create a python programme that solves a VRP model 
and then optimizes the solution. After a natural disaster, we need to deliver the needed supplies to 100 different areas. We need to minimize the sum of the 
total time until we arrive to the last area and we are ready to deliver it the supplies. The time to travel from one area to the other, we assume it's equal with 
their  euclidean distance. Also in every area there is a required time for the supplies to unload from the vehicles.

##Instance.txt: contains the problem's data. Specifically, how many vehicles we have available and their capacity, the latitude, longtitude, demand and service time 
for every area that needs to be delivered supplies, and for the depot.
##Model.py: it reads the Instance.txt and creates the VRP model that needs to be solved.
##Solver.py: it creates an initial solution for the model by using the Nearest Neighbor Algorithm, then it optimizes the solution, using a Tabu Search,
which is consisted of Relocation and Swap Moves.
##Main.py: is the executable file
