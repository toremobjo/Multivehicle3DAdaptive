# imc_ros_interface
Message and vehicle handling interface between vehicles running DUNE and ROS. 

# Python AUV Handler Class
The AuvHandler class is made for easy integration to any vehicle running DUNE.

### AuvHandler(node_name, configpath = "empty" )
node_name (str): the node name of the ROS node initiating the class.  
configpath (str): not implemented yet, default="empty".

### AuvHandler.publishLogentry(what, type)
A function for publishing a log-entry to the Neptus log book, handy for notifications in Neptus.   

what (str): Plaintext of the log entry    
type (int): Type of log entry; 0=info, 1=warning, 2=error, 3=critical, 4=debug

### AuvHandler.setWaypoint(lat, lon, z = 0.0, z_units = 1, speed = 1.5, speed_units = 0)
Function for setting the desired waypoint for the vehicle.    

lat (float): latitude in radians    
lon (float): longitude in radians    
z (float): depth/z in meters   
z_units (int): units of the z-parameter; 0=NONE, 1=Depth, 2=Altitude, 3=Height   
speed (float): desired speed   
speed_units (int): units of the speed variable; 0 = meters per second, 1 = RPM, 2 = percent   

### AuvHandler.getState()
Returns the state of the vehicle, the possible states are:  
"starting" : vehicle is starting and waiting for followreference to be started from Neptus.         
"executing" : vehicle is executing a maneuver commanded from the adaframe.     
"waiting" : vehicle has arrived at the given waypoint and is waiting for your code to give it another.    
"unavailable" : vehicle is unavailable, most likely due to some error. 


