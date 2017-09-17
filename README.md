# Irradiance_sensing
Irradiance sensing with MDA300 data acquisition board and MICAz motes in EPIC-RoofNet


#Description:
The EPIC-RoofNet was deployed in the roof of the Energy Production and Infrastructure Center (EPIC) building at UNC Charlotte
to provide an experimental platform for this research. EPIC-RoofNet has been designed primarily with two objectives. Firstly, it comprises of a set of pyranometer sensor nodes that are essential to obtain solar irradiance data from wireless sensor nodes placed in outdoor settings. This data is used for developing statistical models characterizing the spatial and temporal variations of solar energy available at the nodes. Secondly, it is used for a platform for experimental validation and performance evaluation of adaptive network protocols and energy-conservation schemes that is the focus of this research. 

Each sensor motes run Tinyos-2.x on Micaz sensor motes. Also to avoid idle listening, nodes use low-power listening (LPL) with IEEE 802.15.4 MAC where they sleep most of the time and wakes up in a periodic interval. If they sense the channel to be busy, they remain on. Otherwise, they go back to sleep to conserve energy. The periodic wake-up interval is assumed to be 128 milliseconds. The beacon interval and the data interval are assumed to be 15 minutes and 5 minutes respectively. All the nodes sense every 5 minutes and forward these sensing values to the sink. The laptop computer running the base server/sink is connected to the Internet via UNC Charlotte WiFi network and automatically transfers the data gathered from the testbed to a Dropbox database for offline monitoring. 

To measure the solar irradiance measurements, a pyranometer with a MDA300 data acquisition board attached to a MICAz mote.
The MDA300 is designed as a general measurement platform for the MICAz nodes. These devices can interface with various analog sensors or with digital devices. The mote can sample from the MDA300, and it can actuate other devices via a set of output terminals. Other than sensing solar irradiance values, the mote senses ambient temperature from using the MDA along with the current battery voltage. Nodes are placed in different places of the roof with different orientations, so that a wide variety of
spatial variance of solar irradiance can be captured.

#Collcted data:
The sample irradiance data is captured fro several months. These samples are made availabe inside the "All_Data_For_EPIC_Roof" file. 


