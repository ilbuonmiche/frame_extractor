######## PACKAGE DESCRIPTION ###############

This package has been used for the extraction of the data contained in the bagfiles recorded with the car FLUENCE.
Since the topics has in general different frequencies and we want to have for every frequency 
An approximate time policy has been used in ord


######## USAGE #############################

Don't visualize the images while you want to
actually write the images on the disk because we want to save computational power not to miss frames.

Be carefull not to read the bag and write the extracted data on the same external hard disk because the bus cannot handle
the read and write operations with sufficient speed and some frames will be lost.

frame_gps_odom_sync_extractor_fisheye.launch -->  launches frame_extractor for extracting fisheye images
frame_gps_odom_sync_extractor.launch         -->  launches frame_extractor for extracting normal images
