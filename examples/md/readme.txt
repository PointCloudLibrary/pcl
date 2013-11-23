
Readme file for the multi-descriptor project

(1) to run, use the supplied script (run-md), or the executable directly (md)

(2) contents of the supplied script:

./md inputCloud0.pcd inputCloud1.pcd 65 5 2 70 70  0.01 0.2 0.1 0.01 0 1 0 perf01.txt results01.txt

argv[1]:  input cloud 1
argv[2]:  input cloud 2
argv[3]:  max_simulated rotation in degrees
argv[4]:  resolution of the simulated rotation in degrees
argv[5]:  number of threads, 2 is recommended
argv[6]:  # of best correspondeces to use in the 2D domain, 70-140 is recommended
argv[7]:  # of best correspondeces to use in the 3D domain, 70-140 is recommended
argv[8]:  downsampling leaf size: 0.01 to 0.03 is recommended.
argv[9]:  voting threshold, 0.2-0.3 is recommended
argv[10]: ransac threshold in the 2D domain, 0.1 is recommended
argv[11]: ransac threshold in the 3D domain, 0.01 is recommended
argv[12]: enable_graphics: 1/0, if set to 1, images of the processed clouds are displayed.
argv[13]: debug_level: -1: displays runtime performance
                        0: displays nothing
                        1: displays md error performance summary
                        2: displays associated transformations
                       >2: displays additional details.
argv[14]: live_sensor: whether the input clouds (1 and 2) were rotated externally,
                      ie, no need for simulated rotation.  If set to 1, then 
                      argv[3] and argv[4] should be 5 and 5 respectively.
                      note: the code supports horizontal rotation only as a reference
                            for md error measurement
argv[15], argv[16]: log files for further processing.

Other internal parameters:

MIN_CORRESPONDENCE_COUNT: used to control which corresspondence results would contribute to
                          the voting process.

  
