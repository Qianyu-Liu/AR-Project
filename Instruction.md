# AR-Project instruction

Terminal :`xhost +`

#Run Task1 in Rec file:
Put the task 1 rec file in :~/kiwi-recordings
opendlv-tutorial-kiwi/ForRecFile: Modify the rec file name in h264-replay-viewer.yml line 32
opendlv-tutorial-kiwi/ForRecFile: `docker-compose -f h264-replay-viewer.yml up`
opendlv-tutorial-kiwi/opendlv-perception-helloworld-cpp: `docker-compose -f Run-Kiwi-in-RecFile.yml up`

#Run Task2 in Rec file:
Put the task2 rec file in ~/kiwi-recordings
opendlv-tutorial-kiwi/ForRecFile: Modify the rec file name in h264-replay-viewer.yml line 32
opendlv-tutorial-kiwi/ForRecFile: `docker-compose -f h264-replay-viewer.yml up` 
opendlv-tutorial-kiwi/opendlv-perception-helloworld-cpp: `docker-compose -f Run-Kiwi-in-RecFile.yml up`


#Run Task3 in Rec file:
Put the task3 rec file in ~/kiwi-recordings
opendlv-tutorial-kiwi/ForRecFile: Modify the rec file name in h264-replay-viewer.yml line 32
opendlv-tutorial-kiwi/ForRecFile: `docker-compose -f h264-replay-viewer.yml up` 
opendlv-tutorial-kiwi/opendlv-perception-helloworld-cpp: `docker-compose -f Run-Kiwi-in-RecFile.yml up`

#Run Task 1 in simulation:
opendlv-tutorial-kiwi/ForSimulation:`docker-compose -f simulation-kiwi.yml up`
