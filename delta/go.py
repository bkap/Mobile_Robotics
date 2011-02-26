import os
import csv
import time
import threading
import thread
import shutil
import math
import random

def Run (Job):
	print "Running " + Job
	os.system("rosrun delta "+Job)

JobList = ["lidar", "goalpublisher", "lidarmapper", "planner", "desiredpathcrawler", "profiler", "steering"]#os.listdir("bin")
for Job in JobList:
	t = threading.Thread(None, Run, None, [Job])
	t.start()
	time.sleep(2)
while 1:
	time.sleep(1)