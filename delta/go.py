import os
import csv
import time
import threading
import thread
import shutil
import math
import random
import subprocess
import signal
def Run (Job):
	print "Running " + Job
	os.system("rosrun delta "+Job)

JobList = ["lidar", "goalpublisher", "lidarmapper", "planner", "desiredpathcrawler", "profiler", "steering"]#os.listdir("bin")
pidlist = []
for Job in JobList:
	pidlist.append(subprocess.Popen('rosrun delta ' + Job, shell=True))
	time.sleep(2)
try :
	while 1:
		time.sleep(1)
except KeyboardInterrupt, e:
	for p in pidlist[::-1] :
		os.kill(p.pid, signal.SIGTERM)
	for job in JobList :
		subprocess.call('killall -%d %s' % (signal.SIGINT,job),shell=True)
	exit(signal.SIGINT)
