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
try:
	for Job in JobList:
		pidlist.append(subprocess.Popen('rosrun delta ' + Job, shell=True))
		time.sleep(2)
	while 1:
		time.sleep(1)

except KeyboardInterrupt, e:
	for p in pidlist[::-1] :
		p.send_signal(signal.SIGTERM)
	for job in JobList :
		subprocess.call('killall -%d %s' % (signal.SIGTERM,job),shell=True)
	exit(0)
