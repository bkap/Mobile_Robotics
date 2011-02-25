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
	os.system("screen rosrun delta "+Job)

JobList = os.listdir("bin")
for Job in JobList:
	t = threading.Thread(None, Run, None, [Job])
	t.start()
while 1:
	time.sleep(1)