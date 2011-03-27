#a simple script for calling all of our various programs in a known order in one command line call

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
from optparse import OptionParser

class RosRunner:
	def __init__(self, options):
		self.options = options
		self.jobList = options.jobList.split(",")
	
	#runs a series of jobs until it recieves a keyboard interrupt (ctrl+c), and then kills them all nicely
	def runJobs(self):
		pidlist = []
		try:
			for job in self.jobList:  #for each job in the joblist
				if self.options.verbose:	print "Running %s" % job 
				pidlist.append(subprocess.Popen('rosrun delta ' + job, shell=True))  #run a job
				time.sleep(.5) #half second delay to allow things to boot up one at a time
			while 1:
				time.sleep(1) #we are waiting to be killed, so sleep

		except KeyboardInterrupt, e:   #if we get a ctrl+c
			for p in pidlist[::-1] :   #for each process
				if self.options.verbose:    print "oskilling %s, %s" % (p.pid, signal.SIGTERM)
				os.kill(p.pid, signal.SIGTERM) #send a kill signal
			for job in self.jobList:
				if self.options.verbose:	print "Killing %s" % job
				subprocess.call('killall %s -q' % job, shell=True)
			exit(signal.SIGINT)

# Command Line entry point
if __name__ == "__main__":
	parser = OptionParser()
	parser.add_option("-q", "--quiet", action="store_false", dest="verbose", default=True, help="Don't print extra status messages")
	parser.add_option("-j", "--job-list", action="store", type="string", dest="jobList", default="lidar,camera,goalpublisher,mapper,planner,desiredpathcrawler,profiler,steering", help="Specify which jobs to run, comma-separated list")
	(options, args) = parser.parse_args()
	rr = RosRunner(options)
	rr.runJobs()
