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
	
	def runJobs(self):
		pidlist = []
		try:
			for job in self.jobList:
				if self.options.verbose:	print "Running %s" % job
				pidlist.append(subprocess.Popen('rosrun delta ' + job, shell=True))
				time.sleep(.5)
			while 1:
				time.sleep(1)

		except KeyboardInterrupt, e:
			for p in pidlist[::-1] :
				if self.options.verbose:    print "oskilling %s, %s" % (p.pid, signal.SIGTERM)
				os.kill(p.pid, signal.SIGTERM)
			for job in self.jobList:
				if self.options.verbose:	print "Killing %s" % job
				subprocess.call('killall %s -q' % job, shell=True)
			exit(signal.SIGINT)

# Command Line entry point
if __name__ == "__main__":
	parser = OptionParser()
	parser.add_option("-q", "--quiet", action="store_false", dest="verbose", default=True, help="Don't print extra status messages")
	parser.add_option("-j", "--job-list", action="store", type="string", dest="jobList", default="lidar,goalpublisher,lidarmapper,planner,desiredpathcrawler,profiler,steering", help="Specify which jobs to run, comma-separated list")
	(options, args) = parser.parse_args()
	rr = RosRunner(options)
	rr.runJobs()
