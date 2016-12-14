#!/usr/bin/env python

import urllib
import urllib2
import json
import sys
import os
import tempfile
import time
import tarfile
import shutil

import rospy
from std_msgs.msg import String
from rapp_core_agent.srv import StoreList, StoreListResponse

class store_interactions:

	#
	#
	#
	def get_rapp_list(self):
		url = 'https://api.rapp.cloud/rapps'
		response = urlib2.urlopen(url).read()
		rapps = json.loads(response)
		print rapps

        #
	#
	#
	def get_rapp_info(self, rapp_id):
		url = 'https://api.rapp.cloud/' + str(rapp_id)
		response = urllib2.urlopen(url).read()
		return json.loads(response)[0]
	
	#
	#
	#
	def get_rapp_file(self, rapp):
		url = 'https://api.rapp.cloud/' + rapp["rapp_id"]
		params = urllib.urlencode({
		  'token': '9a7fd032d2141b075eaf262a4478e6a6d17d7c6b6303edc08e815fc06b69cdd0'
		})
		response = urllib2.urlopen(url, params) 
		temp_file = tempfile.NamedTemporaryFile(delete=False, suffix=".tar.bz2")

		temp_file_name = temp_file.name
		self.chunk_read(response, report_hook=self.chunk_report, chunk_size=1024, out_file=temp_file)
		temp_file.close()
		
		# check whether this RApp already exists and remove directory if necessary
		rapp_dir = os.path.join(self.rapps_dir, rapp["package"])
		if os.path.isdir(rapp_dir):
			shutil.rmtree(rapp_dir)
			
			
		t = tarfile.open(temp_file_name, 'r:bz2')
		t.extractall(self.rapps_dir)
		t.close()
		
		manifest_file = open(rapp_dir + "/manifest.json")  
		manifest_data = json.load(manifest_file)
		print(manifest_data)
		return rapp_dir

	#
	#
	#
	def chunk_report(self, bytes_so_far, chunk_size, total_size):
		if total_size > 0:
			percent = float(bytes_so_far) / total_size
			percent = round(percent*100, 2)
			print("Downloaded %d of %d bytes (%0.2f%%)" % (bytes_so_far, total_size, percent))
		else:
			print("Downloaded %d bytes" % (bytes_so_far))


	#
	#
	#
	def chunk_read(self, response, chunk_size=8192, report_hook=None, size_check=False, out_file=None):
		total_size = 0
		if size_check:
			total_size = response.info().getheader('Content-Length').strip()
			total_size = int(total_size)
		
		bytes_so_far = 0

		while 1:
			chunk = response.read(chunk_size)
			bytes_so_far += len(chunk)
			
			if (out_file):
				chunk_bytes = bytearray(chunk)
				out_file.write(chunk_bytes)

			if not chunk:
				break

			if report_hook:
				report_hook(bytes_so_far, chunk_size, total_size)

		return bytes_so_far
	
	def get_rapp_list(self, robot):
		url = 'https://api.rapp.cloud/rapps'
		response = urllib2.urlopen(url).read()
		rapps = json.loads(response)
		ignored_keywords = ["test", "nao"]
		ret = {}
		for rapp in rapps:
			if rapp["robot"] != robot:
				continue
			for k in rapp["keywords"]:
				if k in ignored_keywords:
					continue
				if k in ret:
					continue
				ret[k] = rapp["rapp_id"]

		#print rapp["rapp_id"], ':', rapp["name"], rapp["robot"], rapp["keywords"]
		print ret
		return ret
	
	def handle_store_list(self, req):
		ret = self.get_rapp_list(req.robot)
		resp = StoreListResponse()
		for rapp in ret:
			resp.keys.append(rapp)
			resp.ids.append(ret[rapp])
		print resp
		return resp
	
	def callback(self, data):
		rospy.loginfo(rospy.get_caller_id() + "New request for %s", data.data)
		
		rapp = self.get_rapp_info(data.data)
		print(rapp)
		rapp_dir = self.get_rapp_file(rapp)
		
		# return path to calling process
		self.pub.publish(str(rapp_dir))

	def __init__(self):
		rospy.init_node('store_interactions', anonymous=True)
		
		self.pub = rospy.Publisher('response', String, queue_size=10)
		rospy.Subscriber('request', String, self.callback)
		
		# read base path from parameter server
		self.rapps_dir = rospy.get_param('rapps_dir', '/tmp/rapps')
		
		self.s = rospy.Service('list', StoreList, self.handle_store_list)
		
		rospy.spin()


if __name__ == '__main__':
	try:
		store_interactions()
	except rospy.ROSInterruptException:
		pass
		
# install with:
# catkin_install_python(PROGRAMS fake_store.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
