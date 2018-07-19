import time
import hashlib
import json
import requests

secret_key = b'Eic981234'

def generate_otp():
	timestamp = int(time.time()*1000)
	timestamp = str(timestamp)
	h = hashlib.sha256()
	h.update(timestamp.encode() + secret_key)
	return h.digest()[:8].hex()+timestamp


class DriverMotor(object):
	"""docstring for DriverMotor"""
	def __init__(self, host, id, is_servo = False):
		super(DriverMotor, self).__init__()
		self.url_host = host
		self.id = id
	
	def get_goal(self):
		url = "{}/{}/status/goal".format(self.url_host, self.id)
		result = requests.post(url, data={"token": generate_otp()})

		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

	def get_current(self):
		url = "{}/{}/status/motion".format(self.url_host, self.id)
		result = requests.post(url, data={"token": generate_otp(), ""})

		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

	def get_others(self):
		url = "{}/{}/status/others".format(self.url_host, self.id)
		result = requests.post(url, data={"token": generate_otp(), ""})

		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

	def set_goal_pwm(self, pwm)
		url = "{}/{}/goal/set".format(self.url_host, self.id)
		result = requests.post(url, data={"token": generate_otp(), "goal_pwm": pwm})

		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

	def set_goal_pos(self, pos)
		url = "{}/{}/goal/set".format(self.url_host, self.id)
		result = requests.post(url, data={"token": generate_otp(), "goal_pos": pos})

		if (result.status_code != 200)
			raise Exception('HTTP Error: {}'.format(result.status_code))

		return json.loads(result.text)

class DriverLaser(object):
	"""docstring for DriverLaser"""
	def __init__(self, host, id):
		super(DriverLaser, self).__init__()
		self.url_host = host
		self.id = id

	def get_length(self):
		pass
		