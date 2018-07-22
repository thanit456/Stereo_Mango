import math

class Mango(object):
	"""docstring for Mango"""
	def __init__(self, x, y, z):
		super(Mango, self).__init__()
		# self.id = id
		self.x = x # mm
		self.y = y # mm
		self.z = z # mm

		self.already_stored = False

	def set_id(self, id):
		self.id = id

	def set_stored(self):
		self.already_stored = True

	def get_distance(self, others):
		return math.sqrt((self.x - others.x)**2 + (self.y - others.y)**2 + (self.z - others.z)**2)  # mm
		

class MangoStorage(object):
	"""docstring for MangoStorage"""
	def __init__(self):
		super(MangoStorage, self).__init__()
		self.error_distance = 100

		self.list_mango_object = []
		self.list_mango_front = 0
		self.list_mango_back = 0
		
	def set_error_distance(self, distance):
		self.error_distance = distance

	def add_mango(self, mango_add):
		check = False
		for i in range(self.list_mango_front, self.list_mango_back):
			if mango_add.get_distance(self.list_mango_object[i]) < self.error_distance:
				check = True

		if check:
			return False

		self.list_mango_object.append(mango_add)
		self.list_mango_back += 1

		return True

	def get_mango(self):
		return self.list_mango_object[self.list_mango_front]

	def pop_mango(self):
		self.list_mango_front += 1
