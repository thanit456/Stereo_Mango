import numpy as np
import math

DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 180 / math.pi

AXIS_X = 0
AXIS_Y = 1
AXIS_Z = 2

def translation_martrix_x(x = 0):
	return np.array([x, 1, 1])

def translation_martrix_y(y = 0):
	return np.array([1, y, 1])

def translation_martrix_z(z = 0):
	return np.array([1, 1, z])

def rotation_matrix_x(x = 0):
	return np.array([[1, 0, 0],
					[0, np.cos(x * DEG_TO_RAD), -np.sin(x * DEG_TO_RAD)],
					[0, np.sin(x * DEG_TO_RAD), np.cos(x * DEG_TO_RAD)]])

def rotation_matrix_y(y = 0):
	return np.array([[np.cos(y * DEG_TO_RAD), 0, -np.sin(y * DEG_TO_RAD)],
					[0, 1, 0],
					[np.sin(y * DEG_TO_RAD), 0, np.cos(y * DEG_TO_RAD)]])

def rotation_matrix_z(z = 0):
	return np.array([[np.cos(z * DEG_TO_RAD), -np.sin(z * DEG_TO_RAD), 0],
					[np.sin(z * DEG_TO_RAD), np.cos(z * DEG_TO_RAD), 0],
					[0, 0, 1]])

class box:	
	"""docstring for box"""
	def __init__(self):
		self.origin = np.array([0, 0, 0])
		self.position = np.array([0, 0, 0])
		self.rotation = np.array([0, 0, 0])

		self.dimension = np.array([0, 0, 0])
		self.joint = None
	
	def set_dimensions(self, x, y, z):
		self.dimension = np.array([x, y, z])

	def set_origin(self, x, y, z):
		self.origin = np.array([x, y, z])

	def set_position(self, x, y, z): # from global frame to local frame
		self.position = np.array([x, y, z])

	def set_rotation(self, x, y, z):
		self.rotation = np.array([x, y, z])

	def _joint(self, jointa):
		if not isinstance(jointa, joint):
			raise Exception('Joint type isn\'t joint class')

		if self.joint != None:
			raise Exception('Already have joint.')
		self.joint = jointa

	def _unjoint(self):
		self.joint = None

	def get_origin(self):
		return self.position + self.origin

	def get_position(self): # global frame
		if self.joint != None:
			return self.joint.get_position()

		return self.position + self._calculate_position()

	def _calculate_position(self):
		position = self.dimension - self.origin
		rotation = [rotation_matrix_x, rotation_matrix_y, rotation_matrix_z]
		for i in range(3):
			position = np.dot(position, rotation[i](self.rotation[i]))
		return position

	def __str__(self):
		txt = 'Dimension: x:{}, y:{}, z:{}\n'.format(self.dimension[0], self.dimension[1], self.dimension[2])
		origin = self.joint._get_source_position() if self.joint != None else self.get_origin()
		txt += 'Origin: x:{}, y:{}, z:{}\n'.format(origin[0], origin[1], origin[2])
		position = self.get_position()
		txt += 'Position: x:{}, y:{}, z:{}'.format(position[0], position[1], position[2])
		return txt

class joint:
	"""docstring for joint"""
	def __init__(self, box_a, box_b, axis = 0):
		self.box_base = box_a
		self.box_slave = box_b
		self.box_slave._joint(self)

	def set_offset(self, x, y, z):
		self.offset = np.array([x, y, z])

	def get_position(self):
		return self._get_source_position()

	def _get_source_position(self):
		return self.box_base.get_position() + self.offset

class joint_rotation(joint):
	"""docstring for joint"""
	def __init__(self, box_a, box_b, axis = 0):
		super(joint_rotation, self).__init__(box_a, box_b, axis)

		self.offset = np.array([0, 0, 0])
		self.rotation_on_axis = axis
		self.rotation_deg = 0

	def rotate(self, deg = 0):
		self.rotation_deg += deg

	def get_position(self):
		position = self.box_slave._calculate_position()

		rotation = [rotation_matrix_x, rotation_matrix_y, rotation_matrix_z]
		position = np.dot(rotation[self.rotation_on_axis](self.rotation_deg), position)

		return self._get_source_position() + position

class joint_slide(joint):
	"""docstring for joint"""
	def __init__(self, box_a, box_b, axis = 0):
		super(joint_rotation, self).__init__(box_a, box_b, axis)

		self.offset = np.array([0, 0, 0])
		self.slide_on_axis = axis
		self.slide_position = 0

	def translate(self, pos):
		self.slide_position += pos

	def get_position(self):
		position = self.box_slave._calculate_position()

		slide = [translation_martrix_x, translation_martrix_y, translation_martrix_z]
		position = slide[self.slide_on_axis](self.slide_position) + position

		return self._get_source_position() + position