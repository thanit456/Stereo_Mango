class Node(object):
	"""docstring for Node"""
	def __init__(self, x, y, z):
		super(Node, self).__init__()
		self.x = x
		self.y = y
		self.z = z

		self.cost = 0
		self.parent = None

		self.extend_failed = 0

	def set_cost(self, cost):
		self.cost = cost

	def set_parent(self, parent):
		self.parent = parent

	def get_distance(self, node):
		return math.sqrt((self.x - node.x)**2 + (self.y - node.y)**2 + (self.z - node.z)**2)

	def increse_extend_failed(self):
		self.extend_failed += 1
		

class RRT(object):
	"""docstring for RRT"""
	def __init__(self):
		super(RRT, self).__init__()

		self.nodes = [] # Node
		self.init_node = None
		self.goal_node = None

	def init(self, init_node, goal_node):
		self.init_node = init_node
		self.goal_node = goal_node

	def solve(self, iteration = 50, extend_failed_threshould = 5):
		self.init_node.set_cost(0)

		self.nodes.clear()
		self.nodes.append(self.init_node)

		while len(self.nodes) < iteration:
			new_node = self.__random_node()
			if not self.__extend(new_node):
				return False

		return True

	def __extend(self, new_node, extend_failed_threshould):
		near_node = self.__find_nearest_node(new_node, extend_failed_threshould)
		if near_node is None:
			return False

		if self.__check_collision(near_node, new_node):
			new_node.parent = near_node
			new_node.set_cost(near_node.cost + new_node.get_distance(near_node))
			self.nodes.append(new_node)

		else:
			near_node.increse_extend_failed()

		return True

	def __random_node():
		pass

	def __find_nearest_node(self, new_node, extend_failed_threshould):
		min_dist = 9999999999
		near_node = None
		for node in self.nodes:
			if node.extend_failed > extend_failed_threshould:
				continue

			if new_node.get_distance(node) < min_dist:
				near_node = node
				min_dist = new_node.get_distance(node)

		return near_node

	def __check_collision(self, near_node, new_node):
		pass

	def optimal_path(self):
		pass
		