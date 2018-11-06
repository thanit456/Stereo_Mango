import construct

def test_2():
	print ("Test rotation joint")
	a = construct.box()
	a.set_dimensions(2, 4, 1)
	b = construct.box()
	b.set_dimensions(1, 10, 1)
	print (b)
	joint1 = construct.joint_rotation(a, b, construct.AXIS_Z)
	joint1.set_offset(-1, -2, 0)
	joint1.rotate(90)
	print ("rotate 90 degree on z")
	print (b)

def test_1():
	print ("Test function")
	b = construct.box()
	b.set_dimensions(1, 2, 3)
	print (b)
	b.set_position(1, 2, 3)
	print (b)
	b.set_rotation(0, 0, 90)
	print (b)

	b.set_rotation(0, 0, -90)
	b.set_rotation(90, 0, 0)
	print (b)

def main():
	test_1()
	test_2()

if __name__ == '__main__':
	main()