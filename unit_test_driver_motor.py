import driver

def asset_true(key, value):
	if key != value:
		e = "Error"
		print (e)
		raise e

	print ("Pass")

test_1 = driver.DriverMotor(1)

test_1.set_pulse_per_mm(8000)
asset_true(test_1.ppmm, 8000)