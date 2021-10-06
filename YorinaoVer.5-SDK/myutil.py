import math
import time

class myConv:
	RAD2DEG = 180.0 / math.pi
	DEG2RAD = math.pi / 180.0


class TimeKeeper:
	SamplingTime = 0.01
	StartTime = None

	def __init__(self, sampling_time = 0.01):
		self.__SamplingTime = sampling_time
		self.StartTime = time.time()
		pass

	def SleepToKeep(self):
		elapsed_time = time.time() - self.StartTime
		time_to_sleep = self.SamplingTime - elapsed_time
		if 0 < time_to_sleep:
			time.sleep(time_to_sleep)
		else:
			pass

		self.StartTime = time.time()
