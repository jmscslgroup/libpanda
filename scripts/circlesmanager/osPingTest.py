import os


if os.system("./simplePing.sh") == 0:
	print("Condition Ping Success")
else:
	print("condition Ping Failure")
