import tello
import sys
from datetime import datetime
import time

start_time = str(datetime.now())

#file_name = sys.argv[1]

#f = open(file_name, "r")

tello = tello.Tello('', 8889)
tello.send_command('land')
