from djitellopy import Tello
from tello_control_ui import TelloUI


def main():

    #drone = tello.Tello('', 8889)
    drone = Tello()
    vplayer = TelloUI(drone,"./img/")
    
	# start the Tkinter mainloop
    vplayer.root.mainloop() 

if __name__ == "__main__":
    main()
