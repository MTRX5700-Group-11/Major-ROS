#program to control a chess-playing robot
import tkinter as tk
# Code to add widgets will go here . . .

#create the main window
window = tk.Tk()
window.title('Automated Kasparov')
#get full-screen size
window_width = window.winfo_screenwidth()
window_height = window.winfo_screenheight()
# find the center point
center_x = int(window_width/2)
center_y = int(window_height/2)
# set the position of the window to the center of the screen
window.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')

#add the program icon
img = tk.PhotoImage(file='garry_kasparov.png')
window.tk.call('wm', 'iconphoto', window._w, img)


window.mainloop()
