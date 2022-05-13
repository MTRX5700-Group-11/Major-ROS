#program to control a chess-playing robot
import tkinter as tk
#from PIL import ImageTk, Image  
#global constants used by the GUI
counter = 0
#dimensions of the symbols representing the pieces

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
logo = tk.PhotoImage(file='garry_kasparov.png')
window.tk.call('wm', 'iconphoto', window._w, logo)

#chess piece image are stored as a list where the zeroth entry is on a black background, 1st entry is on a white background, 
#let's load all the chess pieces, first black
black_king = (tk.PhotoImage(file='display_pieces/black_king_on_black.png'),tk.PhotoImage(file='display_pieces/black_king_on_white.png'))
black_queen = (tk.PhotoImage(file='display_pieces/black_queen_on_black.png'),tk.PhotoImage(file='display_pieces/black_queen_on_white.png'))
black_rook = (tk.PhotoImage(file='display_pieces/black_rook_on_black.png'),tk.PhotoImage(file='display_pieces/black_rook_on_white.png'))
black_bishop = (tk.PhotoImage(file='display_pieces/black_bishop_on_black.png'),tk.PhotoImage(file='display_pieces/black_bishop_on_white.png'))
black_knight = (tk.PhotoImage(file='display_pieces/black_knight_on_black.png'),tk.PhotoImage(file='display_pieces/black_knight_on_white.png'))
black_pawn = (tk.PhotoImage(file='display_pieces/black_pawn_on_black.png'),tk.PhotoImage(file='display_pieces/black_pawn_on_white.png'))
#then white
white_king= (tk.PhotoImage(file='display_pieces/white_king_on_black.png'),tk.PhotoImage(file='display_pieces/white_king_on_white.png'))
white_queen = (tk.PhotoImage(file='display_pieces/white_queen_on_black.png'),tk.PhotoImage(file='display_pieces/white_queen_on_white.png'))
white_rook = (tk.PhotoImage(file='display_pieces/white_rook_on_black.png'),tk.PhotoImage(file='display_pieces/white_rook_on_white.png'))
white_bishop = (tk.PhotoImage(file='display_pieces/white_bishop_on_black.png'),tk.PhotoImage(file='display_pieces/white_bishop_on_white.png'))
white_knight = (tk.PhotoImage(file='display_pieces/white_knight_on_black.png'),tk.PhotoImage(file='display_pieces/white_knight_on_white.png'))
white_pawn = (tk.PhotoImage(file='display_pieces/white_pawn_on_black.png'),tk.PhotoImage(file='display_pieces/white_pawn_on_white.png'))
#and the empty square
empty = (tk.PhotoImage(file='display_pieces/empty_black.png'),tk.PhotoImage(file='display_pieces/empty_white.png'))
def click_increase():
    global counter
    counter += 1
    label.config(text=counter)

def click_decrease():
    global counter
    counter -= 1
    label.config(text=counter)

#note, board position is between 0 and 7
#
def board_row_to_y(row):
    piece_size_height = 90
    global center_y
    y_out = center_y+(row-4)*piece_size_height
    return y_out

def board_col_to_x(col):
    piece_size_width = 90
    global center_x
    x_out = center_x+(col-4)*piece_size_width
    return x_out

#determine if a square is white
def is_white(x,y):
    odd_col = x%2 #will be 1 if odd, 0 if even
    odd_row = y%2 
    if odd_row:
       if odd_col:
          is_white = True
       else:
           is_white = False
    else:
        if odd_col:
          is_white = False
        else:
           is_white = True

    return is_white

#get index in the list of all squares from x/y coordinates of a square
def list_index(x,y):
    index = (y*8)+x
    return index

#add a incrementing counter
label = tk.Label(window, text=counter)

#render a blank board
board_squares = []#actual board squares being rendered
for x in range(8):
    for y in range(8):
        white = is_white(x,y)
        #if white:
        #    new_square = tk.Label(window, image=empty_white,border=0)
        #else:
        #    new_square = tk.Label(window, image=empty_black,border=0)
        new_square = tk.Label(window,image=empty[white],border=0)
        new_square.place(x=board_col_to_x(x),y=board_row_to_y(y))
        board_squares.append(new_square)

#update a single square on the board
#def update_board_square(board_squares,x,y,piece):
#    index = list_index(x,y)
#    white = is_white(x,y)
#    if piece=='white_queen':
#        if(white):


    



window.mainloop()
