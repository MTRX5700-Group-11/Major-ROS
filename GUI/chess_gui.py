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

starting_board = ('black_rook','black_knight','black_bishop','black_queen','black_king','black_bishop','black_knight','black_rook',
                  'black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn',
                  'white_rook','white_knight','white_bishop','white_queen','white_king','white_bishop','white_knight','white_rook')

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
def render_blank_board():
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
    return board_squares

#update the image of a  single square on the board 
def update_board_square(board_squares,x,y,piece):
    index = list_index(x,y)
    white = is_white(x,y)
    if piece=='black_king':
        new_image = black_king[white]
    elif piece=='black_queen':
        new_image = black_queen[white]
    elif piece=='black_rook':
        new_image = black_rook[white] 
    elif piece=='black_bishop':
        new_image = black_bishop[white]    
    elif piece=='black_knight':
        new_image = black_knight[white]    
    elif piece=='black_pawn':
        new_image = black_pawn[white]
    elif piece=='white_king':
        new_image = white_king[white]
    elif piece=='white_queen':
        new_image = white_queen[white]
    elif piece=='white_rook':
        new_image = white_rook[white] 
    elif piece=='white_bishop':
        new_image = white_bishop[white]    
    elif piece=='white_knight':
        new_image = white_knight[white]    
    elif piece=='white_pawn':
        new_image = white_pawn[white]         
    elif piece=='empty':
        new_image = empty[white]
    else:
        print('error, invalid piece ', piece)
    new_square = tk.Label(window,image=new_image,border=0)
    new_square.place(x=board_col_to_x(x),y=board_row_to_y(y))
    board_squares[index] = new_square
    return board_squares

#reset board to some default position
def reset_board(board,goal_board):
    for x in range(8):
        for y in range(8):
            index = list_index(x,y)
            new_piece = goal_board[index]
            board = update_board_square(board,x,y,new_piece)
    return board

board_squares = render_blank_board()
board_squares = reset_board(board_squares,starting_board)        
#board_squares = update_board_square(board_squares,2,2,'black_pawn')
#move_controls = tk.Frame()
#input = tk.Entry(master=move_controls,width=20)
#move_controls.pack()
move_controls = tk.Frame(bg='silver')
move_controls.pack(side = tk.LEFT)

move_label = tk.Label(master=move_controls, fg='blue',bg='gold',text="Move Controls")
move_label.pack()
x_set = tk.Label(master=move_controls, fg='blue',bg='gold',text="Set X")
x_set.pack()
enter_x = tk.Entry(master=move_controls, fg='black',bg='silver',width=20)
enter_x.pack()
y_set = tk.Label(master=move_controls, fg='blue',bg='gold',text="Set Y")
y_set.pack()
enter_y = tk.Entry(master=move_controls, fg='black',bg='silver',width=20)
enter_y.pack()
piece_set = tk.Label(master=move_controls, fg='blue',bg='gold',text="Set Piece")
piece_set.pack()
enter_piece = tk.Entry(master=move_controls, fg='black',bg='silver',width=20)
enter_piece.pack()

#what should happen when the user clicks the "Create Piece Button"
def user_create_piece_click():
    global board_squares
    board_squares = update_board_square(board_squares=board_squares,x=4,y=5,piece='black_queen')

button = tk.Button(master=move_controls, text="Create Piece", command=user_create_piece_click)
button.pack()


window.mainloop()
