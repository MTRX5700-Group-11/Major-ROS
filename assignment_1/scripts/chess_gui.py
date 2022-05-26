#program to control a chess-playing robot
import tkinter as tk
import sunfish
import time
import random
from chess_arm import chess_arm
from streamchessboard import StreamChessBoard
from chess_detector import ChessDetector
import numpy as np
import cv2
import rospy
#from PIL import ImageTk, Image  
#global constants used by the GUI
counter = 0
#dimensions of the symbols representing the pieces

#create the main window
window = tk.Tk()
#window.attributes('-zoomed', True)
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
logo = tk.PhotoImage(file='GUI/garry_kasparov.png')
window.tk.call('wm', 'iconphoto', window._w, logo)

#chess piece image are stored as a list where the zeroth entry is on a black background, 1st entry is on a white background, 
#let's load all the chess pieces, first black
black_king = (tk.PhotoImage(file='GUI/display_pieces/black_king_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/black_king_on_white.png'))
black_queen = (tk.PhotoImage(file='GUI/display_pieces/black_queen_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/black_queen_on_white.png'))
black_rook = (tk.PhotoImage(file='GUI/display_pieces/black_rook_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/black_rook_on_white.png'))
black_bishop = (tk.PhotoImage(file='GUI/display_pieces/black_bishop_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/black_bishop_on_white.png'))
black_knight = (tk.PhotoImage(file='GUI/display_pieces/black_knight_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/black_knight_on_white.png'))
black_pawn = (tk.PhotoImage(file='GUI/display_pieces/black_pawn_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/black_pawn_on_white.png'))
#then white
white_king= (tk.PhotoImage(file='GUI/display_pieces/white_king_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/white_king_on_white.png'))
white_queen = (tk.PhotoImage(file='GUI/display_pieces/white_queen_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/white_queen_on_white.png'))
white_rook = (tk.PhotoImage(file='GUI/display_pieces/white_rook_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/white_rook_on_white.png'))
white_bishop = (tk.PhotoImage(file='GUI/display_pieces/white_bishop_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/white_bishop_on_white.png'))
white_knight = (tk.PhotoImage(file='GUI/display_pieces/white_knight_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/white_knight_on_white.png'))
white_pawn = (tk.PhotoImage(file='GUI/display_pieces/white_pawn_on_black.png'),tk.PhotoImage(file='GUI/display_pieces/white_pawn_on_white.png'))
#and the empty square
empty = (tk.PhotoImage(file='GUI/display_pieces/empty_black.png'),tk.PhotoImage(file='GUI/display_pieces/empty_white.png'))

starting_board = ('black_rook','black_knight','black_bishop','black_queen','black_king','black_bishop','black_knight','black_rook',
                  'black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn',
                  'white_rook','white_knight','white_bishop','white_queen','white_king','white_bishop','white_knight','white_rook')

sunfish_state2 = (
    '         \n'  #   0 -  9
    '         \n'  #  10 - 19
    ' rnbqkbnr\n'  #  20 - 29
    ' pppppppp\n'  #  30 - 39
    ' ........\n'  #  40 - 49
    ' ........\n'  #  50 - 59
    ' ........\n'  #  60 - 69
    ' ........\n'  #  70 - 79
    ' PPPPPPPP\n'  #  80 - 89
    ' RNBQKBNR\n'  #  90 - 99
    '         \n'  # 100 -109
    '         \n'  # 110 -119
)

sunfish_state3 = (
    '         \n'  #   0 -  9
    '         \n'  #  10 - 19
    ' rqbqkbqr\n'  #  20 - 29
    ' pppppppp\n'  #  30 - 39
    ' ........\n'  #  40 - 49
    ' .....q..\n'  #  50 - 59
    ' ..K.....\n'  #  60 - 69
    ' ........\n'  #  70 - 79
    ' PPpPPPPP\n'  #  80 - 89
    ' RNBQKBnR\n'  #  90 - 99
    '         \n'  # 100 -109
    '         \n'  # 110 -119
)

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
          is_white = False
       else:
           is_white = True
    else:
        if odd_col:
          is_white = True
        else:
           is_white = False

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
        new_image = empty[white]

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

#what should happen when the user clicks the "Create Piece Button"
#def user_create_piece_click():
#    global board_squares
#    board_squares = update_board_square(board_squares=board_squares,x=4,y=5,piece='black_queen')

  
#board_squares = update_board_square(board_squares,2,2,'black_pawn')
#move_controls = tk.Frame()
#input = tk.Entry(master=move_controls,width=20)
#move_controls.pack()

#convert name of piece from our format to sunfish format
def convert_name_to_sunfish(piece):
    #note, sunfish uses bold for white characters, normal case for black
    if piece=='black_king':
        new_name = 'k'
    elif piece=='black_queen':
        new_name = 'q'
    elif piece=='black_rook':
        new_name = 'r' 
    elif piece=='black_bishop':
        new_name = 'b'    
    elif piece=='black_knight':
        new_name = 'n'    
    elif piece=='black_pawn':
        new_name = 'p'
    elif piece=='white_king':
        new_name = 'K'
    elif piece=='white_queen':
        new_name = 'Q'
    elif piece=='white_rook':
        new_name = 'R' 
    elif piece=='white_bishop':
        new_name = 'B'    
    elif piece=='white_knight':
        new_name = 'N'    
    elif piece=='white_pawn':
        new_name = 'P'         
    elif piece=='empty':
        new_name = '.'
    return new_name

#convert name from sunfish format back to our format
def convert_sunfish_to_name(piece):
    if piece=='k':
        new_name = 'black_king'
    elif piece=='q':
        new_name = 'black_queen'
    elif piece=='r':
        new_name = 'black_rook' 
    elif piece=='b':
        new_name = 'black_bishop'    
    elif piece=='n':
        new_name = 'black_knight'    
    elif piece=='p':
        new_name = 'black_pawn'
    elif piece=='K':
        new_name = 'white_king'
    elif piece=='Q':
        new_name = 'white_queen'
    elif piece=='R':
        new_name = 'white_rook' 
    elif piece=='B':
        new_name = 'white_bishop'    
    elif piece=='N':
        new_name = 'white_knight'    
    elif piece=='P':
        new_name = 'white_pawn'         
    elif piece=='.':
        new_name = 'empty'
    else:
        #this is padding from sunfish's format, which shold be discarded
        new_name = 'pad'
    return new_name



#convert the board state in our format to sunfish format
def convert_board_to_sunfish(board):
    padding = '         \n'
    #add padding at the top, as required by sunfish
    sunfish_string = padding+padding
    for y in range(8):
        sunfish_string +=' '
        for x in range(8):
            index = list_index(x,y)
            piece = board[index]#get the name of the piece under our definition
            sunfish_name = convert_name_to_sunfish(piece)
            sunfish_string += sunfish_name
        sunfish_string +='\n'
    #add padding at the bottom, as required by sunfish
    sunfish_string += padding
    sunfish_string += padding
    return sunfish_string

#convert the board state from the 
def convert_sunfish_to_board(sunfish_board):
    board = []
    for i in sunfish_board:
        piece_name = convert_sunfish_to_name(i)
        if(piece_name=='pad'):
            continue
        else:
            board.append(piece_name)
    return board


#convert the numeric format from streamchess into the format used by the GUI
def streamchess_num_to_name(piece_number):
    piece_name = 'empty'
    if piece_number==1:
        piece_name = 'white_king'
    elif piece_number==2:
        piece_name = 'white_queen'
    elif piece_number==3:
        piece_name = 'white_bishop'
    elif piece_number==4:
        piece_name = 'white_knight'
    elif piece_number==5:
        piece_name = 'white_rook'
    elif piece_number==6:
        piece_name = 'white_pawn'
    elif piece_number==7:
        piece_name = 'black_king'
    elif piece_number==8:
        piece_name = 'black_queen'
    elif piece_number==9:
        piece_name = 'black_bishop'
    elif piece_number==10:
        piece_name = 'black_knight'
    elif piece_number==11:
        piece_name = 'black_bishop'
    elif piece_number==12:
        piece_name = 'black_pawn'
    return piece_name


#convert the state of the board in streamchess format to our own format
def streamchess_state_to_board(chess_state):
    #go through all points in the board
    board = []#generate the blank board
    for r in range(8):
        for c in range(8):
            piece_number = chess_state[r][c]#find the piece number from the streamchess board state
            piece_name = streamchess_num_to_name(piece_number)#convert the number to the correct name for the GUI format
            board.append(piece_name)
    return board




start_position=tk.StringVar()
end_position=tk.StringVar()

def move_arm_between_squares(arm):
    global start_position
    global end_position
    arm.move_piece(start_position.get(),end_position.get())
    return 
 




def generate_random_board_state():
    #generate a board state comparable too that made by the chess_detector class, but random
    chess_state = np.zeros((8,8))
    for r in range(8):
        for c in range(8):
            blank = random.randint(1,2)
            if(blank==1):#setup half the squares to have a piece
                chess_state[r][c] = random.randint(1,12)
            else:
                chess_state[r][c] = 0#and other half to be blank

    return chess_state


def update_board(stream,detector):
    #raw_board_state = generate_random_board_state()
    image = stream.chess_board
    cv2.imshow("Camera_Stream",stream.camera_stream)
    if stream.chess_board is not None:
        labelled_image,chess_state = detector.detect_image(image)
        board_state = streamchess_state_to_board(chess_state)
        board_squares = render_blank_board()
        board_squares = reset_board(board_squares,board_state)
        print("Chess State")
        print(chess_state)
    else:
        print("April Tags not found....")


def main():
    
    arm = chess_arm()#initialise the arm
    #render the board in the GUI
    board_squares = render_blank_board()
    #board_state = starting_board
    #board_squares = reset_board(board_squares,board_state)
    rospy.Rate(1)
    stream = StreamChessBoard()#create the object to stream in the chess images
    detector = ChessDetector()#create the object to detect board state from the chess images
    #create the state of the board
    white_castle_rights = (True,True)#Is it still possible to castle on Queen and King Side Respectively
    black_castle_rights = (True,True)
    move_controls = tk.Frame(bg='silver')
    move_controls.pack(side = tk.LEFT)
    move_label = tk.Label(master=move_controls, fg='blue',bg='gold',text="Piece Move Controls")
    move_label.pack()
    start_position_display = tk.Label(master=move_controls, fg='blue',bg='gold',text="Enter Start Position")
    start_position_display.pack()
    enter_start = tk.Entry(master=move_controls, fg='black',bg='silver',width=20,textvariable=start_position)
    enter_start.pack()
    end_position_display = tk.Label(master=move_controls, fg='blue',bg='gold',text="Enter End Position")
    end_position_display.pack()
    enter_end = tk.Entry(master=move_controls, fg='black',bg='silver',width=20,textvariable=end_position)
    enter_end.pack()
    move_button = tk.Button(master=move_controls,text="MOVE",fg='white',bg='red',command=lambda: move_arm_between_squares(arm))
    move_button.pack()
    refresh_button = tk.Button(master=move_controls,text="REFRESH BOARD",fg='white',bg='green',command=lambda: update_board(stream,detector))
    refresh_button.pack()
    window.mainloop()


#button = tk.Button(master=move_controls, text="Create Piece", command=user_create_piece_click)
#button.pack()

if __name__ == '__main__':
    main()


