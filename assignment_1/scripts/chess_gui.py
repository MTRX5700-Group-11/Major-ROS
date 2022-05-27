#program to control a chess-playing robot
import tkinter as tk
import sunfish
import time
import random
#from chess_arm import chess_arm
from streamchessboard import StreamChessBoard
from chess_detector import ChessDetector
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from assignment_1.msg import arm_command
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
board_state = 'invalid'#the board is initially invalid till we read it
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

starting_board = list(('black_rook','black_knight','black_bishop','black_queen','black_king','black_bishop','black_knight','black_rook',
                  'black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn','black_pawn',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'empty','empty','empty','empty','empty','empty','empty','empty',
                  'white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn','white_pawn',
                  'white_rook','white_knight','white_bishop','white_queen','white_king','white_bishop','white_knight','white_rook'))



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


### Function to convert chess square position (A1 to H8) to x,y
def square2xy(square):

    square_id = list(square.upper())
    x_position = ord(square_id[0])-ord('A')

    y_position = 8-int(square_id[1])

    return x_position,y_position

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
move_msg = tk.StringVar()
capture_msg = tk.StringVar()

def move_arm_between_squares(pub):
    global start_position
    global end_position
    l_start_position = start_position.get()#extract positions from the tkinter widgets
    l_end_position = end_position.get()#extract positions from tkinter widgets
    move_piece(l_start_position,l_end_position,pub)

#attack a square and then move a piece in to replace it 
def move_piece(start_position,end_position,pub):
    command = arm_command()
    global board_state
    start_x,start_y = square2xy(start_position)#get x and y position of moves
    end_x,end_y = square2xy(end_position)#get x and y positions of moves
    start_index = list_index(start_x,start_y)
    start_piece = board_state[start_index]#extract the piece we are starting from from the array
    end_index = list_index(end_x,end_y)
    end_piece = board_state[end_index]#extract the piece at the end position from from the array
    move_text = "MOVE: " + start_position + " " + start_piece + " TO " + end_position
    print(move_text)
    if end_piece!='empty':#if the place we are moving too is not empty, run attack option to remove the piece from the board
        command.command = "Attack"
        capture_text = "CAPTURE: " + end_position + " " + end_piece 
    else:
        command.command = "Move"
        capture_text = "NO CAPTURE"
    #handle promoition
    #print('starty',start_y,'endy',end_y)
    if start_piece=='white_pawn' and end_y ==0:
        start_piece = 'white_queen'
        capture_text = capture_text + ' PROMOTION TO QUEEN'
    elif start_piece=='black_pawn' and end_y ==7:
        start_piece = 'black_queen'
        capture_text = capture_text + ' PROMOTION TO QUEEN'

    print(capture_text)

    command.start = start_position
    command.end = end_position
    #update board state
    global move_msg
    global capture_msg
    move_msg.set(move_text)
    capture_msg.set(capture_text)
    board_state[start_index] = 'empty'
    board_state[end_index] = start_piece
    board_squares = render_blank_board()
    board_squares = reset_board(board_squares,board_state)
    pub.publish(command)



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

#the get move function is a slightly modified version of the main function from sunfish
#Sunfish originally written by Thomas Ahle, Available online at https://github.com/thomasahle/sunfish
def  computer_move(pub,computer_move_msg):
    global board_state
    sunfish_board = convert_board_to_sunfish(board_state)
    hist = [sunfish.Position(sunfish_board, 0, (False,False), (False,False), 0, 0)]#Castling is banned in this game
    searcher = sunfish.Searcher()
    start = time.time()
    for _depth, move, score in searcher.search(hist[-1], hist):
        if time.time() - start > 1:
            break

    if score == sunfish.MATE_UPPER:
        print("Checkmate!")

    # The black player moves from a rotated position, so we have to
    # 'back rotate' the move before printing it.
    computer_move = "COMPUTER MOVE:" + sunfish.render(119-move[0]) + sunfish.render(119-move[1])
    computer_move_msg.set(computer_move)

    print(computer_move)
    start_position = sunfish.render(119-move[0])
    end_position = sunfish.render(119-move[1])
    move_piece(start_position,end_position,pub)

    

#for testing without the camera
def update_board_test():
    #raw_board_state = generate_random_board_state()
    global board_state
    #board_state = streamchess_state_to_board(raw_board_state)
    board_state = starting_board
    board_squares = render_blank_board()
    board_squares = reset_board(board_squares,board_state)

def update_board(stream,detector):
    #raw_board_state = generate_random_board_state()
    if stream.chess_board is not None:
        image = stream.chess_board
        cv2.imshow("Camera_Stream",stream.camera_stream)
        labelled_image,chess_state = detector.detect_image(image)
        global board_state
        board_state = streamchess_state_to_board(chess_state)
        board_squares = render_blank_board()
        board_squares = reset_board(board_squares,board_state)
        print("Chess State")
        cv2.imshow("Chess_Board",image)
        cv2.imshow("labelled_image",labelled_image)
        #print(chess_state) prints board state to command line, unneeded once we have a GUI  
    else:
        print("April Tags not found....")
        image = np.ones((640,640,1), np.uint8)*100
        cv2.imshow('Nothing to see here',image)

    cv2.waitKey()
    cv2.destroyAllWindows()


def main():
    rospy.init_node('GUI')
    pub = rospy.Publisher('arm_command', arm_command, queue_size=10)
    computer_move_msg = tk.StringVar()
    #finalise the arm
    #render the board in the GUI
    board_squares = render_blank_board()
    #board_state = starting_board
    #board_squares = reset_board(board_squares,board_state)
    rospy.Rate(1)
    stream = StreamChessBoard()#create the object to stream in the chess images
    detector = ChessDetector()#create the object to detect board state from the chess images
    #create the state of the board
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
    move_button = tk.Button(master=move_controls,text="MOVE",fg='white',bg='red',command=lambda: move_arm_between_squares(pub))
    move_button.pack()
    computer_move_button = tk.Button(master=move_controls,text="COMPUTER MOVE",fg='white',bg='blue',command=lambda: computer_move(pub,computer_move_msg))
    computer_move_button.pack()
    global computer_move_display
    computer_move_display = tk.Label(master=move_controls,textvariable=computer_move_msg,fg='white',bg='blue')
    computer_move_display.pack()
    global move_msg
    global capture_msg
    all_move_display = tk.Label(master=move_controls,textvariable=move_msg,fg='white',bg='blue')
    all_move_display.pack()
    capture_move_display = tk.Label(master=move_controls,textvariable=capture_msg,fg='white',bg='blue')
    capture_move_display.pack()

    refresh_button = tk.Button(master=move_controls,text="REFRESH BOARD",fg='white',bg='green',command=lambda: update_board(stream,detector))#for real camera
    #refresh_button = tk.Button(master=move_controls,text="REFRESH BOARD",fg='white',bg='green',command=lambda: update_board_test())#for no real camera
    refresh_button.pack()
    window.mainloop()


#button = tk.Button(master=move_controls, text="Create Piece", command=user_create_piece_click)
#button.pack()

if __name__ == '__main__':
    main()


