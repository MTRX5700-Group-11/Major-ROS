        
### Function to convert x,y to chess square position (A1 to H8)
from cv2 import threshold

def xy2square(pix_x,pix_y):
        
    # Define the size of the image (in pixels)
    img_len_x = 640
    img_len_y = 640
    
    # Define the board size (in m)
    board_len_x = 0.48
    board_len_y = 0.48
    
    # Define the bix to board ratio for x and y
    pix2board_x = board_len_x/img_len_x
    pix2board_y = -(board_len_y/img_len_y)
    
    # Convert the pixel to board coordinates
    x = (pix_x * pix2board_x) - board_len_x
    y = (pix_y * pix2board_y) + board_len_y
    
    # 1-8, h-a
    # H1 is top left, A8 is bottom right

    # Define the centre of A8
    a8_position = [0.21,0.44]

    # Get x0 and y0 positions from A8
    x0 = a8_position[0]
    y0 = a8_position[1]

    # Define the width of each square on the chess board
    square_width = 0.06

    square_letter_val =  (x0-x)/square_width
    square_num_val = 8 - (y-y0)/square_width

    # Round to nearest int
    square_letter_int = round(square_letter_val) 
    square_num_int = round(square_num_val)
    
    # Convert letter to ascii
    square_letter_ascii = square_letter_int + ord('A')

    # Convert to letter on chess board
    square_letter_board = chr(square_letter_ascii)

    # Convert the number to a string
    square_num_board = str(square_num_int)

    # Form the chess board square
    square = square_letter_board + square_num_board

    return square

def main():
    
    #x = 0.21 - 7*0.06 
    #y = 0.44 + 7*0.06 
    
    
    # Add pixel coordinates to test
    pix_x = 0
    pix_y = 0
    
    print(xy2square(pix_x,pix_y))


if __name__ == '__main__':
    main()
