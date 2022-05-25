        
### Function to convert x,y to chess square position (A1 to H8)
from cv2 import threshold

def xy2square(pix_x,pix_y):
        
    # Define the size of the image (in pixels)
    img_len_x = 640
    img_len_y = 640
    
    # Define the board size (in cm)
    board_len_x = 48
    board_len_y = 48
    
    # Define the bix to board ratio for x and y
    pix2board_x = board_len_x/img_len_x
    pix2board_y = (board_len_y/img_len_y)
    
    # Convert the pixel to board coordinates
    x = (pix_x * pix2board_x)
    y = (pix_y * pix2board_y)
    
    # 1-8, h-a
    # H1 is top left, A8 is bottom right

    # Define the centre of A8
    a1_position = [45,45]

    # Get x0 and y0 positions from A8
    x0 = a1_position[0]
    y0 = a1_position[1]

    # Define the width of each square on the chess board
    square_width = 6

    r =  (x0-x)/square_width
    c =  (y0-y)/square_width
    

    return int(r),int(c)

def main():
    
    #x = 0.21 - 7*0.06 
    #y = 0.44 + 7*0.06 
    
    
    # Add pixel coordinates to test
    pix_x = 640
    pix_y = 640
    
    print(xy2square(pix_x,pix_y))


if __name__ == '__main__':
    main()
