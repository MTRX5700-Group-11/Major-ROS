
def label2number(label):
        if('king_white' in label):
            return 1
        elif('queen_white' in label):
            return 2
        elif('bishop_white' in label):
            return 3
        elif('knight_white' in label):
            return 4
        elif('rook_white' in label):
            return 5
        elif('pawn_white' in label):
            return 6
        elif('king_black' in label):
            return 7
        elif('queen_black' in label):
            return 8
        elif('bishop_black' in label):
            return 9
        elif('knight_black' in label):
            return 10
        elif('rook_black' in label):
            return 11
        elif('pawn_black' in label):
            return 12

def number2label(number):
        if(number==1):
            return 'king_white'
        elif(number==2):
            return 'queen_white'
        elif(number==3):
            return 'bishop_white'
        elif(number==4):
            return 'knight_white'
        elif(number==5):
            return 'rook_white'
        elif(number==6):
            return 'pawn_white'
        elif(number==7):
            return 'king_black'
        elif(number==8):
            return 'queen_black'
        elif(number==9):
            return 'bishop_black'
        elif(number==10):
            return 'knight_black'
        elif(number==11):
            return 'rook_black'
        elif(number==12):
            return 'pawn_black'