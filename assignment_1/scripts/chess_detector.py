import cv2
import numpy as np
import os
import re
from xy2square import xy2square
from chess_pieces import label2number

class ChessDetector():
    def __init__(self) :
        
        #constants
        self.input_width = 640
        self.input_height = 640
        self.score_threshold = 0.5
        self.nms_threshold = 0.45
        self.confidence_threshold = 0.45

        #text_parameters
        self.font_face = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.7
        self.thickness = 1

        #colors
        self.black=(0,0,0)
        self.blue=(255,178,50)
        self.yellow=(0,255,255)

        dirname = os.path.dirname(__file__)
        classes_filename = os.path.join(dirname, 'detection_files/coco.names')
        # Load class names.
        self.classesFile = classes_filename
        self.classes = None
        with open(self.classesFile, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

        model_filename = os.path.join(dirname, 'detection_files/chess.onnx')
        self.modelWeights = model_filename
        self.net = cv2.dnn.readNet(self.modelWeights)
        
        self.labelled_image = None
    
    

    def detection2matrix(self,labels,x_centres,y_centres):
        chess_state = np.zeros((8,8))

        for i in range(0,len(labels)):
            r,c = xy2square(x_centres[i],y_centres[i])
            chess_state[r][c]=label2number(labels[i])
        return chess_state

    
    def detect_image(self,image):
        self.image = image
        detections = self.pre_process(image,self.net)
        img,labels,x_centres,y_centres = self.post_process(image.copy(), detections)
        t, _ = self.net.getPerfProfile()
        label = 'Inference time: %.2f ms' % (t * 1000.0 /  cv2.getTickFrequency())
        print(label)
        cv2.putText(img, label, (20, 40), self.font_face, self.font_scale,  (0, 0, 255), self.thickness, cv2.LINE_AA)
        self.labelled_image = img
        state = self.detection2matrix(labels,x_centres,y_centres)
        return img,state
        
    def draw_label(self,im, label, x, y):
        """Draw text onto image at location."""
        # Get text size.
        text_size = cv2.getTextSize(label, self.font_face, self.font_scale, self.thickness)
        dim, baseline = text_size[0], text_size[1]
        # Use text size to create a self.black  rectangle.
        cv2.rectangle(im, (x,y), (x + dim[0], y + dim[1] + baseline), (0,0,0), cv2.FILLED)
        # Display text inside the rectangle.
        cv2.putText(im, label, (x, y + dim[1]), self.font_face, self.font_scale, self.yellow, self.thickness, cv2.LINE_AA)

    def pre_process(self,input_image, net):
        # Create a 4D blob from a frame.
        blob = cv2.dnn.blobFromImage(input_image, 1/255,  (self.input_width, self.input_height ), [0,0,0], 1, crop=False)

        # Sets the input to the network.
        net.setInput(blob)

        # Run the forward pass to get output of the output layers.
        outputs = net.forward(net.getUnconnectedOutLayersNames())
        return outputs

    def post_process(self,input_image, outputs):
        # Lists to hold respective values while unwrapping.
        class_ids = []
        confidences = []
        boxes = []
        # Rows.
        rows = outputs[0].shape[1]
        image_height, image_width = input_image.shape[:2]
        # Resizing factor.
        x_factor = image_width / self.input_width
        y_factor =  image_height / self.input_height 
        # Iterate through detections.
        for r in range(rows):
            row = outputs[0][0][r]
            confidence = row[4]
            # Discard bad detections and continue.
            if confidence >= self.confidence_threshold :
                    classes_scores = row[5:]
                    # Get the index of max class score.
                    class_id = np.argmax(classes_scores)
                    #  Continue if the class score is above threshold.
                    if (classes_scores[class_id] > self.score_threshold):
                        confidences.append(confidence)
                        class_ids.append(class_id)
                        cx, cy, w, h = row[0], row[1], row[2], row[3]
                        left = int((cx - w/2) * x_factor)
                        top = int((cy - h/2) * y_factor)
                        width = int(w * x_factor)
                        height = int(h * y_factor)
                        box = np.array([left, top, width, height])
                        boxes.append(box)

        # Define the arrays in which the labels, and it's x and y centrer are stored
        labels = []
        x_centres = []
        y_centres = []
        
        # Perform non maximum suppression to eliminate redundant, overlapping boxes with lower confidences.
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold , self.nms_threshold)
        for i in indices:
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]             
            # Draw bounding box.             
            cv2.rectangle(input_image, (left, top), (left + width, top + height), self.blue, 3*self.thickness)
            # Class label.                      
            label = "{}:{:.2f}".format(self.classes[class_ids[i]], confidences[i])             
            # Draw label.             
            self.draw_label(input_image, label, left, top)
            
            # Find the centre of the label
            x_centre = left + width/2
            y_centre = top + height/2
            
            # Add the label and it's centre to the appropriate array
            labels.append(label)
            x_centres.append(x_centre)
            y_centres.append(y_centre)
            
        return input_image,labels,x_centres,y_centres

if __name__ == '__main__':
    detector = ChessDetector()

    pass    