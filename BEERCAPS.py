import cv2
import numpy as np;

# READ IMAGE
img = cv2.imread("bottlecaps.jpg",1)


# RESIZE IMAGE
resize = cv2.resize(img,(431,576))







# CONVERTING PICTURE TO BINARY
def convert_image(img, blur=3):
    # Convert to grayscale
    conv_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Adaptive thresholding to binarize the image
    conv_img = cv2.adaptiveThreshold(conv_img, 255,   
               cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
               cv2.THRESH_BINARY, 11, 4)
    # Blur the image to reduce noise
    conv_img = cv2.medianBlur(conv_img, blur) 
    
    return conv_img







# FINDING CIRCLES FROM PICTURE
def finding_circles(image_search,image_draw):
    circles = cv2.HoughCircles(image_search,cv2.HOUGH_GRADIENT,1.5,140,
                            param1=20,param2=10,minRadius=45,maxRadius=55)

    circles = np.uint16(np.around(circles))

    for i in circles[0,:]:
     # draw the outer circle
     cv2.circle(image_draw,(i[0],i[1]),i[2],(0,255,0),2)
     # draw the center of the circle
     cv2.circle(image_draw,(i[0],i[1]),2,(0,0,255),3)
     
        
    return image_draw




## EXTRACTING CAPS FROM PICTURE

    
    
def extract_circles(image,picture_caps):
    
    circles = cv2.HoughCircles(image,cv2.HOUGH_GRADIENT,1.5,140,
                            param1=20,param2=10,minRadius=45,maxRadius=55)

    circles = np.uint16(np.around(circles))
    
    print(circles)
    
    cropped=picture_caps[261:360,231:330]
    
    return cropped










# CALLING FUNCTIONS
    
# converting image
converted_image=convert_image(resize,blur=3)

# finding circles
founded_caps=finding_circles(converted_image,resize)

# extract caps
extracted_caps=extract_circles(converted_image,resize)







cv2.imshow('Converted picture',converted_image)
cv2.imshow('Founded caps',founded_caps)
cv2.imshow('Cropped caps',extracted_caps)


cv2.waitKey(0)
cv2.destroyAllWindows()






