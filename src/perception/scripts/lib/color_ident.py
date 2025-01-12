import cv2
import os 
import numpy as np
import time
import sys
import glob

def getColor(img,img_hsv,color):
    
   
    lower_red = np.array([170,80,80])
    upper_red = np.array([179,255,255])
    
    lower_green = np.array([40,80,80])
    upper_green = np.array([90,255,255])
    
    lower_yellow = np.array([25,80,80])
    upper_yellow = np.array([35,255,255])
    
    # Threshold the HSV image to get only rgb colors
    if color.__eq__("red"):
        mask = cv2.inRange(img_hsv, lower_red, upper_red) + cv2.inRange(img_hsv, np.array([0,80,80]), np.array([20,255,255]))
    elif color.__eq__("green"):
        mask = cv2.inRange(img_hsv, lower_green, upper_green)
    elif color.__eq__("yellow"):
        mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
    else:
        print("no such color, use red/green/yellow")
        return
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8,8))
    mask_1 = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # cv2.imshow("mask",mask_1)
    # cv2.waitKey(0)
    # Find contours of the colored areas
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)   
    largest = 10
    crop_contour = None
    for contour in contours:
        
        area = cv2.contourArea(contour)
        # print(area)
        if area > largest:
            largest = area
            # print(largest)
            crop_contour = contour
    if largest > 10:
        height,width = cv2.split(crop_contour)
        crop_hmax = np.max(height)
        crop_hmin = np.min(height)
        crop_wmax = np.max(width)
        crop_wmin = np.min(width)
        
        
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img_hsv,img_hsv, mask = mask_1)
        crop_res = res[crop_wmin:crop_wmax,crop_hmin:crop_hmax]
        # crop_black = np.delete(crop_res,np.where(crop_res == [0,0,0]),axis=None)
        _,s,v = cv2.split(crop_res)
        
        # h = np.delete(h,np.where(v <= 20),None)
        # hue = np.mean(h)
        
        s = np.delete(s,np.where(v <= 80),None)
        v = np.delete(v,np.where(v <= 80),None)
        
        res = cv2.bitwise_and(img,img, mask = mask_1)
        crop_img = res[crop_wmin:crop_wmax,crop_hmin:crop_hmax]
        b,g,r = cv2.split(crop_img)
        
        b = b.flatten()
        g = g.flatten()
        r = r.flatten()
        
        if color=="red":
            r = np.delete(r,np.where(r <= 50),None)
            g = np.delete(g,np.where(r <= 50),None)
            b = np.delete(b,np.where(r <= 50),None)
        elif color=="green":
            r = np.delete(r,np.where(g <= 50),None)
            g = np.delete(g,np.where(g <= 50),None)
            b = np.delete(b,np.where(g <= 50),None)
        elif color=="yellow":
            r = np.delete(r,np.where(g <= 50) and np.where(r<=50),None)
            g = np.delete(g,np.where(g <= 50)and np.where(r<=50),None)
            b = np.delete(b,np.where(g <= 50)and np.where(r<=50),None)
        
        # print("ck1")
        b_mean = np.mean(b)
        g_mean = np.mean(g)
        r_mean = np.mean(r)
        
        sat_mean = np.mean(s)
        val_mean = np.mean(v)
        
        
        # sat_bright = np.mean(v)
        
        
        

        # gray_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # gray_crop = gray_img[crop_wmin:crop_wmax,crop_hmin:crop_hmax]
        # sat_bright = np.mean((gray_crop))
        # print(sat_bright)
        
        # cv2.imshow("gray",gray_crop)
        # cv2.imshow("mask",mask_1)
        # cv2.waitKey(0)
    else:
        b_mean = 0
        g_mean = 0
        r_mean = 0
        
        sat_mean = 0
        val_mean = 0
        # hue = 0
    
    
      
    return b_mean,g_mean,r_mean,sat_mean,val_mean,res
    # return hue, sat_mean,val_mean

def predict(strR,strG,strY):
    
    max = np.max([strR,strG*1.05,strY])
    print(max)
    if max == strR:
        return 1 #"red"
    elif max == strG*1.05:
        return 2 #"green"
    elif max == strY:
        return 2 #"green"
    else:
        return 3 #"undetermined"

def lightColor_identify(img):
    # t1 = time.time()
    img = cv2.GaussianBlur(img,(5,5),0)
    # img = img[int(w*0.2):int(w*0.8),int(h*0.2):int(h*0.8)]
    img_hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    
    b_red,g_red,r_red,sat_red,val_red,mask_r = getColor(img,img_hsv,"red")
    b_green,g_green,r_green,sat_green,val_green,mask_g = getColor(img,img_hsv,"green")
    b_yellow,g_yellow,r_yellow,sat_yellow,val_yellow,mask_y = getColor(img,img_hsv,"yellow")
    # hue_red,sat_red,val_red = getColor(img,img_hsv,"red")
    # hue_green,sat_green,val_green = getColor(img,img_hsv,"green")
    # hue_yellow,sat_yellow,val_yellow = getColor(img,img_hsv,"yellow")
    
    red = 0
    green = 0
    yellow = 0
    if sat_red>0:
        red = (r_red)*val_red/sat_red
    if sat_green>0:
        green = (g_green+b_green)*val_green/sat_green 
    if sat_yellow>0:
        yellow = (r_yellow+g_yellow)/2*val_yellow/sat_yellow    
    
    print(red,green,yellow)
    light_color = predict(red,green,yellow)
    # cv2.imshow("mask",mask_y+mask_g+mask_r)
    # cv2.waitKey(0)
    
    # print("the color of the light is " + light_color)

    return light_color
    
if __name__ == "__main__":
    # t1 = time.time()
    match = 0
    red = 0
    green = 0
    list_imgs = glob.glob("/home/slsecret/Documents/BFMC/TrafficLightClassifier/lights/*png")
    for imgpath in list_imgs:
        
        print(imgpath)
        img = cv2.imread(imgpath)
        # cv2.imshow("img",img)
    # img = cv2.resize(img,(128,64))
        light_color = lightColor_identify(img)
        # color.append(light_color)
        # strength.append(sv)
        # result = imgpath.split("/")[1].split("_")[0]

        if light_color==1:
                red +=1
        if light_color==2:
            green +=1
        # if result == light_color:
        #     print("matched")
        #     match += 1
        # else:
        #     print("not matched")
        # cv2.waitKey(0)
    print("accuracy:", match/len(list_imgs))
    print(red,green)
        
        
    
    # print("time:", time.time()-t1)