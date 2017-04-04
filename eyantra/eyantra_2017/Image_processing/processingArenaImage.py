
```
* Team Id : eyrc-cc#2755
* Author List : Sumit Kumar Pradhan, Ronak Bansal
* Filename: processingArenaImage.py
* Theme: Cross a crater
* Functions: err_calc()
             match_tmplt()
             check_sum()
```



import cv2
import numpy as np
import matplotlib.pyplot as plt
import serial


###############GLOBAL VARIABLES###################

 
```
* Function Name: err_calc
* Input: detected image: det_img
*		 template image: tmplt_img 
* Output: mean square error between detected image and template image: err
* Logic: This function takes two images (detected image and a template image) as input and calculates the mean square error between the two.
*		 The error calculated is used to determine whether the detected image is same as the template image or not. 
* Example Call: error=err_calc(det_img,tmplt_img);
```    
#error calculation function
def err_calc(det_img,tmplt_img):
    err = np.sum((det_img.astype('float') - tmplt_img.astype('float')) ** 2)
    err /= float(det_img.shape[0] * tmplt_img.shape[1])
    return err

```
* Function Name: match_tmplt
* Input: two images: (i) tst_img: the main image in which the template image is to be matched
*					 (ii)tmplt_img: the template image which needs to be matched (or searched) in the main image
* Output:  a list : [ <calculated error between detected image and template image> , [<top left coordinate of detected image>, <bottom right coordinate of detected image>]]
* Logic:  In this function the template image is searched in main image using the cv2.matchTemplate function.
*		  The detected image is then compared with the template image to check whether the detected image is the required image or not by calculating the mean square error using the err_calc function.
* Example Call: result = match_tmplt(tst_img,tmplt_img)
```
#TEMPLATE MATCHING FUNCTION
def match_tmplt(tst_img,tmplt_img):
    template=tmplt_img.copy();

    method=cv2.TM_SQDIFF_NORMED;
    res = cv2.matchTemplate(tst_img,template,method)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        top_left = min_loc
    else:
        top_left = max_loc
        
    bottom_right = (top_left[0] + template.shape[1], top_left[1] + template.shape[0])

    disp_img=tst_img.copy()

    cv2.rectangle(disp_img,top_left, bottom_right, 255, 1)

    detected_img=tst_img.copy()[top_left[1]:top_left[1]+template.shape[0],top_left[0]:top_left[0]+template.shape[1],:];
    error=err_calc(detected_img,template);
    return [error,top_left,bottom_right]; 


```
* Function Name: check_sum
* Input: bz: boulder numbers
*		 sum: Provided sum
*		 noc: number of craters (of bridge 1 or 2)
* Output: a list: [<True or False>,  null: if no bridge satisfies sum 'or' boulder position if any bridge satisfies sum]
* Logic: Sum of all possible combinations of boulder numbers were calculated and were checked whether it is equal to the given sum.
* Example Call: check_sum(bz,sum,noc)
```
#CHECK SUM
def check_sum(bz,sum,noc):

    boulder_pick=[];
    for i in range(len(bz)-noc+1):

        for j in range(i+1,len(bz)-noc+2):

            for k in range(j+1,len(bz)-noc+3):

                temp_sum=bz[i]+bz[j]+bz[k];

                if(temp_sum==sum):
                    boulder_pick=[i+1,j+1,k+1]
                    return [True , boulder_pick];
    return [False,null];


        
#######TEMPLATES##########################################

#ARENA IMAGE INPUT
#####################
cam = cv2.VideoCapture(0)
s, im = cam.read() # captures image
cam.release()
plt_img(im);
img=im.copy();


##NUMBERS TEMPLATES:
tmplt=[];
for i in range(0,10):
    tmp_img=cv2.imread("templates/"+str(i)+".png")
    tmplt.append(tmp_img.copy());

#CRATER TEMPLATE INPUT
crater_tmplt=cv2.imread("templates/crater.png")

#OBSTACLE TEMPLATE INPUT
obs_tmplt=cv2.imread("templates/obs.png")

########################################################

###############################################################
#DETECTION
###############################################################

#DETECTING BOULDERS
bz=[-1,-1,-1,-1];
for i in range(len(tmplt)):
    res=match_tmplt(img.copy(),tmplt[i]);
    if(res[0]<1000):
        if(res[1][1]<img.shape[0]/2):
            if(res[1][0]<img.shape[1]-100):
                bz[2]=i;
            else:
                bz[3]=i;
        else:
            if(res[1][0]<img.shape[1]-100):
                bz[1]=i;
            else:
                bz[0]=i;
        
#DISPLAYING BOULDER NUMBERS
for i in range(1,5):
    print("bz"+str(i)+": "+str(bz[i-1]),end=" ");
print("");

#Calculating number of boulders
no_of_boulders=0;
for i in bz:
    if not(i==-1):
        no_of_boulders=no_of_boulders+1;

###################################
##BRIDGE 1 ########################
###################################
#DETECTING START POSITION OF BRIDGE 1.
###################################
bridge_1_start=cv2.imread('templates/bridge1.png');
res=match_tmplt(img.copy()[:,:,:],bridge_1_start)
crat_start_coord=[res[1][1],res[1][0]];


#DETERMINING CRATER POSITIONS.
###################################
crater_pos_1=[]
for i in range(7):
    roi_crat=img.copy()[crat_start_coord[0]:crat_start_coord[0]+crater_tmplt.shape[0],crat_start_coord[1]-((i+1)*(crater_tmplt.shape[1]+2)):crat_start_coord[1]-i*(crater_tmplt.shape[1]),:];
    res=match_tmplt(roi_crat,crater_tmplt);
    if(res[0]<15000):
        crater_pos_1.append(i+1);
        

###################################
##BRIDGE 2 ########################
###################################

#DETECTING START POSITION OF BRIDGE 2.
#######################################
bridge_2_start=cv2.imread('templates/bridge2.png');
res=match_tmplt(img.copy()[:,:,:],bridge_2_start)
crat_start_coord=[(res[1][1],res[1][0]),(res[2][1],res[2][0])];


#DETERMINING CRATER POSITIONS.
###################################
crater_pos_2=[]
for i in range(7):
    roi_crat=img.copy()[crat_start_coord[0][0]:crat_start_coord[1][0],crat_start_coord[0][1]-((i+1)*(crater_tmplt.shape[1]+2)):crat_start_coord[0][1]-i*(crater_tmplt.shape[1]),:];
    res=match_tmplt(roi_crat,crater_tmplt);
    if(res[0]<15000):
        if(res[1][1]<roi_crat.shape[0]/2):
            crater_pos_2.append([0,(i+1)]);
        else:
            crater_pos_2.append([1,(i+1)]);


###################################
##obstacles########################
###################################

#DETECTING STARTING OF BRIDGE 2.
###################################
bridge_2_start=cv2.imread('templates/bridge2.png');
res=match_tmplt(img.copy()[:,:,:],bridge_2_start)
obs_start_coord=[(res[1][1],res[1][0]),(res[2][1],res[2][0])];


#DETERMINING obstacle POSITIONS.
###################################
obstacles=[]
for i in range(7):
    roi_obs=img.copy()[obs_start_coord[0][0]:obs_start_coord[1][0],obs_start_coord[0][1]-((i+1)*(crater_tmplt.shape[1]+2)):obs_start_coord[0][1]-i*(crater_tmplt.shape[1]),:];
    res=match_tmplt(roi_obs,obs_tmplt);
    if(res[0]<2000):
        if(res[1][1]<roi_obs.shape[0]/2):
            obstacles.append([0,(i+1)]);
        else:
            obstacles.append([1,(i+1)]);
        

    
##INPUT SUM:
sum=int(input('enter sum: '));
print("SUM: ",sum)

bridge_1_choose=0;
bridge_2_choose=0;
bridge=0;
boulder_pick=[];

sum_res=check_sum(bz,sum,len(crater_pos_1))
if( sum_res[0]):
    bridge_1_choose=1;
    boulder_pick=sum_res[1].copy();

else:
    sum_res=check_sum(bz,sum,len(crater_pos_2))
    bridge_2_choose=1;
    boulder_pick=sum_res[1].copy();


if(bridge_1_choose==1):
    bridge=1;
else:
    bridge=2;



#data to be transmitted
########################
crater_1_str="";							#This string contains the number of craters in bridge 1 and their positions
for i in crater_pos_1:
    crater_1_str=crater_1_str+str(i);


crater_2_str="";							#This string contains the number of craters in bridge 2 and their positions
for i in crater_pos_2:
    crater_2_str=crater_2_str+str(i[0])+str(i[1]);
    
obs_str="";									#This string contains the number of obstacles in bridge 2 and their positions
for i in obstacles:
    obs_str=obs_str+str(i[0])+str(i[1]);


boulder_str=""								#This string contains the number of boulders and numbers on it.
for i in bz:
    boulder_str=boulder_str+str(i);


boulder_pick_str=""							#This string contains the number of the boulders to be picked.
for i in boulder_pick:
    boulder_pick_str=boulder_pick_str+str(i);
    
    
#This string below is a data packet to be transmitted to the robot    
data=str(len(crater_pos_1))+crater_1_str+str(len(crater_pos_2))+crater_2_str+str(len(obstacles))+obs_str+str(no_of_boulders)+boulder_str+str(bridge)+boulder_pick_str+"#";
####################################

#creating serial object to transmit data
####################################
xbee=serial.Serial("COM4",9600)
xbee.write(data);     

             
