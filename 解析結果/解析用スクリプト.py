# -*- coding: utf-8 -*-
"""
Created on Sat Jul 30 16:50:56 2022

@author: hirar
"""
from multiprocessing import Pool
import numpy as np
import pandas as pd
from scipy import interpolate as intp
from scipy.signal import butter, lfilter, freqz, filtfilt
from matplotlib import pyplot as plt
import matplotlib.patches as patches
from geopy.distance import geodesic

from PIL import Image

from scipy import integrate

#Global Val
#FPS = 59.94
FPS = 20
DELTA_T = 1/FPS

ADC_FP = 13.3 #Hz
SOLAE_FP = 100 #Hz

G_LOPWASS_FS = 2 #Hz


YAW_USE_HPA_NAVI = True

ANGLE_HIGHT = 4
ANLGE_WIDTH = 2

RUDD_BAR_WIDTH = 30
RUDD_BAR_HIGHT = 1.5
ELEV_BAR_WIDTH = 1.5
ELEV_BAR_HIGHT = 30

MAP_CONSTANT = np.arctanh(np.sin(np.radians(85.05112878)))
ZOOM_LEVEL = 17.29
IMAGE_X, IMAGE_Y = 1543,1123
MONITOR_SCALE = 1.5 #windowsで150%表示の場合、zoom levelが変更される。この補正 150%でマップを取得したので、1.5倍となる。

IMAGE_OX, IMAGE_OY, IMAGE_H0 = 1322,719,300
MAP_OX, MAP_OY= 136.254465,35.2941864
MAP_ROTATE = 60

AC_TOP_SIZE_X = 135
AC_TOP_SIZE_Y = 72
AC_SIDE_SIZE_X = 135
AC_SIDE_SIZE_Y = 35

MAP_H_TEXT_OX = 20
MAP_H_TEXT_OY = 70
MAP_P_TEXT_OX = MAP_H_TEXT_OX
MAP_P_TEXT_OY = MAP_H_TEXT_OY+60
MAP_AOA_TEXT_OX = MAP_H_TEXT_OX
MAP_AOA_TEXT_OY = MAP_P_TEXT_OY+60

MAP_YAW_TEXT_OX = MAP_H_TEXT_OX
MAP_YAW_TEXT_OY = 630
MAP_COURSE_TEXT_OX = MAP_H_TEXT_OX
MAP_COURSE_TEXT_OY = MAP_YAW_TEXT_OY + 60
MAP_AOS_TEXT_OX = MAP_H_TEXT_OX
MAP_AOS_TEXT_OY = MAP_COURSE_TEXT_OY + 60
MAP_LON_TEXT_OX = MAP_H_TEXT_OX
MAP_LON_TEXT_OY = MAP_AOS_TEXT_OY+60
MAP_LAT_TEXT_OX = MAP_H_TEXT_OX
MAP_LAT_TEXT_OY = MAP_LON_TEXT_OY+60
SKIP_PLOT_COURSE_RATIO = 0.18


PS_INIT_VAL = 997.3803711
ALT_OFFSET = 10.8 - 0.4 #水面までの高度　Excelシートより これから0.4m（着水時に0になるようにオフセット）を加える
PS_DP_COEF = 0

SAVE_FIG_PATH_RUDD = "img/rudd"
SAVE_FIG_PATH_ELEV = "img/elev"
SAVE_FIG_PATH_MAP = "img/map"
SAVE_FIG_PATH_ROLL = "img/roll"
SAVE_FIG_PATH_TEXT = "img/text"

PLOT_XY_WIND =True
PLOT_LH_WIND =False

SKIP_MAP_PLOT_LINE_RATIO = 0.03

WIND_PLOT_FLG = True
WIND_XY_SCALE = 20000
WIND_XD_SCALE = 20000
WIND_PLOT_SKIP_RATIO = 0.07
WIND_COLOR = "#ff4500"
SKIP_MAP_PLOT_WIND_RATIO = 0.3

ROLL_IMAGE_SIZE_X = 665
ROLL_IMAGE_SIZE_Y = 302
ROLL_LINE_LENGTH = 700 #px
ROLL_LINE_OX = int(ROLL_IMAGE_SIZE_X/2) 
ROLL_LINE_OY = int(ROLL_IMAGE_SIZE_Y/2)-9
ROLL_TEXT_OX = 680
ROLL_TEXT_OY = ROLL_LINE_OY +15

TEXT_SIZE = 28
TEXT_OFFSET = 90


plt.rcParams['font.family'] = "MS Gothic"

def timeCheck(timeArray):
    timeArray = np.array(timeArray)
    newTImeArray = []
    i = 0
    timeOld = timeArray[0]
    while i < len(timeArray)-1:
        if timeArray[i] <= timeOld:
            newTImeArray.append((timeArray[i] + timeArray[i+1])/2)
        else:
            newTImeArray.append(timeArray[i])
        timeOld = timeArray[i]
        i += 1
    
    newTImeArray.append(timeArray[-1])
    return np.array(newTImeArray)

def plotRudd(rudd, num):
    fig = plt.figure(figsize = (7,3))
    ax = plt.axes()
    r1 = patches.Rectangle(xy=(-15, 0), width=RUDD_BAR_WIDTH, height=RUDD_BAR_HIGHT, ec=None, color = "#00ff7f",  fill=True)
    r2 = patches.Rectangle(xy=(-0.25, -RUDD_BAR_HIGHT*0.5), width=0.5, height=RUDD_BAR_HIGHT*2, ec=None, color = "#00ff7f",  fill=True)
    t = patches.Polygon(xy=[(rudd,0), (rudd-ANLGE_WIDTH/2, -ANGLE_HIGHT), (rudd+ANLGE_WIDTH/2, -ANGLE_HIGHT)], ec=None, color = "#00ff7f",  fill=True)
    ax.add_patch(r1)
    ax.add_patch(r2)
    ax.add_patch(t)
    ax.text(RUDD_BAR_WIDTH/2+RUDD_BAR_WIDTH*0.1, 0, "{:>4} deg".format("%.1f"%rudd), size=20, color="#00ff7f")
    ax.axis("off")
    plt.axis("equal")
    plt.xlim([-20, 30])
    fig.savefig("%s/rudd_%s.png"%(SAVE_FIG_PATH_RUDD, num),transparent=True)
    plt.clf()
    plt.cla()
    plt.close()
    

def plotElev(elev, num):
    fig = plt.figure(figsize = (3,7))
    ax = plt.axes()
    r1 = patches.Rectangle(xy=(-ELEV_BAR_WIDTH, -15), width=ELEV_BAR_WIDTH, height=ELEV_BAR_HIGHT, ec=None, color = "#00ff7f",  fill=True)
    r2 = patches.Rectangle(xy=(-ELEV_BAR_WIDTH*1.5, -0.25), width=ELEV_BAR_WIDTH*2, height=0.5, ec=None, color = "#00ff7f",  fill=True)
    t = patches.Polygon(xy=[(0,elev), (ANGLE_HIGHT, elev-ANLGE_WIDTH/2), (ANGLE_HIGHT, elev+ANLGE_WIDTH/2)], ec=None, color = "#00ff7f",  fill=True)
    ax.add_patch(r1)
    ax.add_patch(r2)
    ax.add_patch(t)
    ax.text(-4, ELEV_BAR_HIGHT/2+ELEV_BAR_HIGHT*0.05, "{:>4} deg".format("%.1f"%elev), size=20, color="#00ff7f")
    ax.axis("off")
    plt.axis("equal")
    plt.ylim([-20, 30])
    fig.savefig("%s/elev_%s.png"%(SAVE_FIG_PATH_ELEV, num),transparent=True)
    plt.clf()
    plt.cla()
    plt.close()

def getRudd(x):
    return -((-3.737302*10**-6)*(x**2) + (3.707187*10**-2)*x - 6.144524*10)

def getElev(x):
    return (-9.220868*10**-7)*(x**2) + (1.662984*10**-2)*x - 3.206976*10

def getALT(ps, dp, tat):
    temp = (PS_INIT_VAL / (ps + dp * 0.01 * PS_DP_COEF))**0.19022256  #動圧の補正 係数はExcelより。
    return ((temp - 1) * (tat + 273.15)) / 0.0065

def getG(ax, ay, az):
    return np.sqrt(ax**2 + ay**2 + az**2)/9.81

def getCourseAngle(velN, velE):
    return np.degrees(np.arctan2(velE, velN)) + 360

def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data)
    return y



def getWindVector(velN, velE, velD, airspeed, pitch, roll, yaw, aoa, aos):
    #YAW：北方位が0deg、機体を上から見て時計回りが正方向、Z軸回り
    #ROLL：水平が0deg、機体を後方から見て時計回り（右バンク）が正方向、X軸周り
    #PITCH：水平が0deg、ピッチ上げが正方向、Y軸周り
    #X軸：機体進行方向、前方が正方向
    #Y軸：機体並進方向、進行方向に対して、右翼方向が正方向
    #Z軸：重力方向、下方向が正方向
    #AOA：機体進行方向と機体下方方向の成す角度、ピッチ下げ方向が正
    #AOS：機体進行方向と進行方向に対して、右翼方向が成す角度、ヨー方向が正

    if yaw > 360:
        yaw -= 360
    elif yaw < 0:
        yaw += 360    
    #airspeed = -airspeed
    aos = -aos
    cr = np.cos(np.radians(-roll))
    sr = np.sin(np.radians(-roll))
    cp = np.cos(np.radians(pitch))
    sp = np.sin(np.radians(pitch))
    cy = np.cos(np.radians(yaw))
    sy = np.sin(np.radians(yaw))
    
    caoa = np.cos(np.radians(aoa))
    saoa = np.sin(np.radians(aoa))
    caos = np.cos(np.radians(aos))
    saos = np.sin(np.radians(aos))

    U = airspeed * caoa * caos #機体進行方向速度, 前方が正, 単位はm/s
    V = airspeed * caoa * saos #体並進方向速度, 右翼正  , 単位はm/s
    W = airspeed * saoa * caos #機体下降方向速度、下方向正, 単位はm/s
    
    #cosφ：cr、sinφ：sr
    #cosΘ：cp, sinΘ：sp
    #cosψ：cy、sinψ：sy
    V_N_ac = (cp*cy)*U + (sr*sp*cy-cr*sy)*V + (cr*sp*cy+sr*sy)*W
    V_E_ac = (cp*sy)*U + (sr*sp*sy+cr*cy)*V + (cr*sp*sy-sr*sy)*W
    V_D_ac = (-sp)  *U + (sr*cp)         *V + (cr*cp)         *W
    
    V_N_w = V_N_ac - velN 
    V_E_w = V_E_ac - velE
    V_D_w = V_D_ac - velD
    WINDSPEED = np.sqrt(V_N_w**2 + V_E_w**2 + V_D_w**2)
    
    return V_N_w, V_E_w, V_D_w, WINDSPEED

def map_to_pix(lon,lat, rot):  
    x0 =  2**(ZOOM_LEVEL+7) * ((MAP_OX)/180 + 1)*MONITOR_SCALE
    y0 = 2**(ZOOM_LEVEL+7)/np.pi*(-np.arctanh(np.sin(np.radians(MAP_OY))) + MAP_CONSTANT)*MONITOR_SCALE
    
    x1 = 2**(ZOOM_LEVEL+7) * ((lon)/180 + 1)*MONITOR_SCALE
    y1 = 2**(ZOOM_LEVEL+7)/np.pi*(-np.arctanh(np.sin(np.radians(lat))) + MAP_CONSTANT)*MONITOR_SCALE

    x1 = x1 - x0
    y1 = y1 - y0
    x2 = x1*np.cos(np.radians(rot)) - y1*np.sin(np.radians(rot))
    y2 = x1*np.sin(np.radians(rot)) + y1*np.cos(np.radians(rot))
    
    x2 = x2 + IMAGE_OX
    y2 = y2 + IMAGE_OY
    return int(x2), int(y2)

def heightToPix(height):
    return IMAGE_H0 - 200 * height/10.0 #200pxで10m　IMAGE_H0が0mのy座標で座標軸は下向きが正

def rotate(x,y, rot): #rot:degrees
    x1 = x*np.cos(np.radians(rot)) - y*np.sin(np.radians(rot))
    y1 = x*np.sin(np.radians(rot)) + y*np.cos(np.radians(rot))
    return x1, y1
    
def plotMap(x, y, l, h, pitch, yaw, height, lat, lon, windVector, aoa, aos, course, bg_image, top_image, side_image, num):
    fig = plt.figure(figsize = (IMAGE_X/100 ,IMAGE_Y/100))
    ax = plt.axes()
    
    bg = Image.new("RGBA", (IMAGE_X, IMAGE_Y), (255,255,255,0))
    top_image = top_image.rotate(yaw, resample = Image.BICUBIC, expand=True)
    side_image = side_image.rotate(-pitch, resample = Image.BICUBIC, expand=True)
    bg.paste(bg_image,(0,0), bg_image)
    bg.paste(top_image,(int(x[num]) - int(top_image.width/2), int(y[num])- int(top_image.height/2)), top_image)
    bg.paste(side_image,(int(l[num]) - int(side_image.width/2), int(h[num])- int(side_image.height/2)), side_image)
    ax.imshow(bg, origin='upper')
    
    #plt.plot(x, y, color = "#ff8c00", linewidth=5, alpha = 0.2)
    skipPlotLineNum = int(SKIP_MAP_PLOT_LINE_RATIO*len(x))
    skipPlotWindNum = int(SKIP_MAP_PLOT_WIND_RATIO*len(x))
    windPlotSkipNum = int(WIND_PLOT_SKIP_RATIO*len(x))
    skipPlotCourseNum = int(SKIP_PLOT_COURSE_RATIO*len(x))
    
    if num > skipPlotLineNum:
        plt.plot(x[:(num-skipPlotLineNum)], y[:(num-skipPlotLineNum)], color = "#ffa500", linewidth=5, alpha = 0.5)
        plt.plot(l[:(num-skipPlotLineNum)], h[:(num-skipPlotLineNum)], color = "#ffa500", linewidth=5, alpha = 0.5)
    
    #plot Text
    ax.text(MAP_H_TEXT_OX, MAP_H_TEXT_OY, "高度　　：{:>5} m".format("%.1f"%height[num]), size=30, color="#ffffff")
    ax.text(MAP_P_TEXT_OX, MAP_P_TEXT_OY, "ピッチ角：{:>5} deg".format("%.1f"%pitch), size=30, color="#ffffff")
    ax.text(MAP_AOA_TEXT_OX, MAP_AOA_TEXT_OY, "ＡＯＡ　：{:>5} deg".format("%.1f"%aoa[num]), size=30, color="#ffffff")

    ax.text(MAP_YAW_TEXT_OX, MAP_YAW_TEXT_OY, "ヨー角：{:>10} deg".format("%.1f"%yaw), size=30, color="#ffffff")
    if num > skipPlotCourseNum:
        ax.text(MAP_COURSE_TEXT_OX, MAP_COURSE_TEXT_OY, "航路角：{:>10} deg".format("%.1f"%course), size=30, color="#ffffff")
    else:
        ax.text(MAP_COURSE_TEXT_OX, MAP_COURSE_TEXT_OY, "航路角：{:>10} deg".format("N/A"), size=30, color="#ffffff")
    ax.text(MAP_AOS_TEXT_OX, MAP_AOS_TEXT_OY, "ＡＯＳ：{:>10} deg".format("%.1f"%aos[num]), size=30, color="#ffffff")
    ax.text(MAP_LAT_TEXT_OX, MAP_LAT_TEXT_OY, "緯度　：{:>10} deg".format("%.6f"%lat[num]), size=30, color="#ffffff")

    ax.text(MAP_LON_TEXT_OX, MAP_LON_TEXT_OY, "経度　：{:>10} deg".format("%.6f"%lon[num]), size=30, color="#ffffff")

    
    
    #plot WindVector
    i = 0
    while i < len(windVector):   
        if WIND_PLOT_FLG == True:
            if i % windPlotSkipNum == 0 and i > skipPlotWindNum or i == 0:
                v_x = windVector[i,1] #V_e 右向きが正
                v_y = -windVector[i,0] #V_n 下向きが正
                v_x, v_y = rotate(v_x, v_y, MAP_ROTATE)
                v_d = windVector[i,2]
                if PLOT_XY_WIND == True:
                    plt.quiver(x[i], y[i], int(-v_x * WIND_XY_SCALE), int(-v_y * WIND_XY_SCALE), width=0.005, angles='xy', scale_units='xy', scale=1000, color = WIND_COLOR, alpha = 0.5)
                    if i > skipPlotWindNum:
                        ax.text(x[i]+int(-v_x * WIND_XY_SCALE/1000*1.5) - 20, y[i]+int(-v_y * WIND_XY_SCALE/1000*1.5), "%.1fm/s"%windVector[i,3], size=15, color = WIND_COLOR, alpha = 0.5)
                if PLOT_LH_WIND == True:
                    plt.quiver(l[i], h[i], 0, int(-v_d * WIND_XD_SCALE), width=0.005, angles='xy', scale_units='xy', scale=1000, color = WIND_COLOR, alpha = 0.5)
            i += 1
    
    ax.axis("off")
    plt.axis("equal")
    fig.savefig("%s/rudd_%s.png"%(SAVE_FIG_PATH_MAP, num),transparent=True)
    plt.clf()
    plt.cla()
    plt.close()


def plotRoll(roll, bg_image, num):
    fig = plt.figure(figsize = (9,3))
    ax = plt.axes()
    ax.imshow(bg_image, origin='upper')
    x = [-ROLL_LINE_LENGTH/2 * np.cos(np.radians(roll)) +  ROLL_LINE_OX, ROLL_LINE_LENGTH/2 * np.cos(np.radians(roll)) +  ROLL_LINE_OX]
    y = [-ROLL_LINE_LENGTH/2 * np.sin(np.radians(roll)) +  ROLL_LINE_OY, ROLL_LINE_LENGTH/2 * np.sin(np.radians(roll)) +  ROLL_LINE_OY]
    plt.plot(x, y, linewidth = 4, color = "#b22222")
    
    ax.text(ROLL_TEXT_OX, ROLL_TEXT_OY, "{:>4} deg".format("%.1f"%roll), size=25, color="#b22222")
    
    ax.axis("off")
    plt.axis("equal")
    
    fig.savefig("%s/roll_%s.png"%(SAVE_FIG_PATH_ROLL, num),transparent=True)
    plt.clf()
    plt.cla()
    plt.close()   


def plotText(airSpeed, gSpeed, tat, ps, hum, rho, g, numSV, hour, minute, sec, milli, num):
    fig = plt.figure(figsize = (9,9), facecolor="#000000")
    
    r1 = patches.Rectangle(xy=(0,0), width=900, height=900, ec=None, edgecolor = None,  fill=False, alpha = 0)
    
    ax = plt.axes()
    ax.add_patch(r1)
    ax.text(0, TEXT_OFFSET*0, "対気速度：{:>6} m/s".format("%.2f"%airSpeed[num]), size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*1, "対地速度：{:>6} m/s".format("%.2f"%gSpeed[num]), size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*2, "気温　　：{:>6} degC".format("%.1f"%tat[num]), size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*3, "気圧　　：{:>6} hPa".format("%.1f"%ps[num]), size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*4, "湿度　　：{:>6} %".format("%.1f"%hum[num]), size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*5, "空気密度：{:>6} ".format("%.3f"%rho[num])+r"$kg/s^{3}$", size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*6, "加速度　：{:>6} G".format("%.2f"%g[num]), size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*7, "可視衛星：{:>6} 個".format("%d"%numSV[num]), size=TEXT_SIZE, color="#ffffff")
    ax.text(0, TEXT_OFFSET*8, "時刻　　：{:>4}".format("2022/7/24/%d:%d:%d.%d"%(hour[num]+9, minute[num], \
                                                                   sec[num], milli[num])), size=TEXT_SIZE, color="#ffffff")
    ax.axis([0, 900, 0, 900])
    
    ax.axis('scaled')
    ax.invert_yaxis()
    ax.axis("off")
    
    fig.savefig("%s/text_%s.png"%(SAVE_FIG_PATH_TEXT, num),transparent=True)
    plt.clf()
    plt.cla()
    plt.close()


def multiPlotRudd(verList):
    print(verList[1])
    plotRudd(verList[0], verList[1])

def multiPlotElev(verList):
    print(verList[1])
    plotElev(verList[0], verList[1])

def multiPlotMap(verList):
    print(verList[16])
    plotMap(verList[0], verList[1], verList[2], verList[3], verList[4], verList[5], verList[6], verList[7], verList[8], \
            verList[9], verList[10] , verList[11] , verList[12] , verList[13] , verList[14] , verList[15], verList[16])

def multiPlotRoll(verList):
    print(verList[2])
    plotRoll(verList[0], verList[1], verList[2])
     
def multiPlotText(verList):
    print(verList[12])
    plotText(verList[0], verList[1], verList[2], verList[3], verList[4], verList[5], verList[6], verList[7], verList[8], \
            verList[9], verList[10] , verList[11] , verList[12])    


class ADCData:
    def __init__(self, filename, timeOffset):
        self.filename = filename
        self.rawData = pd.read_csv(filename)
        self.timeOffset = timeOffset
        self.time = timeCheck(self.rawData["SYS_TIME[ms]"] /1000.0 - self.rawData["SYS_TIME[ms]"][0]/1000.0 + timeOffset)
        self.timeNormalized = np.arange(timeOffset, max(self.rawData["SYS_TIME[ms]"])/1000.0 - min(self.rawData["SYS_TIME[ms]"])/1000.0 + timeOffset, DELTA_T)
        self.dataLen = len(self.time)
        self.dataNormalizedLen = len(self.timeNormalized)
        self.ruddNormalized = getRudd(self.getNormalizedData("RUDD[-]"))
        self.elevNormalized = getElev(self.getNormalizedData("ELEV[-]"))
        self.pitchNormalized = self.getNormalizedData("EX[deg]")
        self.rollNormalized = -self.getNormalizedData("EY[deg]")
        self.lonNormalized = self.getNormalizedData("lon[10^7 deg]")/10**7
        self.latNormalized = self.getNormalizedData("lat[10^7 deg]")/10**7    
        self.dpNormalized = self.getNormalizedData("DP[Pa]")
        self.velNNormalized = self.getNormalizedData("velN[mm/s]")/1000 #m/sに換算する
        self.velENormalized = self.getNormalizedData("velE[mm/s]")/1000 #m/sに換算する
        self.velDNormalized = self.getNormalizedData("velD[mm/s]")/1000 #m/sに換算する
        self.gSpeedNormalized = self.getNormalizedData("gSpeed[mm/s]")/1000 #m/sに換算する
        self.airSpeedNormalized = self.getNormalizedData("AIRSPEED[m/s]")
        self.aoaNormalized = self.getNormalizedData("AOA[deg]") - 7.8 #エクセルより。フェアリングの影響補正分
        #self.aoaNormalized = self.getNormalizedData("AOA[deg]")  #エクセルより。フェアリングの影響補正分        
        self.aosNormalized = self.getNormalizedData("AOS[deg]")
        self.tatNormalized = self.getNormalizedData("TAT[degC]")
        self.psNormalized = self.getNormalizedData("PS[hPa]")
        self.humNormalized = self.getNormalizedData("HUM[%]")
        self.rhoNormalized = self.getNormalizedData("RHO[kg/m^3]")
        self.axNormalized = self.getNormalizedData("AX[m/s^2]")
        self.ayNormalized = self.getNormalizedData("AY[m/s^2]")
        self.azNormalized = self.getNormalizedData("AZ[m/s^2]")
        self.gxNormalized = self.getNormalizedData("GX[deg/s]")
        self.gyNormalized = self.getNormalizedData("GY[deg/s]")
        self.gzNormalized = self.getNormalizedData("GZ[deg/s]")
        self.altNormalized = getALT(self.psNormalized, self.dpNormalized, self.tatNormalized) + ALT_OFFSET
        self.gNormalized = butter_lowpass_filter(getG(self.axNormalized, self.ayNormalized, self.azNormalized) , 2, FPS)
        self.numSVNormalized = self.getNormalizedData("numSV[-]")
        self.hourNormalized = self.getNormalizedData("hour[h]")
        self.minuteNormalized = self.getNormalizedData("minute[min]")
        self.secNormalized = self.getNormalizedData("sec[s]")
        self.milliNormalized = self.getNormalizedData("nano[ns]")/10000000
        self.courseAngleNormalized = getCourseAngle(self.velNNormalized, self.velENormalized)
        
        
    def getNormalizedData(self, label):
        try:
            data = self.rawData[label]
            intpFunc = intp.PchipInterpolator(self.time, data)
            normalizedData = intpFunc(self.timeNormalized)
            return normalizedData
        except:
            return np.zeros(len(self.timeNormalized))



class SolaeData:
    def __init__(self, filename, timeOffset):
        self.filename = filename
        self.rawData = pd.read_csv(filename, encoding='shift_jis')
        self.timeOffset = timeOffset
        self.time = timeCheck(self.rawData["time[s]"] - self.rawData["time[s]"][0] + timeOffset)
        self.timeNormalized = np.arange(timeOffset, max(self.rawData["time[s]"]) - min(self.rawData["time[s]"]) + timeOffset, DELTA_T)       
        self.dataLen = len(self.time)
        self.dataNormalizedLen = len(self.timeNormalized)
        self.pitchNormalized = self.getNormalizedData("Pitch[deg]")
        self.rollNormalized = self.getNormalizedData("Roll[deg]")
        #self.yawNormalized = self.getNormalizedData("Yaw[deg]")  - 18.1931  #Yawの補正は要検討の必要あり。プラホ方位から出した方が正確かも。速度ベクトルから
        #self.yawNormalized = self.getNormalizedData("Yaw[deg]") - 5.806929732  #2021プラホの花道軌跡から
        self.yawNormalized = self.getNormalizedData("Yaw[deg]") - 16.1875  #Androidから 機首方位は300degでほぼ確定
        #self.yawNormalized = self.getNormalizedData("Yaw[deg]")
        self.lonNormalized = self.getNormalizedData("東経[deg]")
        self.latNormalized = self.getNormalizedData("北緯[deg]")  
        self.ax = butter_lowpass_filter(self.rawData["raw_a_x[m/s^2]"], G_LOPWASS_FS, SOLAE_FP)
        self.ay = butter_lowpass_filter(self.rawData["raw_a_y[m/s^2]"], G_LOPWASS_FS, SOLAE_FP)
        self.az = butter_lowpass_filter(self.rawData["raw_a_z[m/s^2]"], G_LOPWASS_FS, SOLAE_FP)
        self.g = butter_lowpass_filter(getG(self.ax, self.ay, self.az) , G_LOPWASS_FS, SOLAE_FP)
        self.psNormalized = self.getNormalizedData("気圧[hPa]")
        
        self.axNormalized = self.getNormalizedData("raw_a_x[m/s^2]")
        self.ayNormalized = self.getNormalizedData("raw_a_y[m/s^2]")
        self.azNormalized = self.getNormalizedData("raw_a_z[m/s^2]")
        
        self.gNormalized = self.dataNormalization(self.g)
    
    def getNormalizedData(self, label):
        try:
            data = self.rawData[label]
            intpFunc = intp.PchipInterpolator(self.time, data)
            normalizedData = intpFunc(self.timeNormalized)
            return normalizedData
        except:
            return np.zeros(len(self.timeNormalized))
    
    def dataNormalization(self, data):
        intpFunc = intp.PchipInterpolator(self.time, data)
        normalizedData = intpFunc(self.timeNormalized)
        return normalizedData        

class HPANaviData:
    def __init__(self, filename, timeOffset):
        self.filename = filename
        self.rawData = pd.read_csv(filename, encoding='shift_jis')
        self.timeOffset = timeOffset
        self.time = timeCheck(self.rawData["# Time (s)"] - self.rawData["# Time (s)"][0] + timeOffset)
        self.timeNormalized = np.arange(timeOffset, max(self.rawData["# Time (s)"]) - min(self.rawData["# Time (s)"]) + timeOffset, DELTA_T)       
        self.dataLen = len(self.time)
        self.dataNormalizedLen = len(self.timeNormalized)
        self.pitchNormalized = self.getNormalizedData(" Pitch (deg)")
        self.rollNormalized = self.getNormalizedData(" Roll (deg)")
        self.yawNormalized = -self.getNormalizedData(" Yaw (deg)") + 300  #2021プラホの花道軌跡から
    
    def getNormalizedData(self, label):
        try:
            data = self.rawData[label]
            intpFunc = intp.PchipInterpolator(self.time, data)
            normalizedData = intpFunc(self.timeNormalized)
            return normalizedData
        except:
            return np.zeros(len(self.timeNormalized))

        
#ADC電装
adc = ADCData("ADC.csv", 0)


#Solae電装
solae = SolaeData("SOLAE.csv", 0)


#HPA Navi
navi = HPANaviData("HPANavi.csv", 0)


if __name__ == "__main__":

    i = 0
    map_x = []
    map_y = []
    while i < adc.dataNormalizedLen:
        x, y = map_to_pix(adc.lonNormalized[i], adc.latNormalized[i], MAP_ROTATE)
        map_x.append(x)
        map_y.append(y)
        i += 1
    map_x = np.array(map_x)
    map_y = np.array(map_y)

    
    bg_image = Image.open('png/map.png').convert("RGBA")
    top_image = Image.open('png/ac_top.png').convert("RGBA")
    side_image = Image.open('png/ac_side.png').convert("RGBA")
    roll_bg_image = Image.open('png/roll.png').convert("RGBA")
    
    top_image = top_image.resize((AC_TOP_SIZE_X,AC_TOP_SIZE_Y), resample = Image.BICUBIC) #456, 245 originalの1/10
    side_image = side_image.resize((AC_SIDE_SIZE_X,AC_SIDE_SIZE_Y), resample = Image.BICUBIC)   #351, 91 originalの1/10
    
    print(adc.dataNormalizedLen)    
 
    #距離の換算はデータが最後までとれているSolaeのデータを使う
    pathLength = [0]
    distance = [0]
    tempPathLength = 0
    tempDistance = 0
    p0 = (solae.latNormalized[0], solae.lonNormalized[0])
    i = 0
    while i < solae.dataNormalizedLen-1:
        p1 = (solae.latNormalized[i], solae.lonNormalized[i])
        p2 = (solae.latNormalized[i+1], solae.lonNormalized[i+1])
        tempPathLength += geodesic(p1, p2).m
        tempDistance = geodesic(p0, p2).m
        pathLength.append(tempPathLength)
        distance.append(tempDistance)
        i += 1
    pathLength = np.array(pathLength)
    distance = np.array(distance)

    map_h = heightToPix(adc.altNormalized)
    map_l = -((max(map_x)-min(map_x))/max(pathLength)) * pathLength + IMAGE_OX

    windVector = []
    i = 0
    while i < adc.dataNormalizedLen:
        pitch = solae.pitchNormalized[i]
        if YAW_USE_HPA_NAVI == True:
            yaw = -navi.yawNormalized[i] - MAP_ROTATE
        else:
            yaw = -solae.yawNormalized[i] - MAP_ROTATE #地図に合わせて回転
        if yaw > 360:
            yaw -= 360
        elif yaw < 0:
            yaw += 360
        windVector.append(getWindVector(adc.velNNormalized[i], adc.velENormalized[i], adc.velDNormalized[i], \
                                         adc.airSpeedNormalized[i], solae.pitchNormalized[i], solae.rollNormalized[i], \
                                         solae.yawNormalized[i], adc.aoaNormalized[i], adc.aosNormalized[i]))
        i += 1
    windVector = np.array(windVector)

    verList = []
    i = 0
    while i < adc.dataNormalizedLen:
        pitch = solae.pitchNormalized[i]
        if YAW_USE_HPA_NAVI == True:
            yaw = -navi.yawNormalized[i] - MAP_ROTATE
        else:
            yaw = -solae.yawNormalized[i] - MAP_ROTATE #地図に合わせて回転
        if yaw > 360:
            yaw -= 360
        elif yaw < 0:
            yaw += 360

        course = -adc.courseAngleNormalized[i] - MAP_ROTATE
        if course > 360:
            course -= 360
        elif course < 0:
            course += 360
            
        verList.append([map_x, map_y, map_l, map_h, pitch, yaw, adc.altNormalized, adc.latNormalized, adc.lonNormalized, windVector,\
                        adc.aoaNormalized, adc.aosNormalized, course, bg_image, top_image, side_image, i])
        i += 1
    
    
    verListText = []
    i = 0
    while i < adc.dataNormalizedLen:
        verListText.append([adc.airSpeedNormalized, adc.gSpeedNormalized, adc.tatNormalized, adc.psNormalized, adc.humNormalized, adc.rhoNormalized, \
                            solae.gNormalized, adc.numSVNormalized, adc.hourNormalized, adc.minuteNormalized, adc.secNormalized, adc.milliNormalized,  i])
        i += 1
     
    
    
    ### グラフ生成 ###
    
    
    """
    DELNUM = 1020
    plt.plot(adc.timeNormalized[DELNUM:], integrate.cumtrapz(adc.gSpeedNormalized[DELNUM:], adc.timeNormalized[DELNUM:], initial=0), 'b')
    plt.plot(adc.timeNormalized[DELNUM:], integrate.cumtrapz(adc.airSpeedNormalized[DELNUM:], adc.timeNormalized[DELNUM:], initial=0), 'r')
    """
    
    
    """
    plt.plot(adc.timeNormalized, windVector[:,0], "b")
    plt.plot(adc.timeNormalized, windVector[:,1], "r")
    plt.plot(adc.timeNormalized, windVector[:,2], "g")
    plt.legend(["風速（北）","風速（東）", "風速（下）"])
    plt.xlabel("time[sec]")
    plt.ylabel("風速[m/s]")
    plt.grid()     
    """

    """
    plt.plot(adc.timeNormalized, adc.aoaNormalized, "b")
    plt.plot(solae.timeNormalized, solae.pitchNormalized, "r--")
    plt.legend(["AoA","ピッチ角"])
    plt.xlabel("time[sec]")
    plt.ylabel("AoA, ピッチ角 [deg]")
    plt.grid() 
    """
    """
    plt.plot(adc.timeNormalized, adc.aoaNormalized, "b")
    plt.plot(adc.timeNormalized, adc.aosNormalized, "r")
    plt.plot(solae.timeNormalized, solae.pitchNormalized, "b--")
    plt.legend(["AoA", "AoS", "ピッチ角"])
    plt.xlabel("time[sec]")
    plt.ylabel("AoA, AoS, ピッチ角 [deg]")
    plt.grid() 
    """

    """
    plt.plot(adc.timeNormalized, adc.airSpeedNormalized, "b")
    plt.plot(adc.timeNormalized, adc.gSpeedNormalized, "r")
    plt.legend(["対気速度", "対地速度"])
    plt.xlabel("time[sec]")
    plt.ylabel("速度[m/s]")
    plt.grid() 
    """
    

    """    
    plt.plot(adc.timeNormalized, adc.rhoNormalized, "b")
    plt.legend(["大気密度"])
    plt.xlabel("time[sec]")
    plt.ylabel("密度[kg/m^3]")
    plt.grid() 
    """
    

    """
    plt.plot(adc.timeNormalized, adc.psNormalized, "b")
    plt.plot(solae.timeNormalized, solae.psNormalized, "r")
    plt.legend(["エアデータ", "Solae"])
    plt.xlabel("time[sec]")
    plt.ylabel("気圧[hPa]")        
    plt.grid() 
    """
    
    """
    plt.plot(adc.timeNormalized, adc.tatNormalized, "b")
    plt.plot(adc.timeNormalized, adc.humNormalized, "r")
    plt.legend(["気温", "湿度"])
    plt.xlabel("time[sec]")
    plt.ylabel("気温[℃], 湿度[%]")        
    plt.grid() 
    """
    
    """
    plt.plot(solae.timeNormalized, distance, "b")
    plt.plot(solae.timeNormalized, pathLength, "r")
    plt.legend(["", "航路長"])
    plt.xlabel("time[sec]")
    plt.ylabel("距離[m], 経路長[m]")    
    plt.grid() 
    """


    
    """
    plt.plot(solae.timeNormalized, solae.gNormalized, "b")
    plt.legend(["G(Solae)"])
    plt.xlabel("time[sec]")
    plt.ylabel("G Force [G]")    
    plt.grid() 
    """
    
    
    ### CSV生成 ###
    
    """
    plt.plot(adc.timeNormalized, adc.pitchNormalized, "b")
    plt.plot(adc.timeNormalized, adc.rollNormalized, "r")
    plt.plot(solae.timeNormalized, solae.pitchNormalized, "b--")
    plt.plot(solae.timeNormalized, solae.rollNormalized, "r--")   
    
    plt.legend(["Pitch(エアデータ)","Pitch(エアデータ)","Roll(Solae)","Roll(Solae)"])
    plt.xlabel("time[sec]")
    plt.ylabel("Roll, Pitch[deg]")
    plt.grid() 
    """
    
    
    outCsv = []
    i = 0
    
    headerCsv = "Time[msec]," +\
                "ラダー舵角[deg]," +\
                "エレベーター舵角[deg]," +\
                "加速度(X軸)[m/s^2]," +\
                "加速度(Y軸)[m/s^2]," +\
                "加速度(Z軸)[m/s^2]," +\
                "角速度(X軸)[deg/sec]," +\
                "角速度(Y軸)[deg/sec]," +\
                "角速度(Z軸)[deg/sec]," +\
                "荷重(2Hzローパス)[G]," +\
                "ロール[deg]," +\
                "ピッチ[deg]," +\
                "ヨー[deg]," +\
                "緯度[deg]," +\
                "経度[deg]," +\
                "進行角[deg]," +\
                "対地速度(北方向)[m/s]," +\
                "対地速度(東方向)[m/s]," +\
                "対地速度(下方向)[m/s]," +\
                "対地速度の大きさ[m/s]," +\
                "対気速度[m/s]," +\
                "AoA[deg]," +\
                "AoS[deg]," +\
                "気温[degC]," +\
                "動圧[Pa]," +\
                "大気圧[hPa]," +\
                "湿度[%]," +\
                "大気密度[kg/m^3]," +\
                "北方向風速[m/s]," +\
                "東方向風速[m/s]," +\
                "下方向風速[m/s]," +\
                "風速[m/s],"+\
                "高度[m]"
                
    
    while i < adc.dataNormalizedLen:
        temp  = [adc.timeNormalized[i],\
                 adc.ruddNormalized[i],\
                 adc.elevNormalized[i],\
                 solae.axNormalized[i],\
                 solae.ayNormalized[i],\
                 solae.azNormalized[i],\
                 adc.gxNormalized[i],\
                 adc.gyNormalized[i],\
                 adc.gzNormalized[i],\
                 solae.gNormalized[i],\
                 solae.rollNormalized[i],\
                 solae.pitchNormalized[i],\
                 navi.yawNormalized[i],\
                 adc.latNormalized[i],\
                 adc.lonNormalized[i],\
                 adc.courseAngleNormalized[i],\
                 adc.velNNormalized[i],\
                 adc.velENormalized[i],\
                 adc.velDNormalized[i],\
                 adc.gSpeedNormalized[i],\
                 adc.airSpeedNormalized[i],\
                 adc.aoaNormalized[i],\
                 adc.aosNormalized[i],\
                 adc.tatNormalized[i],\
                 adc.dpNormalized[i],\
                 adc.psNormalized[i],\
                 adc.humNormalized[i],\
                 adc.rhoNormalized[i],\
                 windVector[i,0],\
                 windVector[i,1],\
                 windVector[i,2],\
                 windVector[i,3],\
                 adc.altNormalized[i]]
        outCsv.append(temp)
        i += 1
    outCsv = np.array(outCsv)
    np.savetxt("飛行データ.csv", outCsv, delimiter = ",", fmt = "%.8f",header = headerCsv)                
    
    
    """
    #時間軸・飛距離　csv　書き出し
    distanceCsv = []
    i = 0
    while i < solae.dataNormalizedLen:
        tempList = [solae.timeNormalized[i], distance[i]]
        distanceCsv.append(tempList)
        i += 1
    
    distanceCsv = np.array(distanceCsv)
    np.savetxt("Distance.csv", distanceCsv, delimiter = ",", fmt = "%.3f",header = "time[sec], Distance[m]")
    """
    
    
    
    ### 動画用画像生成 ###
    """
    p = Pool(32)
    p.map(multiPlotText, verListText)
    """
    
    """
    verListRudd = []
    i = 0
    while i < adc.dataNormalizedLen:
        verListRudd.append([adc.ruddNormalized[i], i])
        i += 1
    """
    """
    p = Pool(32)
    p.map(multiPlotRudd, verListRudd)
    """
    

    """
    verListElev = []
    i = 0
    while i < adc.dataNormalizedLen:
        verListElev.append([adc.elevNormalized[i], i])
        i += 1
    
    p = Pool(32)
    p.map(multiPlotElev, verListElev)
    """
    
    
    """
    p = Pool(32)
    p.map(multiPlotMap, verList)
    """
    
    """
    rollVerList = []
    i = 0
    while i < solae.dataNormalizedLen:
        rollVerList.append([solae.rollNormalized[i], roll_bg_image, i])
        i += 1
    p = Pool(32)
    p.map(multiPlotRoll, rollVerList)      
    """


