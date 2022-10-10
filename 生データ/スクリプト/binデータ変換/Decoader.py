# -*- coding: utf-8 -*-
"""
Created on Thu May 13 19:29:49 2021

@author: hirar
"""

import glob
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.figure import Figure
import os.path as op
from SerialLibrary import *
import time

def getData(filename, filMode):
    print("%sをで変換中"%filename.split("\\")[-1])
    f = open(filename, "rb")
    data = f.read()
    f.close()

    dataPackets = data.split(b'\x03\x3b\x3b\x0d\x0a') #ETX ; ; CR LF T I (終了文字EXT～CRLFと開始文字TIで区切る)
    #255文字の7連続は255^5 = 1.08*10^12回に1度。毎秒100000バイト書き込むとしても、1000万秒に1度の確率
    
    timeData =  SerialData(Time(), ['T', 'I'])
    airCraftData =  SerialData(AirCraftData(), ['A', 'C'])
    airCraftAnalogData =  SerialData(AirCraftAnalogData(), ['A', 'A'])
    sendingSetting = SerialData(SendingSetting(), ['S', 'S'])
    loggingSetting = SerialData(LoggingSetting(), ['L', 'S'])
    airdataSetting = SerialData(AirDataSetting(), ['A', 'S'])
    envdataSetting = SerialData(EnvDataSetting(), ['E', 'S'])
    imudataSetting = SerialData(IMUDataSetting(), ['I', 'S'])
    
    rawAirData = SerialData(AirData(), ['R', 'A'],"_raw")
    rawEnvData = SerialData(EnvData(), ['R', 'E'],"_raw")
    rawIMUData = SerialData(IMUData(), ['R', 'I'],"_raw")
    rawGPSData = SerialData(GPSData(), ['R', 'G'],"_raw")
    
    calibAirData = SerialData(AirData(), ['C', 'A'])
    calibEnvData = SerialData(EnvData(), ['C', 'E'])
    calibIMUData = SerialData(IMUData(), ['C', 'I'])
    calibGPSData = SerialData(GPSData(), ['C', 'G'])
    
    airFLT = SerialData(AirSensorFLT(), ['F', 'A'],"_err")
    envFLT = SerialData(EnvSensorFLT(), ['F', 'E'],"_err")
    imuFLT = SerialData(IMUSensorFLT(), ['F', 'I'],"_err")
    gpsFLT = SerialData(GPSSensorFLT(), ['F', 'G'],"_err")
    canFLT = SerialData(CANIFFLT(), ['F', 'C'],"_err")
    
    wind = SerialData(Wind(), ['W', 'I'])
    status = SerialData(Status(), ['S', 'T'])    

    headerStr = ""
    headerStr += timeData.getHeader()
    headerStr += calibAirData.getHeader()
    headerStr += calibEnvData.getHeader()
    headerStr += calibIMUData.getHeader()
    headerStr += calibGPSData.getHeader()  
    
    headerStr += rawAirData.getHeader()
    headerStr += rawEnvData.getHeader()
    headerStr += rawIMUData.getHeader()
    headerStr += rawGPSData.getHeader()
          
    headerStr += airCraftData.getHeader()
    headerStr += airCraftAnalogData.getHeader()
    headerStr += sendingSetting.getHeader()
    headerStr += loggingSetting.getHeader()
    headerStr += airdataSetting.getHeader()
    headerStr += envdataSetting.getHeader()
    headerStr += imudataSetting.getHeader()
    
    headerStr += airFLT.getHeader()
    headerStr += envFLT.getHeader()
    headerStr += imuFLT.getHeader()
    headerStr += gpsFLT.getHeader()
    headerStr += canFLT.getHeader()
    
    headerStr += wind.getHeader()
    headerStr += status.getHeader()    
    headerStr += "\n"

    dataStrList = []
    
    i = 0
    lostPacketNum = 0
    totalDataPacketNum = len(dataPackets)
    while i < totalDataPacketNum:
    #while i < 10:
        tempDataPacket = dataPackets[i]
        byteList = tempDataPacket.split(b';;')

        timeData.setReceiveDataStatus = False
        airCraftData.setReceiveDataStatus = False  
        airCraftAnalogData.setReceiveDataStatus = False  
        sendingSetting.setReceiveDataStatus = False 
        loggingSetting.setReceiveDataStatus = False 
        airdataSetting.setReceiveDataStatus = False 
        envdataSetting.setReceiveDataStatus = False 
        imudataSetting.setReceiveDataStatus = False 

        rawAirData.setReceiveDataStatus = False 
        rawEnvData.setReceiveDataStatus = False 
        rawIMUData.setReceiveDataStatus = False 
        rawGPSData.setReceiveDataStatus = False 

        calibAirData.setReceiveDataStatus = False 
        calibEnvData.setReceiveDataStatus = False 
        calibIMUData.setReceiveDataStatus = False 
        calibGPSData.setReceiveDataStatus = False 

        airFLT.setReceiveDataStatus = False 
        envFLT.setReceiveDataStatus = False 
        imuFLT.setReceiveDataStatus = False 
        gpsFLT.setReceiveDataStatus = False 
        canFLT.setReceiveDataStatus = False 

        wind.setReceiveDataStatus = False
        status.setReceiveDataStatus = False

        if (len(byteList) == 24) or (len(byteList) == 23): #23はAirCraftAnalogData追加前。
            for byte in byteList:
                timeData.setReceiveData(byte)
                rawAirData.setReceiveData(byte) 
                rawEnvData.setReceiveData(byte) 
                rawIMUData.setReceiveData(byte) 
                rawGPSData.setReceiveData(byte) 
        
                calibAirData.setReceiveData(byte) 
                calibEnvData.setReceiveData(byte) 
                calibIMUData.setReceiveData(byte) 
                calibGPSData.setReceiveData(byte) 
                    
                airCraftData.setReceiveData(byte)  
                airCraftAnalogData.setReceiveData(byte)  
                sendingSetting.setReceiveData(byte) 
                loggingSetting.setReceiveData(byte) 
                airdataSetting.setReceiveData(byte) 
                envdataSetting.setReceiveData(byte) 
                imudataSetting.setReceiveData(byte) 

                airFLT.setReceiveData(byte) 
                envFLT.setReceiveData(byte) 
                imuFLT.setReceiveData(byte) 
                gpsFLT.setReceiveData(byte) 
                canFLT.setReceiveData(byte) 
        
                wind.setReceiveData(byte)
                status.setReceiveData(byte)
            
            dataStr = ""
            dataStr += timeData.getStrData(timeData.setReceiveDataStatus, filMode)
 
            dataStr += calibAirData.getStrData(calibAirData.setReceiveDataStatus, filMode) 
            dataStr += calibEnvData.getStrData(calibEnvData.setReceiveDataStatus, filMode) 
            dataStr += calibIMUData.getStrData(calibIMUData.setReceiveDataStatus, filMode) 
            dataStr += calibGPSData.getStrData(calibGPSData.setReceiveDataStatus, filMode) 

            dataStr += rawAirData.getStrData(rawAirData.setReceiveDataStatus, filMode) 
            dataStr += rawEnvData.getStrData(rawEnvData.setReceiveDataStatus, filMode) 
            dataStr += rawIMUData.getStrData(rawIMUData.setReceiveDataStatus, filMode) 
            dataStr += rawGPSData.getStrData(rawGPSData.setReceiveDataStatus, filMode) 
            
            dataStr += airCraftData.getStrData(airCraftData.setReceiveDataStatus, filMode)  
            dataStr += airCraftAnalogData.getStrData(airCraftData.setReceiveDataStatus, filMode)  
            dataStr += sendingSetting.getStrData(sendingSetting.setReceiveDataStatus, filMode) 
            dataStr += loggingSetting.getStrData(loggingSetting.setReceiveDataStatus, filMode) 
            dataStr += airdataSetting.getStrData(airdataSetting.setReceiveDataStatus, filMode) 
            dataStr += envdataSetting.getStrData(envdataSetting.setReceiveDataStatus, filMode) 
            dataStr += imudataSetting.getStrData(imudataSetting.setReceiveDataStatus, filMode) 
    
            dataStr += airFLT.getStrData(airFLT.setReceiveDataStatus, filMode) 
            dataStr += envFLT.getStrData(envFLT.setReceiveDataStatus, filMode) 
            dataStr += imuFLT.getStrData(imuFLT.setReceiveDataStatus, filMode) 
            dataStr += gpsFLT.getStrData(gpsFLT.setReceiveDataStatus, filMode) 
            dataStr += canFLT.getStrData(canFLT.setReceiveDataStatus, filMode) 
    
            dataStr += wind.getStrData(wind.setReceiveDataStatus, filMode)
            dataStr += status.getStrData(status.setReceiveDataStatus, filMode)
            dataStr += "\n"
            
            dataStrList.append(dataStr)
            
        else:
            lostPacketNum += 1
            
        i += 1
        print("処理中。{:.1f}%".format(float(i*100/totalDataPacketNum)))
    print("ロストパケット数： %s \n"%lostPacketNum)
    f = open(filename.replace(".bin", ".csv"), "w")
    f.write(headerStr)
    for dataStr in dataStrList:
        f.write(dataStr)
    f.close()
        

if __name__ == '__main__':
    fileList = glob.glob("*.bin")
    i = 0
    while i < len(fileList):
        getData(fileList[i], "Zval")
        i += 1
    
    
    
    
    
    
    
    
    
    
    
    
    