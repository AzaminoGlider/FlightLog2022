# -*- coding: utf-8 -*-
"""
Created on Thu Mar 25 00:07:23 2021
@author: hirar
"""
import ctypes
import struct
import time

def setCondition(ser, send_data):
    ser.write(send_data)
    ser.reset_input_buffer()
    time.sleep(0.01) 

def setData(ser, send_data):
    ser.write(send_data)
    time.sleep(0.01) 
    res_byte = ser.read(ser.inWaiting())
    ser.reset_input_buffer()
    time.sleep(0.01) 
    return res_byte

def calCheckSum(byte_array):
    ck_array = [0, 0]
    for byte in byte_array:
        ck_array[0] += int(byte)
        if ck_array[0] >= 256:
            ck_array[0] -= 256
        ck_array[1] += ck_array[0]
        if ck_array[1] >= 256:
            ck_array[1] -= 256
    ck_sum = struct.pack("B",ck_array[0]) + struct.pack("B",ck_array[1])
    return ck_sum

def convertBytesToStructure(struct, byte):
    if (ctypes.sizeof(struct) == len(byte)):
        ctypes.memmove(ctypes.addressof(struct), byte, ctypes.sizeof(struct))


def convertStructToBytes(struct):
    buffer = ctypes.create_string_buffer(ctypes.sizeof(struct))
    ctypes.memmove(buffer, ctypes.addressof(struct), ctypes.sizeof(struct))
    return buffer.raw

class SerialData:
    def __init__(self, CtypeClass, header, structName = ""):
        self.structName = structName
        self.__header_name = header
        self.__header = struct.pack("B",ord(header[0])) + struct.pack("B",ord(header[1]))
        self.t = CtypeClass
        self.dataSize = ctypes.sizeof(self.t)
        self.cksumErr = False
        self.setReceiveDataStatus = False

    def getSendData(self):
        temp_byte_data = convertStructToBytes(self.t)
        ck_sum = calCheckSum(temp_byte_data)
        byte_data = self.__header + temp_byte_data + ck_sum
        return byte_data

    def setReceiveData(self, byte_data):
        header = byte_data[0:2]
        if header == self.__header:
            rsv_data   = byte_data[2:len(byte_data)-2]
            rsv_ck_sum = byte_data[len(byte_data)-2:len(byte_data)]
            ck_sum     = calCheckSum(rsv_data)
            if((ck_sum == rsv_ck_sum) and (len(rsv_data) != 0)):
                self.cksumErr = False
                try:
                    convertBytesToStructure(self.t, rsv_data)
                    self.setReceiveDataStatus = True #受信成功
                except:
                    print(self.__header_name)
            else:
                #print('CKSum does not match at %s%s'%(self.__header_name[0],self.__header_name[1]))
                self.cksumErr = True
    
    
    def getStrData(self, det, filMode = "Blank"):
        tempStr = ""
        for name, dtype in self.t._fields_:
            temp_data =  getattr(self.t, name)
            if (((type(temp_data) == float) or \
                (type(temp_data) == bool) or \
                (type(temp_data) == int) or \
                (type(temp_data) == str))):
                
                tempStrData = "%s"%temp_data
            else:
                tempStrData = ""
            
            if det == True:
                tempStr += tempStrData + ","
                
            else:
                if filMode == "Blank":
                    tempStr += "" + ","
                elif filMode == "Zval":
                    tempStr += tempStrData + ","
                elif filMode == "Nan":
                    tempStr += "Nan" + ","
        return tempStr
        
    
    def getHeader(self):
        tempStr = ""
        for name, dtype in self.t._fields_:
            tempUnit = self.t.unit[name]
            tempStr += "%s%s[%s],"%(name,self.structName,tempUnit)
        return tempStr        
    
    
    def getDictData(self):
        temp_list = []
        for name, dtype in self.t._fields_:
            temp_data =  getattr(self.t, name)
            if ((type(temp_data) == float) or \
                (type(temp_data) == bool) or \
                (type(temp_data) == int) or \
                (type(temp_data) == str)) :
                temp_list.append( [name, temp_data] )
            else:
                temp_list.append( [name, 0] )
        return dict(temp_list)
 
        

class Time(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('SYS_TIME',  ctypes.c_ulong),
        ('T_CNT',     ctypes.c_ulong),
        ('T_PPS',     ctypes.c_ulong),
        ('T_CAN_RSV', ctypes.c_ulong)
    ]

    def __init__(self):
        super(Time, self).__init__(
            SYS_TIME = 0,
            T_CNT = 0,
            T_PPS = 0,
            T_CAN_RSV = 0
        )
        self.unit = {
            'SYS_TIME':  "ms",
            'T_CNT':     "us",
            'T_PPS':     "us",
            'T_CAN_RSV': "us"
        }
    """
    uint32_t SYS_TIME;
    uint32_t T_CNT;
    uint32_t T_PPS;
    uint32_t T_CAN_RSV;
    """


class CANData(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('ID',   ctypes.c_ulong),
        ('LEN',  ctypes.c_ubyte),
        ('DATA', ctypes.c_ubyte*8)
    ]
    def __init__(self):
        super(CANData, self).__init__(
            ID = 0,
            LEN = 0   
        )
        self.unit = {
            'ID':  "-",
            'LEN': "-",
            'DATA':"-"
        }
    """
    uint32_t ID;
    uint8_t LEN;
    uint8_t DATA[8];
    """

class AirCraftData(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('ROLL',  ctypes.c_float),
        ('PITCH', ctypes.c_float),
        ('YAW',   ctypes.c_float)    
    ]
    def __init__(self):
        super(AirCraftData, self).__init__(
            ROLL = 0,
            PITCH = 0,
            YAW = 0
        )
        self.unit = {
            'ROLL' : "deg",
            'PITCH': "deg",
            'YAW'  : "deg"
        }
    """
    float ROLL;
    float PITCH;
    float YAW;
    """    

class AirCraftAnalogData(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('H_US', ctypes.c_int16),
        ('ELEV', ctypes.c_int16),
        ('RUDD', ctypes.c_int16)    
    ]
    def __init__(self):
        super(AirCraftAnalogData, self).__init__(
            H_US = 0,
            ELEV = 0,
            RUDD = 0
        )
        self.unit = {
            'H_US': "-",
            'ELEV': "-",
            'RUDD': "-"
        }

class SendingSetting(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('AIR_DATA_EN',ctypes.c_bool),
        ('ENV_DATA_EN',ctypes.c_bool),
        ('IMU_DATA_EN',ctypes.c_bool),
        ('GPS_DATA_EN',ctypes.c_bool)        
    ]
    def __init__(self):
        super(SendingSetting, self).__init__(
            AIR_DATA_EN = True,
            ENV_DATA_EN = True,
            IMU_DATA_EN = True,
            GPS_DATA_EN = True
        )
        self.unit = {
            'AIR_DATA_EN' : "-",
            'ENV_DATA_EN' : "-",
            'IMU_DATA_EN' : "-",
            'GPS_DATA_EN' : "-"
        }    
    """
    bool AIR_DATA_EN;
    bool ENV_DATA_EN;
    bool IMU_DATA_EN;
    bool GPS_DATA_EN;
    """


class LoggingSetting(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('LOGGING_EN', ctypes.c_bool),
        ('LOGGING_MODE', ctypes.c_ubyte)    
    ]
    def __init__(self):
        super(LoggingSetting, self).__init__(
            LOGGING_EN  = True,
            LOGGING_MODE = 0    
        )
        self.unit = {
            'LOGGING_EN'   : "-",
            'LOGGING_MODE' : "-"
        }
    """
    bool LOGGING_EN;
    byte LOGGING_MODE;
    """

class AirDataSetting(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('AOA_OFFSET', ctypes.c_float),
        ('AOS_OFFSET', ctypes.c_float),
        ('DP_GAIN',    ctypes.c_float),
        ('DP_OFFSET',  ctypes.c_float),
        ('LPF_DP',     ctypes.c_float),
        ('LPF_SW',     ctypes.c_bool)    
    ]
    def __init__(self):
        super(AirDataSetting, self).__init__(
            AOA_OFFSET = 0.0,
            AOS_OFFSET = 0.0,
            DP_GAIN = 1.0,
            DP_OFFSET = 0.0,
            LPF_DP = 1.0,
            LPF_SW = False   
        )
        self.unit = {
            'AOA_OFFSET': "deg",
            'AOS_OFFSET': "deg",
            'DP_GAIN':    "-",
            'DP_OFFSET':  "Pa",
            'LPF_DP':     "-",
            'LPF_SW':     "-"    
        }
    """
    float AOA_OFFSET;
    float AOS_OFFSET;
    float DP_GAIN;
    float DP_OFFSET;
    float LPF_DP;
    bool LPF_SW;
    """


class EnvDataSetting(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('PS_INIT_VAL', ctypes.c_float),
        ('H_PS_OFFSET', ctypes.c_float),
        ('LPF_PS',      ctypes.c_float),
        ('LPF_SW',      ctypes.c_bool)   
    ]
    def __init__(self):
        super(EnvDataSetting, self).__init__(
            PS_INIT_VAL = 1013.15,
            H_PS_OFFSET = 0.0,
            LPF_PS = 1.0,
            LPF_SW = False
        )
        self.unit = {
            'PS_INIT_VAL': "hPa",
            'H_PS_OFFSET': "m",
            'LPF_PS':      "-",
            'LPF_SW':      "-"   
        }
    """
    float PS_INIT_VAL;
    float H_PS_OFFSET;
    float LPF_PS;
    bool LPF_SW;
    """

class IMUDataSetting(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('AX_OFFSET',  ctypes.c_float),
        ('AY_OFFSET',  ctypes.c_float),
        ('AZ_OFFSET',  ctypes.c_float),
        ('GX_OFFSET',  ctypes.c_float),
        ('GY_OFFSET',  ctypes.c_float),
        ('GZ_OFFSET',  ctypes.c_float),
        ('EX_OFFSET',  ctypes.c_float),
        ('EY_OFFSET',  ctypes.c_float),
        ('EZ_OFFSET',  ctypes.c_float),
        ('LPF_ACC',    ctypes.c_float),
        ('LPF_GYR',    ctypes.c_float),
        ('LPF_ACC_SW', ctypes.c_bool),
        ('LPF_GYR_SW', ctypes.c_bool)      
    ]
    def __init__(self):
        super(IMUDataSetting, self).__init__(
            AX_OFFSET = 0.0,
            AY_OFFSET = 0.0,
            AZ_OFFSET = 0.0,
            GX_OFFSET = 0.0,
            GY_OFFSET = 0.0,
            GZ_OFFSET = 0.0,
            EX_OFFSET = 0.0,
            EY_OFFSET = 0.0,
            EZ_OFFSET = 0.0,
            LPF_ACC = 1.0,
            LPF_GYR = 1.0,
            LPF_ACC_SW = False,
            LPF_GYR_SW = False
        )
        self.unit = {
            "AX_OFFSET" : "m/s^2",
            "AY_OFFSET" : "m/s^2",
            "AZ_OFFSET" : "m/s^2",
            "GX_OFFSET" : "deg/s",
            "GY_OFFSET" : "deg/s",
            "GZ_OFFSET" : "deg/s",
            "EX_OFFSET" : "deg",
            "EY_OFFSET" : "deg",
            "EZ_OFFSET" : "deg",
            "LPF_ACC" : "-",
            "LPF_GYR" : "-",
            "LPF_ACC_SW" : "-",
            "LPF_GYR_SW" : "-"
        }
    """
    float AX_OFFSET;
    float AY_OFFSET;
    float AZ_OFFSET;
    float GX_OFFSET;
    float GY_OFFSET;
    float GZ_OFFSET;
    float EX_OFFSET;
    float EY_OFFSET;
    float EZ_OFFSET;
    float LPF_ACC;
    float LPF_GYR;
    bool LPF_ACC_SW;
    bool LPF_GYR_SW;
    """

class AirData(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('DP',          ctypes.c_float),
        ('AOA',         ctypes.c_float),
        ('AOS',         ctypes.c_float),
        ('T_DP_SENSOR', ctypes.c_float)
    ]
    def __init__(self):
        super(AirData, self).__init__(
            DP = 0.0,
            AOA = 0.0,
            AOS = 0.0,
            T_DP_SENSOR = 15.0   
        )
        self.unit = {
            "DP" : "Pa",
            "AOA" : "deg",
            "AOS" : "deg",
            "T_DP_SENSOR" : "degC"   
        }
    """
    float DP;
    float AOA;
    float AOS;
    float T_DP_SENSOR;
    """

class EnvData(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('PS',   ctypes.c_float),
        ('H_PS', ctypes.c_float),
        ('TAT',  ctypes.c_float),
        ('HUM',  ctypes.c_float),
        ('RHO',  ctypes.c_float),
        ('T_PS_SENSOR', ctypes.c_float)  
    ]
    def __init__(self):
        super(EnvData, self).__init__(
            PS = 1013.15,
            H_PS = 0.0,
            TAT = 15.0,
            HUM = 50.0,
            RHO = 1.225,
            T_PS_SENSOR = 15.0  
        )
        self.unit = {
            "PS" : "hPa",
            "H_PS" : "m",
            "TAT" : "degC",
            "HUM" : "%",
            "RHO" : "kg/m^3",
            "T_PS_SENSOR" : "degC"  
        }
    """
    float PS;
    float H_PS;
    float TAT;
    float HUM;
    float RHO;
    float T_PS_SENSOR;
    """

class IMUData(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('AX', ctypes.c_float),
        ('AY', ctypes.c_float),
        ('AZ', ctypes.c_float),
        ('GX', ctypes.c_float),
        ('GY', ctypes.c_float),
        ('GZ', ctypes.c_float),
        ('EX', ctypes.c_float),
        ('EY', ctypes.c_float),
        ('EZ', ctypes.c_float)  
    ]
    def __init__(self):
        super(IMUData, self).__init__(
            AX = 0.0,
            AY = 0.0,
            AZ = 9.81,
            GX = 0.0,
            GY = 0.0,
            GZ = 0.0,
            EX = 0.0,
            EY = 0.0,
            EZ = 0.0
        )
        self.unit = {
            "AX" : "m/s^2",
            "AY" : "m/s^2",
            "AZ" : "m/s^2",
            "GX" : "deg/s",
            "GY" : "deg/s",
            "GZ" : "deg/s",
            "EX" : "deg",
            "EY" : "deg",
            "EZ" : "deg"
        }
    """
    float AX;
    float AY;
    float AZ;
    float GX;
    float GY;
    float GZ;
    float EX;
    float EY;
    float EZ;
    """

class GPSData(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('MICROSECONDS_PPS', ctypes.c_ulong),
        ('cls',              ctypes.c_ubyte),
        ('id',               ctypes.c_ubyte),
        ('len',              ctypes.c_ushort),
        ('iTOW',             ctypes.c_ulong),
        ('year',             ctypes.c_ushort),
        ('month',            ctypes.c_ubyte),
        ('day',              ctypes.c_ubyte),
        ('hour',             ctypes.c_ubyte),
        ('minute',           ctypes.c_ubyte),
        ('sec',              ctypes.c_ubyte),
        ('valid',            ctypes.c_ubyte),
        ('tAcc',             ctypes.c_ulong),
        ('nano',             ctypes.c_long),
        ('fixType',          ctypes.c_ubyte),
        ('flags',            ctypes.c_ubyte),
        ('flags2',           ctypes.c_ubyte),
        ('numSV',            ctypes.c_ubyte),
        ('lon',              ctypes.c_long),
        ('lat',              ctypes.c_long),
        ('height',           ctypes.c_long),
        ('hMSL',             ctypes.c_long),
        ('hAcc',             ctypes.c_ulong),
        ('vAcc',             ctypes.c_ulong),
        ('velN',             ctypes.c_long),
        ('velE',             ctypes.c_long),
        ('velD',             ctypes.c_long),
        ('gSpeed',           ctypes.c_long),
        ('headMot',          ctypes.c_long),
        ('sAcc',             ctypes.c_ulong),
        ('headAcc',          ctypes.c_ulong),
        ('pDOP',             ctypes.c_ushort),
        ('reserved1',        ctypes.c_ubyte*6),
        ('headVeh',          ctypes.c_long),
        ('reserved2',        ctypes.c_ubyte*4)   
    ]
    def __init__(self):
        super(GPSData, self).__init__(
            MICROSECONDS_PPS = 0,
            cls = 0,
            id = 0,
            len = 0,
            iTOW = 0,
            year = 2000,
            month = 1,
            day = 1,
            hour = 0,
            minute = 0,
            sec = 0,
            valid = 0,
            tAcc = 0,
            nano = 0,
            fixType = 0,
            flags = 0,
            flags2 = 0,
            numSV = 0,
            lon = 0,
            lat = 0,
            height = 0,
            hMSL = 0,
            hAcc = 0,
            vAcc = 0,
            velN = 0,
            velE = 0,
            velD = 0,
            gSpeed = 0,
            headMot = 0,
            sAcc = 0,
            headAcc = 0,
            pDOP = 0,
            headVeh = 0
        )
        self.unit = {
            "MICROSECONDS_PPS" : "us",
            "cls" : "-",
            "id" : "-",
            "len" : "-",
            "iTOW" : "ms",
            "year" : "y",
            "month" : "month",
            "day" : "d",
            "hour" : "h",
            "minute" : "min",
            "sec" : "s",
            "valid" : "-",
            "tAcc" : "ns",
            "nano" : "ns",
            "fixType" : "-",
            "flags" : "-",
            "flags2" : "-",
            "numSV" : "-",
            "lon" : "10^7 deg",
            "lat" : "10^7 deg",
            "height" : "mm",
            "hMSL" : "mm",
            "hAcc" : "mm",
            "vAcc" : "mm",
            "velN" : "mm/s",
            "velE" : "mm/s",
            "velD" : "mm/s",
            "gSpeed" : "mm/s",
            "headMot" : "10^5 deg",
            "sAcc" : "mm/s",
            "headAcc" : "10^5 deg",
            "pDOP" : "-",
            "reserved1": "-",
            "headVeh" : "10^5 deg",
            "reserved2": "-"
        }
    """
    uint32_t MICROSECONDS_PPS
    unsigned char cls;
    unsigned char id;
    unsigned short len;
    unsigned long iTOW;
    unsigned short year;
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char sec;
    unsigned char valid;
    unsigned long tAcc;
    long nano;
    unsigned char fixType;
    unsigned char flags;
    unsigned char flags2;
    unsigned char numSV;
    long lon;
    long lat;
    long height;
    long hMSL;
    unsigned long hAcc;
    unsigned long vAcc;
    long velN;
    long velE;
    long velD;
    long gSpeed;
    long headMot;
    unsigned long sAcc;
    unsigned long headAcc;
    unsigned short pDOP;
    unsigned char reserved1[6];
    long headVeh;
    unsigned char reserved2[4];
    """

class Wind(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('AIRSPEED', ctypes.c_float),
        ('WINDN',    ctypes.c_float),
        ('WINDE',    ctypes.c_float),
        ('WINDD',    ctypes.c_float)   
    ]
    def __init__(self):
        super(Wind, self).__init__(
            AIRSPEED = 0.0,
            WINDN = 0.0,
            WINDE = 0.0,
            WINDD = 0.0  
        )
        self.unit = {
            "AIRSPEED" : "m/s",
            "WINDN" : "m/s",
            "WINDE" : "m/s",
            "WINDD" : "m/s"  
        }
    """
    float AIRSPEED;
    float WINDN;
    float WINDE;
    float WINDD;
    """

class AirSensorFLT(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('DP',          ctypes.c_bool),
        ('AOA',         ctypes.c_bool),
        ('AOS',         ctypes.c_bool),
        ('T_DP_SENSOR', ctypes.c_bool),
        ('DP_GAIN',     ctypes.c_bool),
        ('DP_OFFSET',   ctypes.c_bool),
        ('AOA_OFFSET',  ctypes.c_bool),
        ('AOS_OFFSET',  ctypes.c_bool),
        ('LPF_DP',      ctypes.c_bool),
        ('DP_I2C',      ctypes.c_bool),
        ('AOA_I2C',     ctypes.c_bool),
        ('AOS_I2C',     ctypes.c_bool),
        ('SETTING_DATA_READ', ctypes.c_bool)   
    ]
    def __init__(self):
        super(AirSensorFLT, self).__init__(
            DP = False,
            AOA = False,
            AOS = False,
            T_DP_SENSOR = False,
            DP_GAIN = False,
            DP_OFFSET = False,
            AOA_OFFSET = False,
            AOS_OFFSET = False,
            LPF_DP = False,
            DP_I2C = False,
            AOA_I2C = False,
            AOS_I2C = False,
            SETTING_DATA_READ = False 
        )
        self.unit = {
            "DP" : "-",
            "AOA" : "-",
            "AOS" : "-",
            "T_DP_SENSOR" : "-",
            "DP_GAIN" : "-",
            "DP_OFFSET" : "-",
            "AOA_OFFSET" : "-",
            "AOS_OFFSET" : "-",
            "LPF_DP" : "-",
            "DP_I2C" : "-",
            "AOA_I2C" : "-",
            "AOS_I2C" : "-",
            "SETTING_DATA_READ" : "-" 
        }
    """
    bool DP;
    bool AOA;
    bool AOS;
    bool T_DP_SENSOR;
    bool DP_GAIN;
    bool DP_OFFSET;
    bool AOA_OFFSET;
    bool AOS_OFFSET;
    bool LPF_DP;
    bool DP_I2C;
    bool AOA_I2C;
    bool AOS_I2C;
    bool SETTING_DATA_READ;
    """

class EnvSensorFLT(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('PS',          ctypes.c_bool),
        ('T_PS_SENSOR', ctypes.c_bool),
        ('TAT',         ctypes.c_bool),
        ('HUM',         ctypes.c_bool),
        ('PS_INIT_VAL', ctypes.c_bool),
        ('H_PS_OFFSET', ctypes.c_bool),
        ('H_PS',        ctypes.c_bool),
        ('RHO',         ctypes.c_bool),
        ('LPF_PS',      ctypes.c_bool),
        ('ENV_I2C',     ctypes.c_bool),
        ('SETTING_DATA_READ', ctypes.c_bool)    
    ]
    def __init__(self):
        super(EnvSensorFLT, self).__init__(
            PS = False,
            T_PS_SENSOR = False,
            TAT = False,
            HUM = False,
            PS_INIT_VAL = False,
            H_PS_OFFSET = False,
            H_PS = False,
            RHO = False,
            LPF_PS = False,
            ENV_I2C = False,
            SETTING_DATA_READ = False 
        )
        self.unit = {
            "PS" : "-",
            "T_PS_SENSOR" : "-",
            "TAT" : "-",
            "HUM" : "-",
            "PS_INIT_VAL" : "-",
            "H_PS_OFFSET" : "-",
            "H_PS" : "-",
            "RHO" : "-",
            "LPF_PS" : "-",
            "ENV_I2C" : "-",
            "SETTING_DATA_READ" : "-" 
        }
    """
    bool PS;
    bool T_PS_SENSOR;
    bool TAT;
    bool HUM;
    bool PS_INIT_VAL;
    bool H_PS_OFFSET;
    bool H_PS;
    bool RHO;
    bool LPF_PS;
    bool ENV_I2C;
    bool SETTING_DATA_READ;    
    """

class IMUSensorFLT(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('AX',        ctypes.c_bool),
        ('AY',        ctypes.c_bool),
        ('AZ',        ctypes.c_bool),
        ('GX',        ctypes.c_bool),
        ('GY',        ctypes.c_bool),
        ('GZ',        ctypes.c_bool),
        ('LPF_ACC',   ctypes.c_bool),
        ('LPF_GYR',   ctypes.c_bool),
        ('AX_OFFSET', ctypes.c_bool),
        ('AY_OFFSET', ctypes.c_bool),
        ('AZ_OFFSET', ctypes.c_bool),
        ('GX_OFFSET', ctypes.c_bool),
        ('GY_OFFSET', ctypes.c_bool),
        ('GZ_OFFSET', ctypes.c_bool),
        ('EX_OFFSET', ctypes.c_bool),
        ('EY_OFFSET', ctypes.c_bool),
        ('EZ_OFFSET', ctypes.c_bool),
        ('SETTING_DATA_READ', ctypes.c_bool)    
    ]
    def __init__(self):
        super(IMUSensorFLT, self).__init__(
            AX = False,
            AY = False,
            AZ = False,
            GX = False,
            GY = False,
            GZ = False,
            LPF_ACC = False,
            LPF_GYR = False,
            AX_OFFSET = False,
            AY_OFFSET = False,
            AZ_OFFSET = False,
            GX_OFFSET = False,
            GY_OFFSET = False,
            GZ_OFFSET = False,
            EX_OFFSET = False,
            EY_OFFSET = False,
            EZ_OFFSET = False,
            SETTING_DATA_READ = False  
        )
        self.unit = {
            "AX" : "-",
            "AY" : "-",
            "AZ" : "-",
            "GX" : "-",
            "GY" : "-",
            "GZ" : "-",
            "LPF_ACC" : "-",
            "LPF_GYR" : "-",
            "AX_OFFSET" : "-",
            "AY_OFFSET" : "-",
            "AZ_OFFSET" : "-",
            "GX_OFFSET" : "-",
            "GY_OFFSET" : "-",
            "GZ_OFFSET" : "-",
            "EX_OFFSET" : "-",
            "EY_OFFSET" : "-",
            "EZ_OFFSET" : "-",
            "SETTING_DATA_READ" : "-"  
        }
    """
    bool AX;
    bool AY;
    bool AZ;
    bool GX;
    bool GY;
    bool GZ;
    bool LPF_ACC;
    bool LPF_GYR;
    bool AX_OFFSET;
    bool AY_OFFSET;
    bool AZ_OFFSET;
    bool GX_OFFSET;
    bool GY_OFFSET;
    bool GZ_OFFSET;
    bool EX_OFFSET;
    bool EY_OFFSET;
    bool EZ_OFFSET;
    bool SETTING_DATA_READ;
    """

class GPSSensorFLT(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('PPS_INTV', ctypes.c_bool),
        ('MEG_INTV', ctypes.c_bool),
        ('CHK_SUM',  ctypes.c_bool)  
    ]
    def __init__(self):
        super(GPSSensorFLT, self).__init__(
            PPS_INTV = False,
            MEG_INTV = False,
            CHK_SUM = False 
        )
        self.unit = {
            "PPS_INTV" : "-",
            "MEG_INTV" : "-",
            "CHK_SUM" : "-" 
        }
    """
    bool PPS_INTV;
    bool MEG_INTV;
    bool CHK_SUM;
    """

class Status(ctypes.LittleEndianStructure): 
    _pack_ = 1
    _fields_ = [
        ('SD_DETECT',    ctypes.c_bool),
        ('LOG_NUMBER',   ctypes.c_ushort),
        ('LOGGING_MODE', ctypes.c_ubyte),
        ('LOGGING_EN',   ctypes.c_bool),
        ('CPU_USED',     ctypes.c_ubyte),
        ('MEMORY_USED',  ctypes.c_ubyte) 
    ]
    def __init__(self):
        super(Status, self).__init__(
            SD_DETECT = False,
            LOG_NUMBER = 0,
            LOGGING_MODE = 0,
            LOGGING_EN = False,
            CPU_USED = 0,
            MEMORY_USED = 0    
        )
        self.unit = {
            "SD_DETECT" : "-",
            "LOG_NUMBER" : "-",
            "LOGGING_MODE" : "-",
            "LOGGING_EN" : "-",
            "CPU_USED" : "%",
            "MEMORY_USED" : "%"    
        }
    """
    bool SD_DETECT;
    uint16_t LOG_NUMBER;
    byte LOGGING_MODE;
    bool LOGGING_EN;
    byte CPU_USED; //%
    byte MEMORY_USED; //%
    """

class CANIFFLT(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('INIT',     ctypes.c_bool),
        ('SET_MODE', ctypes.c_bool),
        ('SEND',     ctypes.c_bool),
        ('RECEIVE',  ctypes.c_bool)
    ]
    def __init__(self):
        super(CANIFFLT, self).__init__(
            INIT = False,
            SET_MODE = False,
            SEND = False,
            RECEIVE = False  
        )
        self.unit = {
            "INIT" : "-",
            "SET_MODE" : "-",
            "SEND" : "-",
            "RECEIVE" : "-"  
        }
    """
    bool INIT;
    bool SET_MODE;
    bool SEND;
    bool RECEIVE;
    """

class SysMessage(ctypes.LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ('DATA', ctypes.c_ubyte*10)
    ]
    def __init__(self, messeage):
        ctypes_ubyte_Array10 = ctypes.c_ubyte*10
        b_m = messeage.encode()
        c_messeage = ctypes_ubyte_Array10(b_m[0], b_m[1], b_m[2], b_m[3], b_m[4], b_m[5], b_m[6], b_m[7], b_m[8], b_m[9])
        super(SysMessage, self).__init__(
            DATA = c_messeage
        )
        self.unit = {
            "DATA" : "-", 
        }
    """    
    byte DATA[10]; 
    """
