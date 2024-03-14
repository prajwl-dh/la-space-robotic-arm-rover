#!/usr/bin/env python3
#coding: utf-8
import smbus
import time
# V0.0.5

class Arm_Device(object):

    def __init__(self):
        self.addr = 0x15
        self.bus = smbus.SMBus(1)

    # set bus servo：id: 1-6(0 means control all servo) angle: 0-180 
    def Arm_serial_servo_write(self, id, angle, time):
        if id == 0:  # control all servos
            self.Arm_serial_servo_write6(angle, angle, angle, angle, angle, angle, time)
        elif id == 2 or id == 3 or id == 4:  
            angle = 180 - angle
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write I2C error')
        elif id == 5:
            pos = int((3700 - 380) * (angle - 0) / (270 - 0) + 380)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write I2C error')
        else:
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write I2C error')

    # Set any bus servo angle interface: id: 1-250(0 is broadcast) angle: 0-180  900 3100   0 - 180
    def Arm_serial_servo_write_any(self, id, angle, time):
        if id != 0:
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x19, [id & 0xff, value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write_any I2C error')
        elif id == 0: # control all servos
            pos = int((3100 - 900) * (angle - 0) / (180 - 0) + 900)
            # pos = ((pos << 8) & 0xff00) | ((pos >> 8) & 0xff)
            value_H = (pos >> 8) & 0xFF
            value_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
            try:
                self.bus.write_i2c_block_data(self.addr, 0x17, [value_H, value_L, time_H, time_L])
            except:
                print('Arm_serial_servo_write_any I2C error')

    # Set the bus servo mid-position offset, and then send the following function,id:1-6(setting)，0（Restore initial statue）
    def Arm_serial_servo_write_offset_switch(self, id):
        try:
            if id > 0 and id < 7:
                self.bus.write_byte_data(self.addr, 0x1c, id)
            elif id == 0:
                self.bus.write_byte_data(self.addr, 0x1c, 0x00)
                time.sleep(.5)
        except:
            print('Arm_serial_servo_write_offset_switch I2C error')

    # Read the status of the neutral offset of the bus servo, 0 means no servo with the corresponding ID is detected, 1 means success, 2 means failure is out of range
    def Arm_serial_servo_write_offset_state(self):
        try:
            self.bus.write_byte_data(self.addr, 0x1b, 0x01)
            time.sleep(.001)
            state = self.bus.read_byte_data(self.addr, 0x1b)
            return state
        except:
            print('Arm_serial_servo_write_offset_state I2C error')
        return None

    # Set the bus servo angle :array
    def Arm_serial_servo_write6_array(self, joints, time):
        s1, s2, s3, s4, s5, s6 = joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]
        if s1 > 180 or s2 > 180 or s3 > 180 or s4 > 180 or s5 > 270 or s6 > 180:
            print("The parameter input range is not within 0-180！")
            return
        try:
            pos = int((3100 - 900) * (s1 - 0) / (180 - 0) + 900)
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF

            s2 = 180 - s2
            pos = int((3100 - 900) * (s2 - 0) / (180 - 0) + 900)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            s3 = 180 - s3
            pos = int((3100 - 900) * (s3 - 0) / (180 - 0) + 900)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            s4 = 180 - s4
            pos = int((3100 - 900) * (s4 - 0) / (180 - 0) + 900)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            pos = int((3700 - 380) * (s5 - 0) / (270 - 0) + 380)
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int((3100 - 900) * (s6 - 0) / (180 - 0) + 900)
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                    value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
            timeArr = [time_H, time_L]
            s_id = 0x1d
            self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
            self.bus.write_i2c_block_data(self.addr, s_id, data)
        except:
            print('Arm_serial_servo_write6 I2C error')

    # Set the angle of bus servo: s1~S4 and s6: 0-180, S5: 0~270, time is the running time
    def Arm_serial_servo_write6(self, s1, s2, s3, s4, s5, s6, time):
        if s1 > 180 or s2 > 180 or s3 > 180 or s4 > 180 or s5 > 270 or s6 > 180:
            print("参数传入范围不在0-180之内！")
            return
        try:
            pos = int((3100 - 900) * (s1 - 0) / (180 - 0) + 900)
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF

            s2 = 180 - s2
            pos = int((3100 - 900) * (s2 - 0) / (180 - 0) + 900)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            s3 = 180 - s3
            pos = int((3100 - 900) * (s3 - 0) / (180 - 0) + 900)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            s4 = 180 - s4
            pos = int((3100 - 900) * (s4 - 0) / (180 - 0) + 900)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            pos = int((3700 - 380) * (s5 - 0) / (270 - 0) + 380)
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int((3100 - 900) * (s6 - 0) / (180 - 0) + 900)
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF
            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                    value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
            timeArr = [time_H, time_L]
            s_id = 0x1d
            self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
            self.bus.write_i2c_block_data(self.addr, s_id, data)
        except:
            print('Arm_serial_servo_write6 I2C error')

    # Read the angle of the specified servo, id: 1-6 returns 0-180, read error returns None
    def Arm_serial_servo_read(self, id):
        if id < 1 or id > 6:
            print("id must be 1 - 6")
            return None
        try:
            self.bus.write_byte_data(self.addr, id + 0x30, 0x0)
            time.sleep(0.003)
            pos = self.bus.read_word_data(self.addr, id + 0x30)
        except:
            print('Arm_serial_servo_read I2C error')
            return None
        if pos == 0:
            return None
        pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00)
        # print(pos)
        if id == 5:
            pos = int((270 - 0) * (pos - 380) / (3700 - 380) + 0)
            if pos > 270 or pos < 0:
                return None
        else:
            pos = int((180 - 0) * (pos - 900) / (3100 - 900) + 0)
            if pos > 180 or pos < 0:
                return None
        if id == 2 or id == 3 or id == 4:
            pos = 180 - pos
        # print(pos)
        return pos

    # Read bus servo angle, id: 1-250 returns 0-180
    def Arm_serial_servo_read_any(self, id):
        if id < 1 or id > 250:
            print("id must be 1 - 250")
            return None
        try:
            self.bus.write_byte_data(self.addr, 0x37, id)
            time.sleep(0.003)
            pos = self.bus.read_word_data(self.addr, 0x37)
        except:
            print('Arm_serial_servo_read_any I2C error')
            return None
        # print(pos)
        pos = (pos >> 8 & 0xff) | (pos << 8 & 0xff00)
        # print(pos)
        pos = int((180 - 0) * (pos - 900) / (3100 - 900) + 0)
        # print(pos)
        return pos

    # Read the servo status, in normally it return 0xda, return 0x00 if no data is read, other values are servo errors
    def Arm_ping_servo(self, id):
        data = int(id)
        if data > 0 and data <= 250:
            reg = 0x38
            self.bus.write_byte_data(self.addr, reg, data)
            time.sleep(.003)
            value = self.bus.read_byte_data(self.addr, reg)
            times = 0
            while value == 0 and times < 5:
                self.bus.write_byte_data(self.addr, reg, data)
                time.sleep(.003)
                value = self.bus.read_byte_data(self.addr, reg)
                times += 1
                if times >= 5:
                    return None
            return value
        else:
            return None

    # Read the hardware version number
    def Arm_get_hardversion(self):
        try:
            self.bus.write_byte_data(self.addr, 0x01, 0x01)
            time.sleep(.001)
            value = self.bus.read_byte_data(self.addr, 0x01)
        except:
            print('Arm_get_hardversion I2C error')
            return None
        version = str(0) + '.' + str(value)
        # print(version)
        return version

    # Torque switch 1: Turn on the torque 0: Turn off the torque (you can change the angle of the servo)
    def Arm_serial_set_torque(self, onoff):
        try:
            if onoff == 1:
                self.bus.write_byte_data(self.addr, 0x1A, 0x01)
            else:
                self.bus.write_byte_data(self.addr, 0x1A, 0x00)
        except:
            print('Arm_serial_set_torque I2C error')

    # Set the ID number of the bus servo
    def Arm_serial_set_id(self, id):
        try:
            self.bus.write_byte_data(self.addr, 0x18, id & 0xff)
        except:
            print('Arm_serial_set_id I2C error')


    def Arm_Product_Select(self, index):
        try:
            self.bus.write_byte_data(self.addr, 0x04, index & 0xff)
        except:
            print('Arm_Product_Select I2C error')

    # Set RGB light color
    def Arm_RGB_set(self, red, green, blue):
        try:
            self.bus.write_i2c_block_data(self.addr, 0x02, [red & 0xff, green & 0xff, blue & 0xff])
        except:
            print('Arm_RGB_set I2C error')

    # Set K1 key mode, 0: default mode 1: study mode
    def Arm_Button_Mode(self, mode):
        try:
            self.bus.write_byte_data(self.addr, 0x03, mode & 0xff)
        except:
            print('Arm_Button_Mode I2C error')

    # Restart drive board
    def Arm_reset(self):
        try:
            self.bus.write_byte_data(self.addr, 0x05, 0x01)
        except:
            print('Arm_reset I2C error')

    # Control PWD servo id:1-6(0 means control all servo) angle：0-180
    def Arm_PWM_servo_write(self, id, angle):
        try:
            if id == 0:
                self.bus.write_byte_data(self.addr, 0x57, angle & 0xff)
            else:
                self.bus.write_byte_data(self.addr, 0x50 + id, angle & 0xff)
        except:
            print('Arm_PWM_servo_write I2C error')

    # Clear action
    def Arm_Clear_Action(self):
        try:
            self.bus.write_byte_data(self.addr, 0x23, 0x01)
            time.sleep(.5)
        except:
            print('Arm_Clear_Action I2C error')

    # In study mode, record the current action once
    def Arm_Action_Study(self):
        try:
            self.bus.write_byte_data(self.addr, 0x24, 0x01)
        except:
            print('Arm_Action_Study I2C error')

    #Action group runing mode 0: Stop 1: Single run 2: run in loop
    def Arm_Action_Mode(self, mode):
        try:
            self.bus.write_byte_data(self.addr, 0x20, mode & 0xff)
        except:
            print('Arm_Clear_Action I2C error')

    # Read the number of saved action groups
    def Arm_Read_Action_Num(self):
        try:
            self.bus.write_byte_data(self.addr, 0x22, 0x01)
            time.sleep(.001)
            num = self.bus.read_byte_data(self.addr, 0x22)
            return num
        except:
            print('Arm_Read_Action_Num I2C error')

    # Open buzzer, delay is 0xff by default，buzzer keep whistle 
    # delay=1~50，buzzer whistle delay*100 milliseconds, the maximum delay time is 5 seconds.
    def Arm_Buzzer_On(self, delay=0xff):
        try:
            if delay != 0:
                self.bus.write_byte_data(self.addr, 0x06, delay&0xff)
        except:
            print('Arm_Buzzer_On I2C error')

    # Close buzzer
    def Arm_Buzzer_Off(self):
        try:
            self.bus.write_byte_data(self.addr, 0x06, 0x00)
        except:
            print('Arm_Buzzer_Off I2C error')

    def bus_servo_control(self, id, num, time=1000):
        try:
            # if num > 4000 or num < 96:
            #     print("bus_servo_control error, num must be [96, 4000]")
            #     return

            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF
        
            if id == 1 or id == 6:
                if num > 3100 or num < 900:
                    print("bus_servo_control error, num must be [900, 3100]")
                    return
                pos = int(num)
                value_H = (pos >> 8) & 0xFF
                value_L = pos & 0xFF
            elif id == 2 or id == 3 or id == 4:  
                if num > 3100 or num < 900:
                    print("bus_servo_control error, num must be [900, 3100]")
                    return
                pos = int(3100 - num + 900)
                value_H = (pos >> 8) & 0xFF
                value_L = pos & 0xFF
            elif id == 5:
                if num > 4200 or num < 900:
                    print("bus_servo_control error, num must be [900, 4200]")
                    return
                pos = int(num - 514)
                value_H = (pos >> 8) & 0xFF
                value_L = pos & 0xFF
            else:
                print("bus_servo_control error, id must be [1, 6]")
                return
            self.bus.write_i2c_block_data(self.addr, 0x10 + id, [value_H, value_L, time_H, time_L])
        except:
            print('bus_servo_control error')

    def __change_value(self, value):
        try:
            val = 3100 - int(value) + 900
            return int(val)
        except:
            return None

    def bus_servo_control_array6(self, array, time=1000):
        try:
            if len(array) != 6:
                print("bus_servo_control_array6 input error")
                return

            s1, s2, s3, s4, s5, s6 = array[0], array[1], array[2], array[3], array[4], array[5]
            if s1 > 3100 or s2 > 3100 or s3 > 3100 or s4 > 3100 or s5 > 4200 or s6 > 3100:
                print("bus_servo_control_array6 input error")
                return
            elif s1 < 900 or s2 < 900 or s3 < 900 or s4 < 900 or s5 < 900 or s6 < 900:
                print("bus_servo_control_array6 input error")
                return

            pos = int(s1)
            value1_H = (pos >> 8) & 0xFF
            value1_L = pos & 0xFF

            s2 = self.__change_value(s2)
            pos = int(s2)
            value2_H = (pos >> 8) & 0xFF
            value2_L = pos & 0xFF

            s3 = self.__change_value(s3)
            pos = int(s3)
            value3_H = (pos >> 8) & 0xFF
            value3_L = pos & 0xFF

            s4 = self.__change_value(s4)
            pos = int(s4)
            value4_H = (pos >> 8) & 0xFF
            value4_L = pos & 0xFF

            s5 = s5 - 514
            pos = int(s5)
            value5_H = (pos >> 8) & 0xFF
            value5_L = pos & 0xFF

            pos = int(s6)
            value6_H = (pos >> 8) & 0xFF
            value6_L = pos & 0xFF

            time_H = (time >> 8) & 0xFF
            time_L = time & 0xFF

            data = [value1_H, value1_L, value2_H, value2_L, value3_H, value3_L,
                    value4_H, value4_L, value5_H, value5_L, value6_H, value6_L]
            timeArr = [time_H, time_L]
            s_id = 0x1d
            self.bus.write_i2c_block_data(self.addr, 0x1e, timeArr)
            self.bus.write_i2c_block_data(self.addr, s_id, data)
        except:
            print('bus_servo_control_array6 I2C error')
            
