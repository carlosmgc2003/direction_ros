# @file ADS1115.py
# @author Red Lenses Panda Bear <redlensespandabear AT gmail.com>
# @version 1.0

# @section DESCRIPTION

# Este archivo contiene la clase necesaria para configurar
# y manejar el ADS1115 de TI mediante el bus I2C de la NVIDIA
# Jetson TX2

import i2cdev
from time import sleep

# I2C possible addresses
ADS1115_ADDRESS_0 = 0x48 #Address pin connected to GND
ADS1115_ADDRESS_1 = 0x49 #Address pin connected to VDD
ADS1115_ADDRESS_2 = 0x4A #Address pin connected to SDA
ADS1115_ADDRESS_3 = 0x4B #Address pin connected to SCL

# I2C address of the device
ADS1115_DEFAULT_ADDRESS	= 0x48

# ADS1115 Register Map
ADS1115_REG_POINTER_CONVERT	= 0x00 # Conversion register
ADS1115_REG_POINTER_CONFIG	= 0x01 # Configuration register
ADS1115_REG_POINTER_LOWTHRESH	= 0x02 # Lo_thresh register
ADS1115_REG_POINTER_HITHRESH	= 0x03 # Hi_thresh register

# ADS1115 Configuration Register
ADS1115_REG_CONFIG_OS_NOEFFECT = 0x00 # No effect
ADS1115_REG_CONFIG_OS_SINGLE = 0x80 # Begin a single conversion
ADS1115_REG_CONFIG_MUX_DIFF_0_1 = 0x00 # Differential P = AIN0, N = AIN1 (default)
ADS1115_REG_CONFIG_MUX_DIFF_0_3 = 0x10 # Differential P = AIN0, N = AIN3
ADS1115_REG_CONFIG_MUX_DIFF_1_3 = 0x20 # Differential P = AIN1, N = AIN3
ADS1115_REG_CONFIG_MUX_DIFF_2_3 = 0x30 # Differential P = AIN2, N = AIN3
ADS1115_REG_CONFIG_MUX_SINGLE_0 = 0x40 # Single-ended P = AIN0, N = GND
ADS1115_REG_CONFIG_MUX_SINGLE_1 = 0x50 # Single-ended P = AIN1, N = GND
ADS1115_REG_CONFIG_MUX_SINGLE_2 = 0x60 # Single-ended P = AIN2, N = GND
ADS1115_REG_CONFIG_MUX_SINGLE_3 = 0x70 # Single-ended P = AIN3, N = GND
ADS1115_REG_CONFIG_PGA_6_144V = 0x00 # +/-6.144V range = Gain 2/3
ADS1115_REG_CONFIG_PGA_4_096V = 0x02 # +/-4.096V range = Gain 1
ADS1115_REG_CONFIG_PGA_2_048V = 0x04 # +/-2.048V range = Gain 2 (default)
ADS1115_REG_CONFIG_PGA_1_024V = 0x06 # +/-1.024V range = Gain 4
ADS1115_REG_CONFIG_PGA_0_512V = 0x08 # +/-0.512V range = Gain 8
ADS1115_REG_CONFIG_PGA_0_256V = 0x0A # +/-0.256V range = Gain 16
ADS1115_REG_CONFIG_MODE_CONTIN = 0x00 # Continuous conversion mode
ADS1115_REG_CONFIG_MODE_SINGLE = 0x01 # Power-down single-shot mode (default)
ADS1115_REG_CONFIG_DR_8SPS = 0x00 # 8 samples per second
ADS1115_REG_CONFIG_DR_16SPS = 0x20 # 16 samples per second
ADS1115_REG_CONFIG_DR_32SPS = 0x40 # 32 samples per second
ADS1115_REG_CONFIG_DR_64SPS = 0x60 # 64 samples per second
ADS1115_REG_CONFIG_DR_128SPS = 0x80 # 128 samples per second (default)
ADS1115_REG_CONFIG_DR_250SPS = 0xA0 # 250 samples per second
ADS1115_REG_CONFIG_DR_475SPS = 0xC0 # 475 samples per second
ADS1115_REG_CONFIG_DR_860SPS = 0xE0 # 860 samples per second
ADS1115_REG_CONFIG_CMODE_TRAD = 0x00 # Traditional comparator with hysteresis (default)
ADS1115_REG_CONFIG_CMODE_WINDOW = 0x10 # Window comparator
ADS1115_REG_CONFIG_CPOL_ACTVLOW = 0x00 # ALERT/RDY pin is low when active (default)
ADS1115_REG_CONFIG_CPOL_ACTVHI = 0x08 # ALERT/RDY pin is high when active
ADS1115_REG_CONFIG_CLAT_NONLAT = 0x00 # Non-latching comparator (default)
ADS1115_REG_CONFIG_CLAT_LATCH = 0x04 # Latching comparator
ADS1115_REG_CONFIG_CQUE_1CONV = 0x00 # Assert ALERT/RDY after one conversions
ADS1115_REG_CONFIG_CQUE_2CONV = 0x01 # Assert ALERT/RDY after two conversions
ADS1115_REG_CONFIG_CQUE_4CONV = 0x02 # Assert ALERT/RDY after four conversions
ADS1115_REG_CONFIG_CQUE_NONE= 0x03 # Disable the comparator and put ALERT/RDY in high state (default)

bus_0 = i2cdev.I2C(ADS1115_ADDRESS_0,1) # Address = 0x48, I2C bus = 1
bus_1 = i2cdev.I2C(ADS1115_ADDRESS_1,1) # Address = 0x48, I2C bus = 1
bus_2 = i2cdev.I2C(ADS1115_ADDRESS_2,1) # Address = 0x48, I2C bus = 1
bus_3 = i2cdev.I2C(ADS1115_ADDRESS_3,1) # Address = 0x48, I2C bus = 1

# def address_switcher(address_number):
#         switcher = {
#                 0: ADS1115_ADDRESS_0,
#                 1: ADS1115_ADDRESS_1,
#                 2: ADS1115_ADDRESS_2,
#                 3: ADS1115_ADDRESS_3,
#         }
#         return switcher.get(address_number,ADS1115_DEFAULT_ADDRESS)

class ADS1115_0():
        # def set_address_and_bus(self,adress_number,bus_num):
        #         address = address_switcher(adress_number)
        #         bus = i2cdev.I2C(address,bus_num)
        
        # TODO: Ver esta sobrecarga de set_channel, en mi opinion no es viable en Python
        #def set_channel(self):
        #        self.channel = int(input("Enter the Channel No. = "))
        #        while self.channel > 3 :
        #                self.channel = int(input("Enter the Channel No. = "))
        #        return self.channel

        def set_channel(self, channel):
                self.channel = channel


        def config_single_ended(self):
            # Select the Configuration Register data from the
            # given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_0
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_2
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
 #               print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2) )
                bus_0.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
                bus_0.write(bytes([int(0)]))
                sleep(0.1)


        def config_differential(self):
            # Select the Configuration Register
            # data from the given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_1_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_2_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
#                print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2))
                bus_0.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
#                print ("Success: registro de configuracin escrito")
#                print ("Cambiando el puntero al registro de medicin")
                bus_0.write(bytes([int(0)]))
                sleep(0.1)
#                print ("Success: puntero cambiado al registro de medicin")

        def read_adc(self):
                # Read data back from ADS1115_REG_POINTER_CONVERT(0x00),
                # 2 bytes raw_adc MSB, raw_adc LSB
                raw_bytes=bus_0.read(2)
                value = int.from_bytes(raw_bytes, byteorder='big')
                return int(value)

class ADS1115_1():
        # def set_address_and_bus(self,adress_number,bus_num):
        #         address = address_switcher(adress_number)
        #         bus = i2cdev.I2C(address,bus_num)
        def set_channel(self):
                self.channel = int(input("Enter the Channel No. = "))
                while self.channel > 3 :
                        self.channel = int(input("Enter the Channel No. = "))
                return self.channel

        def set_channel(self, channel):
                self.channel = channel


        def config_single_ended(self):
            # Select the Configuration Register data from the
            # given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_0
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_2
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
 #               print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2) )
                bus_1.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
#                print ("Registro de configuracin escrito")
#                print ("Cambiando el puntero al registro de medicin")
                bus_1.write(bytes([int(0)]))
                sleep(0.1)
#                print ("Success: puntero cambiado al registro de medicin")

        def config_differential(self):
            # Select the Configuration Register
            # data from the given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_1_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_2_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
#                print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2))
                bus_1.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
#                print ("Success: registro de configuracin escrito")
#                print ("Cambiando el puntero al registro de medicin")
                bus_1.write(bytes([int(0)]))
                sleep(0.1)
#                print ("Success: puntero cambiado al registro de medicin")

        def read_adc(self):
                # Read data back from ADS1115_REG_POINTER_CONVERT(0x00),
                # 2 bytes raw_adc MSB, raw_adc LSB
                raw_bytes=bus_1.read(2)
                value = int.from_bytes(raw_bytes, byteorder='big')
                return int(value)

class ADS1115_2():
        # def set_address_and_bus(self,adress_number,bus_num):
        #         address = address_switcher(adress_number)
        #         bus = i2cdev.I2C(address,bus_num)
        def set_channel(self):
                self.channel = int(input("Enter the Channel No. = "))
                while self.channel > 3 :
                        self.channel = int(input("Enter the Channel No. = "))
                return self.channel

        def set_channel(self, channel):
                self.channel = channel


        def config_single_ended(self):
            # Select the Configuration Register data from the
            # given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_0
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_2
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
 #               print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2) )
                bus_2.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
#                print ("Registro de configuracion escrito")
#                print ("Cambiando el puntero al registro de medicin")
                bus_2.write(bytes([int(0)]))
                sleep(0.1)
#                print ("Success: puntero cambiado al registro de medicin")

        def config_differential(self):
            # Select the Configuration Register
            # data from the given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_1_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_2_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
#                print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2))
                bus_2.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
#                print ("Success: registro de configuracin escrito")
#                print ("Cambiando el puntero al registro de medicin")
                bus_2.write(bytes([int(0)]))
                sleep(0.1)
#                print ("Success: puntero cambiado al registro de medicin")

        def read_adc(self):
                # Read data back from ADS1115_REG_POINTER_CONVERT(0x00),
                # 2 bytes raw_adc MSB, raw_adc LSB
                raw_bytes=bus_2.read(2)
                value = int.from_bytes(raw_bytes, byteorder='big')
                return int(value)

class ADS1115_3():
        # def set_address_and_bus(self,adress_number,bus_num):
        #         address = address_switcher(adress_number)
        #         bus = i2cdev.I2C(address,bus_num)
        def set_channel(self):
                self.channel = int(input("Enter the Channel No. = "))
                while self.channel > 3 :
                        self.channel = int(input("Enter the Channel No. = "))
                return self.channel

        def set_channel(self, channel):
                self.channel = channel


        def config_single_ended(self):
            # Select the Configuration Register data from the
            # given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_0
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_2
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_SINGLE_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_250SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
 #               print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2) )
                bus_3.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
#                print ("Registro de configuracin escrito")
#                print ("Cambiando el puntero al registro de medicin")
                bus_3.write(bytes([int(0)]))
                sleep(0.1)
#                print ("Success: puntero cambiado al registro de medicin")

        def config_differential(self):
            # Select the Configuration Register
            # data from the given provided value above
                if self.channel == 0:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_1
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 1:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_0_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 2:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_1_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
                elif self.channel == 3:
                        config_reg_1 = ADS1115_REG_CONFIG_OS_NOEFFECT | ADS1115_REG_CONFIG_MUX_DIFF_2_3
                        config_reg_1 = config_reg_1 | ADS1115_REG_CONFIG_PGA_4_096V | ADS1115_REG_CONFIG_MODE_CONTIN
                        config_reg_2 = ADS1115_REG_CONFIG_DR_128SPS | ADS1115_REG_CONFIG_CMODE_TRAD | ADS1115_REG_CONFIG_CPOL_ACTVLOW
                        config_reg_2 = config_reg_2 | ADS1115_REG_CONFIG_CLAT_NONLAT | ADS1115_REG_CONFIG_CQUE_NONE
#                print ("Escribiendo en registro 1: \n", "Registro 0: ", int(config_reg_1), "\n", "Registro 1: ", int(config_reg_2))
                bus_3.write(bytes([int(1),int(config_reg_1), int(config_reg_2)]))
                sleep(0.1)
#                print ("Success: registro de configuracin escrito")
#                print ("Cambiando el puntero al registro de medicin")
                bus_3.write(bytes([int(0)]))
                sleep(0.1)
#                print ("Success: puntero cambiado al registro de medicin")

        def read_adc(self):
                # Read data back from ADS1115_REG_POINTER_CONVERT(0x00),
                # 2 bytes raw_adc MSB, raw_adc LSB
                raw_bytes=bus_3.read(2)
                value = int.from_bytes(raw_bytes, byteorder='big')
                return int(value)
