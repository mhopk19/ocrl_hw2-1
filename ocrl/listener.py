import time
import serial

# TO-DO: HAVE A CLASS VARIABLE THAT KEEPS TRACK OF WHETHER OR NOT THE DEVICE IS STILL CONNECTED AFTER EVERY FUNCTION CALL       
class Listener:
    def __init__(self, baud_rate = 115200, timeout_length = .1, my_com = 'COM7'):
        self.connected = True
        self.port = None
        try:
            self.port = serial.Serial(my_com,baud_rate,timeout = timeout_length)
            print("port connection was successful\n")
        except:
            print("port connection was unsuccessful\n")
            self.port = None
            self.connected = False
        
    def getIntMessage(self, length = None):
        if (self.connected == True):
            received_int = None
            if (length == None):
                ser_bytes = self.port.readline()
                converted_string = str(ser_bytes[0:len(ser_bytes)-2]).replace("b'",'').replace("\r\n",'').replace("'",'')
                if (len(converted_string) > 0):
                    try:
                        received_int = float(converted_string)
                        # print("coming from arduino {}".format(converted_string))
                        return received_int
                    except:
                        print("serial stall")
            else:
                # read line by line
                pass
            return received_int
        else:
            # print("the device is not connected: cannot read message")
            return None
        
    def sendMessage(self, message, length = None):
        if (self.port != None):
            self.port.write(message.encode('utf-8'))
        
    def close(self):
        self.port.close()
            
    