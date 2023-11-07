import can
import cantools
from threading import Thread

class motor_driver():
    def __init__(self, cobid = 0, interface='socketcan', channel='can0', bitrate=500000, dicfile = 'file2.dbc'):
        self.cobid = cobid
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.bus = can.ThreadSafeBus(interface= self.interface, channel=self.channel, bitrate=self.bitrate)
        self.dicfile = dicfile
        self.can_db = cantools.database.load_file(dicfile)
        self.dmesg = self.can_db.encode_message(1,{'P_control' : 30,'I_control': 10,  'D_control' : 10,  'Volt_limit' : 0})
        self.send_msg(1 , self.dmesg)
        self.base_can_frames = {'toInit' : 10,'toReady': 13,  'toStop' : 43}
        self.motor_command_frame = 7 
        
        self.dmesg = self.can_db.encode_message(self.motor_command_frame,{'angle_sp' : 1000.0})
        print(self.motor_command_frame)
        self.send_msg(self.motor_command_frame + self.cobid , self.dmesg)
        print("sendede")
        Thread(target = self.read_bus()).start() #read bus is a blocking fnc, so needs threading
     
     
    def send_command(self, angle):
        encoded_message = self.can_db.encode_message(self.motor_command_frame ,{'angle_sp' : angle})
        self.send_msg(self.motor_command_frame + self.cobid , encoded_message)

    def read_bus(self):
        while True:
            msg = self.bus.recv(1)
            if msg is not None:
                    pass
                    #self.send_msg(0x001, self.dmesg)

                    print(msg)
                    #self.can_db.decode_message(msg.arbitration_id-self.cobid, msg.data)

                    try:
                        print(msg.arbitration_id-self.cobid)
                        decoded_message = self.can_db.decode_message(msg.arbitration_id-self.cobid, msg.data)
                    except:
                        print(ValueError)
                    print(decoded_message)

    def send_msg(self,frame,msg):
        msg = can.Message(arbitration_id=frame, data=msg, is_extended_id=True)

        try:
            self.bus.send(msg)
        except (ValueError, TypeError) as e:
            print("Wrong CAN message data: " +  str(e))
        except can.CanError as e:
            print(" Failed to send CAN message: " + str(e))
            
    def close_bus(self):
        self.bus.shutdown()
        
    def init_motor(self, P_controller = 1.0, I_controller = 0.01, 
                   D_controller = 0, Volt_limit = 20):
        init_cobid = self.base_can_frames['toInit'] + self.cobid
        msg = self.can_db.encode_message(0x1,{'position' : 10,'speed': 13,  'FB3' : 0})
        self.send_msg(init_cobid, msg)
        pass
    
    def preop_motor(self, is_calibrated = False, direction = 1, 
                   D_controller = 0, Volt_limit = 20):
        pass       
    
    def prueba(self, fuera = 'dentro'):
        print(fuera)

if __name__ == '__main__':
    print('Hola')
    motor = motor_driver(cobid = 3, channel='vcan0')
    print(motor.cobid)
    motor.send_command(15.3)
    motor.close_bus()
    motor.prueba()