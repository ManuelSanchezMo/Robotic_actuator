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
        self._base_can_frames = {'init_fb' : 1,'motor_out_elec': 2,  'motor_out_mec' : 3, 'transition': 4, 'motor_sp' : 7}
        self._motor_elec_out ={'Ua': 0.0, 'Ub': 0.0, 'current': 0.0, 'electrical_angle': 0.0}
        self._motor_mec_out ={'shaft_angle': 0.0, 'shaft_angle_sp': 0.0, 'shaft_velocity': 0.0}
        self._transition_table = [[0, 2, 0, 0, 0, 0, 0],[0, 0, 1, 3, 0, 0, 0],
                                  [0, 0, 0, 0, 2, 4, 0], [0, 0, 0, 0, 0, 0, 2]]
        self._states_table = [[1 , 2, 0, 0],[1, 2, 3, 4],
                             [0, 2, 3, 4], [1, 0, 0, 4]]
        print('len ' + str(len(self._transition_table[0])))
        self._current_state = 1
        self._gearbox_ratio = 6.0
        self.motor_command_frame = 7 
        self.fsm_transition_frame = 5
        self._motor_position = 0.0
        self.dmesg = self.can_db.encode_message(self.motor_command_frame,{'angle_sp' : 1000.0})
        print(self.motor_command_frame)
        self.send_msg(self.motor_command_frame + self.cobid , self.dmesg)
        print("sendede")
        #Thread(target = self.read_bus()).start() #read bus is a blocking fnc, so needs threading
    
    def change_state(self, transition):
        if 1 <= transition <len(self._transition_table[0]):
            if self._transition_table[self._current_state - 1][transition] != 0:
                self._current_state = self._transition_table[self._current_state - 1][transition]
           
                self.send_msg(self.motor_command_frame + self.cobid , [transition])

                print('correct ' + str(transition))
                
            else:
             print('incorrect')

    def send_command(self, angle):
        angle = angle*self._gearbox_ratio
        encoded_message = self.can_db.encode_message(self.motor_command_frame ,{'angle_sp' : angle})
        self.send_msg(self.motor_command_frame + self.cobid , encoded_message)

    def get_motor_angle(self):
        return self._motor_mec_out["shaft_angle"]/self._gearbox_ratio
    
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
                        print(self._base_can_frames["motor_out_elec"])
                        if (msg.arbitration_id-self.cobid) == self._base_can_frames["motor_out_elec"]:
                            print('rec motor elec')
                            self._motor_elec_out = self.can_db.decode_message((msg.arbitration_id-self.cobid), msg.data)
                            print(self._motor_elec_out )
                        elif (msg.arbitration_id-self.cobid) == self._base_can_frames["motor_out_mec"]:
                            print('rec motor mec')
                            self._motor_mec_out = self.can_db.decode_message((msg.arbitration_id-self.cobid), msg.data)
                            print(self._motor_mec_out )

                    except:
                        print("Error reading can message")

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

    print("mode " + str(motor._current_state))

    #motor.send_command(15.3)
    while True:
        x = input()
        motor.change_state(int(x))
        print("mode " + str(motor._current_state))
    motor.close_bus()
    motor.prueba()