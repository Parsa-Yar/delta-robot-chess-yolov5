import time
import serial
import serial.tools.list_ports as port_list
import camera as Camera
import client as Client


class DeltaManager():
    SERIAL_BAUD_RATE = 38400

    def __init__(self):
        pass

    ## Gripper functions
    def connect_gripper(self, ):
        self.gripper = None
        available_ports = self.get_all_ports()
        for port in available_ports:
            if 'COM' in self.get_port_name(port) or 'ttyACM' in self.get_port_name(port):  #TODO: check if this works on linux
                self.gripper = serial.Serial(
                    self.get_port_name(port), 
                    self.SERIAL_BAUD_RATE, 
                    timeout=None
                )    
        
        if self.gripper:
            self.gripper.write(f"\r\n".encode("utf-8"))
            time.sleep(5)
        else:
            # raise Exception("Gripper not found")
            print("Gripper not found")

    def get_all_ports(self, ):
        return [tuple(p) for p in list(port_list.comports())]

    def get_port_name(self, port):
        return port[0]
    
    def home_gripper(self, ):
        self.gripper.write(f"h".encode("utf-8"))
        # self.wait_till_done()
    
    def open_gripper(self, ):
        self.gripper.write(f"o".encode("utf-8"))
        # self.wait_till_done()

    def close_gripper(self, ):
        self.gripper.write(f"g".encode("utf-8"))
        # self.wait_till_done()

    def rotate_gripper(self, angle):
        # on degree -90:90 (its ralative to the current angle)
        self.gripper.write(f"r{angle}".encode("utf-8"))
        self.wait_till_done()

    def force_gripper(self, force):
        # int from 1 to 5000
        self.gripper.write(f"f{force}".encode("utf-8"))
        self.wait_till_done()
    
    def wait_till_done(self, ):
        while 1:
            result = self.gripper.readline().decode("utf-8")
            print(f"Gripper: {result}")
            if ("Done" in result) or ("failed" in result):
                break

    ## Delta functions
    def wait_till_done_robot(self, ):
        while 1:
            result = Client.Result
            print(f"Robot: {result}")
            if result == "success":
                break

    def go_home(self, ):
        Client.order("move", f"0,0,-37")
        self.wait_till_done_robot()

    def move(self, x, y, z):
        Client.order("move", f"{x},{y},{z}")
        self.wait_till_done_robot()

    def read_forward(self, ):
        Client.order("command", "forward")
        robot_current_coordinate = [Client.Result[1], Client.Result[2], Client.Result[3]]
        return robot_current_coordinate
