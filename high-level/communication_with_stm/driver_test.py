from multiprocessing import Process
import serial
from serial.tools import list_ports
from cmd_list import REVERSED_CMD_LIST
from cmd_list import CMD_LIST

from packets import encode_packet, decode_packet


PORT_VID = '0483' #064E
PORT_PID = '5740' #9700
PORT_SNR = '325936843235' # HF1019-T838-SN03-REV0101
DEVICE_NAME = '/dev/ttyACM1'

class DriverException(Exception):
    pass

class Driver(Process):
    def __init__(self, input_cmd, fsm_output, lz_output,
            baudrate=9600, timeout=0.01, **kwargs):
        super(Driver, self).__init__(**kwargs)
        self.device = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.input_cmd = input_cmd

        self.fsm_output = fsm_output
        self.lz_output = lz_output

    def connect(self):
	print list(serial.tools.list_ports.comports())	
	for port_no, description, address in list(serial.tools.list_ports.comports()):
		if (PORT_VID in address) and ( PORT_PID in address) :
			DEVICE_NAME = port_no 
        #for port in list_ports.comports():		
	#			if (port.serial_number == PORT_SNR) and (port.pid == PORT_PID) and (port.vid == PORT_VID):
	#				self.device = port.device
	#				break
        self.device = "COM5"  ## Time-Limited correction!
        if self.device is None:
            raise DriverException('Device not found')
        self.port = serial.Serial(self.device,
            baudrate=self.baudrate, timeout=self.timeout)

    def close(self):
        self.port.close()

    def process_cmd(self, cmd):
        cmd_id = CMD_LIST[cmd['cmd']]
        if 'params' in cmd:
            packet = encode_packet(cmd_id, cmd['params'])
        else:
            packet = encode_packet(cmd_id, '')
        #print [c for c in packet]
        self.port.write(packet)
        data  = self.port.read(size=3)
        data = bytearray(data)
	#print "data",data
        data += self.port.read(size = int(data[2])) # TODO check correctness
        # clear buffer if error!
        return decode_packet(data)

    def run(self):

        self.connect()
        try:
            while True:
                cmd = input_command_queue.get()
                if cmd is None:
                    break
                reply = self.process_cmd(cmd)
                if cmd['source'] == 'fsm':
                    self.fsm_output.put(reply)
                elif cmd['source'] == 'localization':
                    self.lz_output.put(reply)
                else:
                    raise DriverException('Incorrect source')
        finally:
            self.close()



dr = Driver(1, 2, 3)
dr.connect()
i=0
f = open('Failed.py', 'w')
command = {'source': 'fsm', 'cmd': 'setspeedofleftwheel', 'params': '0.5'}
dr.process_cmd(command)
while (i<100):
	command = {'source': 'fsm', 'cmd': 'getSpeedOfLeftWheel', 'params': ''}
	leftwheelspeed = dr.process_cmd(command)['data'][0:7]
	with open('Failed.py', 'a') as file:
		file.write(leftwheelspeed)
		file.write('\n')
	i = i + 1
command = {'source': 'fsm', 'cmd': 'enableTraektAndStop', 'params': ''}
print dr.process_cmd(command)

