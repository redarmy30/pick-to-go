from multiprocessing import Process
import serial
from serial.tools import list_ports
from cmd_list import REVERSED_CMD_LIST
from cmd_list import CMD_LIST

from packets import encode_packet, decode_packet


PORT_VID = '0483'
PORT_PID = '5740'
PORT_SNR = '325936843235'
DEVICE_NAME = '/dev/ttyACM1'

class DriverException(Exception):
    pass

# code is sensitive to the types, If input Float use 1.0 not 1!
# message format:
#   {'source': <'fsm' or 'localization'>,
#   'cmd': 'cmd name',
#   'params': [<parameters>]}
# reply format: {'cmd': 'cmd name', 'data': <reply data>}

# You can test it without spawning process:
# >>> from driver import Driver
# >>> d = Driver(1,2,3)
# >>> d.connect()
# >>> d.process_cmd(cmd)
class Driver(Process):
    def __init__(self, input_cmd, fsm_output, lz_output,
            baudrate=9600, timeout=1, **kwargs):
        super(Driver, self).__init__(**kwargs)
        self.device = None
        self.baudrate = baudrate
        self.timeout = timeout
        self.input_cmd = input_cmd
        self.fsm_output = fsm_output
        self.lz_output = lz_output

    def connect(self):
		
        for port in list_ports.comports():		
				if (port.serial_number == PORT_SNR) and (port.pid == PORT_PID) and (port.vid == PORT_VID):
					self.device = port.device
					break
        self.device = DEVICE_NAME  ## Time-Limited correction!
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
        data += self.port.read(size = int(data[2]) - 3) # TODO check correctness
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
command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
print dr.process_cmd(command)
