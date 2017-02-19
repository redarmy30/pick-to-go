import  cmd_list
import  driver_test
import  packets
import time
#import ParticleFilter as pf
#from hokuyolx import HokuyoLX
import matplotlib.pyplot as plt





class Robotishe():
    def __init__(self,lidar_on=True):
        if lidar_on:
            #self.lidar = HokuyoLX(tsync=False)
            self.lidar.convert_time = False
        self.lidar_on = lidar_on
        #self.particles = [pf.Robot() for i in range(pf.particle_number)]
        self.dr = driver.Driver(1, 2, 3)
        self.dr.connect()
        self.x = 170  # mm
        self.y = 170  # mm
        self.angle = 0.0  # pi
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        print self.dr.process_cmd(command)
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x/1000., self.y/1000., self.angle]}
        print self.dr.process_cmd(command)


    def lidar_sense(self):
        """Function returns landmark data"""
        timestamp, scan = self.lidar.get_intens()
        # distances to landmarks(up to 3)!
        ans = []
        ideals = pf.ideal(self.x,self.y)
        #print ideals
        beacons = pf.get_beacons(scan)
        #print beacons
        for i in beacons:
            di = i[1] + 40
            for j in ideals:  # small corectness checker
                if abs(di-j) < 200:
                    ans.append(di)
                    break
        return ans


    def stm_test(self):
        command = {'source': 'fsm', 'cmd': 'echo', 'params': 'ECHO'}
        print self.dr.process_cmd(command)
        #command = {'source': 'fsm', 'cmd': 'switchOnPneumo','params': ''}
        #print d.process_cmd(command)
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [0.0, 0.0, 0.0]}
        print self.dr.process_cmd(command)
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': [0.8, 0.0, 0.0, 4]}
        print self.dr.process_cmd(command)
        # time.sleep(5)
        # command = {'source': 'fsm', 'cmd': 'switchOffPneumo','params': ''}
        # print d.process_cmd(command)

    def stm_local(self):
        particles = [pf.Robot() for i in range(pf.particle_number)]
        main_robot = pf.calculate_main(particles)
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [0.0, 0.0, 0.0]}
        print self.dr.process_cmd(command)
        ax = plt.subplot(111)
        plot = ax.plot([], [], 'ro')[0]
        plt.axis([0, 2000, 0, 3000])
        plt.show()
        cord = [0, 0]
        plot.set_data(cord[0], cord[1])
        # make movement
        parameters = [0.15, 0, 0, 4]
        self.make_mov(parameters)
        cord = [main_robot.x, main_robot.y]
        plot.set_data(cord[0], cord[1])
        print main_robot

    def make_move(self,parameters):
        # in param MM
        pm = [(self.x+parameters[0])/1000.,(self.y+parameters[1])/1000.,parameters[2],parameters[3]]
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        print self.dr.process_cmd(command)
        self.particles = pf.particles_move(self.particles,parameters )#[i/2. for i in parameters]
        # After movement
        # stamp = time.time()
        # while not stm_driver('is_point_was_reached'):
        #     time.sleep(0.3)
        #     if (time.time() - stamp) > 30:
        #         return False
        time.sleep(3)
        lidar_data = self.lidar_sense()
        print lidar_data
        for i in range(30):
            self.particles = pf.particles_sense(self.particles, lidar_data)
            self.particles = pf.particles_sense(self.particles, lidar_data)
            self.particles = pf.particles_sense(self.particles, lidar_data)
            main_robot = pf.calculate_main(self.particles)
            print main_robot
        main_robot = pf.calculate_main(self.particles)
        self.x = main_robot.x
        self.y = main_robot.y
        print main_robot

    def go_to_coord(self):
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        print self.dr.process_cmd(command)

    def go_to_coord_rotation(self,parameters): #  parameters [x,y,angle,speed]
        pm = [parameters[0] / 1000., parameters[1] / 1000., parameters[2], parameters[3]]
        command = {'source': 'fsm', 'cmd': 'addPointToStack', 'params': pm}
        print self.dr.process_cmd(command)
        # After movement
        stamp = time.time()
        cmd = {'source': 'fsm', 'cmd': 'is_point_was_reached'}
        while not self.dr.process_cmd(cmd)['data']:
            time.sleep(0.3)
            if (time.time() - stamp) > 30:
                return False # Error
        print 'reached'
        if self.lidar_on:
            lidar_data = self.lidar_sense()
            for i in range(30):
                self.particles = pf.particles_sense(self.particles, lidar_data)
                self.particles = pf.particles_sense(self.particles, lidar_data)
                self.particles = pf.particles_sense(self.particles, lidar_data)
                main_robot = pf.calculate_main(self.particles)
                print main_robot
        self.x = main_robot.x
        self.y = main_robot.y
        command = {'source': 'fsm', 'cmd': 'setCoordinates', 'params': [self.x / 1000., self.y / 1000., self.angle]}
        print self.dr.process_cmd(command)










def test():
    rb = Robotishe(True)
    rb.make_move([100.0, 0.0, 0.0, 4])

#test()

#

#rb = Robotishe(False)
#print rb.lidar_sense()

def func_test():
    dr = driver.Driver(1, 2, 3)
    dr.connect()
    command = {'source': 'fsm', 'cmd': 'is_point_was_reached'}
    print (dr.process_cmd(command))

#func_test()
def t():
    rb = Robotishe(False)
    rb.go_to_coord_rotation([300, 170, 0.0, 4])


rb = Robotishe(False)







