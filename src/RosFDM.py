#!/usr/bin/env python

import sys, os, pexpect, socket
import math, time, select, struct, signal, errno
import pygame
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from pysim import util
from pymavlink import fgFDM
import atexit
from pexpect import fdpexpect
from geometry_msgs.msg import Pose,Pose2D,Twist,PoseWithCovariance,TwistWithCovariance
from nav_msgs.msg import Odometry
import math
C_EARTH = 6372000
respath = "/Users/xuhao/Develop/FixedwingProj/ros_ws/src/ros_jsbsim/resources/"
os.chdir(respath)

class control_state(object):
    def __init__(self):
        self.aileron = 0
        self.elevator = 0
        self.throttle = 0
        self.rudder = 0
        self.ground_height = 0

sitl_state = control_state()

def interpret_address(addrstr):
    '''interpret a IP:port string'''
    a = addrstr.split(':')
    a[1] = int(a[1])
    return tuple(a)

class rosFDM:
    def __init__(self,opts):
        self.opts = opts
        self.setup_template(opts.home)
        self.launchjsb()
        self.wind = util.Wind(opts.wind)
        self.setup_fdm()
        self.init_pos = 0
        rospy.init_node('rosFDM')
        rospy.Subscriber("/joystick", Joy, self.channel_input)
        self.pos_pub = rospy.Publisher('/fdm/latlon', Pose2D, queue_size=0)
        self.odom_pub = rospy.Publisher('/fdm/odometry', Odometry, queue_size=0)
        self.pose_pub = rospy.Publisher('/fdm/pose',Pose,queue_size=0)

    def exit_handler(self):
        '''exit the sim'''
        jsb = self.jsb
        jsb_console = self.jsb_console
        print("running exit handler")
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # JSBSim really doesn't like to die ...
        if getattr(jsb, 'pid', None) is not None:
            os.kill(jsb.pid, signal.SIGKILL)
        jsb_console.send('quit\n')
        jsb.close(force=True)
        util.pexpect_close_all()
        sys.exit(1)

    def launchjsb(self):
        opts = self.opts
        cmd = "JSBSim --realtime --suspend --nice --simulation-rate=%u --logdirectivefile=%s/fgout.xml --script=%s " % (opts.rate,respath, respath + opts.script)
        print cmd
        jsb = pexpect.spawn(cmd, logfile=sys.stdout, timeout=10)
        jsb.delaybeforesend = 0
        util.pexpect_autoclose(jsb)
        i = jsb.expect(["Successfully bound to socket for input on port (\d+)",
                "Could not bind to socket for input"])
        if i == 1:
            print("Failed to start JSBSim - is another copy running?")
            sys.exit(1)
        self.jsb = jsb

        jsb_out_address = interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
        jsb.expect("Creating UDP socket on port (\d+)")
        jsb_in_address = interpret_address("127.0.0.1:%u" % int(jsb.match.group(1)))
        jsb.expect("Successfully connected to socket for output")
        jsb.expect("JSBSim Execution beginning")
        print("JSBSim console on %s" % str(jsb_out_address))
        jsb_out = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        jsb_out.connect(jsb_out_address)
        self.jsb_out = jsb_out
        jsb_console = fdpexpect.fdspawn(jsb_out.fileno(), logfile=sys.stdout)
        jsb_console.delaybeforesend = 0
        self.jsb_console = jsb_console

        # setup input from jsbsim
        print("JSBSim FG FDM input on %s" % str(jsb_in_address))
        jsb_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        jsb_in.bind(jsb_in_address)
        jsb_in.setblocking(0)
        self.jsb_in = jsb_in
    def channel_input(self,data):
        a = Joy()
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.axes)
        self.axes = data.axes
    def setup_fdm(self):
        self.fdm = fgFDM.fgFDM()
        jsb_console = self.jsb_console
        jsb = self.jsb
        jsb_console.send('info\n')
        jsb_console.send('resume\n')
        jsb.expect(["trim computation time","Trim Results"])
        time.sleep(1.5)
        jsb_console.send('step\n')
        jsb_console.logfile = None

        print "read to fly"


    def jsb_set(self,variable, value):
        '''set a JSBSim variable'''
        jsb_console = self.jsb_console
        jsb_console.send('set %s %s\r\n' % (variable, value))
    def setup_template(self,home):
        '''setup aircraft/Rascal/reset.xml'''
        opts = self.opts
        v = home.split(',')
        if len(v) != 4:
            print("home should be lat,lng,alt,hdg - '%s'" % home)
            sys.exit(1)
        latitude = float(v[0])
        longitude = float(v[1])
        altitude = float(v[2])
        heading = float(v[3])
        sitl_state.ground_height = altitude
        template = os.path.join(respath,'aircraft', 'Rascal', 'reset_template.xml')
        reset = os.path.join(respath,'aircraft', 'Rascal', 'reset.xml')
        xml = open(template).read() % { 'LATITUDE'  : str(latitude),
                                        'LONGITUDE' : str(longitude),
                                        'HEADING'   : str(heading) }
        open(reset, mode='w').write(xml)
        print("Wrote %s" % reset)

        baseport = int(opts.simout.split(':')[1])

        template = os.path.join(respath, 'fgout_template.xml')
        out      = os.path.join(respath, 'fgout.xml')
        xml = open(template).read() % { 'FGOUTPORT'  : str(baseport+3) }
        open(out, mode='w').write(xml)
        print("Wrote %s" % out)

        template = os.path.join(respath, 'rascal_test_template.xml')
        out      = os.path.join(respath, 'rascal_test.xml')
        xml = open(template).read() % { 'JSBCONSOLEPORT'  : str(baseport+4) }
        open(out, mode='w').write(xml)
        print("Wrote %s" % out)

    def update_input(self):
        opts = self.opts
        try:
            c1 , c2 , c3 ,c4 = self.axes[0] , self.axes[1] , self.axes[2] , self.axes[3]
        except:
            c1 ,c2 ,c3 ,c4 = 0,0,0,0
        speed = 0
        direction = 0
        turbulance = 0
        aileron = c1
        elevator = c2
        throttle = (c3 + 1) * 0.5
        rudder = c4
        if opts.elevon:
            # fake an elevon plane
            ch1 = aileron
            ch2 = elevator
            aileron  = (ch2-ch1)/2.0
            # the minus does away with the need for RC2_REV=-1
            elevator = -(ch2+ch1)/2.0

        if opts.vtail:
            # fake an elevon plane
            ch1 = elevator
            ch2 = rudder
            # this matches VTAIL_OUTPUT==2
            elevator = (ch2-ch1)/2.0
            rudder   = (ch2+ch1)/2.0

        buf = ''
        if aileron != sitl_state.aileron:
            buf += 'set fcs/aileron-cmd-norm %s\n' % aileron
            sitl_state.aileron = aileron
        if elevator != sitl_state.elevator:
            buf += 'set fcs/elevator-cmd-norm %s\n' % elevator
            sitl_state.elevator = elevator
        if rudder != sitl_state.rudder:
            buf += 'set fcs/rudder-cmd-norm %s\n' % rudder
            sitl_state.rudder = rudder
        if throttle != sitl_state.throttle:
            buf += 'set fcs/throttle-cmd-norm %s\n' % throttle
            sitl_state.throttle = throttle
        buf += 'step\n'
        jsb_console = self.jsb_console
        jsb_console.send(buf)

    def update_wind(self,wind):
        '''update wind simulation'''
        (speed, direction) = wind.current()
        self.jsb_set('atmosphere/psiw-rad', math.radians(direction))
        self.jsb_set('atmosphere/wind-mag-fps', speed/0.3048)

    def process_jsb_input(self,buf, simtime):
        '''process FG FDM input from JSBSim'''
        fdm = self.fdm
        fdm.parse(buf)
        timestamp = int(simtime*1.0e6)
        simbuf = struct.pack('<Q17dI',
                             timestamp,
                             fdm.get('latitude', units='degrees'),
                             fdm.get('longitude', units='degrees'),
                             fdm.get('altitude', units='meters'),
                             fdm.get('psi', units='degrees'),
                             fdm.get('v_north', units='mps'),
                             fdm.get('v_east', units='mps'),
                             fdm.get('v_down', units='mps'),
                             fdm.get('A_X_pilot', units='mpss'),
                             fdm.get('A_Y_pilot', units='mpss'),
                             fdm.get('A_Z_pilot', units='mpss'),
                             fdm.get('phidot', units='dps'),
                             fdm.get('thetadot', units='dps'),
                             fdm.get('psidot', units='dps'),
                             fdm.get('phi', units='degrees'),
                             fdm.get('theta', units='degrees'),
                             fdm.get('psi', units='degrees'),
                             fdm.get('vcas', units='mps'),
                             0x4c56414f)
        #lon E lat N
        latlondata = Pose2D(fdm.get('longitude', units='radians'),
                             fdm.get('latitude', units='radians'),fdm.get('altitude',units="meters"))
        self.pos_pub.publish(latlondata)
        roll = fdm.get("phi",units = 'radians')
        pitch = fdm.get("theta",units = 'radians')
        yaw = fdm.get("psi",units='radians')
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose0 = Pose()
        pose0.orientation.x = quaternion[0]
        pose0.orientation.y = quaternion[1]
        pose0.orientation.z = quaternion[2]
        pose0.orientation.w = quaternion[3]
        if self.init_pos == 0:
            pose0.position.x = 0
            pose0.position.y = 0
            pose0.position.z = - fdm.get("altitude",units="meters")
            self.init_pos = latlondata
        else:
            d_lon = latlondata.x - self.init_pos.x
            d_lat = latlondata.y - self.init_pos.y
            pose0.position.x = d_lat * C_EARTH
            pose0.position.y = d_lon * C_EARTH * math.cos(self.init_pos.y)
            pose0.position.z = - fdm.get("altitude",units="meters")
        twist0 = Twist()
        odom = Odometry(pose = PoseWithCovariance( pose = pose0),twist = TwistWithCovariance(twist = twist0))
        self.odom_pub.publish(odom)
        self.pose_pub.publish(pose0)


    def main_loop(self):
        signal.signal(signal.SIGINT, self.exit_handler)
        signal.signal(signal.SIGTERM, self.exit_handler)
        opts = self.opts
        jsb_in = self.jsb_in
        jsb_console = self.jsb_console
        fdm = self.fdm
        jsb = self.jsb
        tnow = time.time()
        last_report = tnow
        last_sim_input = tnow
        last_wind_update = tnow
        frame_count = 0
        paused = False
        simstep = 1.0/opts.rate
        simtime = simstep
        frame_time = 1.0/opts.rate
        scaled_frame_time = frame_time/opts.speedup
        last_wall_time = time.time()
        achieved_rate = opts.speedup

        while True:
            new_frame = False
            rin = [jsb_in.fileno(), jsb_console.fileno(), jsb.fileno()]
            try:
                (rin, win, xin) = select.select(rin, [], [], 1.0)
            except select.error:
                util.check_parent()
                continue

            tnow = time.time()

            if jsb_in.fileno() in rin:
                buf = jsb_in.recv(fdm.packet_size())
                self.process_jsb_input(buf, simtime)
                frame_count += 1
                new_frame = True

            #if sim_in.fileno() in rin:
            #simbuf = sim_in.recv(28)
            self.update_input()
            simtime += simstep
            last_sim_input = tnow

            # show any jsbsim console output
            if jsb_console.fileno() in rin:
                util.pexpect_drain(jsb_console)
            if jsb.fileno() in rin:
                util.pexpect_drain(jsb)

            # only simulate wind above 5 meters, to prevent crashes while
            # waiting for takeoff
            if tnow - last_wind_update > 0.1:
                self.update_wind(self.wind)
                last_wind_update = tnow

            if tnow - last_report > 0.1:
                print("FPS %u asl=%.1f agl=%.1f roll=%.1f pitch=%.1f a=(%.2f %.2f %.2f) AR=%.1f" % (
                    frame_count / (time.time() - last_report),
                    fdm.get('altitude', units='meters'),
                    fdm.get('agl', units='meters'),
                    fdm.get('phi', units='degrees'),
                    fdm.get('theta', units='degrees'),
                    fdm.get('A_X_pilot', units='mpss'),
                    fdm.get('A_Y_pilot', units='mpss'),
                    fdm.get('A_Z_pilot', units='mpss'),
                    achieved_rate))

                frame_count = 0
                last_report = time.time()

            if new_frame:
                now = time.time()
                if now < last_wall_time + scaled_frame_time:
                    dt = last_wall_time+scaled_frame_time - now
                    time.sleep(last_wall_time+scaled_frame_time - now)
                    now = time.time()

                if now > last_wall_time and now - last_wall_time < 0.1:
                    rate = 1.0/(now - last_wall_time)
                    achieved_rate = (0.98*achieved_rate) + (0.02*rate)
                    if achieved_rate < opts.rate*opts.speedup:
                        scaled_frame_time *= 0.999
                    else:
                        scaled_frame_time *= 1.001

                last_wall_time = now


if __name__ == "__main__":
    opts = {
        "home" : "37.4003371,-122.0800351,100,353",
        "script" : "rascal_test.xml",
        "options" : "",
        "elevon" : False,
        "revthr" : False,
        "vtail" : False,
        "wind" :"0,0,0",
        "rate" : 1000,
        "speedup" : 1,
        "simout" : "127.0.0.1:5051"
    }
    from collections import namedtuple
    opts = namedtuple('Struct', opts.keys())(*opts.values())

    rosFDM(opts).main_loop()

