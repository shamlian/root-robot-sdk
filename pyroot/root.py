# External Dependencies
import threading
import time
from struct import pack, unpack
import queue
import numpy

# Internal Dependencies
from .packet import Packet

class Root(object):
    """Simplifies communication with a real Root robot.

    Unless otherwise indicated, all methods are non-blocking.
    Packets are sent to the robot and received in separate threads;
    replies are interpreted when received and responses are placed
    in internal class state variables.

    Full descriptions of Root BLE packets can be found at
    RootRobotics/root-robot-ble-protocol
    """

    def __init__(self, phy):
        """Sets up data link layer for Root robot. Kicks off some threads
        used to manage the connection, and uses initialize_state() to
        populate some information about the robot into the class.

        Parameters
        ----------
        phy: RootPhy
            Initialized RootPhy object. Used to pick physical layer.
        """

        self._phy = phy
        # do some check here to be sure phy is an initialized RootPhy object

        try:
            self._tx_q = queue.SimpleQueue()
        except AttributeError:
            self._tx_q = queue.Queue()
        self._rx_q = self._phy.rx_q

        self.pending_lock = threading.Lock()
        self.pending_resp = []
        """list: List of responses pending from the robot."""

        self.sniff_mode = False
        """bool: If True, shows the raw transactions to and from the robot."""

        self.ignore_crc_errors = False
        """bool: If true, ignores CRC errors in packets from the robot."""

        self.stop_project_flag = threading.Event()
        """Event: signals that Stop Project message was received."""

        self.state = {}
        """dict: Contains local state of robot"""

        self._last_coord = (0+0j)
        """complex: Contains last known coordinates of robot."""
        self._last_theta_x10 = 900
        """int: Contains last known heading of robot."""

        self.create_empty_state()

        threading.Thread(target = self._sending_thread).start()
        threading.Thread(target = self._receiving_thread).start()
        threading.Thread(target = self._expiration_thread).start()

        self.initialize_state()

    def is_running(self):
        """Utility function for determining state of phy thread."""
        return self._phy.is_connected()

    def disconnect(self, timeout = 3):
        """Request disconnect from the robot and shut down connection.

        Parameters
        -------
        timeout : float
            Number of seconds to wait for all pending transmissions to
            be sent before forcing the disconnect at the physical layer.
        """

        self._tx_q.put((Packet(0, 6, 0), False))

        t = time.time() + timeout
        
        while time.time() < t and self.transmissions_pending():
            time.sleep(0.1)
        self._phy.disconnect()

    def create_empty_state(self):
        """Set up internal state dictionary with all state set to None.
        """

        for devnum, device in self.supported_devices.items():
            self.state[device] = None
        for devnum, device in self.virtual_devices.items():
            self.state[device] = None

    def initialize_state(self):
        """Initialize internal state dictionary.

        Since certain versions of the main board protocol don't support
        CRC properly, also request version information and set some
        internal flags so that warnings are thrown appropriately.
        """

        self.disable_events()
        time.sleep(1) # not sure why this is necessary

        self.get_versions(self.main_board)
        self.get_versions(self.color_board)

        timeout = time.time() + 5
        blocked = True
        while time.time() < timeout and blocked:
            try:
                if self.state['General'][self.main_board] < 1.011:
                    self.ignore_crc_errors = True
                blocked = False
            except TypeError:
                time.sleep(0.1)
        if blocked == True:
                print('Warning: could not get main board version')

        self.get_name()
        self.get_serial_number()
        self.get_battery_level()

        self.enable_events()

    #TODO: Use enums here and elsewhere
    main_board = 0xA5
    color_board = 0xC6
    def get_versions(self, board):
        """Requests the firmware version of a particular board in the robot.

        Parameters
        ----------
        board : byte
            Byte defining the board whose version is being requested.
            For convenience, the following constants are defined:
            * main_board
            * color_board
        """

        self._tx_q.put((Packet(0, 0, 0, payload=bytes([board])), True))

    def set_name(self, name):
        """Sets the robot's name.

        Parameters
        ----------
        name : str
            New name for the robot.

        Returns
        -------
        utf_name : bytes
        Actual name set for the robot, since the name requested may not fit.
        Truncates the name if its UTF-8 representation is longer than 16 bytes.
        Returns None if the name supplied cannot be converted.
        """

        try:
            utf_name = name.encode('utf-8')
            while len(utf_name) > Packet.PAYLOAD_LEN:
                name = name[:-1]
                utf_name = name.encode('utf-8')
        except AttributeError:
            return None

        if utf_name == b'':
            name = b'\x46\x4c\x45\x41'

        self._tx_q.put((Packet(0, 1, 0, payload=utf_name), False))
        return utf_name

    def get_name(self):
        """Requests the robot's name."""
        self._tx_q.put((Packet(0, 2, 0), True))

    def stop_and_reset(self):
        """Requests robot stop and cancel all pending actions."""
        self._tx_q.put((Packet(0, 3, 0), False))

    def enable_events(self):
        """Currently enables all events from the robot.

        TODO: Request better documentation about the payload of
        this packet and implement it.
        """
        payload = bytes([0xFF] * Packet.PAYLOAD_LEN)
        self._tx_q.put((Packet(0, 7, 0, payload=payload), False))

    def disable_events(self):
        """Currently disables all events from the robot.

        TODO: Request better documentation about the payload of
        this packet and implement it.
        """
        payload = bytes([0xFF] * Packet.PAYLOAD_LEN)
        self._tx_q.put((Packet(0, 8, 0, payload=payload), False))

    def get_enabled_events(self):
        """Request bitfield of enabled devices."""
        self._tx_q.put((Packet(0, 11, 0), True))

    def get_serial_number(self):
        """Request robot serial number."""
        self._tx_q.put((Packet(0, 14, 0), True))

    def set_motor_speeds(self, left, right):
        """Set left and right motor linear velocities.

        Parameters
        ----------
        left : int
            Left motor speed in units of mm/s.
        right : int
            Right motor speed in units of mm/s.
        """
        left = pack('>i', self._bound(left, -100, 100))
        right = pack('>i', self._bound(right, -100, 100))
        self._tx_q.put((Packet(1, 4, 0, payload=left + right), False))

    def set_left_motor_speed(self, left):
        """Set left motor linear velocity.

        Parameters
        ----------
        left : int
            Left motor speed in units of mm/s.
        """
        left = pack('>i', self._bound(left, -100, 100))
        self._tx_q.put((Packet(1, 6, 0, payload=left), False))

    def set_right_motor_speed(self, right):
        """Set right motor linear velocity.

        Parameters
        ----------
        right : int
            Right motor speed in units of mm/s.
        """
        right = pack('>i', self._bound(right, -100, 100))
        self._tx_q.put((Packet(1, 7, 0, payload=right), False))

    def drive_distance(self, distance):
        """Drive in a straight line for a certain distance.

        Parameters
        ----------
        distance : int
            Distance to travel in units of mm.
        """
        self._tx_q.put((Packet(1, 8, 0, payload=pack('>i', distance)), True))

    def rotate_angle(self, angle):
        """Turn the robot in place by a particular angle.

        Parameters
        ----------
        angle : int
            Angle to turn in units of deci-degrees.
        """
        self._tx_q.put((Packet(1, 12, 0, payload=pack('>i', angle)), True))

    def drive_arc(self, angle, radius):
        """Drive in an arc subtending a particular angle along a circle.

        Parameters
        ----------
        angle : int
            Angle of an arc to drive along, in deci-degrees.
        radius : int
            Radius of the circle upon which to travel.
        """
        payload = pack('>ii', angle, radius)
        self._tx_q.put((Packet(1, 27, 0, payload=payload), True))

    def drive_xy(self, x, y):
        """Drive to a particular coordinate in the XY plane.

        Parameters
        ----------
        x : float
            X coordinate to which to head.
        y : float
            Y coordinate to which to head.
        """
        self.drive_complex(x + y * 1j)

    def drive_complex(self, coord):
        """Drive to a particular coordinate in the XY plane.

        Parameters
        ----------
        coord : complex
            Coordinate to which to head, described as a complex number.
        """

        vector    = (coord - self._last_coord)
        dist      = numpy.linalg.norm(vector)
        theta     = numpy.angle(vector, deg=True)
        theta_x10 = int(theta * 10)
        turn      = ((self._last_theta_x10 - theta_x10 + 1800) % 3600) - 1800
        dist      = int(dist)

        #print(self._last_theta_x10, '->', theta_x10, ':', turn)
        #print('turn', turn/10, ' drive', dist)
        self.rotate_angle(turn)
        self.drive_distance(dist)

        self._last_coord = (numpy.real(self._last_coord) + dist * numpy.cos(theta_x10/10*numpy.pi/180)) + \
                          (numpy.imag(self._last_coord) + dist * numpy.sin(theta_x10/10*numpy.pi/180))*1j
        self._last_theta_x10 = theta_x10

    marker_up_eraser_up = 0
    marker_down_eraser_up = 1
    marker_up_eraser_down = 2

    def set_marker_eraser_pos(self, pos):
        """Set the Marker/Eraser actuator to a particular position.

        Parameters
        ----------
        pos : byte
            Byte describing the position to acquire.
            For convenience, the following constants are defined.
            * marker_up_eraser_up
            * marker_down_eraser_up
            * marker_up_eraser_down
        """
        pos = self._bound(pos, 0, 2)
        #print('Set pen', pos)
        self._tx_q.put((Packet(2, 0, 0, payload=bytes([pos])), True))

    led_animation_off = 0
    led_animation_on = 1
    led_animation_blink = 2
    led_animation_spin = 3

    def set_led_animation(self, state, red, green, blue):
        """Animate the LED lights on top of the robot.

        Parameters
        ----------
        state : byte
            Byte describing the animation style.
            For convenience, the following constants are defined.
            * led_animation_off
            * led_animation_on
            * led_animation_blink
            * led_animation_spin
        red : byte
            Brightness level of the red channel.
        green : byte
            Brightness level of the green channel.
        blue : byte
            Brightness level of the blue channel.
        """
        payload = bytes([self._bound(state, 0, 3), red, green, blue])
        self._tx_q.put((Packet(3, 2, 0, payload=payload), False))

    def get_color_sensor_data(self, bank, lighting, fmt):
        """Request raw color sensor data.

        Parameters
        ----------
        bank : byte
            Which (of four) banks from which to get data.
        lighting : byte
            Which (of five) styles to illuminate the sensor.
        fmt : bye
            Which (of two) formats to receive the data.
        """
        bank = self._bound(bank, 0, 3)
        lighting = self._bound(lighting, 0, 4)
        fmt = self._bound(fmt, 0, 1)
        payload = bytes([bank, lighting, fmt])
        self._tx_q.put((Packet(4, 1, 0, payload=payload), True))

    def play_note(self, frequency, duration):
        """Play a frequency using the buzzer.

        Parameters
        ----------
        frequency : int
            Frequency of square wave to play, in units of Hertz
        duration : int
            Duration to play, in units of milliseconds
        """
        payload = pack('>IH', frequency, duration)
        self._tx_q.put((Packet(5, 0, 0, payload=payload), True))

    def stop_note(self):
        """Stop playing sound through the buzzer immediately."""
        self._tx_q.put((Packet(5, 1, 0), False))

    def say_phrase(self, phrase):
        """Speak a phrase in Root's language.

        Parameters
        ----------
        phrase : str
            Phase to "speak."

        Returns
        -------

        utf_phrase: bytes
        Actual phrase may be truncated to fit in packet payload.
        Returns None if phrase cannot be converted.
        """
        # TODO: loop over a longer string with multiple packets and responses
        try:
            utf_phrase = phrase.encode('utf-8')
            while len(utf_phrase) > Packet.PAYLOAD_LEN:
                phrase = phrase[:-1]
                utf_phrase = phrase.encode('utf-8')
        except AttributeError:
            return None
        self._tx_q.put((Packet(5, 4, 0, payload=utf_phrase), True))

    def get_battery_level(self):
        """Request the current battery level."""
        self._tx_q.put((Packet(14, 1, 0), True))

    @staticmethod
    def _bound(value, low, high):
        """Helper function to keep numbers in bounds.

        Parameter
        ---------
        value : number
            Value to keep in bounds.
        low : number
            Minimum of bounds check.
        high : number
            Maximum of bounds check.

        Returns
        -------
        new_value
            The original value, guaranteed between low and high, inclusive
        """
        return min(high, max(low, value))

    @staticmethod
    def _calculate_timeout(packet):
        """Helper function to calculate a timeout for packets expecting a response.

        Parameter
        ---------
        packet : Packet
            Packet for which to calculate a timeout

        Returns
        -------
        timeout : float
            Number of seconds to wait for message receipt.
        """
        timeout = 4  # minimum to wait
        cmd_type = (packet.dev, packet.cmd)
        if cmd_type == (1, 8):  # drive distance
            distance = unpack('>i', packet.payload[0:4])
            timeout += 1 + abs(*distance) / 10  # mm/s, drive speed
        elif cmd_type == (1, 12):  # rotate angle
            angle = unpack('>i', packet.payload[0:4])
            timeout += 1 + abs(*angle) / 1000  # decideg/s
        elif cmd_type == (2, 0):  # set marker/eraser position
            timeout += 1
        elif cmd_type == (5, 0):  # play note finished
            duration = unpack('>H', packet.payload[4:6])
            timeout += duration / 1000  # ms/s
        elif cmd_type == (5, 1):  # say phrase finished
            timeout += 16  # need to figure out how to calculate this
        return timeout

    def _responses_pending(self):
        """Helper function to determine whether any response packets are pending.

        Returns
        -------
        value : bool
            True if any response packets are pending.
        """
        self.pending_lock.acquire()
        pending_resp_len = len(self.pending_resp)
        self.pending_lock.release()
        return pending_resp_len > 0

    def transmissions_pending(self):
        """Helper function to determine whether any transmissions are
           waiting to be sent.

        Returns
        -------
        value : bool
            True if any transmissions are pending.
        """
        return not self._tx_q.empty()

    def _sending_thread(self):
        """Manages the sending of packets to the robot.

        Sends messages in order from the tx queue, waiting if any
        messages have responses pending.

        If sniff mode is on, will print packet to standard out as it's sent.
        """
        inc = 0
        while self._phy.is_connected():

            # block sending new commands until no responses pending
            if self._responses_pending():
                continue

            if not self._tx_q.empty():
                packet, expectResponse = self._tx_q.get()
                packet.inc = inc

                if expectResponse:
                    self.pending_lock.acquire()
                    # need a timeout because responses are not guaranteed.
                    resp_expire = time.time() + self._calculate_timeout(packet)
                    self.pending_resp.append(
                        (packet, resp_expire))
                    self.pending_lock.release()

                self._phy.send_raw(packet.bytes)
                if self.sniff_mode:
                    print('>>>', list(packet.bytes))

                inc += 1
                if inc > 255:
                    inc = 0

    def _expiration_thread(self):
        """Manages the expiration of packets in the receiving queue."""
        def tfilter(x, t):
            if t < x[1]:
                return True
            print("Warning: message with header {} expired!".format([x[0].dev, x[0].cmd, x[0].inc]))
            return False

        while self._phy.is_connected():
            time.sleep(0.5)
            self.pending_lock.acquire()
            now = time.time()
            self.pending_resp = [x for x in self.pending_resp if tfilter(x, now)]
            self.pending_lock.release()

    supported_devices = { 0: 'General',
                          1: 'Motors',
                          2: 'MarkEraser',
                          4: 'Color',
                         12: 'Bumper',
                         13: 'Light',
                         14: 'Battery',
                         17: 'Touch',
                         20: 'Cliff'}

    virtual_devices = {   4: 'ColorRaw'}

    event_messages = ( ( 0,  4),
                       ( 1, 29),
                       ( 4,  2),
                       (12,  0),
                       (13,  0),
                       (14,  0),
                       (17,  0),
                       (20,  0) )

    resp_msg_acked = ( (1,  8),
                       (1, 12),
                       (1, 27),
                       (5,  0),
                       (5,  4) )

    def _receiving_thread(self):
        """Manages the receipt of packets from the robot.

        Interprets packets recieved in the rx queue in order and
        acts upon them, if necessary.

        If sniff mode is set, will print packets to standard out as
        they are received.
        """
        last_event = 255
        while self._phy.is_connected():
            if self._rx_q is not None and not self._rx_q.empty():
                packet = Packet.from_bytes(self._rx_q.get())
                state   = packet.payload[4]
                crc_fail = not packet.check_crc()

                event_fail = None
                if (packet.dev, packet.cmd) in self.event_messages:
                    event_fail = (packet.inc - last_event) & 0xFF != 1
                    last_event = packet.inc

                if self.sniff_mode:
                    print('C' if crc_fail else ' ', 'E' if event_fail else ' ', list(packet.bytes) )

                if crc_fail and not self.ignore_crc_errors:
                    continue

                dev_name = self.supported_devices[packet.dev] if packet.dev in self.supported_devices else None

                if (packet.dev, packet.cmd) in self.event_messages:

                    if dev_name == 'General' and packet.cmd == 4:  # stop project
                        print('Warning: Stop Project!')
                        self.stop_project_flag.set()
                        # purge all pending transmissions
                        while not self._tx_q.empty():
                            packet, expectResponse = self._tx_q.get()
                        # stop waiting for any responses
                            self.pending_lock.acquire()
                            self.pending_resp.clear()
                            self.pending_lock.release()
                    elif dev_name == 'Motors' and packet.cmd == 29:  # motor stall
                        m = ['left', 'right', 'markeraser']
                        c = ['none', 'overcurrent', 'undercurrent', 'underspeed', 'saturated', 'timeout']
                        print("Stall: {} motor {}.".format(m[state], c[packet.payload[5]]))

                    elif dev_name == 'Color' and packet.cmd == 2:
                        if self.state[dev_name] is None:
                            self.state[dev_name] = [0]*32
                        i = 0
                        for byte in packet.payload:
                            self.state[dev_name][i*2+0] = (byte & 0xF0) >> 4
                            self.state[dev_name][i*2+1] = byte & 0x0F
                            i += 1

                    elif dev_name == 'Bumper' and packet.cmd == 0:
                        if state == 0:
                            self.state[dev_name] = (False, False)
                        elif state == 0x40:
                            self.state[dev_name] = (False, True)
                        elif state == 0x80:
                            self.state[dev_name] = (True, False)
                        elif state == 0xC0:
                            self.state[dev_name] = (True, True)
                        else:
                            self.state[dev_name] = packet.payload

                    elif dev_name == 'Light' and packet.cmd == 0:
                        if state == 4:
                            self.state[dev_name] = (False, False)
                        elif state == 5:
                            self.state[dev_name] = (False, True)
                        elif state == 6:
                            self.state[dev_name] = (True, False)
                        elif state == 7:
                            self.state[dev_name] = (True, True)
                        else:
                            self.state[dev_name] = packet.payload

                    elif dev_name == 'Battery' and packet.cmd == 0:
                        self.state[dev_name] = packet.payload[6]

                    elif dev_name == 'Touch' and packet.cmd == 0:
                        if self.state[dev_name] is None:
                            self.state[dev_name] = {}
                        self.state[dev_name]['FL'] = state & 0x80 == 0x80
                        self.state[dev_name]['FR'] = state & 0x40 == 0x40
                        self.state[dev_name]['RR'] = state & 0x20 == 0x20
                        self.state[dev_name]['RL'] = state & 0x10 == 0x10

                    elif dev_name == 'Cliff' and packet.cmd == 0:
                        self.state[dev_name] = state == 1

                    else:
                        self.state[dev_name] = packet.bytes
                        print('Unhandled event message from ' + dev_name)
                        print(list(packet.bytes))
                else:  # response message
                    orig_packet = None
                    
                    self.pending_lock.acquire()

                    # see if (dev, cmd, inc, _) exists in pending_resp
                    result = [
                        resp for resp in self.pending_resp
                        if (resp[0].dev == packet.dev
                        and resp[0].cmd == packet.cmd
                        and resp[0].inc == packet.inc)
                    ]
                    #print(result)

                    if len(result) != 1:
                        print('Warning: unexpected response for message',
                              packet.dev, packet.cmd, packet.inc)
                    else:
                        #print ('got resp for', result[0][0].dev, result[0][0].cmd, result[0][0].inc)
                        orig_packet = result[0][0]
                        self.pending_resp.remove(result[0])

                    self.pending_lock.release()

                    if (packet.dev, packet.cmd) in self.resp_msg_acked:
                        pass  # no side effects
                    elif dev_name == 'General':
                        if self.state[dev_name] is None:
                            self.state[dev_name] = {}
                        if packet.cmd == 0:  # get versions
                            self.state[dev_name][packet.payload[0]] = packet.payload[1] + packet.payload[2] / 1000
                        elif packet.cmd == 2:  # get name
                            self.state[dev_name]['Name'] = packet.payload.decode('utf-8').rstrip('\0')
                        elif packet.cmd == 11:  # get enabled events
                            self.state[dev_name]['EnabledEvents'] = packet.payload
                        elif packet.cmd == 14:  # get serial number
                            self.state[dev_name]['Serial'] = packet.payload.decode('utf-8').rstrip('\0')
                    elif dev_name == 'MarkEraser' and packet.cmd == 0:  # set marker/eraser position
                        pos = packet.payload[0]
                        if pos == 0:
                            self.state[dev_name] = 'marker_up_eraser_up'
                        elif pos == 1:
                            self.state[dev_name] = 'marker_down_eraser_up'
                        elif pos == 2:
                            self.state[dev_name] = 'marker_up_eraser_down'
                        else:
                            self.state[dev_name] = pos # undefined
                    elif dev_name == 'Color' and packet.cmd == 1 and orig_packet is not None:
                        if self.state['ColorRaw'] is None:
                            self.state['ColorRaw'] = [ [None]*32 for _ in range(5) ]
                        offset = orig_packet.payload[0] * 8
                        for i in range(8):
                            self.state['ColorRaw'][orig_packet.payload[1]][offset + i] = \
                                packet.payload[i*2]*256 + packet.payload[i*2+1]
                    elif dev_name == 'Battery' and packet.cmd == 1:  # get battery level
                        self.state[dev_name] = packet.payload[6]
                    else:
                        print('Unsupported message ', list(packet.bytes))

    def get_sniff_mode(self):
        """Helper function to determine whether we are in sniff mode."""
        return self.sniff_mode

    def set_sniff_mode(self, mode):
        """Helper function to set sniff mode on or off.

        Parameters
        ----------
        mode : bool
            True to turn sniff mode on; False to turn it off.
        """
        self.sniff_mode = bool(mode)
