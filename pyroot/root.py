# External Dependencies
import threading
import crc8
import time
import struct
import queue
import numpy

# Internal Dependencies
from .ble_helpers import BluetoothDeviceManager, RootDevice

class Root(object):
    """Simplifies communication with a real Root robot.
    
    Unless otherwise indicated, all methods are non-blocking.
    Packets are sent to the robot and received in separate threads;
    replies are interpreted when received and responses are placed
    in internal class state variables.

    Full descriptions of Root BLE packets can be found at
    RootRobotics/root-robot-ble-protocol
    """

    root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'

    def __init__(self, name = None):
        """Sets up Bluetooth manager to look for robots.
        
        Parameters
        ----------
        name : str, optional
            Name of the robot to connect to; if no name supplied, will connect
            to the first robot it sees.
        """

        try:
            self._tx_q = queue.SimpleQueue()
        except AttributeError:
            self._tx_q = queue.Queue()
        self._rx_q = None # set up in RootDevice class

        self.pending_lock = threading.Lock()
        self.pending_resp = []
        """list: List of responses pending from the robot."""

        self.sniff_mode = False
        """bool: If True, shows the raw BLE transactions to and from the robot."""

        self.ignore_crc_errors = False
        """bool: If true, ignores CRC errors in packets from the robot."""
        
        self.stop_project_flag = threading.Event()
        """Event: signals that Stop Project message was received."""

        self.state = {}
        """dict: Contains local state of robot"""

        _last_coord = (0+0j)
        """complex: Contains last known coordinates of robot."""
        _last_theta_x10 = 900
        """int: Contains last known heading of robot."""

        self._ble_manager = BluetoothDeviceManager(adapter_name = 'hci0')
        self._ble_manager.desired_name = name
        self._ble_manager.start_discovery(service_uuids=[self.root_identifier_uuid])
        self._ble_thread = threading.Thread(target = self._ble_manager.run)
        self._ble_thread.start()

    def wait_for_connect(self, timeout = float('inf')):
        """Blocking function initializing robot connection.

        Connects to the first Root robot it sees, kicks off some threads
        used to manage the connection, and uses initialize_state() to
        populate some information about the robot into the class.

        Parameters
        ----------
        timeout : float, optional
            Time to wait for connection; if None, will wait forever. Will throw
            TimeoutError if timeout exceeded.
        """

        timeout += time.time()

        while self._ble_manager.robot is None and time.time() < timeout:
            time.sleep(0.1) # wait for a root robot to be discovered
        if self._ble_manager.robot is None:
            raise TimeoutError('Timed out waiting for ' + self._ble_manager.desired_name)

        while not self._ble_manager.robot.service_resolution_complete:
            time.sleep(0.1) # allow services to resolve before continuing

        self._rx_q = self._ble_manager.robot.rx_q

        threading.Thread(target = self._sending_thread).start()
        threading.Thread(target = self._receiving_thread).start()
        threading.Thread(target = self._expiration_thread).start()

        self.initialize_state()

    def is_running(self):
        """Utility function for determining state of bluetooth thread."""
        return self._ble_thread.is_alive()

    def disconnect(self):
        """Request disconnect from the robot and shut down connection."""
        command = struct.pack('>BBBqq', 0, 6, 0, 0, 0)
        self._tx_q.put((command, False))
        self._ble_manager.stop()
        self._ble_manager.robot.disconnect()
        self._ble_thread.join()

    def initialize_state(self):
        """Set up internal state dictionary.

        Since certain versions of the main board protocol don't support
        CRC properly, also request version information and set some
        internal flags so that warnings are thrown appropriately.
        """

        for devnum, device in self.supported_devices.items():
            self.state[device] = None

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

        command = struct.pack('>BBBBbhiq', 0, 0, 0, board, 0, 0, 0, 0)
        self._tx_q.put((command, True))

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
            while len(utf_name) > 16:
                name = name[:-1]
                utf_name = name.encode('utf-8')
        except AttributeError:
            return None

        if utf_name == b'':
            name = b'\x46\x4c\x45\x41'

        command = struct.pack('>BBB16s', 0, 1, 0, utf_name.ljust(16, b'\0'))
        self._tx_q.put((command, False))
        return utf_name

    def get_name(self):
        """Requests the robot's name."""
        command = struct.pack('>BBBqq', 0, 2, 0, 0, 0)
        self._tx_q.put((command, True))

    def stop_and_reset(self):
        """Requests robot stop and cancel all pending actions."""
        command = struct.pack('>BBBqq', 0, 3, 0, 0, 0)
        self._tx_q.put((command, False))

    def enable_events(self):
        """Currently enables all events from the robot.

        TODO: Request better documentation about the payload of
        this packet and implement it.
        """
        command = struct.pack('>BBBQQ', 0, 7, 0, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF)
        self._tx_q.put((command, False))

    def disable_events(self):
        """Currently disables all events from the robot.

        TODO: Request better documentation about the payload of
        this packet and implement it.
        """
        command = struct.pack('>BBBQQ', 0, 8, 0, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF)
        self._tx_q.put((command, False))

    def get_enabled_events(self):
        """Request bitfield of enabled devices."""
        command = struct.pack('>BBBqq', 0, 11, 0, 0, 0)
        self._tx_q.put((command, True))

    def get_serial_number(self):
        """Request robot serial number."""
        command = struct.pack('>BBBqq', 0, 14, 0, 0, 0)
        self._tx_q.put((command, True))

    def set_motor_speeds(self, left, right):
        """Set left and right motor linear velocities.

        Parameters
        ----------
        left : int
            Left motor speed in units of mm/s.
        right : int
            Right motor speed in units of mm/s.
        """
        left = self._bound(left, -100, 100)
        right = self._bound(right, -100, 100)
        command = struct.pack('>BBBiiq', 1, 4, 0, left, right, 0)
        self._tx_q.put((command, False))

    def set_left_motor_speed(self, left):
        """Set left motor linear velocity.

        Parameters
        ----------
        left : int
            Left motor speed in units of mm/s.
        """
        left = self._bound(left, -100, 100)
        command = struct.pack('>BBBiiq', 1, 6, 0, left, 0, 0)
        self._tx_q.put((command, False))

    def set_right_motor_speed(self, right):
        """Set right motor linear velocity.

        Parameters
        ----------
        right : int
            Right motor speed in units of mm/s.
        """
        right = self._bound(right, -100, 100)
        command = struct.pack('>BBBiiq', 1, 7, 0, right, 0, 0)
        self._tx_q.put((command, False))

    def drive_distance(self, distance):
        """Drive in a straight line for a certain distance.

        Parameters
        ----------
        distance : int
            Distance to travel in units of mm.
        """
        command = struct.pack('>BBBiiq', 1, 8, 0, distance, 0, 0)
        self._tx_q.put((command, True))

    def rotate_angle(self, angle):
        """Turn the robot in place by a particular angle.

        Parameters
        ----------
        angle : int
            Angle to turn in units of deci-degrees.
        """
        command = struct.pack('>BBBiiq', 1, 12, 0, angle, 0, 0)
        self._tx_q.put((command, True))

    def drive_arc(self, angle, radius):
        """Drive in an arc subtending a particular angle along a circle.

        Parameters
        ----------
        angle : int
            Angle of an arc to drive along, in deci-degrees.
        radius : int
            Radius of the circle upon which to travel.
        """
        command = struct.pack('>BBBiiq', 1, 27, 0, angle, radius, 0)
        self._tx_q.put((command, True))

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
        command = struct.pack('>BBBbbhiq', 2, 0, 0, pos, 0, 0, 0, 0)
        self._tx_q.put((command, True))

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
        state = self._bound(state, 0, 3)
        command = struct.pack('>BBBbBBBiq', 3, 2, 0, state, red, green, blue, 0, 0)
        self._tx_q.put((command, False))

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
        command = struct.pack('>BBBbbbBiq', 4, 1, 0, bank, lighting, fmt, 0, 0, 0)
        self._tx_q.put((command, True))

    def play_note(self, frequency, duration):
        """Play a frequency using the buzzer.

        Parameters
        ----------
        frequency : int
            Frequency of square wave to play, in units of Hertz
        duration : int
            Duration to play, in units of milliseconds
        """
        command = struct.pack('>BBBIHhq', 5, 0, 0, frequency, duration, 0, 0)
        self._tx_q.put((command, True))

    def stop_note(self):
        """Stop playing sound through the buzzer immediately."""
        command = struct.pack('>BBBqq', 5, 1, 0, 0, 0)
        self._tx_q.put((command, False))

    def say_phrase(self, phrase):
        """Speak a phrase in Root's language.

        Parameters
        ----------
        phrase : str
            Phase to "speak."
        """
        phrase = phrase.encode('utf-8')[0:16]
        if len(phrase) < 16:
            phrase += bytes(16-len(phrase))
        command = struct.pack('>BBBs', 5, 4, 0, phrase)
        self._tx_q.put((command, True))

    def get_battery_level(self):
        """Request the current battery level."""
        command = struct.pack('>BBBqq', 14, 1, 0, 0, 0)
        self._tx_q.put((command, True))

    def _bound(self, value, low, high):
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

    def _calculate_timeout(self, message):
        """Helper function to calculate a timeout for packets expecting a response.

        Parameter
        ---------
        message : bytes
            Message for which to calculate a timeout

        Returns
        -------
        timeout : float
            Number of seconds to wait for message receipt.
        """
        timeout = 1 # minimum to wait
        timeout += 3
        msg_type = message[0:2]
        if msg_type == bytes([1,8]): # drive distance
            distance = struct.unpack('>i', message[3:7])
            timeout += 1 + abs(*distance) / 10 # mm/s, drive speed
        elif msg_type == bytes([1,12]): # rotate angle
            angle = struct.unpack('>i', message[3:7])
            timeout += 1 + abs(*angle) / 1000 # decideg/s
        elif msg_type == bytes([2,0]): # set marker/eraser position
            timeout += 1
        elif msg_type == bytes([5,0]): # play note finished
            duration = struct.unpack('>H', message[7:9])
            timeout += duration / 1000 # ms/s
        elif msg_type == bytes([5,1]): # say phrase finished
            timeout += 16 # need to figure out how to calculate this
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
        return True if pending_resp_len > 0 else False

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
        """
        inc = 0
        while self._ble_thread.is_alive():

            # block sending new commands until no responses pending
            if self._responses_pending():
                continue

            if not self._tx_q.empty():
                packet, expectResponse = self._tx_q.get()
                packet = bytearray(packet)
                packet[2] = inc

                if expectResponse:
                    self.pending_lock.acquire()
                    # need a timeout because responses are not guaranteed.
                    resp_expire = time.time() + self._calculate_timeout(packet)
                    self.pending_resp.append((packet[0:3], resp_expire))
                    self.pending_lock.release()
                
                self._send_raw_ble(packet + crc8.crc8(packet).digest())
                inc += 1
                if inc > 255:
                    inc = 0

    def _send_raw_ble(self, packet):
        """Helper method to send raw BLE packets to the robot.

        If sniff mode is enabled, this function also prints the
        packet to stdout.

        Parameters
        ----------
        packet : bytes
            20-byte packet to send to the robot.
        """
        if len(packet) == 20:
            self._ble_manager.robot.tx_characteristic.write_value(packet)
        else:
            print('Error: send_raw_ble: Packet wrong length.')
        if self.sniff_mode:
            print('>>>', list(packet))

    def _expiration_thread(self):
        """Manages the expiration of packets in the receiving queue."""
        while self._ble_thread.is_alive():
            time.sleep(0.5)

            self.pending_lock.acquire()
            #TODO: Figure out a more pythonic way to do this
            now = time.time()

            for i in range(len(self.pending_resp)):
                if self.pending_resp[i][1] <= now:
                    print('Warning: message with header', list(self.pending_resp[i][0]), 'expired!')
                    self.pending_resp[i] = None
            while None in self.pending_resp:
                self.pending_resp.remove(None)

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

    event_messages = ( bytes([ 0,  4]),
                       bytes([ 1, 29]),
                       bytes([ 4,  2]),
                       bytes([12,  0]),
                       bytes([13,  0]),
                       bytes([14,  0]),
                       bytes([17,  0]),
                       bytes([20,  0]) )

    resp_msg_acked = ( bytes([1,  8]),
                       bytes([1, 12]),
                       bytes([1, 27]),
                       bytes([5,  0]),
                       bytes([5,  4]) )

    def _receiving_thread(self):
        """Manages the receipt of packets from the robot.

        Interprets messages recieved in the rx queue in order and
        acts upon them, if necessary.
        """
        last_event = 255
        while self._ble_thread.is_alive():
            if self._rx_q is not None and not self._rx_q.empty():
                message = self._rx_q.get()

                device  = message[0]
                command = message[1]
                id      = message[2]
                state   = message[7]

                crc_fail = True if crc8.crc8(message).digest() != b'\x00' else False

                event_fail = None
                if message[0:2] in self.event_messages:
                    event_fail = True if (id - last_event) & 0xFF != 1 else False
                    last_event = id

                if self.sniff_mode:
                    print('C' if crc_fail else ' ', 'E' if event_fail else ' ', list(message) )

                if crc_fail and not self.ignore_crc_errors:
                    continue

                dev_name = self.supported_devices[device] if device in self.supported_devices else None

                if message[0:2] in self.event_messages:

                    if dev_name == 'General' and command == 4: # stop project
                        print('Warning: Stop Project!')
                        self.stop_project_flag.set()
                        # purge all pending transmissions
                        while not self._tx_q.empty():
                            packet, expectResponse = self._tx_q.get()
                        # stop waiting for any responses
                            self.pending_lock.acquire()
                            self.pending_resp.clear()
                            self.pending_lock.release()
                    elif dev_name == 'Motors' and command == 29: # motor stall
                        m = ['left', 'right', 'markeraser']
                        c = ['none', 'overcurrent', 'undercurrent', 'underspeed', 'saturated', 'timeout']
                        print("Stall: {} motor {}.".format(m[state], c[message[8]]))

                    elif dev_name == 'Color' and command == 2:
                        if self.state[dev_name] is None:
                            self.state[dev_name] = [0]*32
                        i = 0
                        for byte in message[3:19]:
                            self.state[dev_name][i*2+0] = (byte & 0xF0) >> 4
                            self.state[dev_name][i*2+1] = byte & 0x0F
                            i += 1

                    elif dev_name == 'Bumper' and command == 0:
                        if state == 0:
                            self.state[dev_name] = (False, False)
                        elif state == 0x40:
                            self.state[dev_name] = (False, True)
                        elif state == 0x80:
                            self.state[dev_name] = (True, False)
                        elif state == 0xC0:
                            self.state[dev_name] = (True, True)
                        else:
                            self.state[dev_name] = message

                    elif dev_name == 'Light' and command == 0:
                        if state == 4:
                            self.state[dev_name] = (False, False)
                        elif state == 5:
                            self.state[dev_name] = (False, True)
                        elif state == 6:
                            self.state[dev_name] = (True, False)
                        elif state == 7:
                            self.state[dev_name] = (True, True)
                        else:
                            self.state[dev_name] = message

                    elif dev_name == 'Battery' and command == 0:
                        self.state[dev_name] = message[9]

                    elif dev_name == 'Touch' and command == 0:
                        if self.state[dev_name] is None:
                            self.state[dev_name] = {}
                        self.state[dev_name]['FL'] = True if state & 0x80 == 0x80 else False
                        self.state[dev_name]['FR'] = True if state & 0x40 == 0x40 else False
                        self.state[dev_name]['RR'] = True if state & 0x20 == 0x20 else False
                        self.state[dev_name]['RL'] = True if state & 0x10 == 0x10 else False

                    elif dev_name == 'Cliff' and command == 0:
                        self.state[dev_name] = True if state == 1 else False

                    else:
                        self.state[dev_name] = message
                        print('Unhandled event message from ' + dev_name)
                        print(list(message))
                else: # response message
                    self.pending_lock.acquire()
                    header = message[0:3]
                    #TODO: Figure out a more pythonic way to do this
                    try:
                        if header in list(zip(*self.pending_resp))[0]:
                            for i in range(len(self.pending_resp)):
                                if self.pending_resp[i][0] == header:
                                    break
                            del self.pending_resp[i]
                        else:
                            print('Warning: unexpected response for message', list(header))
                    except IndexError:
                        print('Warning: unexpected response with empty list for message', list(header))
                    self.pending_lock.release()

                    msg_type = message[0:2]
                    if msg_type in self.resp_msg_acked:
                        pass # no side effects
                    elif dev_name == 'General':
                        if self.state[dev_name] is None:
                            self.state[dev_name] = {}
                        if command == 0: # get versions
                            self.state[dev_name][message[3]] = message[4] + message[5]/1000
                        elif command == 2: # get name
                            self.state[dev_name]['Name'] = message[3:19].decode('utf-8').rstrip('\0')
                        elif command == 11: # get enabled events
                            self.state[dev_name]['EnabledEvents'] = message[3:19]
                        elif command == 14: # get serial number
                            self.state[dev_name]['Serial'] = message[3:19].decode('utf-8').rstrip('\0')
                    elif dev_name == 'MarkEraser' and command == 0: # set marker/eraser position
                        pos = message[3]
                        if pos == 0:
                            self.state[dev_name] = 'marker_up_eraser_up'
                        elif pos == 1:
                            self.state[dev_name] = 'marker_down_eraser_up'
                        elif pos == 2:
                            self.state[dev_name] = 'marker_up_eraser_down'
                        else:
                            self.state[dev_name] = pos # undefined
                    elif dev_name == 'Battery' and command == 1: # get battery level
                        self.state[dev_name] = message[9]
                    else:
                        print('Unsupported message ', list(message))

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
        self.sniff_mode = True if mode else False

