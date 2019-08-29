#!/usr/bin/env python3

import gatt
import threading
import crc8
import time
import struct
import queue
import threading
import numpy

class Root(object):
    ble_manager = None
    ble_thread = None
    root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'

    tx_q = queue.SimpleQueue()
    rx_q = None # set up in RootDevice class

    pending_lock = threading.Lock()
    pending_resp = []

    sniff_mode = False
    ignore_crc_errors = False
    
    stop_project_flag = threading.Event() # signals that Stop Project message was received

    def __init__(self):
        self.ble_manager = BluetoothDeviceManager(adapter_name = 'hci0')
        self.ble_manager.start_discovery(service_uuids=[self.root_identifier_uuid])
        self.ble_thread = threading.Thread(target = self.ble_manager.run)
        self.ble_thread.start()

    def wait_for_connect(self):
        while self.ble_manager.robot is None:
            time.sleep(0.1) # wait for a root robot to be discovered
        while not self.ble_manager.robot.service_resolution_complete:
            time.sleep(0.1) # allow services to resolve before continuing

        self.rx_q = self.ble_manager.robot.rx_q

        threading.Thread(target = self.sending_thread).start()
        threading.Thread(target = self.receiving_thread).start()
        threading.Thread(target = self.expiration_thread).start()

        self.initialize_state()

    def is_running(self):
        return self.ble_thread.is_alive()

    def disconnect(self):
        command = struct.pack('>BBBqq', 0, 6, 0, 0, 0)
        self.tx_q.put((command, False))
        self.ble_manager.stop()
        self.ble_manager.robot.disconnect()
        self.ble_thread.join()

    # Robot States
    state = {}

    def initialize_state(self):
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
        command = struct.pack('>BBBBbhiq', 0, 0, 0, board, 0, 0, 0, 0)
        self.tx_q.put((command, True))

    def get_name(self):
        command = struct.pack('>BBBqq', 0, 2, 0, 0, 0)
        self.tx_q.put((command, True))

    def enable_events(self): #TODO: Make smarter
        command = struct.pack('>BBBQQ', 0, 7, 0, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF)
        self.tx_q.put((command, False))

    def disable_events(self): #TODO: Make smarter
        command = struct.pack('>BBBQQ', 0, 8, 0, 0xFFFFFFFFFFFFFFFF, 0xFFFFFFFFFFFFFFFF)
        self.tx_q.put((command, False))

    def get_serial_number(self):
        command = struct.pack('>BBBqq', 0, 14, 0, 0, 0)
        self.tx_q.put((command, True))

    def set_motor_speeds(self, left, right):
        left = self.bound(left, -100, 100)
        right = self.bound(right, -100, 100)
        command = struct.pack('>BBBiiq', 1, 4, 0, left, right, 0)
        self.tx_q.put((command, False))

    def set_left_motor_speed(self, left):
        left = self.bound(left, -100, 100)
        command = struct.pack('>BBBiiq', 1, 6, 0, left, 0, 0)
        self.tx_q.put((command, False))

    def set_right_motor_speed(self, right):
        right = self.bound(right, -100, 100)
        command = struct.pack('>BBBiiq', 1, 7, 0, right, 0, 0)
        self.tx_q.put((command, False))

    def drive_distance(self, distance):
        command = struct.pack('>BBBiiq', 1, 8, 0, distance, 0, 0)
        self.tx_q.put((command, True))

    def rotate_angle(self, angle):
        command = struct.pack('>BBBiiq', 1, 12, 0, angle, 0, 0)
        self.tx_q.put((command, True))

    def drive_arc(self, angle, radius):
        command = struct.pack('>BBBiiq', 1, 27, 0, angle, radius, 0)
        self.tx_q.put((command, True))

    last_coord = (0+0j)
    last_theta_x10 = 900
    def drive_complex(self, coord):
        vector    = (coord - self.last_coord)
        dist      = numpy.linalg.norm(vector)
        theta     = numpy.angle(vector, deg=True)
        theta_x10 = int(theta * 10)
        turn      = ((self.last_theta_x10 - theta_x10 + 1800) % 3600) - 1800
        dist      = int(dist)

        #print(self.last_theta_x10, '->', theta_x10, ':', turn)
        #print('turn', turn/10, ' drive', dist)
        self.rotate_angle(turn)
        self.drive_distance(dist)

        self.last_coord = (numpy.real(self.last_coord) + dist * numpy.cos(theta_x10/10*numpy.pi/180)) + \
                          (numpy.imag(self.last_coord) + dist * numpy.sin(theta_x10/10*numpy.pi/180))*1j
        self.last_theta_x10 = theta_x10

    marker_up_eraser_up = 0
    marker_down_eraser_up = 1
    marker_up_eraser_down = 2

    def set_marker_eraser_pos(self, pos):
        pos = self.bound(pos, 0, 2)
        #print('Set pen', pos)
        command = struct.pack('>BBBbbhiq', 2, 0, 0, pos, 0, 0, 0, 0)
        self.tx_q.put((command, True))

    led_animation_off = 0
    led_animation_on = 1
    led_animation_blink = 2
    led_animation_spin = 3

    def set_led_animation(self, state, red, green, blue):
        state = self.bound(state, 0, 3)
        command = struct.pack('>BBBbBBBiq', 3, 2, 0, state, red, green, blue, 0, 0)
        self.tx_q.put((command, False))

    def get_color_sensor_data(self, bank, lighting, fmt):
        bank = self.bound(bank, 0, 3)
        lighting = self.bound(lighting, 0, 4)
        fmt = self.bound(fmt, 0, 1)
        command = struct.pack('>BBBbbbBiq', 4, 1, 0, bank, lighting, fmt, 0, 0, 0)
        self.tx_q.put((command, True))

    def play_note(self, frequency, duration):
        command = struct.pack('>BBBIHhq', 5, 0, 0, frequency, duration, 0, 0)
        self.tx_q.put((command, True))

    def stop_note(self):
        command = struct.pack('>BBBqq', 5, 1, 0, 0, 0)
        self.tx_q.put((command, False))

    def say_phrase(self, phrase):
        phrase = phrase.encode('utf-8')[0:16]
        if len(phrase) < 16:
            phrase += bytes(16-len(phrase))
        command = struct.pack('>BBBs', 5, 4, 0, phrase)
        self.tx_q.put((command, True))

    def get_battery_level(self):
        command = struct.pack('>BBBqq', 14, 1, 0, 0, 0)
        self.tx_q.put((command, True))

    def bound(self, value, low, high):
        return min(high, max(low, value))

    def calculate_timeout(self, message):
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

    def responses_pending(self):
        self.pending_lock.acquire()
        pending_resp_len = len(self.pending_resp)
        self.pending_lock.release()
        return True if pending_resp_len > 0 else False

    def sending_thread(self):
        inc = 0
        while self.ble_thread.is_alive():

            # block sending new commands until no responses pending
            if self.responses_pending():
                continue

            if not self.tx_q.empty():
                packet, expectResponse = self.tx_q.get()
                packet = bytearray(packet)
                packet[2] = inc

                if expectResponse:
                    self.pending_lock.acquire()
                    # need a timeout because responses are not guaranteed.
                    resp_expire = time.time() + self.calculate_timeout(packet)
                    self.pending_resp.append((packet[0:3], resp_expire))
                    self.pending_lock.release()
                
                self.send_raw_ble(packet + crc8.crc8(packet).digest())
                inc += 1
                if inc > 255:
                    inc = 0

    def send_raw_ble(self, packet):
        if len(packet) == 20:
            self.ble_manager.robot.tx_characteristic.write_value(packet)
        else:
            print('Error: send_raw_ble: Packet wrong length.')
        if self.sniff_mode:
            print('>>>', list(packet))

    def expiration_thread(self):
        while self.ble_thread.is_alive():
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

    def receiving_thread(self):
        last_event = 255
        while self.ble_thread.is_alive():
            if self.rx_q is not None and not self.rx_q.empty():
                message = self.rx_q.get()

                device  = message[0]
                command = message[1]
                id      = message[2]
                state   = message[7]
                crc     = message[19]

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
                        while not self.tx_q.empty():
                            packet, expectResponse = self.tx_q.get()
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
                            self.state[dev_name]['Name'] = str(message[3:19])
                        elif command == 14: # get serial number
                            self.state[dev_name]['Serial'] = str(message[3:19])
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
        return self.sniff_mode

    def set_sniff_mode(self, mode):
        self.sniff_mode = True if mode else False

class BluetoothDeviceManager(gatt.DeviceManager):
    robot = None # root robot device

    def device_discovered(self, device):
        print("[%s] Discovered: %s" % (device.mac_address, device.alias()))
        self.stop_discovery() # Stop searching
        self.robot = RootDevice(mac_address=device.mac_address, manager=self)
        self.robot.connect()

class RootDevice(gatt.Device):
    uart_service_uuid = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
    tx_characteristic_uuid = '6e400002-b5a3-f393-e0a9-e50e24dcca9e' # Write
    rx_characteristic_uuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e' # Notify
    rx_q = queue.SimpleQueue()

    service_resolution_complete = False

    def connect_succeeded(self):
        super().connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        self.service_resolution_complete = False
        print("[%s] Disconnected" % (self.mac_address))

    def services_resolved(self):
        super().services_resolved()
        print("[%s] Resolved services" % (self.mac_address))

        self.uart_service = next(
            s for s in self.services
            if s.uuid == self.uart_service_uuid)

        self.tx_characteristic = next(
            c for c in self.uart_service.characteristics
            if c.uuid == self.tx_characteristic_uuid)

        self.rx_characteristic = next(
            c for c in self.uart_service.characteristics
            if c.uuid == self.rx_characteristic_uuid)

        self.rx_characteristic.enable_notifications() # listen to RX messages
        self.service_resolution_complete = True

    def characteristic_value_updated(self, characteristic, value):
        self.rx_q.put(value)

# starting a simulator
class Turtle(Root):
    stop_project_flag = threading.Event() # signals that Stop Project message was received
    robot = None

    def __init__(self):
        import turtle
        self.robot = turtle.Turtle()
        self.robot.setheading(90)
        self.robot.penup()

    def wait_for_connect(self):
        pass

    def is_running(self):
        return not self.stop_project_flag()

    def disconnect(self):
        self.stop_project_flag.set()

    def drive_distance(self, distance):
        self.robot.forward(distance)

    def rotate_angle(self, angle):
        self.robot.right(angle/10)

    def set_marker_eraser_pos(self, pos):
        if pos == self.marker_up_eraser_up:
            self.robot.penup()
        else:
            self.robot.pendown()
