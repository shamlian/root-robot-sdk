#!/usr/bin/env python3

import gatt
import threading
import crc8
import time
import struct
import queue
import threading

class Root(object):
    ble_manager = None
    ble_thread = None
    root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'
    sensor = None

    tx_q = queue.SimpleQueue()
    rx_q = queue.SimpleQueue()

    pending_lock = threading.Lock()
    pending_resp = []

    def __init__(self):
        self.ble_manager = BluetoothDeviceManager(adapter_name = 'hci0')
        self.ble_manager.start_discovery(service_uuids=[self.root_identifier_uuid])
        self.ble_thread = threading.Thread(target = self.ble_manager.run)
        self.ble_thread.start()

        threading.Thread(target = self.sending_thread).start()

    def wait_for_connect(self):
        while self.ble_manager.robot is None:
            time.sleep(0.1) # wait for a root robot to be discovered
        while not self.ble_manager.robot.service_resolution_complete:
            time.sleep(0.1) # allow services to resolve before continuing
        self.sensor = self.ble_manager.robot.sensor

    def is_running(self):
        return self.ble_thread.is_alive()

    def disconnect(self):
        self.ble_manager.stop()
        self.ble_manager.robot.disconnect()
        self.ble_thread.join()

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

    #TODO: Use enums here and elsewhere
    marker_up_eraser_up = 0
    marker_down_eraser_up = 1
    marker_up_eraser_down = 2

    def set_marker_eraser_pos(self, pos):
        pos = self.bound(pos, 0, 2)
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

    def sending_thread(self):
        inc = 0
        while self.ble_thread.is_alive():
            if not self.tx_q.empty():
                packet, expectResponse = self.tx_q.get()
                print(expectResponse)
                if expectResponse:
                    self.pending_lock.acquire()
                    pending_resp.append(packet)
                    self.pending_lock.release()
                
                packet = bytearray(packet)
                packet[2] = inc
                print(packet)
                self.send_raw_ble(packet + crc8.crc8(packet).digest())
                inc += 1

    def send_raw_ble(self, packet):
        if len(packet) == 20:
            self.ble_manager.robot.tx_characteristic.write_value(packet)
        else:
            print('send_raw_ble: Packet wrong length.')

    def get_sniff_mode(self):
        return self.ble_manager.robot.sniff_mode

    def set_sniff_mode(self, mode):
        self.ble_manager.robot.sniff_mode = True if mode else False

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

    supported_dev_msg = { 0: 'General',
                          1: 'Motors',
                          2: 'MarkEraser',
                          4: 'Color',
                          12: 'Bumper',
                          13: 'Light',
                          14: 'Battery',
                          17: 'Touch',
                          20: 'Cliff'}
    # Sensor States
    sensor = {'Color':   None,
              'Bumper':  None,
              'Light':   None,
              'Battery': None,
              'Touch':   None,
              'Cliff':   None}

    sniff_mode = False
    ignore_crc_errors = False
    service_resolution_complete = False

    last_event = 255
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
        message = value

        device  = message[0]
        command = message[1]
        event   = message[2]
        state   = message[7]
        crc     = message[19]

        crc_fail = True if crc8.crc8(value).digest() != b'\x00' else False
        event_fail = True if (event - self.last_event) & 0xFF != 1 else False
        if self.sniff_mode:
            print('C' if crc_fail else ' ', 'E' if event_fail else ' ', list(message) )

        self.last_event = event

        if crc_fail and not self.ignore_crc_errors:
            return

        if device in self.supported_dev_msg:
            dev_name = self.supported_dev_msg[device]

            if dev_name == 'Motors' and command == 29:
                m = ['left', 'right', 'markeraser']
                c = ['none', 'overcurrent', 'undercurrent', 'underspeed', 'saturated', 'timeout']
                print("Stall: {} motor {}.".format(m[state], c[message[8]]))

            elif dev_name == 'Color' and command == 2:
                if self.sensor[dev_name] is None:
                    self.sensor[dev_name] = [0]*32
                i = 0
                for byte in message[3:19]:
                    self.sensor[dev_name][i*2+0] = (byte & 0xF0) >> 4
                    self.sensor[dev_name][i*2+1] = byte & 0x0F
                    i += 1

            elif dev_name == 'Bumper' and command == 0:
                if state == 0:
                    self.sensor[dev_name] = (False, False)
                elif state == 0x40:
                    self.sensor[dev_name] = (False, True)
                elif state == 0x80:
                    self.sensor[dev_name] = (True, False)
                elif state == 0xC0:
                    self.sensor[dev_name] = (True, True)
                else:
                    self.sensor[dev_name] = message

            elif dev_name == 'Light' and command == 0:
                if state == 4:
                    self.sensor[dev_name] = (False, False)
                elif state == 5:
                    self.sensor[dev_name] = (False, True)
                elif state == 6:
                    self.sensor[dev_name] = (True, False)
                elif state == 7:
                    self.sensor[dev_name] = (True, True)
                else:
                    self.sensor[dev_name] = message

            elif dev_name == 'Battery' and command == 0:
                self.sensor[dev_name] = message[9]

            elif dev_name == 'Touch' and command == 0:
                if self.sensor[dev_name] is None:
                    self.sensor[dev_name] = {}
                self.sensor[dev_name]['FL'] = True if state & 0x80 == 0x80 else False
                self.sensor[dev_name]['FR'] = True if state & 0x40 == 0x40 else False
                self.sensor[dev_name]['RR'] = True if state & 0x20 == 0x20 else False
                self.sensor[dev_name]['RL'] = True if state & 0x10 == 0x10 else False

            elif dev_name == 'Cliff' and command == 0:
                self.sensor[dev_name] = True if state == 1 else False
            else:
                self.sensor[dev_name] = message
                print('Unhandled message from ' + dev_name)
                print(list(message))
        else:
            print('Unsupported device ' + str(device))
            print(list(message))
