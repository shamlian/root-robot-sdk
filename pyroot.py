#!/usr/bin/env python3

import gatt
import threading
import crc8
import time

class Root(object):
    ble_manager = None
    ble_thread = None
    root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'
    sensor = None

    def __init__(self):
        self.ble_manager = self.BluetoothDeviceManager(adapter_name = 'hci0')
        self.ble_manager.start_discovery(service_uuids=[self.root_identifier_uuid])
        self.ble_thread = threading.Thread(target = self.ble_manager.run)
        self.ble_thread.start()

    def wait_for_connect(self):
        while self.ble_manager.robot is None:
            time.sleep(0.1) # wait for a root robot to be discovered
        while not self.ble_manager.robot.service_resolution_complete:
            time.sleep(0.1) # allow services to resolve before continuing
        self.sensor = self.ble_manager.robot.sensor

    def is_running(self):
        return self.ble_thread.is_alive()

    def stop(self):
        self.ble_manager.stop()
        self.ble_manager.robot.disconnect()
        self.ble_thread.join()

    def send_raw_ble(self, packet):
        self.ble_manager.robot.tx_characteristic.write_value(packet)

    def get_sniff_mode(self):
        return self.ble_manager.robot.sniff_mode

    def set_sniff_mode(self, mode):
        self.ble_manager.robot.sniff_mode = True if mode else False

    class BluetoothDeviceManager(gatt.DeviceManager):
        robot = None # root robot device

        def device_discovered(self, device):
            print("[%s] Discovered: %s" % (device.mac_address, device.alias()))
            self.stop_discovery() # Stop searching
            self.robot = Root.RootDevice(mac_address=device.mac_address, manager=self)
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
            message = []
            for byte in value:
                message.append(byte)
            device  = message[0]
            command = message[1]
            event   = message[2]
            state   = message[7]
            crc     = message[19]

            crc_fail = True if crc8.crc8(value).digest() != b'\x00' else False
            event_fail = True if (event - self.last_event) & 0xFF != 1 else False
            if self.sniff_mode:
                print('C' if crc_fail else ' ', 'E' if event_fail else ' ', message)

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
                    print(message)
            else:
                print('Unsupported device ' + str(device))
                print(message)
