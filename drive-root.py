#!/usr/bin/env python3

import gatt
import threading


# BLE UUID's
root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'
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

class BluetoothDeviceManager(gatt.DeviceManager):
    robot = None # root robot device

    def device_discovered(self, device):
        print("[%s] Discovered: %s" % (device.mac_address, device.alias()))
        self.stop_discovery() # Stop searching
        self.robot = RootDevice(mac_address=device.mac_address, manager=self)
        self.robot.connect()

class RootDevice(gatt.Device):
    def connect_succeeded(self):
        super().connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        print("[%s] Disconnected" % (self.mac_address))

    def services_resolved(self):
        super().services_resolved()
        print("[%s] Resolved services" % (self.mac_address))

        self.uart_service = next(
            s for s in self.services
            if s.uuid == uart_service_uuid)

        self.tx_characteristic = next(
            c for c in self.uart_service.characteristics
            if c.uuid == tx_characteristic_uuid)

        self.rx_characteristic = next(
            c for c in self.uart_service.characteristics
            if c.uuid == rx_characteristic_uuid)

        self.rx_characteristic.enable_notifications() # listen to RX messages

    def characteristic_value_updated(self, characteristic, value):
        message = []
        type = ""
        for byte in value:
            message.append(byte)
#        print ("Messages from Root:")
        device = message[0]
        command = message[1]
        ident = message[2]
        state = message[7]

        if device in supported_dev_msg:
            dev_name = supported_dev_msg[device]

            if dev_name == 'Color' and command == 2:
                if sensor[dev_name] is None:
                    sensor[dev_name] = [0]*32
                i = 0
                for byte in message[3:19]:
                    sensor[dev_name][i*2+0] = (byte & 0xF0) >> 4
                    sensor[dev_name][i*2+1] = byte & 0x0F
                    i += 1

            elif dev_name == 'Bumper' and command == 0:
                if state == 0:
                    sensor[dev_name] = (False, False)
                elif state == 0x40:
                    sensor[dev_name] = (False, True)
                elif state == 0x80:
                    sensor[dev_name] = (True, False)
                elif state == 0xC0:
                    sensor[dev_name] = (True, True)
                else:
                    sensor[dev_name] = message

            elif dev_name == 'Light' and command == 0:
                if state == 4:
                    sensor[dev_name] = (False, False)
                elif state == 5:
                    sensor[dev_name] = (False, True)
                elif state == 6:
                    sensor[dev_name] = (True, False)
                elif state == 7:
                    sensor[dev_name] = (True, True)
                else:
                    sensor[dev_name] = message

            elif dev_name == 'Battery' and command == 0:
                sensor[dev_name] = message[9]

            elif dev_name == 'Touch' and command == 0:
                if sensor[dev_name] is None:
                    sensor[dev_name] = {}
                sensor[dev_name]['FL'] = True if state & 0x80 == 0x80 else False
                sensor[dev_name]['FR'] = True if state & 0x40 == 0x40 else False
                sensor[dev_name]['RR'] = True if state & 0x20 == 0x20 else False
                sensor[dev_name]['RL'] = True if state & 0x10 == 0x10 else False

            elif dev_name == 'Cliff' and command == 0:
                sensor[dev_name] = True if state == 1 else False
            else:
                sensor[dev_name] = message
                print('Unhandled message from ' + dev_name)
                print(message)
        else:
            print('Unsupported device ' + str(device))
            print(message)

    def drive_forward(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD1])

    def drive_left(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8A])

    def drive_right(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x25])

    def stop(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E])

    def drive_backwards(self):
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, 0xFF, 0xFF, 0xFF, 0x9C, 0xFF, 0xFF, 0xFF, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x71])

    def pen_up(self):
        self.tx_characteristic.write_value([0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def pen_down(self):
        self.tx_characteristic.write_value([0x02, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

    def turn_rate(self, rate):
        left = 0
        right = 0
        if rate >= 0:
            left = rate
        if rate < 0:
            right = -1*rate
        leftbytes = left.to_bytes(4,byteorder='big',signed=True)  # need to convert to byte string
        rightbytes = right.to_bytes(4,byteorder='big',signed=True)
        # note that we're not dynamically calculating the CRC at the end, so just leaving it 0 (unchecked)
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, leftbytes[0], leftbytes[1], leftbytes[2], leftbytes[3], rightbytes[0], rightbytes[1], rightbytes[2], rightbytes[3], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0])

    def steer(self, left, right):
        leftbytes = left.to_bytes(4,byteorder='big',signed=True)  # need to convert to byte string
        rightbytes = right.to_bytes(4,byteorder='big',signed=True)
        # note that we're not dynamically calculating the CRC at the end, so just leaving it 0 (unchecked)
        self.tx_characteristic.write_value([0x01, 0x04, 0x00, leftbytes[0], leftbytes[1], leftbytes[2], leftbytes[3], rightbytes[0], rightbytes[1], rightbytes[2], rightbytes[3], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0])


def drive_root(command):
    angle = 0
    if command == "f":
        print ("Drive forward")
        manager.robot.drive_forward()
    if command == "b":
        print ("Drive backwards")
        manager.robot.drive_backwards()
    if command == "r":
        print ("Drive right")
        manager.robot.drive_right()
    if command == "l":
        print ("Drive left")
        manager.robot.drive_left()
    if command == "s":
        print ("Stop")
        manager.robot.stop()
    if command == "u":
        print ("Pen up")
        manager.robot.pen_up()
    if command == "d":
        print ("Pen down")
        manager.robot.pen_down()
    if command == "t":
        print ("Enter turn rate (up to +-90):")
        char = input()
        rate = int(char)
        print ("Turning ", rate)
        manager.robot.turn_rate(rate)
    if command == "p":
        print ("Steer")
        manager.robot.steer(30,40)
    if command == "z":
        for s, v in sensor.items():
            print(s, v)

# start here if run as program / not imported as module
if __name__ == "__main__":
    manager = BluetoothDeviceManager(adapter_name = 'hci0')
    manager.start_discovery(service_uuids=[root_identifier_uuid])
    thread = threading.Thread(target = manager.run)
    thread.start()
    char = ""
    try:
        while manager.robot is None:
            pass # wait for a root robot to be discovered
        print("Press letter (f,b,l,r) to drive robot (t) to turn, (s) to stop, (u or d) raise pen up or down, (z) to get sensor states, (q) to quit")
        while char != "q":
            char = input() # wait for keyboard input
            drive_root(char)
    except KeyboardInterrupt:
        pass

    print("Quitting")
    manager.stop()
    manager.robot.disconnect()
    thread.join()