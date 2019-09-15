# External Dependencies
import pygatt
import queue
import threading
from uuid import UUID

class BluetoothDeviceManager(object):
    robot = None # root robot device
    desired_name = None
    _service_uuid = None
    _cap = 'connectable_advertisement_packet'
    _suid = 'incomplete_list_128-bit_service_class_uuids'


    def __init__(self, adapter_name = None):
        #TODO: If adapter_name specified, use it for the serial port
        self.adapter = pygatt.BGAPIBackend()
        self._running = threading.Event()

    def start_discovery(self, service_uuids):
        self._service_uuid = UUID(service_uuids[0]).bytes[::-1]
        # I honestly have no idea why I have to reverse the byte order.
        # I'd love to buy a beer for you if you can tell me why!

    def run(self):
        self._running.set()
        self.adapter.start()

        while self._running.is_set():
            self.adapter.scan(timeout = 1, scan_cb = self.device_discovered)

    def stop(self):
        try:
            self._running.clear()
            self.adapter.stop()
        except pygatt.exceptions.NotConnectedError:
            pass

    def device_discovered(self, devices, addr, packet_type):
        device = devices[addr]
        # print("**", device.name, device.address, device.rssi)
        if self._cap in device.packet_data:
            cap = device.packet_data[self._cap]
            if self._suid in cap:
                if cap[self._suid] == self._service_uuid:
                    print("[%s] Discovered: %s" % (device.address, device.name))
                    if self.desired_name == None:
                        self.desired_name = device.name

                    if self.desired_name == device.name:
                        self.adapter._evt.clear() # Stop searching
                        self.robot = RootDevice(mac_address=device.address, manager=self)
                        self.robot.connect()

class RootDevice(object): #gatt.Device
    uart_service_uuid = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
    tx_characteristic_uuid = '6e400002-b5a3-f393-e0a9-e50e24dcca9e' # Write
    rx_characteristic_uuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e' # Notify

    service_resolution_complete = False

    def __init__(self, mac_address, manager):
        try:
            self.rx_q = queue.SimpleQueue()
        except AttributeError:
            self.rx_q = queue.Queue()

        self.mac_address = mac_address
        self.manager = manager

    def connect(self):
        try:
            self.device = self.manager.adapter.connect(self.mac_address, timeout = 1)
        except pygatt.exceptions.NotConnectedError:
            print('Failed to connect.')
        
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
