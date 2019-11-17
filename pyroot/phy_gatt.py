# External Dependencies
import gatt
import queue
import time
import threading

class RootGATT(object): # TODO: Make RootPhy ABC
    root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'

    def __init__(self, name = None, dev = 'hci0', wait_for_connect = True):
        """Sets up Bluetooth manager to look for robots.
        
        Parameters
        ----------
        name : str, optional
            Name of the robot to connect to; if no name supplied, will connect
            to the first robot it sees.
        dev : str, optional
            Name of the device to connect to; default is hci0
        wait_for_connect : bool, optional
            If true (default), blocks until a connection is made
        """
        self._ble_manager = BluetoothDeviceManager(adapter_name = dev)
        self._ble_manager.desired_name = name
        self._ble_manager.start_discovery(service_uuids=[self.root_identifier_uuid])
        self._ble_thread = threading.Thread(target = self._ble_manager.run)
        self._ble_thread.start()

        if wait_for_connect:
            self.wait_for_connect()

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

        self.rx_q = self._ble_manager.robot.rx_q

    def is_connected(self):
        """Utility function for determining state of bluetooth thread."""
        return self._ble_thread.is_alive()

    def disconnect(self):
        """Disconnects BLE from robot and stops comms thread."""
        self._ble_manager.stop()
        self._ble_manager.robot.disconnect()
        self._ble_thread.join()

    def send_raw(self, packet):
        """Helper method to send raw BLE packets to the robot.

        Parameters
        ----------
        packet : bytes
            20-byte packet to send to the robot.
        """
        if len(packet) == 20:
            self._ble_manager.robot.tx_characteristic.write_value(packet)
        else:
            print('Error: send_raw_ble: Packet wrong length.')

class BluetoothDeviceManager(gatt.DeviceManager):
    robot = None # root robot device
    desired_name = None

    def device_discovered(self, device):
        print("[%s] Discovered: %s" % (device.mac_address, device.alias()))
        if self.desired_name == None:
            self.desired_name = device.alias()

        if self.desired_name == device.alias():
            self.stop_discovery() # Stop searching
            self.robot = RootDevice(mac_address=device.mac_address, manager=self)
            self.robot.connect()

class RootDevice(gatt.Device):
    uart_service_uuid = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
    tx_characteristic_uuid = '6e400002-b5a3-f393-e0a9-e50e24dcca9e' # Write
    rx_characteristic_uuid = '6e400003-b5a3-f393-e0a9-e50e24dcca9e' # Notify

    service_resolution_complete = False

    def __init__(self, mac_address, manager, managed=True):
        try:
            self.rx_q = queue.SimpleQueue()
        except AttributeError:
            self.rx_q = queue.Queue()
        super().__init__(mac_address, manager, managed)
        
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
