# External Dependencies
import bleak
import queue
import time
import threading
import asyncio

# Internal Dependencies
from pyroot import RootPhy

class RootBleak(RootPhy):
    root_identifier_uuid = '48c5d828-ac2a-442d-97a3-0c9822b04979'

    def __init__(self, name = None, dev = None, wait_for_connect = True):
        """Sets up Bluetooth manager to look for robots.
        
        Parameters
        ----------
        name : str, optional
            Name of the robot to connect to; if no name supplied, will
            connect to the first robot it sees.
        dev : str, optional
            Name of the device to connect to. Note that due to the way
            discovery is implemented, it will go much faster if you supply
            the device name.
        wait_for_connect : bool, optional
            If true (default), blocks until a connection is made
        """
        self._desired_name = name
        self._target_address = None

        if wait_for_connect:
            self.wait_for_connect()

        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self._discover())

    def _discover(self):
            print('Discovering devices...')
            devices = await bleak.discover()
            for d in devices:
                if self._desired_name is None or self._desired_name == d.name:
                    print('\nDiscovered', d.address, d.name)
                    try:
                        async with bleak.BleakClient(d.address, loop=self.loop) as client:
                            s = await client.get_services()
                            print(s.services, self.root_identifier_uuid in s.services)
                            if self.root_identifier_uuid in s.services:
                                print("Found a Root!")
                                self._target_address = d.address # probably should store the client here instead?
                                break
                                
                    except (bleak.exc.BleakError, AttributeError) as e:
                        print(e)
                        pass

    def wait_for_connect(self, timeout = float('inf')):
        """Blocking function initializing robot connection.

        Parameters
        ----------
        timeout : float, optional
            Time to wait for connection; if None, will wait forever. Will throw
            TimeoutError if timeout exceeded.
        """

        timeout += time.time()

        # connect to discovered robot
        # or if we time out, 
        #    raise TimeoutError('Timed out waiting for ' + self._desired_name)

        # Hang out until services are resolved
        # Need to do something to launch serial port connection, including creating the RX queue

    #def is_connected(self):
    #    """Utility function for determining state of bluetooth thread."""

    #def disconnect(self):
    #    """Disconnects BLE from robot and stops comms thread."""

    def send_raw(self, packet):
        """Helper method to send raw BLE packets to the robot.

        Parameters
        ----------
        packet : bytes
            20-byte packet to send to the robot.
        """
        if len(packet) == 20:
            # do packet write
            pass
        else:
            print('Error: send_raw_ble: Packet wrong length.')

