# External Dependencies
import serial
import queue
import time
import threading
from binascii import hexlify, unhexlify

# Internal Dependencies
from pyroot import RootPhy

class RootSerial(RootPhy): # TODO: Make RootPhy ABC

    def __init__(self, name = None, dev = None, wait_for_connect = True):
        """Sets up serial port connection with robot.
        
        Parameters
        ----------
        name : str, optional
            Name of the robot to connect to; currently not checked.
        dev : str, optional
            Name of the device to connect to. If not supplied, will attempt
            to auto-discover.
        wait_for_connect : bool, optional
            If true (default), blocks until a connection is made
        """
        if dev == None:
            raise NotImplementedError('Serial port discovery not implemented.')

        self._serial_port = serial.Serial(port=dev, baudrate=115200)
        self._desired_name = name
        self._comms_thread = threading.Thread(target = self._maintain_connection)
        self._comms_thread.start()

        if wait_for_connect:
            self.wait_for_connect()

    def wait_for_connect(self, timeout = float('inf')):
        """Blocking function initializing robot connection.

        Parameters
        ----------
        timeout : float, optional
            Time to wait for connection; if None, will wait forever. Will throw
            TimeoutError if timeout exceeded.
        """

        timeout += time.time()

        while self._serial_port.is_open is False and time.time() < timeout:
            time.sleep(0.1) # wait for a root robot to be attached
        if self._serial_port.is_open is False:
            raise TimeoutError('Timed out waiting for ' + self._desired_name)

        try:
            self.rx_q = queue.SimpleQueue()
        except AttributeError:
            self.rx_q = queue.Queue()

    def is_connected(self):
        """Utility function for determining state of serial thread."""
        return self._comms_thread.is_alive()

    def disconnect(self):
        """Disconnects from robot and stops comms thread."""
        self._serial_port.close()
        self._comms_thread.join()

    def send_raw(self, packet):
        """Helper method to send raw packets to the robot.

        Parameters
        ----------
        packet : bytes
            20-byte packet to send to the robot.
        """
        if len(packet) == 20:
            try:
                self._serial_port.write(hexlify(packet) + b'\n')
            except serial.SerialException as e:
                print('Warning from send: ', end='')
                print(e)
            except TypeError as e:
                pass # serial port shut down while sending
        else:
            print('Error: send_raw: Packet wrong length.')

    def _maintain_connection(self):
        """Assembles received data into packets for parsing.
        Safely shuts things down if the connection is interrupted.
        """

        while self._serial_port.is_open:
            try:
                packet = self._serial_port.readline()
                if len(packet) == 41:
                    packet = unhexlify(packet[:-1])
                    self.rx_q.put(packet)
                else:
                    print('ERROR: Received packet of improper length')
            except serial.SerialException as e:
                print('Warning from receive: ', end='')
                print(e)
            except TypeError as e:
                pass # serial port shut down while sending
