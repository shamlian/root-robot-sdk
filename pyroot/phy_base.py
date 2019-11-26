class RootPhy(object): # TODO: Use ABC

    def __init__(self, name = None, dev = None, wait_for_connect = True):
        """Sets up physical connection with robot.
        
        Parameters
        ----------
        name : str, optional
            Name of the robot to connect to.
        dev : str, optional
            Name of the device to connect to. If not supplied, will attempt
            to auto-discover.
        wait_for_connect : bool, optional
            If true (default), blocks until a connection is made
        """
        raise NotImplementedError('__init__ not implemented.')

    def wait_for_connect(self, timeout = float('inf')):
        """Blocking function initializing robot connection.

        Parameters
        ----------
        timeout : float, optional
            Time to wait for connection; if None, will wait forever. Will throw
            TimeoutError if timeout exceeded.
        """

        raise NotImplementedError('wait_for_connect not implemented.')

    def is_connected(self):
        """Utility function for determining state of comms thread."""
        raise NotImplementedError('is_connected not implemented.')

    def disconnect(self):
        """Disconnects from robot and stops comms thread."""
        raise NotImplementedError('disconnect not implemented.')

    def send_raw(self, packet):
        """Helper method to send raw packets to the robot.

        Parameters
        ----------
        packet : bytes
            20-byte packet to send to the robot.
        """
        raise NotImplementedError('send_raw not implemented.')
