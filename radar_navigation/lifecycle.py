"""Keep ROS context alive until shutdown cleanup has stopped the sensor/output."""
import signal
import threading


def initialize():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    requested=threading.Event()
    def stop(signum, frame):
        # Raising inside a TF C-extension callback can corrupt exception handling.
        requested.set()
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    return requested


def begin_cleanup():
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
