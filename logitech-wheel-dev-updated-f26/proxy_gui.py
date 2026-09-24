from asyncio.proactor_events import _ProactorDuplexPipeTransport
import ctypes
import struct
import sys
import time
import socket

from PyQt5 import QtGui, QtWidgets, QtCore

import logitech_steering_wheel as lsw

TX_INTERVAL_MS = 50

## Configure this ##
REMOTE_HOST = "169.254.100.16"
S_PORT = 8000

LOCAL_HOST = "169.254.227.87" # IP of local interface
R_PORT = 8001
##

class MyMainwindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.update_timer = QtCore.QTimer()
        self.update_timer.setInterval(TX_INTERVAL_MS)
        self.update_timer.setSingleShot(False)
        self.update_timer.timeout.connect(self._update_sw)

        self.transmit_timer = QtCore.QTimer()
        self.transmit_timer.setInterval(TX_INTERVAL_MS)
        self.transmit_timer.setSingleShot(False)
        self.transmit_timer.timeout.connect(self._transmit)

        self.connect_button = QtWidgets.QPushButton('connect')
        self.current_angle = QtWidgets.QSpinBox()
        self.current_angle.setMaximum(2147483647)
        self.current_angle.setMinimum(-2147483648)

        self.throttle_angle = QtWidgets.QSpinBox()
        self.throttle_angle.setMaximum(2147483647)
        self.throttle_angle.setMinimum(-2147483648)

        self.brake_angle = QtWidgets.QSpinBox()
        self.brake_angle.setMaximum(2147483647)
        self.brake_angle.setMinimum(-2147483648)

        self.stop_button = QtWidgets.QPushButton('stop')
        self.effect_button = QtWidgets.QPushButton('Toggle feedback')

        self.connect_button.clicked.connect(self.connect_to_wheel)
        self.stop_button.clicked.connect(self.stop)
        self.effect_button.clicked.connect(self.toggle_feedback)

        self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_ctr = 0

        self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receive_socket.bind((LOCAL_HOST, R_PORT))
        self.receive_socket.setblocking(False)
        self.receive_ctr = 0

        layout = QtWidgets.QVBoxLayout()
        widget = QtWidgets.QWidget()

        widget.setLayout(layout)
        layout.addWidget(self.connect_button)
        layout.addWidget(self.effect_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.current_angle)
        layout.addWidget(self.throttle_angle)
        layout.addWidget(self.brake_angle)

        self.setCentralWidget(widget)

    def connect_to_wheel(self) -> None:
        w = self.winId()
        initialized = lsw.initialize_with_window(True, w)

        if initialized:
            print('initialized successfully')
        else:
            return

        if lsw.is_connected(0):
            print('connected to a steering wheel at index 0')

        lsw.update()
        # Move wheel to register initial state
        lsw.play_bumpy_road_effect(0, 20)
        time.sleep(0.04)
        lsw.stop_bumpy_road_effect(0)

        self.update_timer.start()
        self.transmit_timer.start()
        print(lsw.get_current_controller_properties(0))

    def _transmit(self):
        # Send logitech wheel state
        #print("Transmit")
        state_c = lsw.get_c_state(0)
        pdata = bytearray(ctypes.string_at(ctypes.byref(state_c), ctypes.sizeof(state_c)))
        # Append packet counter
        pdata = self.send_ctr.to_bytes(4, 'little') + pdata
        self.send_ctr += 1
        self.send_socket.sendto(pdata, (REMOTE_HOST, S_PORT))
        

    def _update_sw(self):
        lsw.update()
        state = lsw.get_state(0)

        # Set GUI vars
        self.current_angle.setValue(state.lX)
        self.throttle_angle.setValue(state.lY)
        self.brake_angle.setValue(state.lRz)

        try:
            force_byte = self.receive_socket.recvfrom(10)[0]
            force = int.from_bytes(force_byte, 'little', signed=True)
            print("Received | (Pkt {}) : {}".format(self.receive_ctr, force))
            self.receive_ctr += 1
            lsw.play_constant_force(0, int(force))

        except socket.error as e:
            print("No data")
            

    def toggle_feedback(self):
        if lsw.is_playing(0, lsw.ForceType.BUMPY_ROAD):
            lsw.stop_bumpy_road_effect(0)
        else:
            lsw.play_bumpy_road_effect(0, 20)

    def stop(self):
        print('disconnecting')
        lsw.shutdown()
        self.update_timer.stop()
        self.transmit_timer.stop()

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self.stop()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyMainwindow()
    window.show()
    sys.exit(app.exec_())
