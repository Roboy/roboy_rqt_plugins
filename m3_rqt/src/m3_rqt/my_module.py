import os
import rospy
import rospkg
from python_qt_binding import QT_BINDING
import sys
from python_qt_binding.QtCore import qDebug
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QSlider
from python_qt_binding.QtCore import QObject
# import socket programming library
import socket, select
import matplotlib
# import thread module
from _thread import *
import threading
from struct import *

class MyPlugin(Plugin):
    target_pos = []
    target_force = []
    s = []
    s_cmd = []
    port = 8002  # where do you expect to get a msg?
    bufferSize = 12 # whatever you need
    server_thread = []
    client_address = ('192.168.0.102', 8000)
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('m3_rqt'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('M3')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self.target_pos = self._widget.findChild(QSlider, "target_pos")
        self.target_force = self._widget.findChild(QSlider, "target_force")
        self.target_pos.valueChanged.connect(self.pos_target_change)
        self.target_force.valueChanged.connect(self.force_target_change)

        import select, socket

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(('255.255.255.255', self.port))
        self.s.setblocking(0)
        self.server_thread = threading.Thread(target=self.receiveStatus)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.s_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # thread fuction
    def receiveStatus(self):
        while not rospy.is_shutdown():
            result = select.select([self.s],[],[])
            msg = result[0][0].recv(self.bufferSize)
            print(unpack('fff',msg))

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    def pos_target_change(self):
        setpoint = self.target_pos.value()
        print(setpoint)
        self.s_cmd.sendto(pack('fB', setpoint,0),self.client_address)

    def force_target_change(self):
        setpoint = self.target_force.value()
        print(setpoint)
        self.s_cmd.sendto(pack('fB', setpoint,1),self.client_address)