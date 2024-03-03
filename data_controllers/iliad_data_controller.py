import math

import crc
import dearpygui.dearpygui as gui
from pymavlink.dialects.v10.common import MAVLink_message, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_STATE_BOOT, \
    MAVLink_global_position_int_message, MAVLink_scaled_imu_message

from components.data_series import DataSeries
from pymavlink import mavutil
import serial.tools.list_ports


class IliadDataController:

    def __init__(self, app) -> None:
        self.connection_log = None
        self.connect_button = None
        self.disconnect_button = None
        self.com_ports_combo = None
        self.status_text = None
        self.app = app

        self.mavlink: mavutil.mavserial | None = None
        self.mavlink_connected = False
        self.data_buffer = bytearray()
        self.checksum_calculator = crc.Calculator(crc.Crc16.CCITT)

        self.high_g_accelerometer_x = DataSeries('time', 'High G Acceleration X')  # float, float
        self.high_g_accelerometer_y = DataSeries('time', 'High G Acceleration Y')  # float, float
        self.high_g_accelerometer_z = DataSeries('time', 'High G Acceleration Z')  # float, float
        self.gyroscope_x = DataSeries('time', 'Gyroscope X')  # float, float
        self.gyroscope_y = DataSeries('time', 'Gyroscope Y')  # float, float
        self.gyroscope_z = DataSeries('time', 'Gyroscope Z')  # float, float
        self.accelerometer_x = DataSeries('time', 'Acceleration X')  # float, float
        self.accelerometer_y = DataSeries('time', 'Acceleration Y')  # float, float
        self.accelerometer_z = DataSeries('time', 'Acceleration Z')  # float, float
        self.barometer_altitude = DataSeries('time', 'Altitude')  # float, float
        self.gps_altitude = DataSeries('time', 'GPS Altitude')  # float, float
        self.gps_satellite_count = DataSeries('time', 'Satellite Count')  # float, int
        self.gps_latitude = DataSeries('time', 'Latitude')  # float, float
        self.gps_longitude = DataSeries('time', 'Longitude')  # float, float
        self.gps_ascent = DataSeries('time', 'Ascent')  # float, float
        self.gps_ground_speed = DataSeries('time', 'Ground Speed')  # float, float
        self.telemetrum_status = DataSeries('time', 'Telemetrum Status')  # float, int
        self.telemetrum_current = DataSeries('time', 'Telemetrum Current')  # float, float
        self.telemetrum_voltage = DataSeries('time', 'Telemetrum Voltage')  # float, float
        self.stratologger_status = DataSeries('time', 'Stratologger Status')  # float, int
        self.stratologger_current = DataSeries('time', 'Stratologger Current')  # float, float
        self.stratologger_voltage = DataSeries('time', 'Stratologger Voltage')  # float, float
        self.camera_status = DataSeries('time', 'Camera Status')  # float, int
        self.camera_current = DataSeries('time', 'Camera Current')  # float, float
        self.camera_voltage = DataSeries('time', 'Camera Voltage')  # float, float
        self.battery_voltage = DataSeries('time', 'Battery Voltage')  # float, float
        self.battery_temperature = DataSeries('time', 'Battery Temperature')  # float, float

    def add(self):
        # Create a gui for opening / closing the connection
        with gui.tab(label='Telemetry Connection') as self.telemetry_connection_tab:
            with gui.table(header_row=False):
                gui.add_table_column()
                gui.add_table_column()

                with gui.table_row():
                    with gui.table_cell():
                        ports = serial.tools.list_ports.comports()
                        named_ports = []
                        for port, desc, hwid in sorted(ports):
                            named_ports.append(port)

                        with gui.group() as self.connection_options:
                            gui.add_text("Please select a COM port to connect to rocket telemetry using MAVLink.")
                            gui.add_text("To refresh ports, please restart the application.")
                            gui.add_text("Available COM ports:")
                            self.com_ports_combo = gui.add_combo(named_ports, callback=self._on_com_port_selected)
                            self.connect_button = gui.add_button(label='CONNECT', width=400 * self.app.scaling_factor,
                                                                 height=200 * self.app.scaling_factor,
                                                                 callback=self._on_connect_button_clicked)
                            gui.hide_item(self.connect_button)
                            gui.bind_item_font(self.connect_button, self.app.large_font)

                        with gui.group() as self.connection_status:
                            self.status_text = gui.add_text("")
                            gui.bind_item_font(self.status_text, self.app.large_font)
                            self.disconnect_button = gui.add_button(label='Cancel Connection',
                                                                    width=400 * self.app.scaling_factor,
                                                                    height=200 * self.app.scaling_factor,
                                                                    callback=self._on_disconnect_button_clicked)
                            gui.bind_item_font(self.disconnect_button, self.app.large_font)

                        gui.hide_item(self.connection_status)
                    with gui.table_cell():
                        self.connection_log = gui.add_group()
                        self.info("Ready to connect to MAVLink")

    def info(self, text: str):
        gui.add_text(f"[INFO] {text}\n", parent=self.connection_log)

    def error(self, text: str):
        gui.add_text(f"[ERROR] {text}\n", parent=self.connection_log, color=[255, 0, 0])

    def status(self, text: str):
        gui.set_value(self.status_text, text)

    def info_status(self, text: str):
        self.info(text)
        self.status(text)

    def _on_com_port_selected(self):
        gui.show_item(self.connect_button)
        self.selected_com_port = gui.get_value(self.com_ports_combo)
        gui.set_item_label(self.connect_button, f"Connect to {self.selected_com_port}")

    def gui_connecting(self):
        gui.hide_item(self.connection_options)
        gui.show_item(self.connection_status)

    def gui_connected(self):
        self.info_status(f"Connected to MAVLink on port {self.selected_com_port}")
        gui.hide_item(self.disconnect_button)

    def gui_disconnect(self):
        gui.show_item(self.connection_options)
        gui.hide_item(self.connection_status)

    def send_heartbeat_packet(self):
        self.info(f'Sent MAVLink heartbeat packet on port {self.selected_com_port}...')
        self.mavlink.mav.heartbeat_send(MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_BOOT)

    def _on_connect_button_clicked(self) -> None:
        self.gui_connecting()
        self.info(f'Initializing MAVLink connection on port {self.selected_com_port}...')
        try:
            self.mavlink = mavutil.mavlink_connection(self.selected_com_port)
            self.send_heartbeat_packet()
            self.info_status(f'Waiting for MAVLink heartbeat packet on port {self.selected_com_port}...')
        except PermissionError:
            self.error(f"Permission denied to open port {self.selected_com_port}")
            self.gui_disconnect()
        except Exception as e:
            self.error(repr(e))
            self.gui_disconnect()

    def _on_disconnect_button_clicked(self) -> None:
        self.gui_disconnect()
        if self.mavlink:
            self.mavlink.close()
            self.mavlink = None
        if self.mavlink_connected:
            self.info("Disconnected from MAVLink")
        else:
            self.info("Cancelled connecting to MAVLink")

    def on_mavlink_packet(self, msg: MAVLink_message):
        match msg.get_type():
            case 'HEARTBEAT':
                if not self.mavlink_connected:
                    self.info("Initial heartbeat packet received")
                    self.send_heartbeat_packet()
                    self.gui_connected()
                    self.mavlink_connected = True
            case 'SCALED_IMU':
                msg: MAVLink_scaled_imu_message
                # scale time to s from ms
                t = msg.time_boot_ms / 1000
                # scale accel to m/s^2 from milliG
                accel_factor = 9.81 / 1000
                self.accelerometer_x.add_point(t, msg.xacc * accel_factor)
                self.accelerometer_y.add_point(t, msg.yacc * accel_factor)
                self.accelerometer_z.add_point(t, msg.zacc * accel_factor)
                # scale accel to rotations/s from millirad/s
                gyro_factor = 1 / 1000 / (2 * math.pi)
                self.gyroscope_x.add_point(t, msg.xgyro * gyro_factor)
                self.gyroscope_y.add_point(t, msg.ygyro * gyro_factor)
                self.gyroscope_z.add_point(t, msg.zgyro * gyro_factor)
            case 'GLOBAL_POSITION_INT':
                msg: MAVLink_global_position_int_message
                t = msg.time_boot_ms / 1000
                # scale degE7 to deg
                gps_lat_long_factor = 1 / 10000000
                self.gps_latitude.add_point(t, msg.lat * gps_lat_long_factor)
                self.gps_longitude.add_point(t, msg.lon * gps_lat_long_factor)
                # scale mm to m
                gps_altitude_factor = 1 / 1000
                self.gps_altitude.add_point(t, msg.alt * gps_altitude_factor)
                # scale cm/s to m
                gps_ground_speed_factor = 1 / 100
                self.gps_ground_speed.add_point(t, msg.vx * gps_ground_speed_factor)

    def update(self) -> None:
        if self.mavlink is not None:
            msg: MAVLink_message = self.mavlink.recv_msg()
            if msg is not None:
                self.on_mavlink_packet(msg)

    def destroy(self):
        if self.mavlink is not None:
            self.mavlink.close()
            self.mavlink = None
