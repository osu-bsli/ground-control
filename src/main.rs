#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")] // hide console window on Windows in release

mod data;
mod plot_tab;
mod serial_connection;

use std::{collections::VecDeque, io::Cursor};

use data::{Data, DataSeries};
use ground_control::mavlink_generated::bsli2025::MavMessage;
use mavlink_core::peek_reader::PeekReader;
use serial_connection::SerialConnection;

use eframe::egui::{self};
use plot_tab::PlotTab;
use serialport::SerialPortInfo;

// mG to m/s^2
fn scale_mG_to_mps2(val: i16) -> f64 {
    const G: f64 = 9.81;
    val as f64 / 1000.0 * G
}

fn main() -> eframe::Result {
    env_logger::init(); // Log to stderr (if you run with `RUST_LOG=debug`).

    let native_options = eframe::NativeOptions {
        vsync: false,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([400.0, 300.0])
            .with_min_inner_size([300.0, 220.0])
            .with_icon(
                // NOTE: Adding an icon is optional
                eframe::icon_data::from_png_bytes(&include_bytes!("assets/bsli_logo.png")[..])
                    .expect("Failed to load icon"),
            ),
        ..Default::default()
    };
    eframe::run_native(
        "BSLI IREC Ground Control",
        native_options,
        Box::new(|cc| Ok(Box::new(GroundControlApp::new(cc)))),
    )
}

#[derive(PartialEq)]
enum AppTab {
    Plot,
}

struct GroundControlApp {
    plot_tab: PlotTab,

    data: data::Data,

    serial: SerialConnection,
    serial_received: PeekReader<VecDeque<u8>>,

    ui_showsidebar: bool,
    ui_selected_tab: AppTab,

    frame_count: u64,
    packets_received: u32,
}

impl GroundControlApp {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        // if let Some(storage) = cc.storage {
        //     return eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default();
        // }

        let mut app = Self {
            plot_tab: PlotTab::new(),

            data: Data::new(),

            serial: serial_connection::SerialConnection::new(),
            serial_received: PeekReader::new(VecDeque::new()),

            ui_showsidebar: true,
            ui_selected_tab: AppTab::Plot,

            frame_count: 0,
            packets_received: 0,
        };

        app.serial.refresh_known_ports();

        app
    }

    fn ui_add_serialportui(&mut self, ui: &mut egui::Ui) {
        let settings_isenabled = self.serial.connection_allowed();
        let connect_isenabled =
            self.serial.connection_allowed() && !self.serial.selected_port.is_empty();
        let disconnect_isenabled = self.serial.disconnection_allowed();

        // port settings
        ui.add_enabled_ui(settings_isenabled, |ui| {
            // port selection
            ui.horizontal(|ui| {
                GroundControlApp::ui_draw_serialportdropdown(
                    ui,
                    &mut self.serial.known_ports,
                    &mut self.serial.selected_port,
                );
                ui.label("Port");
                if ui.button("Refresh").clicked() {
                    self.serial.refresh_known_ports();
                }
            });

            // baud rate
            egui::ComboBox::from_label("Baud rate")
                .selected_text(format!("{}", self.serial.baud_rate))
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.serial.baud_rate, 50, "50");
                    ui.selectable_value(&mut self.serial.baud_rate, 75, "75");
                    ui.selectable_value(&mut self.serial.baud_rate, 110, "110");
                    ui.selectable_value(&mut self.serial.baud_rate, 134, "134");
                    ui.selectable_value(&mut self.serial.baud_rate, 150, "150");
                    ui.selectable_value(&mut self.serial.baud_rate, 200, "200");
                    ui.selectable_value(&mut self.serial.baud_rate, 300, "300");
                    ui.selectable_value(&mut self.serial.baud_rate, 600, "600");
                    ui.selectable_value(&mut self.serial.baud_rate, 1200, "1200");
                    ui.selectable_value(&mut self.serial.baud_rate, 1800, "1800");
                    ui.selectable_value(&mut self.serial.baud_rate, 2400, "2400");
                    ui.selectable_value(&mut self.serial.baud_rate, 4800, "4800");
                    ui.selectable_value(&mut self.serial.baud_rate, 9600, "9600");
                    ui.selectable_value(&mut self.serial.baud_rate, 19200, "19200");
                    ui.selectable_value(&mut self.serial.baud_rate, 28800, "28800");
                    ui.selectable_value(&mut self.serial.baud_rate, 38400, "38400");
                    ui.selectable_value(&mut self.serial.baud_rate, 57600, "57600");
                    ui.selectable_value(&mut self.serial.baud_rate, 76800, "76800");
                    ui.selectable_value(&mut self.serial.baud_rate, 115200, "115200");
                    ui.selectable_value(&mut self.serial.baud_rate, 230400, "230400");
                    ui.selectable_value(&mut self.serial.baud_rate, 460800, "460800");
                    ui.selectable_value(&mut self.serial.baud_rate, 576000, "576000");
                    ui.selectable_value(&mut self.serial.baud_rate, 921600, "921600");
                });
        });

        // status
        ui.label(format!(
            "Status: {}",
            match self.serial.connection_status() {
                serial_connection::Status::Connected => "Connected",
                serial_connection::Status::Connecting => "Connecting...",
                serial_connection::Status::Disconnected => "Disconnected",
                serial_connection::Status::Disconnecting => "Disconnecting...",
                serial_connection::Status::Failed => "Failed",
            }
        ));
        ui.label(format!("Bytes read: {}", self.serial.bytes_read()));
        ui.label("Error rate: 0% (placeholder)");

        // connect/disconnect buttons
        ui.horizontal(|ui| {
            ui.add_enabled_ui(connect_isenabled, |ui| {
                if ui.button("Connect").clicked() {
                    self.serial.connect(self.serial.selected_port.clone());
                }
            });

            ui.add_enabled_ui(disconnect_isenabled, |ui| {
                if ui.button("Disconnect").clicked() {
                    self.serial.disconnect();
                }
            });
        });
    }

    fn ui_draw_serialportdropdown(
        ui: &mut egui::Ui,
        availableports: &mut Vec<SerialPortInfo>,
        selectedport: &mut String,
    ) {
        egui::ComboBox::from_id_salt("serialport-name")
            .selected_text(selectedport.clone())
            .show_ui(ui, |ui| {
                for p in availableports {
                    ui.selectable_value(selectedport, p.port_name.clone(), p.port_name.clone());
                    // TODO: cloning port info every time is probably horrible lol
                }
            });
    }
}

impl eframe::App for GroundControlApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.frame_count += 1;

        /* Read all pending data from serial port buffer and place in recieve buffer for MAVLink library */
        loop {
            if let Ok(b) = self.serial.read_byte() {
                self.serial_received.reader_mut().push_back(b);
            } else {
                break;
            }
        }

        /* Loop to keep decoding packets until we're out of data */
        loop {
            /* Decode MAVLink packet */
            let msg: Result<
                (mavlink_core::MavHeader, MavMessage),
                mavlink_core::error::MessageReadError,
            > = mavlink_core::read_v2_msg(&mut self.serial_received);
            if let Ok(msg) = msg {
                /* If we're here we know the packet was successfully decoded */
                /* msg is of type enum ground_control::mavlink_generated::bsli2025::MavMessage */
                self.packets_received += 1;
                match msg.1 {
                    MavMessage::BSLI2025_COMPOSITE(data) => {
                        /* milliG to m/s^2 */
                        self.data.x_acc.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            scale_mG_to_mps2(data.xacc),
                        );
                        self.data.y_acc.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            scale_mG_to_mps2(data.yacc),
                        );
                        self.data.z_acc.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            scale_mG_to_mps2(data.zacc),
                        );

                        /* mrad/s to rad/s */
                        self.data.x_gyro.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            (data.xgyro as f64) / 1000.0,
                        );
                        self.data.y_gyro.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            (data.ygyro as f64) / 1000.0,
                        );
                        self.data.z_gyro.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            (data.zgyro as f64) / 1000.0,
                        );

                        /* mgauss to mgauss */
                        self.data.x_mag.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            data.xmag as f64,
                        );
                        self.data.y_mag.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            data.ymag as f64,
                        );
                        self.data.z_mag.add_point(
                            data.time_boot_ms as f64 / 1000.0,
                            data.zmag as f64,
                        );
                        // TODO: The other measured values (gyro, magn)
                    }
                    _ => println!("Unhandled MAVLink packet type"),
                }
            } else {
                break;
            }
        }

        /* Make sure packets are read at least every 0.1 seconds */
        ctx.request_repaint_after_secs(0.1);

        egui::TopBottomPanel::top("menubar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.label("BSLI Ground Control");

                ui.separator();

                ui.menu_button("File", |ui| {
                    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);

                    if ui.add(egui::Button::new("Placeholder")).clicked() {
                        ui.close_menu();
                    }
                    if ui.add(egui::Button::new("Placeholder")).clicked() {
                        ui.close_menu();
                    }
                    if ui.add(egui::Button::new("Placeholder")).clicked() {
                        ui.close_menu();
                    }
                });

                egui::global_theme_preference_switch(ui);

                ui.separator();

                ui.toggle_value(&mut self.ui_showsidebar, "Sidebar");
                ui.selectable_value(&mut self.ui_selected_tab, AppTab::Plot, "Plot");
            });
        });

        egui::SidePanel::left("sidebar")
            .resizable(false)
            .default_width(160.0)
            .min_width(160.0)
            .show_animated(ctx, self.ui_showsidebar, |ui| {
                // ui.add_space(4.0);
                egui::ScrollArea::vertical().show(ui, |ui| {
                    // serial port connection ui
                    ui.collapsing("Info", |ui| {
                        ui.label(format!("UI frame count: {}", self.frame_count));
                    });

                    ui.collapsing("Serial port", |ui| {
                        self.ui_add_serialportui(ui);
                    });

                    ui.collapsing("Logging", |ui| {
                        ui.label("placeholder");
                    });

                    ui.collapsing("MAVLink", |ui| {
                        ui.label(format!("Packets received: {}", self.packets_received));
                        ui.label("Error rate: 0% (placeholder)");
                    });

                    ui.collapsing("Data", |ui| {
                        egui::Grid::new("sidebar-data-grid")
                            .num_columns(2)
                            .striped(true)
                            .show(ui, |ui| {
                                let mut display_data_series_label = |s: &DataSeries| {
                                    ui.label(format!("{}:", s.name));
                                    ui.label(format!("{} {}", s.last_y_str(), s.units));
                                    ui.end_row();
                                };

                                display_data_series_label(&self.data.y_acc);
                                display_data_series_label(&self.data.x_acc);
                                display_data_series_label(&self.data.z_acc);
                                display_data_series_label(&self.data.x_gyro);
                                display_data_series_label(&self.data.y_gyro);
                                display_data_series_label(&self.data.z_gyro);
                                display_data_series_label(&self.data.x_mag);
                                display_data_series_label(&self.data.y_mag);
                                display_data_series_label(&self.data.z_mag);
                            });
                    });
                });
            });

        egui::CentralPanel::default().show(ctx, |ui| match self.ui_selected_tab {
            AppTab::Plot => {
                self.plot_tab.ui(ui, &self.data);
            }
        });
    }
}
