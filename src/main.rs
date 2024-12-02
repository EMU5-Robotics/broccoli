#![no_main]
#![no_std]

extern crate alloc;

use brain_helper::{
    protocol::{self, *},
    vexide::{self, prelude::*},
};

use vexide::devices::screen::{self, Screen};

struct Robot(ComponentHandler);

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let handler = &mut self.0;
        let mut num_packets = 0;
        let mut quadrant: u8 = 0;
        let mut text_col: u32 = 0;
        loop {
            if let Some(ref pkt) = brain_helper::read_pkt_serial() {
                screen::Text::new(
                    &alloc::format!("{num_packets:?}"),
                    screen::TextSize::Medium,
                    [0, 0],
                )
                .fill(&mut handler.screen, 0xFFFFFFu32);
                num_packets += 1;

                handler.set_motors(pkt.set_motors);
                handler.set_gearsets(pkt.set_motor_gearsets);
                handler.set_triports(pkt.set_triports);
            }

            let touch_status = handler.screen.touch_status();

            if touch_status.state == screen::TouchState::Pressed {
                match (
                    touch_status.x < Screen::HORIZONTAL_RESOLUTION / 2,
                    touch_status.y < Screen::VERTICAL_RESOLUTION / 2,
                ) {
                    (true, true) => {
                        quadrant = 0;
                        text_col = 0xFFFFFFu32;
                    }
                    (false, true) => {
                        quadrant = 1;
                        text_col = 0xFF0000u32;
                    }
                    (true, false) => {
                        quadrant = 2;
                        text_col = 0x00FF00u32;
                    }
                    (false, false) => {
                        quadrant = 3;
                        text_col = 0x0000FFu32;
                    }
                }
            }

            screen::Text::new(
                &alloc::format!("quadrant: {quadrant:?}"),
                screen::TextSize::Large,
                [0, 64],
            )
            .fill(&mut handler.screen, text_col);

            let mut to_robot_packet = ToRobot::default();
            to_robot_packet.comp_state = protocol::CompState::Auton(quadrant);
            to_robot_packet.device_list = Some(handler.get_device_list());
            to_robot_packet.encoder_state = handler.get_encoder_states();

            brain_helper::write_pkt_serial(to_robot_packet);
            vexide::async_runtime::time::sleep(core::time::Duration::from_millis(1)).await;
        }
    }

    async fn driver(&mut self) {
        let handler = &mut self.0;
        let mut num_packets = 0;
        loop {
            if let Some(ref pkt) = brain_helper::read_pkt_serial() {
                screen::Text::new(
                    &alloc::format!("{num_packets:?}"),
                    screen::TextSize::Medium,
                    [0, 0],
                )
                .fill(&mut handler.screen, 0xFFFFFFu32);
                num_packets += 1;

                handler.set_motors(pkt.set_motors);
                handler.set_gearsets(pkt.set_motor_gearsets);
                handler.set_triports(pkt.set_triports);
            }

            let mut to_robot_packet = ToRobot::default();
            to_robot_packet.comp_state = protocol::CompState::Driver;
            to_robot_packet.device_list = Some(handler.get_device_list());
            to_robot_packet.controller_state = Some(handler.get_controller_state());
            to_robot_packet.encoder_state = handler.get_encoder_states();
            to_robot_packet.imu_state = handler.get_imu_states();

            brain_helper::write_pkt_serial(to_robot_packet);
            vexide::async_runtime::time::sleep(core::time::Duration::from_millis(1)).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot(peripherals.into());
    robot.compete().await;
}

#[derive(Default)]
pub enum SmartPortDevice {
    #[default]
    Empty,
    Raw(SmartPort),
    Motor(Motor),
    Encoder(RotationSensor),
    Imu(InertialSensor),
}

use protocol::{AdiPortState, ConfigureAdiPort, PortState};
use vexide::devices::{adi::digital::LogicLevel, screen::Fill};
impl SmartPortDevice {
    pub fn get_type(&self) -> PortState {
        match self {
            Self::Empty => PortState::Unplugged,
            Self::Raw(sp) => sp.device_type().into(),
            Self::Motor(_) => PortState::Motor,
            Self::Encoder(_) => PortState::Encoder,
            Self::Imu(_) => PortState::Imu,
        }
    }
    // TODO: improve state handling
    pub fn transform(&mut self, dest_type: PortState) {
        match dest_type {
            PortState::Motor => {
                let old = core::mem::take(self);
                if let SmartPortDevice::Raw(sp) = old {
                    *self =
                        SmartPortDevice::Motor(Motor::new(sp, Gearset::Green, Direction::Forward));
                } else {
                    *self = old;
                }
            }
            PortState::Encoder => {
                let old = core::mem::take(self);
                if let SmartPortDevice::Raw(sp) = old {
                    *self = SmartPortDevice::Encoder(RotationSensor::new(sp, Direction::Forward));
                } else {
                    *self = old;
                }
            }
            PortState::Imu => {
                let old = core::mem::take(self);
                if let SmartPortDevice::Raw(sp) = old {
                    let mut imu = InertialSensor::new(sp);
                    let _ = imu.calibrate();
                    *self = SmartPortDevice::Imu(imu);
                } else {
                    *self = old;
                }
            }
            _ => {}
        }
    }
}

#[derive(Debug)]
pub enum AdiPortDevice {
    Raw(AdiPort),
    AnalogIn(AdiAnalogIn),
    DigitalOut(AdiDigitalOut, bool),
    DigitalIn(AdiDigitalIn),
    TransferState,
}

impl AdiPortDevice {
    pub fn get_state(&self) -> AdiPortState {
        match self {
            Self::Raw(_) => AdiPortState::Other,
            Self::AnalogIn(p) => match (p.value(), p.voltage()) {
                (Ok(v), Ok(v2)) => AdiPortState::AnalogIn(Some((v, v2))),
                _ => AdiPortState::AnalogIn(None),
            },
            Self::DigitalIn(p) => {
                AdiPortState::DigitalIn(p.level().ok().map(|v| v == LogicLevel::High))
            }
            Self::DigitalOut(_, is_high) => AdiPortState::DigitalOut(*is_high),
            Self::TransferState => AdiPortState::Other,
        }
    }
    pub fn into_raw(&mut self) {
        if let Self::Raw(_) = self {
            return;
        }

        match core::mem::replace(self, AdiPortDevice::TransferState) {
            AdiPortDevice::DigitalOut(p, _) => {
                *self = Self::Raw(p.into());
            }
            Self::AnalogIn(p) => {
                *self = Self::Raw(p.into());
            }
            Self::DigitalIn(p) => {
                *self = Self::Raw(p.into());
            }
            Self::TransferState | Self::Raw(_) => unreachable!(),
        };
    }
    pub fn into_analog_in(&mut self) {
        if let Self::AnalogIn(_) = self {
            return;
        }

        self.into_raw();
        if let Self::Raw(p) = core::mem::replace(self, AdiPortDevice::TransferState) {
            *self = Self::AnalogIn(AdiAnalogIn::new(p.into()));
        };
    }
    pub fn into_digital_in(&mut self) {
        if let Self::DigitalIn(_) = self {
            return;
        }

        self.into_raw();
        if let Self::Raw(p) = core::mem::replace(self, AdiPortDevice::TransferState) {
            *self = Self::DigitalIn(AdiDigitalIn::new(p.into()));
        };
    }
    pub fn into_digital_out(&mut self) {
        if let Self::DigitalOut(_, _) = self {
            return;
        }

        self.into_raw();
        if let Self::Raw(p) = core::mem::replace(self, AdiPortDevice::TransferState) {
            *self = Self::DigitalOut(AdiDigitalOut::new(p.into()), false);
        };
    }
}

pub struct ComponentHandler {
    smart_ports: [SmartPortDevice; 21],
    adi_ports: [AdiPortDevice; 8],
    primary_controller: Controller,
    screen: vexide::devices::screen::Screen,
}

impl ComponentHandler {
    pub fn set_motors(&mut self, targets: [protocol::MotorControl; 21]) {
        for (target, motor) in targets.iter().zip(self.smart_ports.iter_mut()) {
            let v: Option<vexide::devices::smart::motor::MotorControl> = (*target).into();
            if let Some(motor_control) = v {
                motor.transform(PortState::Motor);
                if let SmartPortDevice::Motor(m) = motor {
                    let _ = m.set_target(motor_control);
                }
            }
        }
    }
    pub fn set_triports(&mut self, targets: [protocol::ConfigureAdiPort; 8]) {
        for (target, triport) in targets.iter().zip(self.adi_ports.iter_mut()) {
            match target {
                ConfigureAdiPort::None => {}
                ConfigureAdiPort::AnalogIn => triport.into_analog_in(),
                ConfigureAdiPort::DigitalLow => {
                    triport.into_digital_out();
                    if let AdiPortDevice::DigitalOut(p, is_high) = triport {
                        if *is_high {
                            if let Ok(_) = p.set_low() {
                                *is_high = false;
                            }
                        }
                    }
                }
                ConfigureAdiPort::DigitalHigh => {
                    triport.into_digital_out();
                    if let AdiPortDevice::DigitalOut(p, is_high) = triport {
                        if !*is_high {
                            if let Ok(_) = p.set_high() {
                                *is_high = true;
                            }
                        }
                    }
                }
                ConfigureAdiPort::DigitalIn => triport.into_digital_in(),
            }
        }
    }
    pub fn set_gearsets(&mut self, gearsets: [protocol::GearSetChange; 21]) {
        for (gearset_change, motor) in gearsets.iter().zip(self.smart_ports.iter_mut()) {
            let v: Option<vexide::devices::smart::motor::Gearset> = (*gearset_change).into();
            if let Some(gearset) = v {
                motor.transform(PortState::Motor);
                if let SmartPortDevice::Motor(m) = motor {
                    let _ = m.set_gearset(gearset);
                }
            }
        }
    }
    pub fn get_device_list(&self) -> DevicesList {
        DevicesList {
            smart_ports: self.smart_ports.each_ref().map(|v| v.get_type()),
            adi_ports: self.adi_ports.each_ref().map(|v| v.get_state()),
        }
    }
    pub fn get_encoder_states(&mut self) -> [EncoderState; 21] {
        let mut encoder_states = [protocol::EncoderState::None; 21];
        for (i, possible_smart_port) in self.smart_ports.iter_mut().enumerate() {
            if possible_smart_port.get_type() == PortState::Encoder {
                possible_smart_port.transform(PortState::Encoder);
                match possible_smart_port {
                    SmartPortDevice::Encoder(encoder) => {
                        if let Ok(angle) = encoder.angle() {
                            encoder_states[i] = protocol::EncoderState::Radians(angle.as_radians());
                        }
                    }
                    SmartPortDevice::Motor(motor) => {
                        if let Ok(position) = motor.position() {
                            encoder_states[i] =
                                protocol::EncoderState::Radians(position.as_radians());
                        }
                    }
                    _ => {}
                }
            }
        }
        encoder_states
    }
    pub fn get_imu_states(&mut self) -> [ImuState; 21] {
        let mut imu_states = [protocol::ImuState::None; 21];
        for (i, possible_smart_port) in self.smart_ports.iter_mut().enumerate() {
            if possible_smart_port.get_type() == PortState::Imu {
                possible_smart_port.transform(PortState::Imu);
                if let SmartPortDevice::Imu(imu) = possible_smart_port {
                    if let (Ok(z_rotation), Ok(rate)) = (imu.rotation(), imu.gyro_rate()) {
                        imu_states[i] = protocol::ImuState::State {
                            z_rotation,
                            z_rate: rate.x,
                        }
                    }
                }
            }
        }
        imu_states
    }

    pub fn get_controller_state(&self) -> ControllerState {
        use protocol::controller::*;
        let mut controller_state = protocol::ControllerState::default();

        for (button, actual_button) in [
            (A, &self.primary_controller.button_a),
            (B, &self.primary_controller.button_b),
            (X, &self.primary_controller.button_x),
            (Y, &self.primary_controller.button_y),
            (UP, &self.primary_controller.button_up),
            (DOWN, &self.primary_controller.button_down),
            (LEFT, &self.primary_controller.button_left),
            (RIGHT, &self.primary_controller.button_right),
            (LEFT_TRIGGER_1, &self.primary_controller.left_trigger_1),
            (LEFT_TRIGGER_2, &self.primary_controller.left_trigger_2),
            (RIGHT_TRIGGER_1, &self.primary_controller.right_trigger_1),
            (RIGHT_TRIGGER_2, &self.primary_controller.right_trigger_2),
        ] {
            if let Ok(true) = actual_button.is_pressed() {
                controller_state.buttons |= button;
            }
        }

        controller_state.axis[0] = self.primary_controller.left_stick.x().unwrap_or(0.0);
        controller_state.axis[1] = self.primary_controller.left_stick.y().unwrap_or(0.0);
        controller_state.axis[2] = self.primary_controller.right_stick.x().unwrap_or(0.0);
        controller_state.axis[3] = self.primary_controller.right_stick.y().unwrap_or(0.0);

        controller_state
    }
}

impl From<Peripherals> for ComponentHandler {
    fn from(v: Peripherals) -> Self {
        Self {
            smart_ports: [
                v.port_1, v.port_2, v.port_3, v.port_4, v.port_5, v.port_6, v.port_7, v.port_8,
                v.port_9, v.port_10, v.port_11, v.port_12, v.port_13, v.port_14, v.port_15,
                v.port_16, v.port_17, v.port_18, v.port_19, v.port_20, v.port_21,
            ]
            .map(|v| SmartPortDevice::Raw(v)),
            adi_ports: [
                v.adi_a, v.adi_b, v.adi_c, v.adi_d, v.adi_e, v.adi_f, v.adi_g, v.adi_h,
            ]
            .map(|v| AdiPortDevice::Raw(v)),
            primary_controller: v.primary_controller,
            screen: v.screen,
        }
    }
}
