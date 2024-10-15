#![no_main]
#![no_std]

use brain_helper::{
    protocol::{self, ToBrain, ToRobot},
    vexide::{self, prelude::*},
};

struct Robot(ComponentHandler);

impl Compete for Robot {
    async fn autonomous(&mut self) {
        let handler = &mut self.0;
        loop {
            if let Some(vec) = brain_helper::read_pkts_serial() {
                for pkt in vec {
                    match pkt {
                        ToBrain::RequestDeviceInfo => {
                            brain_helper::write_pkt_serial(handler.get_device_list());
                        }
                        ToBrain::RequestControllerInfo => {}
                        ToBrain::SetMotors(targets) => handler.set_motors(targets),
                        ToBrain::RequestCompState => {
                            brain_helper::write_pkt_serial(ToRobot::CompState(
                                protocol::CompState::Auton,
                            ));
                        }
                        ToBrain::RequestEncoderState => {}
                        ToBrain::SetMotorGearsets(gearsets) => handler.set_gearsets(gearsets),
                        ToBrain::Ping(v) => {
                            brain_helper::write_pkt_serial(ToRobot::Pong(v));
                        }
                    }
                }
            }

            vexide::async_runtime::time::sleep(core::time::Duration::from_millis(1)).await;
        }
    }

    async fn driver(&mut self) {
        let handler = &mut self.0;
        loop {
            if let Some(vec) = brain_helper::read_pkts_serial() {
                for pkt in vec {
                    match pkt {
                        ToBrain::RequestDeviceInfo => {
                            brain_helper::write_pkt_serial(handler.get_device_list());
                        }
                        ToBrain::RequestControllerInfo => {
                            brain_helper::write_pkt_serial(handler.get_controller_state());
                        }
                        ToBrain::SetMotors(targets) => handler.set_motors(targets),
                        ToBrain::RequestCompState => {
                            brain_helper::write_pkt_serial(ToRobot::CompState(
                                protocol::CompState::Driver,
                            ));
                        }
                        ToBrain::RequestEncoderState => {}
                        ToBrain::SetMotorGearsets(gearsets) => handler.set_gearsets(gearsets),
                        ToBrain::Ping(v) => {
                            brain_helper::write_pkt_serial(ToRobot::Pong(v));
                        }
                    }
                }
            }

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
}

use protocol::{AdiPortState, PortState};
impl SmartPortDevice {
    pub fn get_type(&self) -> PortState {
        match self {
            Self::Empty => PortState::Unplugged,
            Self::Raw(sp) => sp.device_type().into(),
            Self::Motor(_) => PortState::Motor,
            Self::Encoder(_) => PortState::Encoder,
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
            _ => {}
        }
    }
}

pub enum AdiPortDevice {
    Raw(AdiPort),
}
impl AdiPortDevice {
    pub fn get_type(&self) -> AdiPortState {
        match self {
            _ => AdiPortState::Other,
        }
    }
}

pub struct ComponentHandler {
    smart_ports: [SmartPortDevice; 21],
    adi_ports: [AdiPortDevice; 8],
    primary_controller: Controller,
}

impl ComponentHandler {
    pub fn set_motors(&mut self, targets: [protocol::MotorControl; 21]) {
        for (target, motor) in targets.iter().zip(self.smart_ports.iter_mut()) {
            let v: Option<vexide::devices::smart::motor::MotorControl> = (*target).into();
            if let Some(motor_control) = v {
                motor.transform(PortState::Motor);
                if let SmartPortDevice::Motor(m) = motor {
                    m.set_target(motor_control).unwrap();
                }
            }
        }
    }
    pub fn set_gearsets(&mut self, gearsets: [protocol::GearSetChange; 21]) {
        for (gearset_change, motor) in gearsets.iter().zip(self.smart_ports.iter_mut()) {
            let v: Option<vexide::devices::smart::motor::Gearset> = (*gearset_change).into();
            if let Some(gearset) = v {
                motor.transform(PortState::Motor);
                if let SmartPortDevice::Motor(m) = motor {
                    m.set_gearset(gearset).unwrap();
                }
            }
        }
    }
    pub fn get_device_list(&self) -> ToRobot {
        ToRobot::DevicesList(protocol::DevicesList {
            smart_ports: self.smart_ports.each_ref().map(|v| v.get_type()),
            adi_ports: self.adi_ports.each_ref().map(|v| v.get_type()),
        })
    }
    pub fn get_encoder_states(&mut self) -> ToRobot {
        let mut encoder_states = [protocol::EncoderState::None; 21];
        for (i, possible_smart_port) in self.smart_ports.iter_mut().enumerate() {
            if possible_smart_port.get_type() == PortState::Encoder {
                possible_smart_port.transform(PortState::Encoder);
                if let SmartPortDevice::Encoder(encoder) = possible_smart_port {
                    if let Ok(angle) = encoder.angle() {
                        encoder_states[i] = protocol::EncoderState::Radians(angle.as_radians());
                    }
                }
            }
        }
        ToRobot::EncoderState(encoder_states)
    }
    pub fn get_controller_state(&self) -> ToRobot {
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

        ToRobot::ControllerState(controller_state)
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
        }
    }
}
