#![no_main]
#![no_std]

use brain_helper::protocol::{self, ToBrain, ToRobot};
use vexide::prelude::*;

struct Robot(Peripherals);

impl Compete for Robot {
    async fn autonomous(&mut self) {
        unimplemented!();
    }

    async fn driver(&mut self) {
        let perfs = &self.0;
        loop {
            match brain_helper::read_pkt_serial() {
                Some(ToBrain::EstablishConnection) => {
                    // send back device list
                    brain_helper::write_pkt_serial(get_device_list(&perfs));
                }
                Some(ToBrain::Ping(v)) => {
                    brain_helper::write_pkt_serial(ToRobot::Pong(v));
                }
                Some(ToBrain::Control) => {}
                _ => {}
            }

            vexide::async_runtime::time::sleep(core::time::Duration::from_millis(1)).await;
        }
    }
}

#[vexide::main(banner = false)]
async fn main(peripherals: Peripherals) {
    let robot = Robot(peripherals);
    robot.compete().await;
}

fn get_device_list(p: &vexide::devices::peripherals::Peripherals) -> ToRobot {
    use vexide::devices::adi::AdiDeviceType;
    use vexide::devices::smart::SmartDeviceType;
    let map_smartport_device = |v: &vexide::devices::smart::SmartPort| -> protocol::PortState {
        match v.device_type() {
            Ok(SmartDeviceType::Motor) => protocol::PortState::Motor,
            Ok(SmartDeviceType::Rotation) => protocol::PortState::Encoder,
            Ok(v) => {
                protocol::PortState::Other(unsafe { *(&v as *const SmartDeviceType as *const u8) })
            }
            Err(_) => protocol::PortState::Unplugged,
        }
    };
    let map_adiport_device = |v: &vexide::devices::adi::AdiPort| -> protocol::AdiPortState {
        match v.configured_type() {
            Ok(AdiDeviceType::Undefined) => protocol::AdiPortState::Undefined,
            Ok(AdiDeviceType::DigitalIn) => protocol::AdiPortState::DigitalIn,
            Ok(AdiDeviceType::DigitalOut) => protocol::AdiPortState::DigitalOut,
            Ok(AdiDeviceType::PwmOut) => protocol::AdiPortState::PwmOut,
            Ok(_) => protocol::AdiPortState::Other,
            Err(_) => protocol::AdiPortState::Unplugged,
        }
    };

    let smart_ports = [
        &p.port_1, &p.port_2, &p.port_3, &p.port_4, &p.port_5, &p.port_6, &p.port_7, &p.port_8,
        &p.port_9, &p.port_10, &p.port_11, &p.port_12, &p.port_13, &p.port_14, &p.port_15,
        &p.port_16, &p.port_17, &p.port_18, &p.port_19, &p.port_10,
    ]
    .map(map_smartport_device);

    let adi_ports = [
        &p.adi_a, &p.adi_b, &p.adi_c, &p.adi_d, &p.adi_e, &p.adi_f, &p.adi_g, &p.adi_h,
    ]
    .map(map_adiport_device);

    ToRobot::DevicesList(protocol::DevicesList {
        smart_ports,
        adi_ports,
    })
}
