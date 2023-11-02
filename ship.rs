// Tutorial: Radio
// Destroy the enemy ship. Your radar is broken, but a radio signal on channel
// 2 will give you its position and velocity.

use oort_api::prelude::*;
use std::collections::VecDeque;

const BULLET_SPEED: f64 = 1000.0; // m/s
const E: f64 = f64::EPSILON;

#[derive(Debug)]
enum Quadrant {
    One = 1,
    Two = 2,
    Three = 3,
    Four = 4,
}

trait UnitCircleQuadrant {
    fn get_quadrant(&self) -> Quadrant;

    fn get_relative_quadrant(&self, other: Vec2) -> Quadrant;
}

impl UnitCircleQuadrant for Vec2 {
    fn get_quadrant(&self) -> Quadrant {
        if self.x > 0.0 && self.y > 0.0 {
            Quadrant::One
        } else if self.x < 0.0 && self.y > 0.0 {
            Quadrant::Two
        } else if self.x < 0.0 && self.y < 0.0 {
            Quadrant::Three
        } else {
            Quadrant::Four
        }
    }

    // returns relative quadrant of SELF compared to OTHER
    // i.e. target_position.get_relative_quadrant(ship.position)
    fn get_relative_quadrant(&self, other: Vec2) -> Quadrant {
        if self.x > other.x && self.y > other.y {
            Quadrant::One
        } else if self.x < other.x && self.y > other.y {
            Quadrant::Two
        } else if self.x < other.x && self.y < other.y {
            Quadrant::Three
        } else {
            Quadrant::Four
        }
    }
}

enum ShipState {
    NoTarget,
    Searching,
    Engaged,
    OutOfTargetRange,
    OutOfRadarRange,
}

pub struct Radar {
    // count ticks since contact to switch to extended radar sweep
    ticks_since_contact: u32,

    // collect current target positions for time-based calculations
    target_positions: Option<Vec2>,
}

pub struct Ship {
    // does the ship have a target
    target_lock: bool,

    // the current target
    target: Option<ScanResult>,

    // current ship state
    state: ShipState,

    // ship radar component
    radar: Radar,
}

trait RadarControl {
    // main radar control loop
    fn radar_control(&mut self);

    // sets tracking parameters for an acquired target
    fn set_tracking(&mut self, tracking: bool, object: Option<ScanResult>);

    // stop tracking, for any reason
    fn abort_tracking(&mut self);

    // tracks target currently set to Ship.target
    fn radar_tracking(&self);

    // performs a standard radar sweep
    fn standard_radar_sweep(&mut self);

    // performs a long range radar sweep
    fn long_range_radar_sweep(&mut self);

    // returns current target position
    fn get_target_position(&self) -> Vec2;

    // returns distance to target
    fn get_target_distance(&self) -> f64;

    // returns target velocity in x,y
    fn get_target_velocity(&self) -> Vec2;

    // returns x,y values as distance representation to target
    fn get_target_direction(&self) -> Vec2;

    // returns closing speed to target in scalar m/s
    fn get_closing_speed_to_target(&self) -> f64;

    // returns predicted Vec2 of target lead
    fn get_target_lead(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2;

    fn get_target_lead_in_ticks(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2;

    fn get_angle_to_target(&self) -> f64;

    // scan contact handler
    fn radar_scan(&mut self);
}

impl RadarControl for Ship {
    fn set_tracking(&mut self, tracking: bool, object: Option<ScanResult>) {
        self.target_lock = tracking;

        if object.is_none() {
            self.target = None;
            self.set_state(ShipState::Searching);
        } else {
            self.target = Some(ScanResult { ..object.unwrap() });
            self.set_state(ShipState::Engaged);
        }
    }

    fn abort_tracking(&mut self) {
        self.target_lock = false;
        self.target = None;
        self.set_state(ShipState::Searching);
    }

    fn radar_tracking(&self) {
        if self.target_lock {
            let target_distance = self.get_target_distance();
            let radar_heading = (self.get_target_direction()).angle();
            set_radar_heading(radar_heading);

            // focus radar on target
            set_radar_width(PI / target_distance.log(2.0));
            set_radar_max_distance(target_distance + (target_distance * 0.1));
            set_radar_min_distance(target_distance - (target_distance * 0.3));
        }
    }

    fn radar_control(&mut self) {
        self.radar_scan();
        match self.get_state() {
            ShipState::NoTarget => self.standard_radar_sweep(),
            ShipState::Searching => self.standard_radar_sweep(),
            ShipState::Engaged => self.radar_tracking(),
            ShipState::OutOfTargetRange => self.standard_radar_sweep(),
            ShipState::OutOfRadarRange => self.long_range_radar_sweep(),
        }
    }

    fn get_target_position(&self) -> Vec2 {
        self.target.as_ref().unwrap().position
    }

    fn get_target_distance(&self) -> f64 {
        self.get_target_direction().length()
    }

    fn get_target_direction(&self) -> Vec2 {
        self.target.as_ref().unwrap().position - position()
    }

    fn get_target_velocity(&self) -> Vec2 {
        self.target.as_ref().unwrap().velocity
    }

    fn get_closing_speed_to_target(&self) -> f64 {
        -((self.get_target_velocity() - velocity()).dot(self.get_target_direction()) / self.get_target_distance())
    }

    fn get_target_lead(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
        let delta_position = target_position - position();
        let delta_velocity = target_velocity - velocity();
        let prediction = delta_position + delta_velocity * delta_position.length() / BULLET_SPEED;
        prediction
    }

    fn get_target_lead_in_ticks(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
        let delta_position = target_position - position();
        let delta_velocity = (target_velocity - velocity()) / 60.0; // divide down to ticks
        delta_position + delta_velocity * delta_position.length() / (BULLET_SPEED / 60.0).ceil()
    }

    // TODO: broken af
    fn get_angle_to_target(&self) -> f64 {
        // let dir = self.get_target_lead(self.target.as_ref().unwrap().position, self.target.as_ref().unwrap().velocity) - position();
        let prediction_time: f64 = 1.0;
        // experimenting with different approaches
        let dir: Vec2 = self.get_target_position() + prediction_time * self.get_target_velocity() - position();
        dir.y.atan2(dir.x)
    }

    fn standard_radar_sweep(&mut self) {
        // if we've been looking for a while, look harder
        if self.radar.ticks_since_contact > 30 {
            self.set_state(ShipState::OutOfRadarRange);
        }

        set_radar_heading(radar_heading() + radar_width());
        set_radar_width(PI / 2.0);
        set_radar_max_distance(10_000.0);
        set_radar_min_distance(25.0);
    }

    fn long_range_radar_sweep(&mut self) {
        debug!("long range radar sweep");
        set_radar_heading(radar_heading() + radar_width());
        set_radar_width(PI / 8.0);
        set_radar_max_distance(1_000_000.0);
        set_radar_min_distance(25.0);
    }

    fn radar_scan(&mut self) {
        // do scanning, found a contact if we enter block
        if let Some(contact) = scan() {
            // reset contact counter
            self.radar.ticks_since_contact = 0;

            // set tracking object
            self.set_tracking(true, Some(contact.clone()));
        } else {
            debug!("no target, incrementing radar ticks: {}", self.radar.ticks_since_contact);
            // no target this scan, increment ticks
            self.radar.ticks_since_contact += 1;
            self.set_state(ShipState::Searching);
        }

    }
}

trait FigherGeometry {
    // act upon target with deadly force
    fn engage_target(&self);

    // time for ship to intercept target
    fn seconds_to_intercept(&self) -> f64;

    fn ticks_to_intercept(&self) -> f64;

    // TODO: improve the next two methods
    // turns ship to focus on target lead coordinates
    fn turn_to_lead_target(&self, lead: Vec2);
    
    fn heading_to_target(&self, target: Vec2);

    fn basic_maneuver_to_target(&self);
}

impl FigherGeometry for Ship {
    fn seconds_to_intercept(&self) -> f64 {
        let delta_position = position() - self.target.as_ref().unwrap().position;
        let delta_velocity = velocity() - self.target.as_ref().unwrap().velocity;
        // TODO: divide by delta velocity length or just my velocity?
        delta_position.length() / velocity().length()
    }

    fn ticks_to_intercept(&self) -> f64 {
        (position() - self.target.as_ref().unwrap().position).length() / ((velocity() / 60.0) - (self.target.as_ref().unwrap().velocity / 60.0)).length()
    }

    // engage fighter geometry with target
    // TODO: this maybe should be changed to setup an attack orbit
    fn engage_target(&self) {
        if !self.target.is_none() {
            // let lead_point = self.get_target_lead(self.target.as_ref().unwrap().position, self.target.as_ref().unwrap().velocity);
            let lead_point = self.get_target_lead_in_ticks(self.target.as_ref().unwrap().position, self.target.as_ref().unwrap().velocity);

            // draws line from target point to their velocity
            draw_line(self.target.as_ref().unwrap().position, self.target.as_ref().unwrap().velocity, 0xffff00);
            debug!("lead_point: {}", lead_point);
            draw_line(position(), lead_point, 0xff00f0);
            self.turn_to_lead_target(lead_point);
        }
    }

    fn turn_to_lead_target(&self, lead: Vec2) {
        // debug!("lead.angle {}", lead.angle());
        // debug!("self.angle {}", self.get_angle_to_target());
        // let current_diff = angle_diff(heading(), self.get_angle_to_target());
        let current_diff = angle_diff(heading(), lead.angle());
        let distance = self.get_target_distance();
        // if closer turn differently
        // if further, turn differentlyer
        if distance > 500.0 {
            if current_diff.abs() > 0.01 {
                torque(calculate_angular_velocity(69.0, current_diff));
            } else {
                turn(calculate_angular_velocity(50_000.0, current_diff));
                fire(0);
            }
        } else {
            if current_diff.abs() > 0.3 {
                torque(calculate_angular_velocity(50.0, current_diff));
            } else if current_diff.abs() > 0.1 {
                torque(calculate_angular_velocity(1.0, current_diff));
            } else {
                torque(calculate_angular_velocity(5_000.0, current_diff));
                fire(0);
            }
        }
    }

    fn heading_to_target(&self, target: Vec2) {
        // let current_diff = angle_diff(heading(), self.get_angle_to_target());
        let current_diff = angle_diff(heading(), target.angle());
        if current_diff.abs() > 0.01 {
            torque(calculate_angular_velocity(69.0, current_diff));
        } else {
            turn(calculate_angular_velocity(50_000.0, current_diff));
        }
    }
    fn basic_maneuver_to_target(&self) {
        if self.target.is_none() {
            return;
        }
        let contact_distance: f64 = self.get_target_distance();
        let contact_direction: Vec2 = self.get_target_direction();
        let contact_velocity: Vec2 = self.get_target_velocity();
        let contact_position: Vec2 = self.get_target_position();
        let contact_future = contact_position + (contact_velocity);
        let contact_future_distance = (position() - contact_future).length();
        let mut target_distance_increasing = false;

        let tti = self.seconds_to_intercept();
        debug!("time to intercept: {}", tti);

        draw_line(position(), contact_future, 0xff0000);

        if contact_future_distance > contact_distance {
            // target moving relatively away
            debug!("target distance increasing!");
            target_distance_increasing = true;
        } else {
            // target moving relatively closer
            debug!("target distance decreasing!");
            target_distance_increasing = false;
        }

        let normal_vec = contact_direction.normalize();
        debug!("contact direction: {}", contact_direction);
        debug!("contact distance: {}", contact_distance);
        debug!("contact future distance: {}", contact_future_distance);
        debug!("contact normal_vec: {}", normal_vec);

        let relative_quadrant = self.get_target_position().get_relative_quadrant(position());
        debug!("target in relative quadrant {:?}!", relative_quadrant);

        let closing_speed = self.get_closing_speed_to_target();

        debug!("closing speed: {}", closing_speed);

        let time_to_stop: f64 = velocity().length() / max_forward_acceleration();
        debug!("time to stop: {}", time_to_stop);
        debug!("time to stop in ticks: {}", (time_to_stop * 60.0).ceil());

        if time_to_stop < tti {
            // time to stop less than time to intercept, keep going!
            // handle fighter moves based on distance to target
            // current best ranges seem to be [0, 500], [500, 1000], [1000, +]
            if contact_distance < 500.0 {
                // close to target, just float, probably needs to be smarter here
                if target_distance_increasing {
                    accelerate(10.0 * normal_vec);
                } else {
                    accelerate(-10.0 * normal_vec);
                }
            } else if contact_distance > 500.0 && contact_distance < 1000.0 {
                // attempts to match contact motion for combat engagement
                accelerate(10.0 * (contact_velocity));
            } else if contact_distance > 1000.0 {
                // refactored math from target_position - position to pre-calc'd variable of the same
                // need to change to a unit vector in the direction of the target to accelerate
                // back into optimal combat range
                accelerate(100.0 * normal_vec);
            }
        } else {
            // need to figure out how to slow down here
            accelerate(-velocity());
        }
    }
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            target_lock: false,
            target: None,
            state: ShipState::NoTarget,
            radar: Radar {
                ticks_since_contact: 0,
                target_positions: None,
            },
        }
    }

    pub fn set_state(&mut self, state: ShipState) {
        self.state = state;
    }

    pub fn get_state(&self) -> &ShipState {
        &self.state
    }

    // TODO: improve no_target operations
    // no_target might be the ship initializer
    pub fn no_target(&mut self) {
        // pick a random initial vector
        let dir: Vec2 = Vec2::new(rand(-1.0, 1.0), rand(-1.0, 1.0));
        debug!("random dir: {}", dir);
        let mag = 42.20;
        accelerate(dir * mag);

        // set ship to searching for target
        self.set_state(ShipState::Searching);
    }

    pub fn searching_for_target(&mut self) {
        debug!("searching for target");

        // look for a target
    }

    pub fn engaging_target(&mut self) {
        debug!("engaging target");
        // TODO: comment/uncomment to make ship actually work again
        self.basic_maneuver_to_target();
        self.engage_target();

        // TODO: broken stuff below
        // if self.get_target_distance() > 1000.0 {
        //     debug!("maximum course to target");
        //     // target is pretty far, fly course to target at max thrust
        //     let ticks_to_intercept = self.ticks_to_intercept();
        //     debug!("ticks to intercept: {}", ticks_to_intercept);
        //     let velocity_in_ticks = velocity() / 60.0;
        //     debug!("velocity in ticks: {}", velocity_in_ticks);

        //     // gets stopping ticks assuming main thrusters are used to slow down
        //     let minimum_stopping_time = velocity_in_ticks.length() / (max_forward_acceleration() / 60.0);

        //     // gets stopping ticks assuming reverse thrusters are used to slow down
        //     let maximum_stopping_time = velocity_in_ticks.length() / (max_backward_acceleration() / 60.0);
        //     debug!("minimum stopping time: {}", minimum_stopping_time);
        //     if minimum_stopping_time <= ticks_to_intercept && minimum_stopping_time != 0.0 {
        //         debug!("BRAKING!");
        //         // hit earliest time to brake

        //         // flip 180*
        //         turn(-heading());

        //         // get inverse velocity
        //         let inverse_velocity = -velocity();
        //         // scalar inverse_velocity by max forward acceleration
        //         let max_deceleration_velocity = inverse_velocity * max_forward_acceleration();

        //         // maximum thrusters in opposite direction
        //         accelerate(max_deceleration_velocity);
        //     } else {
        //         debug!("heading to target");
        //         let lead = self.get_target_lead_in_ticks(self.get_target_position(), self.get_target_velocity());
        //         self.heading_to_target(lead);
        //         accelerate(self.get_target_direction() * max_forward_acceleration());
        //     }
        // } else if self.get_target_distance() > 500.0 {
        //     // target in firing range, but still pretty far
        // } else {
        //     // close quarters combat
        //     // TODO: orbit calculations, maintain distance, get normal vector of target position
        //     self.basic_maneuver_to_target();
        //     self.engage_target();
        // }
    }

    pub fn out_of_range_target(&mut self) {
        debug!("target out of range, maneuver closer!");

        // fly ship to target
    }

    pub fn out_of_radar_range(&mut self) {
        debug!("extending radar to maximum distance!");

        // fly ship somewhere
    }

    pub fn ship_control(&mut self) {
        match self.get_state() {
            ShipState::NoTarget => self.no_target(),
            ShipState::Searching => self.searching_for_target(),
            ShipState::Engaged => self.engaging_target(),
            ShipState::OutOfTargetRange => self.out_of_range_target(),
            ShipState::OutOfRadarRange => self.out_of_radar_range(),
        }
    }

    pub fn tick(&mut self) {
        // pseudo code for ship loop when target identified in radar scope
        // check acquired target distance
        // check for FoF tags (future)
        // set radar to track ship in (less?) narrow window
        // get ship to optimal firing range (determine)
        // destroy target
        // reset scanner to find next target
        // adjust position to hunting patterns
        
        set_radio_channel(2);
        if let Some(msg) = receive() {
            debug!("msg: {msg:?}");
            let pos = Vec2::new(msg[0], msg[1]);
            let vel = Vec2::new(msg[2], msg[3]);
            let target: Option<ScanResult> = Some(ScanResult {
                position: pos,
                velocity: vel,
                class: Class::Unknown,
                rssi: 42.0,
                snr: 42.0
            });
            self.set_tracking(true, target);
        } else {
            debug!("no message received");
        }

        // self.radar_control();
        self.ship_control();
    }
}

// better but still sucks
fn iterative_approximation(target_position: Vec2, target_velocity: Vec2) -> Vec2 {
    let mut t: f64 = 0.0;
    let mut iterations = 10;
    while iterations > 0 {
        let old_t: f64 = t;
        t = ((target_position - position()) + (t * target_velocity)).length() / BULLET_SPEED;
        if t - old_t < E {
            break;
        }
        iterations = iterations - 1;
    }

    return target_position + (t * target_velocity);
}
// discriminant is the part under the sqrt when solved for x
// d = b^2-4ac
fn get_smallest_quadratic_solution(a: f64, b: f64, c: f64) -> f64 {
    let discriminant: f64 = (b * b) - (4.0 * a * c);

    if discriminant < 0.0 {
        return -1.0;
    }

    // solved for x: x = (-b +/- sqrt(b^2-4ac)) / 2a
    // getting one or both x solutions from above
    // only the positive solutions are useful
    let s: f64 = discriminant.sqrt();
    let x2: f64 = (-b + s) / (2.0 * a);
    let x1: f64 = (-b - s) / (2.0 * a);
    if x2 > 0.0 && x1 > 0.0 {
        return x2.min(x1);
    } else if x1 > 0.0 {
        return x1;
    } else if x2 > 0.0 {
        return x2;
    }
    return -1.0; //no positive solution
}
// uses quadratic math from available info to produce useful lead vector
// remember, quadratic formula is: ax^2+bx+c=0
// solved for x: x = (-b +/- sqrt(b^2-4ac)) / 2a
fn quadratic_lead(target_position: Vec2, target_velocity: Vec2) -> Vec2 {
    let a: f64 = target_velocity.dot(target_velocity) - (BULLET_SPEED * BULLET_SPEED);
    let b: f64 = 2.0 * target_position.dot(target_velocity);
    let c: f64 = target_position.dot(target_position);

    let t: f64 = get_smallest_quadratic_solution(a, b, c);
    if t <= 0.0 {
        return position();
    }
    return target_position + t * target_velocity;
}

fn calculate_angular_velocity(tune_factor: f64, angle_to_mark: f64) -> f64 {
    let c1: f64 = 2.0 * tune_factor.sqrt();
    tune_factor * angle_to_mark - c1 * angular_velocity()
}