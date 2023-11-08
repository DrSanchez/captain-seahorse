// Tutorial: Missiles
// Destroy the enemy ship with your missiles.
// Hint: https://en.wikipedia.org/wiki/Proportional_navigation
//     pub fn tick(&mut self) {
//         if class() == Class::Missile {
//             if let Some(contact) = scan() {
//                 let dp = contact.position - position();
//                 let dv = contact.velocity - velocity();
//                 turn_to(dp.angle());
//                 accelerate(dp + dv);
//                 if dp.length() < 20.0 {
//                     explode();
//                 }
//             }
//         } else {
//             fire(1);
//         }
//     }
// }
use oort_api::prelude::*;
use std::collections::VecDeque;
use std::collections::HashMap;
use std::cell::RefCell;
use std::rc::Rc;

const BULLET_SPEED: f64 = 1000.0; // m/s
const E: f64 = f64::EPSILON;
const STICKY_TARGET_TICKS: u32 = 1;

pub enum Ship {
    Fighter(Fighter),
    Missile(Missile),
}

// game loop router
impl Ship {
    pub fn new() -> Ship {
        match class() {
            Class::Fighter => Ship::Fighter(Fighter::new()),
            Class::Missile => Ship::Missile(Missile::new()),
            _ => { todo!() },//TODO: future stuff
        }
    }
    pub fn tick(&mut self) {
        match self {
            Ship::Fighter(fighter) => { fighter.tick() },
            Ship::Missile(missile) => { missile.tick() },
            _ => { () },//TODO: future ticks
        }
    }
}

struct Missile {
    radio_target: Vec2,
    radar: Radar,
}

impl Missile {
    pub fn new() -> Self {
        Missile {
            radio_target: Vec2::new(0.0, 0.0),
            radar: Radar {
                id_gen: 0,
                potential_targets: HashMap::new(),
                ticks_since_contact: 0,
            }
        }
    }
    pub fn standard_radar_sweep(&mut self) {
        // if we've been looking for a while, look harder
        set_radar_heading(radar_heading() + radar_width());
        set_radar_width(PI / 4.0);
        set_radar_max_distance(10_000.0);
        set_radar_min_distance(25.0);
    }
    pub fn tick(&mut self) {
        // from initial tutorial hint
        self.standard_radar_sweep();
        self.radar.radar_loop();
        if let Some(contact) = scan() {
            let dp = contact.position - position();
            let dv = contact.velocity - velocity();
            turn_to(dp.angle());
            accelerate(100.0 * (dp + dv));
            if dp.length() < 20.0 {
                explode();
            }
        }
    }
}

fn turn_to(target_heading: f64) {
    let heading_error = angle_diff(heading(), target_heading);
    turn(10.0 * heading_error);
}

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

// used to drive general ship behavior
enum ShipState {
    NoTarget,
    Searching,
    Engaged,
    OutOfTargetRange,
    OutOfRadarRange,
}

// used to drive engaged state behavior
enum CombatState {
    Attack,
    Evade,
    Flee,
}

/*******************************
pseudo scratch redesign area

ship contains:
 - radar
 - flight controls
 - situational awareness
 - firing controls
 - radio comms

radar contains:
 - radar control
 - radar tracker
 - collection of tracks

track:
 - position
 - velocity
 - uuid
 - uncertainty / error information

********************************/

// RadarTrack is a specific item formulated by the radar tracker algorithm
// it will be used to house revolving data for the object as well as
// having useful derived data members
#[derive(Debug)]
pub struct RadarTrack {
    // list of scan results tracked for this item
    // these need to be parsed and filtered
    scans: VecDeque<ScanResult>,

    // resolved position estimate
    position: Vec2,

    // resolved velocity estimate
    velocity: Vec2,

    // velocity.y.atan2(velocity.x)
    heading: f64,
    
    // need to create a uuid
    id: u128,

    // track classification
    class: TrackType,

    // gate is the predicted target fence for position estimates
    gate: RadarTrackGate,

    // lifetime manager
    contact_tick: u32,


}

// classifier to apply to a RadarTrack
#[derive(Debug)]
enum TrackType {
    Tentative,
    Friend,
    Foe,
    Missile,

}

// geometry helpers specific to a single RadarTrack
trait RadarTrackGeometry {
    fn heading(&self) -> f64;

    fn push_plot(&mut self, plot: Option<ScanResult>);

    fn update(&mut self);

    fn check_gate(&mut self, point: Vec2) -> bool;

    fn distance_from(&self, point: Vec2) -> f64;
}

impl RadarTrackGeometry for RadarTrack {
    fn heading(&self) -> f64 {
        self.velocity.y.atan2(self.velocity.x)
    }

    fn push_plot(&mut self, plot: Option<ScanResult>) {
        self.scans.push_back(plot.unwrap());
    }

    fn update(&mut self) {
        if self.scans.is_empty() {
            // no new scans in queue, just update one tick of velocity
            debug!("using estimated velocity");
            self.position += self.velocity / 60.0;
        } else {
            // we have scans to consider
            if self.scans.len() == 1 {
                debug!("one scans to consider for radartrack: {}", self.id);
                // only one element front and back are the same here
                let scan = self.scans.pop_front().unwrap();
                debug!("scan position: {}", scan.position);

                // cur_vel(t-1) - scan.vel(t) => delta_vel
                // delta_vel needs to be in ticks as well / 2 ticks => 
                let current_velocity_in_ticks = self.velocity  / 60.0;
                let acceleration = (current_velocity_in_ticks - (scan.velocity / 60.0)) / 2.0;
                // ^^ acceleration should be in meters / second / tick (m/s/t)
                // add velocity in ticks with new acceleration, mult*60.0 should convert back to meters / second
                let new_velocity = (current_velocity_in_ticks + acceleration) * 60.0;
                debug!("old velocity: {}", self.velocity);
                debug!("new velocity: {}", new_velocity);
                self.velocity = new_velocity;
                // add acceleration experienced in the last tick to the current estimated position
                self.position += acceleration;
            } else {
                debug!("multiple scans to consider for radartrack: {}", self.id);
                // multiple scans case
                // TODO: can this happen? means update wasnt called on this for multiple ticks
            }
        }

        // done processing, update RadarTrackGate::center
        self.gate.update_center(self.position);
    }

    fn check_gate(&mut self, point: Vec2) -> bool {
        self.gate.point_in_gate(point)
    }

    fn distance_from(&self, point: Vec2) -> f64 {
        (self.position - point).length()
    }
}

// defines a square field for a given radartrack
#[derive(Debug)]
pub struct RadarTrackGate {
    // scalar value (in meters) for how big a window we think the position may be in
    // determines corners from point
    error_magnitude: f64,

    center: Vec2,

    radius: f64,
}

impl RadarTrackGate {
    pub fn new(point: Vec2, radius: f64) -> RadarTrackGate {
        RadarTrackGate {
            error_magnitude: 10.0,
            center: point,
            radius,
        }
    }
    pub fn draw_gate(&self, id: u128) {
        draw_square(self.center, self.radius, 0xff0000);
        let p4: Vec2 = Vec2::new(self.center.x + self.radius / 2.0, self.center.y - self.radius / 2.0);
        draw_text!(p4, 0xff0000, "id: {}", id);
    }

    pub fn update_center(&mut self, center: Vec2) {
        self.center = center;
    }

    // find current points of square based on radius or error magnitude
    pub fn point_in_gate(&self, point: Vec2) -> bool {
        let p1: Vec2 = Vec2::new(self.center.x + self.radius / 2.0, self.center.y + self.radius / 2.0);
        let p2: Vec2 = Vec2::new(self.center.x - self.radius / 2.0, self.center.y + self.radius / 2.0);
        let p3: Vec2 = Vec2::new(self.center.x - self.radius / 2.0, self.center.y - self.radius / 2.0);
        let p4: Vec2 = Vec2::new(self.center.x + self.radius / 2.0, self.center.y - self.radius / 2.0);

        if point.x >= p1.x || point.y >= p1.y {
            return false;
        }
        if point.x <= p2.x || point.y >= p2.y {
            return false;
        }
        if point.x <= p3.x || point.y <= p3.y {
            return false;
        }
        if point.x >= p4.x || point.y <= p4.y {
            return false;
        }
        true
    }
}

pub struct Radar {
    // count ticks since contact to switch to extended radar sweep
    ticks_since_contact: u32,

    // collect current target positions for time-based calculations
    potential_targets: HashMap<u128, Rc<RefCell<RadarTrack>>>,

    // simple unsigned integer id to use for uuids
    id_gen: u128,
}

trait RadarTracker {
    // main loop
    fn radar_loop(&mut self);
    
    // handle unique id creation
    fn new_id_gen(&mut self) -> u128;

    fn has_contacts(&self) -> bool;
    
    fn update_tracks(&mut self);

    fn show_tracks(&self);
    
    fn insert_new_potential_target(&mut self, plot: Option<ScanResult>);

    // used to add a new ScanResult plot to the potential_targets data
    fn add_detection_point(&mut self, plot: Option<ScanResult>);

    fn get_closest_target_to_point(&self, point: Vec2) -> u128;

    fn get_track(&self, id: u128) -> Rc<RefCell<RadarTrack>>;
}

// impl against Radar struct to remove dependency on Ship
impl RadarTracker for Radar {
    fn radar_loop(&mut self) {
        self.update_tracks();
        self.show_tracks();
        if let Some(plot) = scan() {
            self.add_detection_point(Some(plot));
        }
    }

    // use current value as next, then increment id counter
    fn new_id_gen(&mut self) -> u128 {
        let next = self.id_gen;
        self.id_gen += 1;
        debug!("new_id_gen: next: {}, incremented: {}", next, self.id_gen);
        next
    }

    fn has_contacts(&self) -> bool {
        !self.potential_targets.is_empty()
    }

    fn insert_new_potential_target(&mut self, plot: Option<ScanResult>) {
        let mut scans: VecDeque<ScanResult> = VecDeque::new();
        debug!("insert_new_potential_target: new plot position: {}", plot.as_ref().unwrap().position);
        scans.push_back(plot.clone().unwrap());
        let id = self.new_id_gen();
        // populate initial RadarTrack with baseline values
        let track = Rc::new(RefCell::new(RadarTrack {
            scans,
            position: plot.as_ref().unwrap().position,
            velocity: plot.as_ref().unwrap().velocity,
            heading: plot.as_ref().unwrap().velocity.y.atan2(plot.as_ref().unwrap().velocity.x),
            id,
            class: TrackType::Tentative,
            gate: RadarTrackGate::new(plot.as_ref().unwrap().position, 50.0),
            contact_tick: current_tick(),
        }));
        self.potential_targets.insert(id, track);
    }

    fn show_tracks(&self) {
        for (id, track) in &self.potential_targets {
            track.borrow().gate.draw_gate(*id);
        }
    }

    // iterate over existing tracks and call their update method
    fn update_tracks(&mut self) {
        for (id, track) in &self.potential_targets {
            track.borrow_mut().update();
        }
    }
    fn get_closest_target_to_point(&self, point: Vec2) -> u128 {
        let mut distance: f64 = 90_000_000.0;
        let mut target_id: u128 = 0;

        // iterate over potential targets looking for 
        for (id, track) in &self.potential_targets {
            let dist = track.borrow().distance_from(point);
            if dist < distance {
                distance = dist;
                target_id = *id;
            }
        }
        target_id
    }

    fn get_track(&self, id: u128) -> Rc<RefCell<RadarTrack>> {
        self.potential_targets.get(&id).unwrap().clone()
    }

    fn add_detection_point(&mut self, plot: Option<ScanResult>) {
        debug!("adding detection point");
        debug!("potential_targets.len: {}", self.potential_targets.len());
        if self.potential_targets.is_empty() {
            // first result, no values to compare with
            self.insert_new_potential_target(plot);
        } else {
            let mut found = false;
            let mut found_id = 0;
            let mut old_tracks: Vec<u128> = Vec::new();
            // TODO: improve detection point association
            // check radartracks for potential match
            for (id, track) in &self.potential_targets {
                if found {
                    break;
                }
                let mut t = track.borrow_mut();
                if t.check_gate(plot.as_ref().unwrap().position) {
                    debug!("associating new plot with existing target");
                    found = true;
                    // update current track with new data
                    t.push_plot(plot.clone());
                    t.update();
                } else {
                    // check current track lifetime
                    let delta_tick: f64 = (current_tick() - t.contact_tick).into();

                    // check if num ticks hits 2 second window, remove outdated track
                    if delta_tick >= 30.0 {
                    // if delta_tick / 60.0 >= 1.0 {
                        debug!("adding old_track id: {}", id);
                        old_tracks.push(*id);
                    }
                }
            }
            // clear out of date tracks
            for i in &old_tracks {
                self.potential_targets.remove(i);
            }
            old_tracks.clear();
            if !found {
                // new potential target discovered
                debug!("new target discovered");
                self.insert_new_potential_target(plot);
            }
        }
    }
}


pub struct Radio {
    // current radio channel
    current_channel: u8,

    // queue of messages to process
    message_queue: VecDeque<String>,
}

pub struct Fighter {
    // does the ship have a target
    target_lock: bool,

    target: Option<Rc<RefCell<RadarTrack>>>,

    // current ship state
    state: ShipState,

    radio: Radio,

    // ship radar component
    radar: Radar,

    sticky_target_ticks: u32,

    rotation: Rotator,

    // TODO:
    // lateral_throttle
    // longitudinal_throttle
}

pub struct Rotator {
    // current movement estimated ticks to accomplish desired rotation
    estimated_ticks_to_angle: u32,
    throttle: f64,
}

trait FigherGeometry {
    fn shoot(&self);

    fn fly_to_target(&self);

    fn engage_target(&mut self);

    fn seconds_to_intercept(&self) -> f64;

    fn ticks_to_intercept(&self) -> f64;

    fn turn_to_lead_target(&self, lead: Vec2);
    fn turn_to_lead_target_aggressive(&self, lead: Vec2);
    
    fn heading_to_target(&self, target: Vec2);

    fn basic_maneuver_to_target(&self);

    fn set_current_target(&mut self, target: Rc<RefCell<RadarTrack>>);

    fn quadratic_lead(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2;
}

impl FigherGeometry for Fighter {
    fn shoot(&self) {
        if self.get_target_distance() < 1000.0 {
            fire(0);
        }
    }
    
    fn seconds_to_intercept(&self) -> f64 {
        let delta_position = position_fixed() - self.target.as_ref().unwrap().borrow().position;
        let _delta_velocity = velocity() - self.target.as_ref().unwrap().borrow().velocity;
        delta_position.length() / velocity().length()
    }

    fn ticks_to_intercept(&self) -> f64 {
        (position_fixed() - self.target.as_ref().unwrap().borrow().position).length() / ((velocity() / 60.0) - (self.target.as_ref().unwrap().borrow().velocity / 60.0)).length()
    }
    fn fly_to_target(&self) {
        self.turn_to_lead_target(self.get_target_direction());
    }

    // uses quadratic math from available info to produce useful lead vector
    // remember, quadratic formula is: ax^2+bx+c=0
    // solved for x: x = (-b +/- sqrt(b^2-4ac)) / 2a
    fn quadratic_lead(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
        let a: f64 = target_velocity.dot(target_velocity) - (BULLET_SPEED * BULLET_SPEED);
        let b: f64 = 2.0 * target_position.dot(target_velocity);
        let c: f64 = target_position.dot(target_position);

        let t: f64 = get_smallest_quadratic_solution(a, b, c);
        if t <= 0.0 {
            return self.get_adjusted_target_lead_in_ticks(target_position, target_velocity);
            // return position_fixed();
        }
        return target_position + t * target_velocity;
    }
    // engage fighter geometry with target
    // TODO: this maybe should be changed to setup an attack orbit
    fn engage_target(&mut self) {
        if self.target.is_some() {

            // TODO: still no idea which of these works best / least worst
            // let lead_point = self.quadratic_lead(self.target.as_ref().unwrap().borrow().position, self.target.as_ref().unwrap().borrow().velocity);
            let lead_point = self.get_target_lead_in_ticks(self.target.as_ref().unwrap().borrow().position, self.target.as_ref().unwrap().borrow().velocity);
            // let lead_point = self.get_adjusted_target_lead_in_ticks(self.target.as_ref().unwrap().borrow().position, self.target.as_ref().unwrap().borrow().velocity);
            draw_triangle(self.target.as_ref().unwrap().borrow().position, 50.0, 0x00ff00);
            draw_line(position_fixed(), lead_point, 0xff00f0);

            // TODO: fighter is dumb and flies straight at target which usually wins in the fight
            if self.get_target_distance() < 1000.0 {
                self.turn_to_lead_target_aggressive(lead_point);
            } else {
                self.fly_to_target();
            }
        }
    }

    fn set_current_target(&mut self, target: Rc<RefCell<RadarTrack>>) {
        self.target = Some(target);
    }

    // NOTE: just write a new method, this works well enough now
    fn turn_to_lead_target(&self, lead: Vec2) {
        let current_diff = angle_diff(heading(), lead.angle());
        if current_diff.abs() > 0.1 {
            let next_ang_v = calculate_angular_velocity(50.0, current_diff);
            debug!("turning angle velocity: {}", next_ang_v);
            torque(next_ang_v);
        } else {
            let next_ang_v = calculate_angular_velocity(1_000.0, current_diff);
            debug!("firing angle velocity: {}", next_ang_v);
            torque(next_ang_v);
            self.shoot();
        }
    }
        // NOTE: just write a new method, this works well enough now
    fn turn_to_lead_target_aggressive(&self, lead: Vec2) {
        let current_diff = angle_diff(heading(), lead.angle());
        if current_diff.abs() > 0.1 {
            let next_ang_v = calculate_angular_velocity(4.0, current_diff);
            debug!("turning angle velocity: {}", next_ang_v);
            torque(next_ang_v);
        } else {
            let next_ang_v = calculate_angular_velocity(10.0, current_diff);
            debug!("firing angle velocity: {}", next_ang_v);
            torque(next_ang_v);
            self.shoot();
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
        let contact_future_distance = (position_fixed() - contact_future).length();
        let mut target_distance_increasing = false;

        let tti = self.seconds_to_intercept();
        debug!("time to intercept: {}", tti);

        draw_line(position_fixed(), contact_future, 0xff0000);

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

        let relative_quadrant = self.get_target_position().get_relative_quadrant(position_fixed());
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

fn position_fixed() -> Vec2 {
    position() - vec2(1.0, 0.0).rotate(heading()) * 1.33333333
}
impl Fighter {
    pub fn new() -> Self {
        Fighter {
            target_lock: false,
            target: None,
            state: ShipState::NoTarget,
            radio: Radio {
                current_channel: 0,
                message_queue: VecDeque::new(),
            },
            radar: Radar {
                ticks_since_contact: 0,
                potential_targets: HashMap::new(),
                id_gen: 0,
            },
            sticky_target_ticks: STICKY_TARGET_TICKS,
            rotation: Rotator {
                estimated_ticks_to_angle: 0,
                throttle: 0.0,
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
    }

    pub fn searching_for_target(&mut self) {
        debug!("searching for target");

        // look for a target
    }

    pub fn engaging_target(&mut self) {
        debug!("engaging target");

        // TODO: 
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
        // TODO: hmmm
        match self.get_state() {
            ShipState::NoTarget => self.no_target(),
            ShipState::Searching => self.searching_for_target(),
            ShipState::Engaged => self.engaging_target(),
            ShipState::OutOfTargetRange => self.out_of_range_target(),
            ShipState::OutOfRadarRange => self.out_of_radar_range(),
        }
        
        self.radar.radar_loop();
        if self.radar.has_contacts() {
            match self.get_state() {
                ShipState::Engaged => { () },
                _ => { self.set_state(ShipState::Engaged); }
            }
            
            if self.target.is_some() {
                if self.sticky_target_ticks > 0 {
                    debug!("sticky ticks remaining: {}", self.sticky_target_ticks);
                    self.sticky_target_ticks -= 1;
                } else {
                    debug!("setting new target");
                    self.sticky_target_ticks = STICKY_TARGET_TICKS;
                    debug!("setting latest target values");
                    let id = self.radar.get_closest_target_to_point(position_fixed());
                    let track = self.radar.get_track(id);
                    self.set_current_target(track);
                }
            } else {
                // first target
                debug!("getting FIRST target");
                let id = self.radar.get_closest_target_to_point(position_fixed());
                    let track = self.radar.get_track(id);
                    self.set_current_target(track);
            }
        }
        self.radar_control();
    }

    pub fn snap_to_heading(&mut self, angle: f64) {
        debug!("heading: {}", heading());
        debug!("target angle: {}", angle);
        let delta = angle_diff(heading(), angle);

        // amount of radians for max acceleration
        let halfway = delta / 2.0;

        debug!("delta angle: {}", delta);

        let max_angular_acceleration_ticks = max_angular_acceleration() / 60.0;
        debug!("max angular acceleration: {}", max_angular_acceleration());
        debug!("max angular acceleration in ticks: {}", max_angular_acceleration_ticks);

        // set latest estimate for updated heading
        self.rotation.estimated_ticks_to_angle = (delta.abs() / max_angular_acceleration_ticks).ceil() as u32;
        debug!("estimated ticks to angle: {}", self.rotation.estimated_ticks_to_angle);
        let future_heading: f64 = 0.0;
        // while future_heading

        // latest ticks to stop
        let ticks_to_stop = ((angular_velocity().abs() / 60.0) / (max_angular_acceleration() / 60.0)).ceil();

        if ticks_to_stop < self.rotation.estimated_ticks_to_angle as f64 {
            self.rotation.throttle = 1.0 * delta.signum();
        } else {
            self.rotation.throttle = -1.0 * delta.signum();
        }

        // note: this correctly defines future heading per tick, so far so good
        // let future_heading = heading() + (angular_velocity() / 60.0);
        // debug!("future heading: {}", future_heading);

        debug!("angular velocity: {}", angular_velocity());

        // if delta > 0.0 {
        //     // positive delta, turning ccw

        //     if future_heading < angle {
        //         // future heading will be beyond target angle
        //         self.rotational_throttle = 1.0; // counter throttle
        //     } else {
        //         self.rotational_throttle = -1.0;
        //     }
        // } else {
        //     // negative delta, turning cw
        //     if future_heading > angle {
        //         // future heading will be beyond target angle
        //         self.rotational_throttle = -1.0; // counter throttle
        //     } else {
        //         self.rotational_throttle = 1.0; // counter throttle
        //     }
        // }
        // e.g. delta => -0.14 radians
        // current_velocity in radians per second per second per tick, e.g. => 0.0
        // throttle => -1.0
        // 

        // fastest possible snap to heading
        // count number of ticks to rotate to target
            // based on current angular velocity
        // find number of ticks to stop on target at max deceleration

        torque(self.rotation.throttle * max_angular_acceleration());
    }

    pub fn missile_control(&mut self) {
        // self.radar.radar_loop();
        // self.fly_to_target();
        if let Some(contact) = scan() {
            let dp = contact.position - position();
            let dv = contact.velocity - velocity();
            turn_to(dp.angle());
            accelerate(dp + dv);
            if dp.length() < 20.0 {
                explode();
            }
        }
    }

    pub fn tick(&mut self) {
        // debug!("target: {}", target());
        // debug!("target velocity: {}", target_velocity());
        // draw_line(position_fixed(), target(), 0xffffff);
        // draw_line(target(), target() + target_velocity(), 0xff0000);


        // for completeness even though velocity() is 0,0 for this tutorial
        let delta_velocity = target_velocity() - velocity();
        let bullet_delta = delta_velocity - BULLET_SPEED;
        debug!("bullet delta: {}", bullet_delta);
        let ab = (target() - position_fixed()).length();
        let cat = target() + target_velocity();
        let bc = cat.length();
        draw_line(position_fixed(), cat, 0x0000ff);

        // TODO: this seems to be quadrant dependant
        let head = Vec2::new(heading().cos(), heading().sin());
        draw_line(position_fixed(), head*1000.0, 0xff0000);

        // self.snap_to_heading(cat.angle());
        // draw_line(position_fixed(), 69.0*Vec2::new(cat.angle().cos(), cat.angle().sin()), 0x0f0fff);
        // turn(270.0 * angle_diff(heading(), cat.angle()));

        // fire(0);
        // pseudo code for ship loop when target identified in radar scope
        // check acquired target distance
        // check for FoF tags (future)
        // set radar to track ship in (less?) narrow window
        // get ship to optimal firing range (determine)
        // destroy target
        // reset scanner to find next target
        // adjust position to hunting patterns

        match class() {
            Class::Missile => {
                self.missile_control();
            },
            Class::Fighter => {
                self.ship_control();

                // if we have a target just start launching missiles!
                if self.target.is_some() {
                    fire(1);
                }
            },
            _ => {
                ()
            }
        }
    }
}

// better but still sucks
fn iterative_approximation(target_position: Vec2, target_velocity: Vec2) -> Vec2 {
    let mut t: f64 = 0.0;
    let mut iterations = 10;
    while iterations > 0 {
        let old_t: f64 = t;
        t = ((target_position - position_fixed()) + (t * target_velocity)).length() / BULLET_SPEED;
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

fn calculate_angular_velocity(tune_factor: f64, angle_to_mark: f64) -> f64 {
    let c1: f64 = 2.0 * tune_factor.sqrt();
    tune_factor * angle_to_mark - c1 * angular_velocity()
}
trait RadarControl {
    // main radar control loop
    fn radar_control(&mut self);
    // tracks target currently set to Ship.target
    fn lock_radar_to_target(&self);
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
    // returns predicted Vec2 of target lead in seconds
    fn get_target_lead(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2;
    // returns predicted Vec2 of target lead in ticks
    fn get_target_lead_in_ticks(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2;
    fn get_adjusted_target_lead_in_ticks(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2;
}

impl RadarControl for Fighter {
    fn lock_radar_to_target(&self) {
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
        match self.get_state() {
            ShipState::NoTarget => self.standard_radar_sweep(),
            ShipState::Searching => self.standard_radar_sweep(),
            ShipState::Engaged => self.standard_radar_sweep(),
            ShipState::OutOfTargetRange => self.standard_radar_sweep(),
            ShipState::OutOfRadarRange => self.long_range_radar_sweep(),
        }
    }

    fn get_target_position(&self) -> Vec2 {
        self.target.as_ref().unwrap().borrow().position
    }

    fn get_target_distance(&self) -> f64 {
        self.get_target_direction().length()
    }

    fn get_target_direction(&self) -> Vec2 {
        self.target.as_ref().unwrap().borrow().position - position_fixed()
    }

    fn get_target_velocity(&self) -> Vec2 {
        self.target.as_ref().unwrap().borrow().velocity
    }

    fn get_closing_speed_to_target(&self) -> f64 {
        -((self.get_target_velocity() - velocity()).dot(self.get_target_direction()) / self.get_target_distance())
    }

    fn get_target_lead(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
        let delta_position = target_position - position_fixed();
        let delta_velocity = target_velocity - velocity();
        let prediction = delta_position + delta_velocity * delta_position.length() / BULLET_SPEED;
        prediction
    }

    fn get_target_lead_in_ticks(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
        let delta_position = target_position - position_fixed();
        let delta_velocity = (target_velocity - velocity()) / 60.0; // divide down to ticks
        delta_position + delta_velocity * delta_position.length() / (BULLET_SPEED / 60.0).ceil()
    }

    fn get_adjusted_target_lead_in_ticks(&self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
        let delta_position = target_position - position_fixed();
        let delta_velocity = (target_velocity - velocity()) / 60.0; // divide down to ticks
        let bullet_delta = BULLET_SPEED - target_velocity;
        delta_position + delta_velocity * delta_position.length() / (bullet_delta / 60.0)
    }

    fn standard_radar_sweep(&mut self) {
        // if we've been looking for a while, look harder
        if self.radar.ticks_since_contact > 30 {
            self.set_state(ShipState::OutOfRadarRange);
        }

        set_radar_heading(radar_heading() + radar_width());
        set_radar_width(PI / 4.0);
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
}