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

    fn get_closing_speed_to_target(&self) -> f64;

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
    // returns angle off target's 6'o-clock position
    fn angle_off_tail(&self) -> f64;

    // returns the closing speed
    fn rate_of_closure(&self) -> f64;

    // gets nose heading of mark
    fn bandit_heading(&self) -> f64;

    // act upon target with deadly force
    fn engage_target(&self);

    // time for ship to intercept target
    fn time_to_intercept(&self) -> f64;

    fn fly_to_mark(&self, mark: Vec2, pursuit_type: PursuitGeometry);

    // TODO: improve the next two methods
    // turns ship to focus on target lead coordinates
    fn turn_to_lead_target(&self, lead: Vec2);

    fn basic_maneuver_to_target(&self);
}

enum PursuitGeometry {
    // lag pursuit is primarily used on approach
    // if in lag, you cant shoot the target
    Lag,
    // primarily used during missile engagements
    // flying a pure pursuit course all the way into target will overshoot
    // should only hold pure pursuit when ready to shoot
    Pure,
    // primarily used to close in on bandit and for gun shots
    // a lead course will also be used during intercept runs
    Lead,
}

impl FigherGeometry for Ship {
    // TODO: implement
    fn angle_off_tail(&self) -> f64 {
        -1.0
    }

    // TODO: implement
    fn rate_of_closure(&self) -> f64 {
        -1.0
    }

    // TODO: implement
    fn bandit_heading(&self) -> f64 {
        -1.0
    }

    fn time_to_intercept(&self) -> f64 {
        let delta_position = position() - self.target.as_ref().unwrap().position;
        let delta_velocity = velocity() - self.target.as_ref().unwrap().velocity;
        // TODO: divide by delta velocity length or just my velocity?
        delta_position.length() / velocity().length()
    }

    fn fly_to_mark(&self, mark: Vec2, pursuit_type: PursuitGeometry) {
        match pursuit_type {
            PursuitGeometry::Lag => {
                debug!("fly slightly behind mark");
            },
            PursuitGeometry::Pure => {
                debug!("fly directly to mark");
            },
            PursuitGeometry::Lead => {
                debug!("fly slightly ahead of mark");
            },
        }
    }

    // engage fighter geometry with target
    fn engage_target(&self) {
        if !self.target.is_none() {
            // let lead = get_target_lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity, true);
            // let lead = self.lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity);
            let lead_point = lead(self.target.as_ref().unwrap().position,self.target.as_ref().unwrap().velocity);
            draw_line(self.target.as_ref().unwrap().position, self.target.as_ref().unwrap().velocity, 0xffff00);
            // let mut lead_point: Vec2 = quadratic_lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity);
            // if lead_point == position() {
            //     lead_point = lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity);
            // }
            // if engage {
            draw_line(position(), lead_point, 0xff0000);
            self.turn_to_lead_target(lead_point);
            // } else {
            // draw_line(position(), lead_point, 0x0000ff);
            // }
        }
    }

    fn turn_to_lead_target(&self, lead: Vec2) {
        let current_diff = angle_diff(heading(), lead.angle());
        let distance = self.get_target_distance();
        // if closer turn differently
        // if further, turn differentlyer
        // if distance > 1_500.0 {
        //     if current_diff.abs() > 0.01 {
        //         torque(calculate_angular_velocity(69.0, current_diff));
        //     } else {
        //         turn(calculate_angular_velocity(10_000.0, current_diff));
        //         fire(0);
        //     }
        // } else {
        if current_diff.abs() > 0.1 {
            torque(calculate_angular_velocity(69.0, current_diff));
        } else {
            torque(calculate_angular_velocity(20_000.0, current_diff));
            fire(0);
        }
        // }
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

        let tti = self.time_to_intercept();
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

        let mut new_velocity: Vec2 = Vec2::new(0.0, 0.0);
        let relative_quadrant = self.get_target_position().get_relative_quadrant(position());
        debug!("target in relative quadrant {:?}!", relative_quadrant);

        let closing_speed = self.get_closing_speed_to_target();

        debug!("closing speed: {}", closing_speed);

        // check if ship is moving faster than target
        if velocity().x.abs() > contact_velocity.x.abs() {
            // amount of x to reduce by
            new_velocity.x = velocity().x.abs() - contact_velocity.x.abs();
        } else {
            // amount of x to increase by
            new_velocity.x = contact_velocity.x.abs() - velocity().x.abs();
        }
        if velocity().y.abs() > contact_velocity.y.abs() {
            // amount of y to reduce by
            new_velocity.y = velocity().y.abs() - contact_velocity.y.abs();
        } else {
            // amount of y to increase by
            new_velocity.y = contact_velocity.y.abs() - velocity().y.abs();
        }
        // TODO: might be useful for quadrant specific logic?
        // match self.get_target_position().get_relative_quadrant(position()) {
        //     Quadrant::One => {
        //         // x positive, y positive
        //         debug!("target in relative quadrant 1!");
        //     },
        //     Quadrant::Two => {
        //         // x negative, y positive
        //         debug!("target in relative quadrant 2!");
        //     },
        //     Quadrant::Three => {
        //         // x negative, y negative
        //         debug!("target in relative quadrant 3!");
        //     },
        //     Quadrant::Four => {
        //         // x positive, y negative
        //         debug!("target in relative quadrant 4!");
        //     },
        // }

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

        self.engage_target();
        self.basic_maneuver_to_target();
    }

    pub fn out_of_range_target(&mut self) {
        debug!("target out of range, maneuver closer!");

        // fly ship to target
    }

    pub fn out_of_radar_range(&mut self) {
        debug!("extending radar to maximum distance!");

        // fly ship somewhere
    }

    pub fn heading_to_target(&self, target: Vec2) {
        // turns to target, will be behind a moving target
        turn(angle_diff(heading(), (target - position()).angle()));
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
        self.radar_control();
        self.ship_control();
    }
}

// better but still sucks
fn iterative_approximation(target_position: Vec2, target_velocity: Vec2) -> Vec2 {
    let mut t: f64 = 0.0;
    let mut iterations = 100;
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
fn get_target_lead(target: Vec2, target_velocity: Vec2, debug: bool) -> Vec2 {
    let target_vector = target - position();
    let target_distance = target_vector.length();
    let time_to_target = target_distance / BULLET_SPEED;
    let lead = quadratic_lead(target, target_velocity);
    draw_line(position(), lead, 0xff0000);

    if debug {
        debug!("distance to target: {}", target_vector.length());
        debug!("target vector: {}", target_vector);
        debug!("time to target: {}", time_to_target);
        debug!("target: {}", target);
        debug!("target_velocity: {}", target_velocity);
        debug!("lead: {}", lead);
        debug!("my heading: {}", heading());
        debug!("angular velocity: {}", angular_velocity());
    }
    lead
}

fn calculate_angular_velocity(tune_factor: f64, angle_to_mark: f64) -> f64 {
    let c1: f64 = 2.0 * tune_factor.sqrt();
    tune_factor * angle_to_mark - c1 * angular_velocity()
}

fn turn_to_lead_target_delta_position(lead: Vec2) {
    let current_diff = angle_diff(heading(), (lead - position()).angle());
    // if closer turn differently
    // if further, turn differentlyer
    if current_diff.abs() > 0.1 {
        torque(calculate_angular_velocity(40.0, current_diff));
    } else {
        turn(calculate_angular_velocity(10_000.0, current_diff));
    }
}
fn turn_to_lead_target_from_angle(lead: f64) {
    let current_diff = angle_diff(heading(), lead);
    // if closer turn differently
    // if further, turn differentlyer
    if current_diff.abs() > 0.1 {
        torque(calculate_angular_velocity(420.0, current_diff));
    } else {
        turn(calculate_angular_velocity(10_000.0, current_diff));
    }
}

fn lead(target_position: Vec2, target_velocity: Vec2) -> Vec2 {
    let delta_position = target_position - position();
    let delta_velocity = target_velocity - velocity();
    let prediction = delta_position + delta_velocity * delta_position.length() / BULLET_SPEED;
    prediction
}