
use oort_api::prelude::*;
use std::collections::VecDeque;

const BULLET_SPEED: f64 = 1000.0; // m/s
const E: f64 = f64::EPSILON;

pub struct Ship {
    target_lock: bool,
    target: Option<ScanResult>,
}

trait ObjectTracking {
    fn set_tracking(&mut self, tracking: bool, object: Option<ScanResult>);

    fn abort_tracking(&mut self);

    fn track_target(&self, engage: bool);

    fn radar_lock(&self, engage: bool);

    fn get_target_position(&self) -> Vec2;

    fn get_target_distance(&self) -> f64;

    fn standard_radar_sweep(&self);
 
    fn turn_to_lead_target(&self, lead: Vec2);
}

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

impl ObjectTracking for Ship {
    fn set_tracking(&mut self, tracking: bool, object: Option<ScanResult>) {
        self.target_lock = tracking;
        self.target = Some(ScanResult { ..object.unwrap() });
    }

    fn abort_tracking(&mut self) {
        self.target_lock = false;
        self.target = None;
    }

    fn track_target(&self, engage: bool) {
        if let None = self.target {
            return;
        }
        // let lead = get_target_lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity, true);
        // let lead = self.lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity);
        let lead_point = lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity);
        // let mut lead_point: Vec2 = quadratic_lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity);
        // if lead_point == position() {
        //     lead_point = lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity);
        // }
        if engage {
            draw_line(position(), lead_point, 0xff0000);
            self.turn_to_lead_target(lead_point);
        } else {
            draw_line(position(), lead_point, 0x0000ff);
        }
    }

    fn radar_lock(&self, engage: bool) {
        debug!("radar_lock: {}", engage);
        if engage && !self.target.is_none(){
            let diff_to_radar_mark = angle_diff(radar_heading(), (self.target.clone().unwrap().position - position()).angle());
            set_radar_heading((self.target.clone().unwrap().position - position()).angle());

            let targ_dist = self.get_target_distance();
            // focus radar on target
            set_radar_width(PI / targ_dist.log(2.0));
            set_radar_max_distance(targ_dist + (targ_dist * 0.1));
            set_radar_min_distance(targ_dist - (targ_dist * 0.3));
        } else {
            set_radar_heading(radar_heading() + radar_width());

            // set standard radar sweep
            set_radar_width(PI / 2.0);
            set_radar_max_distance(10_000.0);
            set_radar_min_distance(25.0);
        }
    }

    fn get_target_position(&self) -> Vec2 { self.target.clone().unwrap().position }

    fn get_target_distance(&self) -> f64 { (self.target.clone().unwrap().position - position()).length() }

    fn standard_radar_sweep(&self) {
        set_radar_heading(radar_heading() + radar_width());

        // set standard radar sweep
        set_radar_width(PI / 2.0);
        set_radar_max_distance(10_000.0);
        set_radar_min_distance(25.0);
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

    let t: f64 = get_smallest_quadratic_solution(a,b,c);
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

fn distance_between_points(a: Vec2, b: Vec2) -> f64 { (a - b).length() }

impl Ship {
    pub fn new() -> Ship {
        Ship {
            target_lock: false,
            target: None,
        }
    }

    pub fn heading_to_target(&mut self, target: Vec2) {
        // turns to target, will be behind a moving target
        turn(angle_diff(heading(), (target - position()).angle()));
    }

    // better but still sucks
    pub fn iterative_approximation(&mut self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
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

    pub fn tick(&mut self) {

        if let Some(contact) = scan() {
            // pseudo code for ship loop when target identified in radar scope
            // check acquired target distance
            // check for FoF tags (future)
            // set radar to track ship in (less?) narrow window
            // get ship to optimal firing range (determine)
            // destroy target
            // reset scanner to find next target
            // adjust position to hunting patterns

            
            let object = Some(contact.clone());
            self.set_tracking(true, object);
            self.radar_lock(true);
            self.track_target(true);

            let dist = distance_between_points(contact.position, position());
            let unit_vector_to_target = (contact.position - position()).normalize();
            let target_angular_velocity = calculate_angular_velocity(420.0, angle_diff(heading(), (self.get_target_position() - position()).angle()));

            debug!("distance to current target: {}", dist);
            debug!("unit vector: {}", unit_vector_to_target);

            debug!("target - position: {}" , self.get_target_position() - position());
            debug!("target - velocity: {}", contact.velocity);
            debug!("my velocity: {}", velocity());
            debug!("my angular velocity: {}", angular_velocity());
            debug!("my position: {}", position());

            if dist < 500.0 {
                // accelerate(10.0 * (self.get_target_position() + position()));
                accelerate(Vec2::new(0.0, 0.0));
            } else if dist > 500.0 && dist < 1_000.0 {
                accelerate(100.0 * (contact.velocity));
            } else if dist > 1_000.0 {
                accelerate(1_000.0 * (self.get_target_position() - position()));
            }
        } else {
            self.radar_lock(false);
            self.track_target(false);
        }
        // self.lead_target(target(), true, positions);
        // fire(0);
    }
}
