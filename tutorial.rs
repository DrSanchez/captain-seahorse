
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
}

impl ObjectTracking for Ship {
    fn set_tracking(&mut self, tracking: bool, object: Option<ScanResult>) {
        self.target_lock = tracking;
        self.target = Some(ScanResult { ..object.unwrap() });
        debug!("Ship tracking: {}", self.target_lock);
    }

    fn abort_tracking(&mut self) {
        self.target_lock = false;
        self.target = None;
    }

    fn track_target(&self, engage: bool) {
        if let None = self.target {
            return;
        }
        let lead = get_target_lead(self.target.clone().unwrap().position, self.target.clone().unwrap().velocity, true);
        if engage {
            draw_line(position(), lead, 0xff0000);
            turn_to_lead_target(lead);
        } else {
            draw_line(position(), lead, 0x0000ff);
        }
    }

    fn radar_lock(&self, engage: bool) {
        if let None = self.target {
            return;
        }
        debug!("radar_lock: {}", engage);
        if engage {
            set_radar_heading(angle_diff(radar_heading(), (self.target.clone().unwrap().position - position()).angle()));

            // focus radar on target
            set_radar_width(PI / 8.0);
            set_radar_max_distance(2000.0);
            set_radar_min_distance(25.0);
        } else {
            set_radar_heading(radar_heading() + radar_width());

            // set standard radar sweep
            set_radar_width(PI / 2.0);
            set_radar_max_distance(5000.0);
            set_radar_min_distance(25.0);
        }
    }

    fn get_target_position(&self) -> Vec2 { self.target.clone().unwrap().position }
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
    let a: f64 = target_velocity.dot(target_velocity) - BULLET_SPEED.powf(2.0);
    let b: f64 = 2.0 * target_position.dot(target_velocity);
    let c: f64 = target_position.dot(target_position);

    // seems to be the same math as above, just done without explicit vector maths
    // let local_position = position();
    // let a: f64 = target_velocity.x.powf(2.0) + target_velocity.y.powf(2.0) - BULLET_SPEED.powf(2.0);
    // let b: f64 = 2.0 * (target_velocity.x * (target_position.x - local_position.x) + target_velocity.y * (target_position.y - local_position.y));
    // let c: f64 = (target_position.x - local_position.x).powf(2.0) + (target_position.y - local_position.y).powf(2.0);

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

fn turn_to_lead_target(lead: Vec2) {
    let current_diff = angle_diff(heading(), (lead - position()).angle());

    if current_diff.abs() > 0.1 {
        let c0: f64 = 40.0;
        let c1: f64 = 2.0 * c0.sqrt();

        let angular_acceleration: f64 = c0 * current_diff - c1 * angular_velocity();
        torque(angular_acceleration);
    } else {
        let c0: f64 = 10_000.0;
        let c1: f64 = 2.0 * c0.sqrt();

        let angular_acceleration: f64 = c0 * current_diff - c1 * angular_velocity();
        turn(angular_acceleration);
    }
}

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


    // get unit circle quadrant id from object position
    pub fn which_quadrant(&mut self, object: Vec2) -> u8 {
        if object.x > 0.0 && object.y > 0.0 {
            return 1;
        } else if object.x < 0.0 && object.y > 0.0 {
            return 2;
        } else if object.x < 0.0 && object.y < 0.0 {
            return 3;
        } else if object.x > 0.0 && object.y < 0.0 {
            return 4;
        } else {
            // should never see
            return 0;
        }
    }


    pub fn lead(&mut self, target_position: Vec2, target_velocity: Vec2) -> f64 {
        let delta_position = target_position - position();
        let delta_velocity = target_velocity - velocity();
        let prediction = delta_position + delta_velocity * delta_position.length() / BULLET_SPEED;
        prediction.angle()
    }

    pub fn tick(&mut self) {
        // draw_line(position(), target(), 0x00ff00);
        // let mut positions: VecDeque<Vec2> = VecDeque::new();

        // debug!("ship.target_lock: {}", self.target_lock);
        if let Some(contact) = scan() {
            self.set_tracking(true, Some(contact));
            self.radar_lock(true);
            self.track_target(true);
            // self.lead_target(contact.position, contact.velocity, true, positions);
            // let something = angle_diff(heading(), targ_heading);
            accelerate(0.1 * (self.get_target_position() - position()));
            fire(0);
        } else {
            self.radar_lock(false);
            self.track_target(false);
        }
        // self.lead_target(target(), true, positions);
        // fire(0);
    }
}
