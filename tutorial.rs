// Tutorial: Lead
// Destroy the enemy ship. Its position is given by the "target" function and velocity by the
// "target_velocity" function. Your ship is not able to accelerate in this scenario.
//
// This is where the game becomes challenging! You'll need to lead the target
// by firing towards where the target will be by the time the bullet gets there.
//
// Hint: target() + target_velocity() * t gives the position of the target after t seconds.
//
// You can scale a vector by a number: vec2(a, b) * c == vec2(a * c, b * c)
//
// p.s. You can change your username by clicking on it at the top of the page.
use oort_api::prelude::*;
use std::collections::VecDeque;

const BULLET_SPEED: f64 = 1000.0; // m/s
const E: f64 = f64::EPSILON;
pub struct Ship {
    target_lock: bool,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            target_lock: false,
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

    // discriminant is the part under the sqrt when solved for x
    // d = b^2-4ac
    pub fn get_smallest_quadratic_solution(&mut self, a: f64, b: f64, c: f64) -> f64 {
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
    pub fn quadratic_lead(&mut self, target_position: Vec2, target_velocity: Vec2) -> Vec2 {
        let a: f64 = target_velocity.dot(target_velocity) - BULLET_SPEED.powf(2.0);
        let b: f64 = 2.0 * target_position.dot(target_velocity);
        let c: f64 = target_position.dot(target_position);

        // seems to be the same math as above, just done without explicit vector maths
        // let local_position = position();
        // let a: f64 = target_velocity.x.powf(2.0) + target_velocity.y.powf(2.0) - BULLET_SPEED.powf(2.0);
        // let b: f64 = 2.0 * (target_velocity.x * (target_position.x - local_position.x) + target_velocity.y * (target_position.y - local_position.y));
        // let c: f64 = (target_position.x - local_position.x).powf(2.0) + (target_position.y - local_position.y).powf(2.0);

        let t: f64 = self.get_smallest_quadratic_solution(a,b,c);
        if t <= 0.0 {
            return position();
        }
        return target_position + t * target_velocity;
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

    pub fn lead_target(&mut self, target: Vec2, target_velocity: Vec2, debug: bool, mut positions: VecDeque<Vec2>) {
        positions.push_back(target);

        let target_vector = target - position();
        let target_distance = target_vector.length();
        let time_to_target = target_distance / BULLET_SPEED;
        let lead = self.quadratic_lead(target, target_velocity);
        draw_line(position(), lead, 0xff0000);
        let current_diff = angle_diff(heading(), (lead - position()).angle());

        // let last_known_position = positions.pop_front();
        // if last_known_position == None {
        //     debug!("last_known_position is None: {}", last_known_position.unwrap());
        //     return;
        // }
        // let tracked_angular_velocity = (lead.y - target.y).atan2(lead.x - target.x) - (target.y - last_known_position.unwrap().y).atan2(target.x - last_known_position.unwrap().x) / (current_time() - TICK_LENGTH);
        // debug!("tracking angular velocity: {}", tracked_angular_velocity);

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

        if debug {
            debug!("distance to target: {}", target_vector.length());
            debug!("target vector: {}", target_vector);
            debug!("time to target: {}", time_to_target);
            debug!("target: {}", target);
            debug!("target_velocity: {}", target_velocity);
            debug!("lead: {}", lead);
            debug!("current diff: {}", current_diff);
            debug!("my heading: {}", heading());
            debug!("angular velocity: {}", angular_velocity());
        }
    }

    pub fn tick(&mut self) {
        // draw_line(position(), target(), 0x00ff00);
        let mut positions: VecDeque<Vec2> = VecDeque::new();
        set_radar_heading(heading());
        // set_radar_width(PI / 4.0);
        // set_radar_max_distance(3000.0);
        // set_radar_min_distance(25.0);
        debug!("ship.target_lock: {}", self.target_lock);
        if let Some(contact) = scan() {
            self.target_lock = true;
            debug!("class: {:?}", contact.class);
            self.lead_target(contact.position, contact.velocity, true, positions);
            accelerate(0.1 * (contact.position - position()));
            fire(0);
        }
        // self.lead_target(target(), true, positions);
        // fire(0);
    }
}
