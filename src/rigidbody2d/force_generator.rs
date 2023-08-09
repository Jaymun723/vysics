use crate::linalg::Vec2D;

use super::RigidBody2D;

pub trait ForceGenerator {
    fn compute(&self, body: &RigidBody2D) -> (Vec2D, f64);
}

pub struct GravityGenerator {}

impl ForceGenerator for GravityGenerator {
    fn compute(&self, _body: &RigidBody2D) -> (Vec2D, f64) {
        (Vec2D { x: 0., y: 981. }, 0.)
    }
}
