use crate::{linalg::Vec2D, rigidbody2d::RigidBody2D};

#[derive(Debug)]
pub struct ContactManifold<'a> {
    pub a: &'a RigidBody2D,
    pub b: &'a RigidBody2D,

    pub normal: Option<Vec2D>,
    pub points: Vec<Vec2D>,
}

impl<'a> ContactManifold<'a> {
    pub fn new(a: &'a RigidBody2D, b: &'a RigidBody2D) -> Self {
        ContactManifold {
            a,
            b,
            normal: None,
            points: vec![],
        }
    }
}
