use derivative::Derivative;

// use self::force_generator::ForceGenerator;
// use crate::collider::*;
// use crate::linalg::{Mat22, Vec2D};

use crate::{
    collision::collider::Collider,
    linalg::{Mat22, Vec2D},
    rigidbody2d::force_generator::ForceGenerator,
};

pub mod force_generator;

#[derive(Derivative)]
#[derivative(Debug)]
pub struct RigidBody2D {
    pub position: Vec2D,
    pub velocity: Vec2D,
    pub angle: f64,
    pub angular_velocity: f64,
    pub collider: Collider,
    // pub is temp
    pub mass: f64,
    pub inertia: f64,

    // pub is temp
    #[derivative(Debug = "ignore")]
    pub force_generators: Vec<Box<dyn ForceGenerator>>,
}

impl Clone for RigidBody2D {
    fn clone(&self) -> Self {
        Self {
            position: self.position.clone(),
            collider: self.collider.clone(),
            angle: self.angle.clone(),
            angular_velocity: self.angular_velocity.clone(),
            force_generators: vec![],
            mass: self.mass.clone(),
            velocity: self.velocity.clone(),
            inertia: self.inertia.clone(),
        }
    }
}

impl RigidBody2D {
    pub fn new(position: Vec2D, collider: Collider, mass: f64) -> Self {
        let inertia = collider.get_inertia();
        Self {
            position,
            collider,
            velocity: Vec2D::zero(),
            angle: 0.,
            angular_velocity: 0.,
            force_generators: vec![],
            mass,
            inertia,
        }
    }

    pub fn from_polygon(vertices: &Vec<Vec2D>, mass: f64) -> Self {
        let collider = Collider::PolygonCollider {
            vertices: vertices.to_vec(),
        };
        let inertia = collider.get_inertia();

        // let position = collider.center();
        let position = Vec2D::zero();

        // collider = Collider::PolygonCollider {
        //     vertices: vertices.iter().map(|v| *v - position).collect(),
        // };

        Self {
            position,
            collider,
            velocity: Vec2D::zero(),
            angle: 0.,
            angular_velocity: 0.,
            force_generators: vec![],
            mass,
            inertia,
        }
    }

    pub fn get_mass_inv(&self) -> f64 {
        if self.mass == 0. {
            0.
        } else {
            1. / self.mass
        }
    }

    pub fn get_inertia_inv(&self) -> f64 {
        if self.inertia == 0. {
            0.
        } else {
            1. / self.inertia
        }
    }

    pub fn add_force_generator(&mut self, force_generator: Box<dyn ForceGenerator>) {
        self.force_generators.push(force_generator);
    }

    pub fn step(&mut self, dt: f64) {
        let mut force_acc = Vec2D::zero();
        let mut torque_acc = 0.;

        for force_generator in self.force_generators.as_slice() {
            // acceleration += force_generator.compute(self);
            let (force, torque) = force_generator.compute(&self);
            force_acc += force;
            torque_acc += torque;
        }

        let acceleration = self.get_mass_inv() * force_acc;

        self.velocity += acceleration * dt;
        self.position += self.velocity * dt;

        // This is SUSSSSSS
        self.angular_velocity += torque_acc * dt;
        self.angle += self.angular_velocity * dt;
    }

    pub fn to_global(&self, point: Vec2D) -> Vec2D {
        let rot = Mat22::from_angle(self.angle);
        self.position + (rot * point)
    }

    pub fn to_local(&self, point: Vec2D) -> Vec2D {
        let rot = Mat22::from_angle(-self.angle);
        rot * point
    }
}
