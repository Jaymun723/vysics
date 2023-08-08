use crate::{
    collision::collider::Collider::{CircleCollider, PolygonCollider},
    linalg::Vec2D,
    rigidbody2d::RigidBody2D,
};

use super::narrow::{
    polygon_v_polygon::polygon_v_polygon, sphere_v_polygon::sphere_v_polygon,
    sphere_v_sphere::sphere_v_sphere,
};

#[derive(Debug)]
pub struct ContactManifold<'a> {
    pub a: &'a RigidBody2D,
    pub b: &'a RigidBody2D,

    pub normal: Option<Vec2D>,
    pub points: Vec<(Vec2D, f64)>,
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

    pub fn compute(&mut self) {
        match (&self.a.collider, &self.b.collider) {
            (CircleCollider { .. }, CircleCollider { .. }) => sphere_v_sphere(self),
            (CircleCollider { .. }, PolygonCollider { .. })
            | (PolygonCollider { .. }, CircleCollider { .. }) => sphere_v_polygon(self),
            (PolygonCollider { .. }, PolygonCollider { .. }) => polygon_v_polygon(self),
        }
    }

    pub fn set_colliding(&mut self, normal: Vec2D) {
        self.normal = Some(normal);
    }
    pub fn unset_colliding(&mut self) {
        self.normal = None;
        self.points.clear();
    }
    pub fn add_point(&mut self, point: Vec2D, depth: f64) {
        self.points.push((point, depth))
    }
}
