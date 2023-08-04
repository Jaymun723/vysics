use std::ops::Mul;

use super::Vec2D;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Mat22 {
    pub m11: f64,
    pub m12: f64,
    pub m21: f64,
    pub m22: f64,
}

impl Mat22 {
    pub fn from_angle(angle: f64) -> Self {
        Mat22 {
            m11: angle.cos(),
            m12: -angle.sin(),
            m21: angle.sin(),
            m22: angle.cos(),
        }
    }

    pub fn new(m11: f64, m12: f64, m21: f64, m22: f64) -> Self {
        Mat22 { m11, m12, m21, m22 }
    }
}

impl Mul<Vec2D> for Mat22 {
    type Output = Vec2D;

    fn mul(self, rhs: Vec2D) -> Vec2D {
        Vec2D {
            x: self.m11 * rhs.x + self.m12 * rhs.y,
            y: self.m21 * rhs.x + self.m22 * rhs.y,
        }
    }
}
