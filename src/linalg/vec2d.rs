use std::{
    fmt::Display,
    ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign},
};

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vec2D {
    pub x: f64,
    pub y: f64,
}

pub const TOLERANCE: f64 = 1e-3;

impl Vec2D {
    pub fn new(x: f64, y: f64) -> Self {
        Vec2D { x, y }
    }

    pub fn zero() -> Self {
        Vec2D { x: 0., y: 0. }
    }

    pub fn norm(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    pub fn normalize(self) -> Self {
        self / self.norm()
    }

    pub fn squared_norm(&self) -> f64 {
        self.x.powi(2) + self.y.powi(2)
    }

    pub fn cross(&self, other: Vec2D) -> f64 {
        self.x * other.y - self.y * other.x
    }

    pub fn left(self) -> Vec2D {
        Self::new(-self.y, self.x)
    }

    pub fn right(self) -> Vec2D {
        Self::new(self.y, -self.x)
    }

    pub fn to_3d(self) -> crate::linalg::Vec3D {
        crate::linalg::Vec3D {
            x: self.x,
            y: self.y,
            z: 0.,
        }
    }

    pub fn near_zero(&self) -> bool {
        (self.x.abs() < TOLERANCE) && (self.y.abs() < TOLERANCE)
    }

    pub fn is_correct(&self) -> bool {
        !self.x.is_nan() && !self.y.is_nan()
    }
}

impl Display for Vec2D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "({:.4}, {:.4})", self.x, self.y)
    }
}

impl Add for Vec2D {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl Sub for Vec2D {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl Neg for Vec2D {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl AddAssign for Vec2D {
    fn add_assign(&mut self, other: Self) {
        *self = Self {
            x: self.x + other.x,
            y: self.y + other.y,
        };
    }
}

impl SubAssign for Vec2D {
    fn sub_assign(&mut self, rhs: Self) {
        *self = Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        };
    }
}

impl Mul for Vec2D {
    type Output = f64;

    /// Perform the dot product
    fn mul(self, rhs: Self) -> Self::Output {
        (self.x * rhs.x) + (self.y * rhs.y)
    }
}

impl Mul<f64> for Vec2D {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Div<f64> for Vec2D {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl Mul<Vec2D> for f64 {
    type Output = Vec2D;

    fn mul(self, rhs: Vec2D) -> Self::Output {
        Self::Output {
            x: rhs.x * self,
            y: rhs.y * self,
        }
    }
}

impl Div<Vec2D> for f64 {
    type Output = Vec2D;

    fn div(self, rhs: Vec2D) -> Self::Output {
        Self::Output {
            x: rhs.x / self,
            y: rhs.y / self,
        }
    }
}

impl MulAssign<f64> for Vec2D {
    fn mul_assign(&mut self, rhs: f64) {
        *self = Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl DivAssign<f64> for Vec2D {
    fn div_assign(&mut self, rhs: f64) {
        *self = Self {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}
