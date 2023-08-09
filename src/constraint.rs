use crate::{linalg::Matrix, rigidbody2d::RigidBody2D};
use derivative::Derivative;

#[derive(Derivative)]
#[derivative(Debug)]
pub struct Constraint<'a> {
    pub dt: f64,
    #[derivative(Debug = "ignore")]
    pub a: &'a RigidBody2D,

    #[derivative(Debug = "ignore")]
    pub b: &'a RigidBody2D,

    pub c_min: f64,
    pub c_max: f64,

    pub jacobian: Matrix,
    pub push_factor: Matrix,

    pub lambda_accumulated: Matrix,
}

impl<'a> Constraint<'a> {
    pub fn new(a: &'a RigidBody2D, b: &'a RigidBody2D, c_min: f64, c_max: f64, dt: f64) -> Self {
        let jacobian = Matrix::zeroes(6, 6);
        let push_factor = Matrix::zeroes(6, 1);
        let lambda_accumulated = Matrix::zeroes(6, 1);

        Self {
            a,
            b,
            dt,
            c_max,
            c_min,
            jacobian,
            lambda_accumulated,
            push_factor,
        }
    }

    fn get_v_1(&self) -> Matrix {
        let mut v_1 = Matrix::zeroes(6, 1);
        v_1[(0, 0)] = self.a.velocity.x;
        v_1[(1, 0)] = self.a.velocity.y;
        v_1[(2, 0)] = self.a.angular_velocity;

        v_1[(3, 0)] = self.b.velocity.x;
        v_1[(4, 0)] = self.b.velocity.y;
        v_1[(5, 0)] = self.b.angular_velocity;

        v_1
    }

    fn get_inverted_mass(&self) -> Matrix {
        let mut m_inv = Matrix::zeroes(6, 6);
        m_inv[(0, 0)] = self.a.get_mass_inv();
        m_inv[(1, 1)] = self.a.get_mass_inv();
        m_inv[(2, 2)] = self.a.get_inertia_inv();

        m_inv[(3, 3)] = self.b.get_mass_inv();
        m_inv[(4, 4)] = self.b.get_mass_inv();
        m_inv[(5, 5)] = self.b.get_inertia_inv();

        m_inv
    }

    pub fn get_lambda(&self) -> Matrix {
        let mut lambda = &self.jacobian * &self.get_v_1();
        lambda *= -1.;

        lambda += &self.push_factor;

        let mut denom = &(&self.jacobian * &self.get_inverted_mass()) * &self.jacobian.transpose();
        denom *= self.dt;

        denom = denom.inv();

        lambda * denom
        // (&self.push_factor - (&self.jacobian * &self.get_v_1()))
        //     * (self.jacobian * self.get_inverted_mass() * self.jacobian.transpose() * self.dt).inv()
    }
}
