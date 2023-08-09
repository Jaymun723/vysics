use crate::linalg::Vec2D;

trait Broadphase {
    fn get_contact_constraint(bodies: Vec<Vec2D>) {}
}
