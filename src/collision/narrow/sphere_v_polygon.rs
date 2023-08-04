use crate::collider::Collider::{CircleCollider, PolygonCollider};
use crate::collision::algorithms::epa::{epa, CollisionResult};
use crate::collision::algorithms::gjk::{self, gjk_collision};
use crate::collision::manifold::ContactManifold;

pub fn sphere_v_polygon(manifold: &mut ContactManifold) {
    let mut a_radius = 0.;
    let mut b_verticies = &(vec![]);

    // Ensure a is the circle and b the polygon
    match (&manifold.a.collider, &manifold.b.collider) {
        (CircleCollider { radius }, PolygonCollider { vertices }) => {
            a_radius = *radius;
            b_verticies = vertices;
        }
        (PolygonCollider { vertices }, CircleCollider { radius }) => {
            a_radius = *radius;
            b_verticies = vertices;
            let tmp = manifold.a;
            manifold.a = manifold.b;
            manifold.b = tmp;
        }
        _ => panic!("Inappropriate fonction used for the narrow phase."),
    }

    // match gjk_collision(manifold.a, manifold.b) {

    // }
}
