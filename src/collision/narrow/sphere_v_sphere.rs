use crate::{collision::manifold::ContactManifold, linalg::Vec2D};

pub fn sphere_v_sphere(manifold: &mut ContactManifold) {
    let radius_a = match manifold.a.collider {
        crate::collider::Collider::CircleCollider { radius } => radius,
        _ => panic!("Inappropriate fonction used for the narrow phase (for element a)."),
    };

    let radius_b = match manifold.b.collider {
        crate::collider::Collider::CircleCollider { radius } => radius,
        _ => panic!("Inappropriate fonction used for the narrow phase (for element b)."),
    };

    let l = manifold.b.position - manifold.a.position;

    if l.near_zero() {
        manifold.set_colliding(Vec2D::new(0., 1.));
        manifold.add_point(manifold.a.position + Vec2D::new(0., 0.01), 0.01);
        return;
    }

    let distance_squared = l.squared_norm();
    let min_dist = (radius_a + radius_b).powi(2);

    // clg!("distance_squared: {distance_squared}, min_dist: {min_dist}");

    if distance_squared > min_dist {
        manifold.unset_colliding();
    } else {
        let normal = l.normalize();

        // manifold.normal = Some(normal);
        manifold.set_colliding(normal);

        let mid_distance_from_a = 0.5 * (l.norm() + radius_a - radius_b);

        let contact_point = manifold.a.position + normal * mid_distance_from_a;

        // manifold.points = vec![contact_point];

        let depth = l.norm() - radius_a - radius_b;

        manifold.add_point(contact_point, depth)

        // clg!("{:#?}", manifold);
    }
}
