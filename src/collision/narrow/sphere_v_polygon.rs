use crate::{
    collision::{
        algorithms::{
            gjk::{gjk_collision, gjk_distance},
            sat::sat,
        },
        collider::Collider::{CircleCollider, PolygonCollider},
        manifold::ContactManifold,
    },
    rigidbody2d::RigidBody2D,
};

pub fn sphere_v_polygon(manifold: &mut ContactManifold) {
    // Ensure a is the circle and b the polygon
    let (a_radius, b_vertices) = match (&manifold.a.collider, &manifold.b.collider) {
        (CircleCollider { radius }, PolygonCollider { vertices }) => (*radius, vertices),
        (PolygonCollider { vertices }, CircleCollider { radius }) => {
            // a_radius = *radius;
            // b_vertices = vertices;
            let tmp = manifold.a;
            manifold.a = manifold.b;
            manifold.b = tmp;
            (*radius, vertices)
        }
        _ => panic!("Inappropriate fonction used for the narrow phase."),
    };

    // Following: http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf page 54

    let fake_a = RigidBody2D::new(manifold.a.position, CircleCollider { radius: 1. }, 1.);

    // Test if this is shallow collision of not:
    match gjk_collision(&fake_a, manifold.b) {
        // Shallow
        None => match gjk_distance(&fake_a, manifold.b) {
            // No collision (but should not happen)
            None => manifold.unset_colliding(),
            Some((distance, point_a, point_b)) => {
                // Collision !
                if distance <= a_radius {
                    let l = point_b - point_a;
                    manifold.set_colliding(l.normalize());
                    manifold.add_point(point_b, (l.norm() - a_radius).abs())
                } else {
                    manifold.unset_colliding()
                }
            }
        },
        Some(_simplex) => {
            println!("hey !");

            // let CollisionResult { normal, .. } = epa(simplex, manifold.a, manifold.b);
            let normal = match sat(manifold.a, manifold.b) {
                Some(n) => n.normalize(),
                None => panic!("Sat does not give a collision whereas GJK gives one."),
            };

            println!("sat computed the normal: {normal}");

            let n = b_vertices.len();
            let mut min_vertex = manifold.b.to_global(b_vertices[0]);
            let mut min_edge =
                manifold.b.to_global(b_vertices[1]) - manifold.b.to_global(b_vertices[0]);
            let mut min_dist = min_edge.normalize().cross(manifold.a.position - min_vertex);

            println!(
                "Edge 0: {min_edge}: ({} - {}) is at {min_dist}",
                manifold.b.to_global(b_vertices[1]),
                min_vertex
            );

            for i in 1..n {
                let j = (i + 1) % n;
                let vertex = manifold.b.to_global(b_vertices[i]);
                let edge = manifold.b.to_global(b_vertices[j]) - vertex;
                let dist = edge.normalize().cross(manifold.a.position - vertex);

                println!(
                    "Edge {i}: {edge}: ({} - {}) is at {dist}",
                    manifold.b.to_global(b_vertices[j]),
                    manifold.b.to_global(b_vertices[i])
                );
                if dist < min_dist {
                    min_dist = dist;
                    min_vertex = vertex;
                    min_edge = edge;
                    println!(" Update !, min_dist: {min_dist}");
                }
            }

            println!("The min_edge is: {min_edge}, with vertex: {min_vertex}");

            let t = ((manifold.a.position - min_vertex) * min_edge) / min_edge.squared_norm();

            let point = min_vertex + min_edge * t;

            println!("min_dist: {min_dist}");

            let depth = min_dist + a_radius;

            manifold.set_colliding(normal);
            manifold.add_point(point, depth);
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        collision::{collider::Collider, manifold::ContactManifold},
        linalg::Vec2D,
        rigidbody2d::RigidBody2D,
    };

    #[test]
    fn example_one() {
        let a = RigidBody2D::new(
            Vec2D::new(0., 0.5),
            Collider::CircleCollider { radius: 1. },
            1.,
        );
        let b = RigidBody2D::new(Vec2D::zero(), Collider::rectangle(2., 2.), 1.);

        let mut manifold = ContactManifold::new(&a, &b);
        manifold.compute();

        assert_eq!(manifold.normal, Some(Vec2D::new(0., -1.)));

        assert_eq!(manifold.points, vec![(Vec2D::new(0., 1.), 0.5 + 1.)]);
    }

    #[test]
    fn example_two() {
        let a = RigidBody2D::new(
            Vec2D::new(0., -0.5),
            Collider::CircleCollider { radius: 1. },
            1.,
        );
        let b = RigidBody2D::new(Vec2D::zero(), Collider::rectangle(2., 2.), 1.);

        let mut manifold = ContactManifold::new(&a, &b);
        manifold.compute();

        assert_eq!(manifold.normal, Some(Vec2D::new(0., 1.)));

        assert_eq!(manifold.points, vec![(Vec2D::new(0., -1.), 0.5 + 1.)]);
    }

    #[test]
    fn example_three() {
        let a = RigidBody2D::new(Vec2D::zero(), Collider::CircleCollider { radius: 4. }, 1.);
        let b = RigidBody2D::new(Vec2D::new(4., 0.), Collider::rectangle(10., 4.), 1.);

        let mut manifold = ContactManifold::new(&a, &b);
        manifold.compute();

        assert_eq!(manifold.normal, Some(Vec2D::new(1., 0.)));

        assert_eq!(manifold.points, vec![(Vec2D::new(-1., 0.), 1. + 4.)]);
    }
}
