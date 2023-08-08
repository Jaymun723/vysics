// From: http://media.steampowered.com/apps/valve/2015/DirkGregorius_Contacts.pdf page 80
// And: https://github.com/erincatto/box2d/blob/main/src/collision/b2_collide_polygon.cpp
// But mainly: https://github.com/dyn4j/dyn4j/blob/master/src/main/java/org/dyn4j/collision/manifold/ClippingManifoldSolver.java

use crate::{
    collider::Collider::PolygonCollider,
    collision::{algorithms::sat::sat, manifold::ContactManifold},
    linalg::Vec2D,
    rigidbody2d::RigidBody2D,
};

fn vertices_to_global(body: &RigidBody2D) -> Vec<Vec2D> {
    match &body.collider {
        PolygonCollider {
            vertices: local_vertices,
        } => local_vertices.iter().map(|v| body.to_global(*v)).collect(),
        _ => panic!("Inappropriate fonction used for the narrow phase."),
    }
}

fn get_farthest_vertex_index(vertices: &Vec<Vec2D>, dir: Vec2D) -> usize {
    let mut max_index = 0;
    let n = vertices.len();
    let mut max = dir * vertices[0];

    for i in 1..n {
        let dist = dir * vertices[i];

        if dist > max {
            max = dist;
            max_index = i;
        }
    }

    max_index

    // let mut max_index = 0;
    // let n = vertices.len();
    // let mut max = dir * vertices[0];
    // let mut candidate_max = dir * vertices[1];

    // if max < candidate_max {
    //     // Search to the right
    //     max = candidate_max;
    //     max_index += 1;

    //     candidate_max = dir * vertices[max_index + 1];

    //     while (max_index + 1) < n && max < candidate_max {
    //         max = candidate_max;
    //         max_index += 1;

    //         candidate_max = dir * vertices[max_index + 1];
    //     }
    // } else {
    //     candidate_max = dir * vertices[n - 1];

    //     if max < candidate_max {
    //         max = candidate_max;
    //         max_index = n - 1;

    //         candidate_max = dir * vertices[max_index - 1];

    //         while (max_index > 0) && max <= candidate_max {
    //             max = candidate_max;
    //             max_index -= 1;
    //             candidate_max = dir * vertices[max_index - 1];
    //         }
    //     }
    // }

    // max_index
}

// fn get_edges_normals(vertices: &Vec<Vec2D>) -> (Vec<Vec2D>, Vec<Vec2D>) {
//     let mut edges = vec![];
//     let mut normals = vec![];
//     let n = vertices.len();

//     for i in 0..n {
//         let j = (i + 1) % n;
//         let edge = vertices[j] - vertices[i];
//         edges.push(edge);

//         let normal = edge.left().normalize();
//         normals.push(normal);
//     }

//     (edges, normals)
// }

#[derive(Debug)]
struct EdgeFeature {
    first: Vec2D,
    second: Vec2D,
    max: Vec2D,
}

impl EdgeFeature {
    fn new(first: Vec2D, second: Vec2D, max: Vec2D) -> Self {
        Self { first, second, max }
    }

    fn to_vec(&self) -> Vec2D {
        self.second - self.first
    }
}

fn get_farthest_edge(vertices: &Vec<Vec2D>, dir: Vec2D) -> EdgeFeature {
    println!("Getting the farthest with dir: {dir}");
    let n = vertices.len();
    let index = get_farthest_vertex_index(vertices, dir);

    let vertex = vertices[index];
    let prev_vertex = vertices[(index + n - 1) % n];
    let next_vertex = vertices[(index + 1) % n];

    println!(
        "The vertex the farthest is {index}: {vertex} (prev: {prev_vertex}, next: {next_vertex})"
    );

    let left_normal = (vertex - prev_vertex).left().normalize();
    let right_normal = (next_vertex - vertex).left().normalize();

    println!("Left_normal: {left_normal}, Right_normal: {right_normal}");

    if left_normal * dir < right_normal * dir {
        println!("Choosed left");

        EdgeFeature::new(prev_vertex, vertex, vertex)
    } else {
        println!("Choosed right");

        EdgeFeature::new(vertex, next_vertex, vertex)
    }
}

fn clip(p1: Vec2D, p2: Vec2D, n: Vec2D, offset: f64) -> Vec<Vec2D> {
    println!("Clipping p1: {p1}, p2: {p2}, n: {n}, offset: {offset}");
    let mut points = Vec::with_capacity(3);

    // calculate the distance between the end points of the edge and the clip line
    let d1 = n * p1 - offset;
    let d2 = n * p2 - offset;

    // add the points if they are behind the line
    if d1 <= 0.0 {
        println!("We keep p1");
        points.push(p1);
    }
    if d2 <= 0.0 {
        println!("We keep p2");
        points.push(p2);
    }

    // check if they are on opposing sides of the line
    if d1 * d2 < 0.0 {
        // get the edge vector
        let mut e = p2 - p1;
        // clip to obtain another point
        let u = d1 / (d1 - d2);
        e *= u;
        e += p1;
        println!("We add a mid_point: {e}");
        points.push(e);
    }
    return points;
}

pub fn polygon_v_polygon(manifold: &mut ContactManifold) {
    let (a_vertices, b_vertices) = (
        vertices_to_global(manifold.a),
        vertices_to_global(manifold.b),
    );

    println!("Vertices: a: {:#?}\nb: {:#?}", a_vertices, b_vertices);
    println!("=====================================================");

    match sat(manifold.a, manifold.b) {
        None => {
            manifold.unset_colliding();
        }
        Some(mtv) => {
            println!("End of sat:\n\n");
            let normal = mtv.normalize();

            manifold.set_colliding(normal);

            let farthest_a = get_farthest_edge(&a_vertices, normal);

            println!("Farthest on a: {:#?} ({})", farthest_a, farthest_a.to_vec());
            println!("a dot n: {}", farthest_a.to_vec() * normal);

            let farthest_b = get_farthest_edge(&b_vertices, -normal);

            println!("Farthest on b: {:#?} ({})", farthest_b, farthest_b.to_vec());
            println!("b dot n: {}", farthest_b.to_vec() * normal);

            let (reference, incident, flipped) =
                if (farthest_a.to_vec() * normal).abs() <= (farthest_b.to_vec() * normal).abs() {
                    (farthest_a, farthest_b, false)
                } else {
                    println!("Flipping");
                    (farthest_b, farthest_a, true)
                };

            println!(
                "The reference edge: {}, the incident one: {}",
                reference.to_vec(),
                incident.to_vec()
            );

            let refev = reference.to_vec().normalize();
            let offset_1 = -refev * reference.first;

            println!("The first offset is {offset_1}");

            let cp = clip(incident.first, incident.second, -refev, offset_1);

            if cp.len() < 2 {
                return;
            }

            let offset_2 = refev * reference.second;

            let cp = clip(cp[0], cp[1], refev, offset_2);

            if cp.len() < 2 {
                return;
            }

            let front_normal = refev.left(); // sus but ok
            let front_offset = front_normal * reference.max;

            println!("Front_normal: {front_normal}, front_offset: {front_offset}");

            if flipped {
                manifold.set_colliding(-normal);
            }

            for point in cp {
                let depth = front_normal * point - front_offset;
                println!("Point: {point} has depth {depth}");
                if depth >= 0. {
                    println!("Keeping it");
                    manifold.add_point(point, depth);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{
        collider::Collider, collision::manifold::ContactManifold, linalg::Vec2D,
        rigidbody2d::RigidBody2D,
    };

    #[test]
    fn example_one() {
        let a = RigidBody2D::new(Vec2D::zero(), Collider::rectangle(2., 2.), 1.);

        let mut b = RigidBody2D::new(Vec2D::new(0., 2.), Collider::rectangle(2., 2.), 1.);
        b.angle = std::f64::consts::FRAC_PI_4;

        let mut manifold = ContactManifold::new(&a, &b);
        manifold.compute();

        assert_eq!(
            manifold.points,
            vec![(
                Vec2D {
                    x: -1.1102230246251565e-16,
                    y: 0.5857864376269051
                },
                0.4142135623730949
            )]
        );
    }
}
