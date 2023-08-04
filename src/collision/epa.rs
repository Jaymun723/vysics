// use crate::{linalg::Vec2D, rigidbody2d::RigidBody2D};

// use super::gjk::{support_point, triple_product};

// fn get_normal(v: Vec2D) -> Vec2D {
//     Vec2D::new(v.y, -v.x).normalize()
// }

// fn closest_edge(simplex: &Vec<(Vec2D, Vec2D)>) -> (f64, usize, Vec2D) {
//     let n = simplex.len();
//     let mut min_index = 0;
//     let mut min_normal = None;
//     let mut min_dist = std::f64::INFINITY;

//     for i in 0..n {
//         let j = (i + 1) % n;
//         let edge = (simplex[j].0 - simplex[j].1) - (simplex[i].0 - simplex[i].1);
//         let normal = triple_product(edge, simplex[i].0 - simplex[i].1, edge).normalize();
//         let dist = normal * (simplex[i].0 - simplex[i].1);
//         if dist < min_dist {
//             min_dist = dist;
//             min_index = i;
//             min_normal = Some(normal);
//         }
//     }

//     (min_dist, min_index, min_normal.unwrap())
// }

// pub struct CollisionResult {
//     pub normal: Vec2D,
//     pub depth: f64,
//     pub point_a: Vec2D,
//     pub point_b: Vec2D,
// }

// pub fn epa(simplex: &mut Vec<(Vec2D, Vec2D)>, a: &RigidBody2D, b: &RigidBody2D) -> CollisionResult {
//     loop {
//         let (depth, index, normal) = closest_edge(simplex);
//         let support = support_point(a, b, normal);
//         let r = support.0 - support.1;
//         if ((normal * r) - depth).abs() < 0.001 {
//             let jndex = (index + 1) % simplex.len();
//             let a = simplex[index].0 - simplex[index].1;
//             let b = simplex[jndex].0 - simplex[jndex].1;

//             let l = b - a;

//             let mut point_a = simplex[index].0;
//             let mut point_b = simplex[index].1;

//             if l.near_zero() {
//                 return CollisionResult {
//                     normal,
//                     depth,
//                     point_a,
//                     point_b,
//                 };
//             }

//             let lambda = -(a * l) / l.squared_norm();

//             if 1. - lambda < 0. {
//                 point_a = simplex[jndex].0;
//                 point_b = simplex[jndex].1;
//             } else if lambda > 0. {
//                 point_a = (1. - lambda) * simplex[index].0 + lambda * simplex[jndex].0;
//                 point_b = (1. - lambda) * simplex[index].1 + lambda * simplex[jndex].1;
//             }

//             return CollisionResult {
//                 normal,
//                 depth,
//                 point_a,
//                 point_b,
//             };
//         }
//         simplex.insert(index + 1, support);
//     }
// }
