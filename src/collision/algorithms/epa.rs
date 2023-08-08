use crate::{linalg::Vec2D, rigidbody2d::RigidBody2D};

use super::gjk::{triple_product, CSOVertex, Simplex};

#[derive(Debug, PartialEq)]
pub struct CollisionResult {
    pub normal: Vec2D,
    pub depth: f64,
    pub point_a: Vec2D,
    pub point_b: Vec2D,
}

fn closest_edge(simplex: &Simplex) -> (f64, usize, Vec2D) {
    let n = simplex.len();
    let mut min_index = 0;
    let mut min_normal = None;
    let mut min_dist = std::f64::INFINITY;

    for i in 0..n {
        let j = (i + 1) % n;
        // println!(
        //     "Looking at edge from {i}: {} and {j}: {}",
        //     simplex[i].to_vec(),
        //     simplex[j].to_vec()
        // );
        let edge = simplex[j].to_vec() - simplex[i].to_vec();

        if !edge.is_correct() {
            panic!("Stop: edge is incorrect.");
        }
        // let normal = if simplex[i].to_vec().cross(simplex[j].to_vec()) < 0. {
        //     Vec2D::new(-edge.y, edge.x)
        // } else {
        //     Vec2D::new(edge.y, -edge.x)
        // }
        // .normalize();

        let normal = triple_product(edge, simplex[i].to_vec(), edge).normalize();

        if normal.near_zero() || !normal.is_correct() {
            // clg!("Triple prodcut was bad.");
            continue;
            // normal = if simplex[i].to_vec().cross(simplex[j].to_vec()) > 0. {
            //     Vec2D::new(-edge.y, edge.x)
            // } else {
            //     Vec2D::new(edge.y, -edge.x)
            // }
            // .normalize();
            // normal = edge.right();

            // println!("With edge: {edge} we computed this normal: {normal}");
        }

        if !normal.is_correct() {
            panic!("Stop: normal is incorrect.");
        }

        if !simplex[i].to_vec().is_correct() {
            panic!("Stop: simplex edge is incorect.");
        }

        // let normal = triple_product(edge, simplex[i].to_vec(), edge).normalize();
        let dist = (normal * (simplex[i].to_vec())).abs();
        // println!("Got dist: {dist}");
        if dist < min_dist {
            min_dist = dist;
            min_index = i;
            min_normal = Some(normal);
        }
    }

    (
        min_dist,
        min_index,
        min_normal.expect(format!("Can't find the closest edge, simplex: {:#?}", simplex).as_str()),
    )
}

const EPA_ITERATIONS: u32 = 100;

pub fn epa(mut simplex: Simplex, a: &RigidBody2D, b: &RigidBody2D) -> CollisionResult {
    // let mut winding = 0;
    // for i in 0..simplex.len() {
    //     let j = if i + 1 == simplex.len() { 0 } else { i + 1 };
    //     let a = simplex[i].to_vec();
    //     let b = simplex[j].to_vec();
    //     let cross = a.x * b.y - a.y * b.x;
    //     if cross > 0. {
    //         winding = 1;
    //         break;
    //     } else if cross < 0. {
    //         winding = -1;
    //         break;
    //     }
    // }

    // println!("Winding: {winding}");

    // clg!("Iteration 0:");
    // clg!("a: {:#?}\nb: {:#?}\nsimplex: {:#?}", a, b, simplex);
    for _ in 0..EPA_ITERATIONS {
        let (depth, index, normal) = closest_edge(&simplex);
        let support = CSOVertex::get(a, b, normal);
        let r = support.to_vec();
        // clg!("Iteration {}: adding support {}", i + 1, r);
        // clg!("a: {:#?}\nb: {:#?}\nsimplex: {:#?}", a, b, simplex);
        if ((normal * r) - depth).abs() < 0.001 {
            let jndex = (index + 1) % simplex.len();
            let a = simplex[index].to_vec();
            let b = simplex[jndex].to_vec();

            let l = b - a;

            let mut point_a = simplex[index].a;
            let mut point_b = simplex[index].b;

            if l.near_zero() {
                return CollisionResult {
                    normal,
                    depth,
                    point_a,
                    point_b,
                };
            }

            let lambda = -(a * l) / l.squared_norm();

            if 1. - lambda < 0. {
                point_a = simplex[jndex].a;
                point_b = simplex[jndex].b;
            } else if lambda > 0. {
                point_a = (1. - lambda) * simplex[index].a + lambda * simplex[jndex].a;
                point_b = (1. - lambda) * simplex[index].b + lambda * simplex[jndex].b;
            }

            // println!("concluded in {i} steps");
            return CollisionResult {
                normal: -normal,
                depth,
                point_a,
                point_b,
            };
        }
        simplex.insert(index + 1, support);
    }

    panic!(
        "Epa runned out of iterations. \nTo reproduce: a: {:#?}\nb:{:#?}",
        a, b
    );
}

#[cfg(test)]
mod tests {
    use std::vec;

    use crate::{
        collider::Collider::{CircleCollider, PolygonCollider},
        collision::algorithms::{epa::CollisionResult, gjk::gjk_collision},
        linalg::Vec2D,
        rigidbody2d::RigidBody2D,
    };

    use super::epa;

    #[test]
    fn example_one() {
        let a = RigidBody2D {
            position: Vec2D { x: 400.0, y: 250.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: PolygonCollider {
                vertices: vec![
                    Vec2D { x: -50.0, y: -50.0 },
                    Vec2D { x: -50.0, y: 50.0 },
                    Vec2D { x: 50.0, y: 50.0 },
                    Vec2D { x: 50.0, y: -50.0 },
                ],
            },
            mass: 1.0,
            force_generators: vec![],
        };

        let b = RigidBody2D {
            position: Vec2D { x: 400.0, y: 165.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: PolygonCollider {
                vertices: vec![
                    Vec2D {
                        x: -100.0,
                        y: -35.0,
                    },
                    Vec2D { x: -100.0, y: 35.0 },
                    Vec2D { x: 100.0, y: 35.0 },
                    Vec2D { x: 100.0, y: -35.0 },
                ],
            },
            mass: 1.0,
            force_generators: vec![],
        };

        println!("===== GJK =====");
        let simplex = match gjk_collision(&a, &b) {
            Some(s) => s,
            None => panic!("GJK didn't get a collision."),
        };

        println!("===== SPL =====");
        println!("{:#?}", simplex);

        println!("===== EPA =====");
        let result = epa(simplex, &a, &b);

        assert_eq!(
            result,
            CollisionResult {
                normal: Vec2D { x: 1.0, y: -0.0 },
                depth: 150.0,
                point_a: Vec2D { x: 350.0, y: 200.0 },
                point_b: Vec2D { x: 500.0, y: 200.0 }
            }
        )
    }

    #[test]
    fn example_two() {
        let a = RigidBody2D {
            position: Vec2D { x: 400.0, y: 250.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: PolygonCollider {
                vertices: vec![
                    Vec2D { x: -50.0, y: -50.0 },
                    Vec2D { x: -50.0, y: 50.0 },
                    Vec2D { x: 50.0, y: 50.0 },
                    Vec2D { x: 50.0, y: -50.0 },
                ],
            },
            mass: 1.0,
            force_generators: vec![],
        };

        let b = RigidBody2D {
            position: Vec2D { x: 406.0, y: 254.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: PolygonCollider {
                vertices: vec![
                    Vec2D {
                        x: -100.0,
                        y: -50.0,
                    },
                    Vec2D { x: -100.0, y: 50.0 },
                    Vec2D { x: 100.0, y: 50.0 },
                    Vec2D { x: 100.0, y: -50.0 },
                ],
            },
            mass: 1.0,
            force_generators: vec![],
        };

        let simplex = match gjk_collision(&a, &b) {
            Some(s) => s,
            None => panic!("GJK didn't get a collision."),
        };

        let result = epa(simplex, &a, &b);
        assert_eq!(
            result,
            CollisionResult {
                normal: Vec2D { x: -0.0, y: 1.0 },
                depth: 104.0,
                point_a: Vec2D { x: 402.0, y: 200.0 },
                point_b: Vec2D { x: 402.0, y: 304.0 }
            }
        )
    }

    #[test]
    fn example_three() {
        let a = RigidBody2D {
            position: Vec2D { x: 400.0, y: 250.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: CircleCollider { radius: 80.0 },
            mass: 1.0,
            force_generators: vec![],
        };

        let b = RigidBody2D {
            position: Vec2D { x: 404.0, y: 248.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: PolygonCollider {
                vertices: vec![
                    Vec2D {
                        x: -100.0,
                        y: -50.0,
                    },
                    Vec2D { x: -100.0, y: 50.0 },
                    Vec2D { x: 100.0, y: 50.0 },
                    Vec2D { x: 100.0, y: -50.0 },
                ],
            },
            mass: 1.0,
            force_generators: vec![],
        };

        println!("===== GJK =====");
        let simplex = match gjk_collision(&a, &b) {
            Some(s) => s,
            None => panic!("GJK didn't get a collision."),
        };

        println!("===== SPL =====");
        println!("{:#?}", simplex);

        println!("===== EPA =====");
        let result = epa(simplex, &a, &b);
        assert_eq!(
            result,
            CollisionResult {
                normal: Vec2D {
                    x: -1.0825472913881718e-6,
                    y: 0.999999999999414
                },
                depth: 127.99988572773604,
                point_a: Vec2D {
                    x: 400.089269166606,
                    y: 170.00011427233898
                },
                point_b: Vec2D {
                    x: 400.0891306006764,
                    y: 298.0
                }
            }
        )
    }

    #[test]
    pub fn example_four() {
        let a = RigidBody2D {
            position: Vec2D { x: 400.0, y: 250.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: CircleCollider { radius: 80.0 },
            mass: 1.0,
            force_generators: vec![],
        };

        let b = RigidBody2D {
            position: Vec2D { x: 426.0, y: 301.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: PolygonCollider {
                vertices: vec![
                    Vec2D {
                        x: -100.0,
                        y: -50.0,
                    },
                    Vec2D { x: 100.0, y: -50.0 },
                    Vec2D { x: 100.0, y: 50.0 },
                    Vec2D { x: -100.0, y: 50.0 },
                ],
            },
            mass: 1.0,
            force_generators: vec![],
        };

        println!("===== GJK =====");
        let simplex = match gjk_collision(&a, &b) {
            Some(s) => s,
            None => panic!("GJK didn't get a collision."),
        };

        println!("===== SPL =====");
        println!("{:#?}", simplex);

        // clg!("{:#?}", simplex);

        println!("===== EPA =====");
        let result = epa(simplex, &a, &b);
        assert_eq!(
            result,
            CollisionResult {
                normal: Vec2D {
                    x: -4.736703088154728e-9,
                    y: -1.0
                },
                depth: 78.99999910497503,
                point_a: Vec2D {
                    x: 400.0063376179095,
                    y: 329.999999104975
                },
                point_b: Vec2D {
                    x: 400.00633724370994,
                    y: 251.0
                }
            }
        )
    }
    // #[test]
    // fn example_four() {
    //     let a = RigidBody2D {
    //         position: Vec2D { x: 400.0, y: 250.0 },
    //         velocity: Vec2D { x: 0.0, y: 0.0 },
    //         angle: 0.0,
    //         angular_velocity: 0.0,
    //         collider: CircleCollider { radius: 80.0 },
    //         mass: 1.0,
    //         force_generators: vec![],
    //     };

    //     let b = RigidBody2D {
    //         position: Vec2D { x: 490.0, y: 205.0 },
    //         velocity: Vec2D { x: 0.0, y: 0.0 },
    //         angle: 0.0,
    //         angular_velocity: 0.0,
    //         collider: PolygonCollider {
    //             vertices: vec![
    //                 Vec2D {
    //                     x: -100.0,
    //                     y: -50.0,
    //                 },
    //                 Vec2D { x: -100.0, y: 50.0 },
    //                 Vec2D { x: 100.0, y: 50.0 },
    //                 Vec2D { x: 100.0, y: -50.0 },
    //             ],
    //         },
    //         mass: 1.0,
    //         force_generators: vec![],
    //     };
    // }
}
