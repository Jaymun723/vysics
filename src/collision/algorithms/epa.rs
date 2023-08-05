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
        let edge = simplex[j].to_vec() - simplex[i].to_vec();

        // let normal = if simplex[i].to_vec().cross(simplex[j].to_vec()) < 0. {
        //     Vec2D::new(-edge.y, edge.x)
        // } else {
        //     Vec2D::new(edge.y, -edge.x)
        // }
        // .normalize();

        let normal = triple_product(edge, simplex[i].to_vec(), edge).normalize();

        if normal.near_zero() {
            panic!("oh shit");
        }

        // let normal = triple_product(edge, simplex[i].to_vec(), edge).normalize();
        let dist = normal * (simplex[i].to_vec());
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

const EPA_ITERATIONS: u32 = 1000;

pub fn epa(mut simplex: Simplex, a: &RigidBody2D, b: &RigidBody2D) -> CollisionResult {
    for _ in 0..EPA_ITERATIONS {
        let (depth, index, normal) = closest_edge(&simplex);
        let support = CSOVertex::get(a, b, normal);
        let r = support.to_vec();
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
    use crate::{
        collider::Collider::PolygonCollider,
        collision::algorithms::{
            epa::CollisionResult,
            gjk::{self, gjk_collision},
        },
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

        let simplex = match gjk_collision(&a, &b) {
            Some(s) => s,
            None => panic!("GJK didn't get a collision."),
        };

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
}
