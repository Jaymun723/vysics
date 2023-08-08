use core::panic;
use std::ops::{Index, IndexMut};

use crate::{
    linalg::{Vec2D, TOLERANCE},
    rigidbody2d::RigidBody2D,
};

// use wasm_bindgen::prelude::wasm_bindgen;

// #[wasm_bindgen]
// extern "C" {
//     #[wasm_bindgen(js_namespace = console)]
//     fn log(s: &str);
// }

// macro_rules! clg {
//     ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
// }

pub fn support_point(body: &RigidBody2D, dir: Vec2D) -> Vec2D {
    let local_dir = body.to_local(dir);

    let local_support = body.collider.support(local_dir);

    body.to_global(local_support)
}

pub fn triple_product(a: Vec2D, b: Vec2D, c: Vec2D) -> Vec2D {
    // let dot = a.x * b.y - b.x * a.y;
    // Vec2D::new(-c.y * dot, c.y * dot)
    -(c * b) * a + (c * a) * b
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CSOVertex {
    pub a: Vec2D,
    pub b: Vec2D,
}

impl CSOVertex {
    pub fn new(a: Vec2D, b: Vec2D) -> Self {
        Self { a, b }
    }

    pub fn get(a: &RigidBody2D, b: &RigidBody2D, dir: Vec2D) -> Self {
        let supp_a = support_point(a, dir);
        let supp_b = support_point(b, -dir);

        let res = Self::new(supp_a, supp_b);

        if !res.is_corect() {
            panic!(
                "Incorrect point generated.\nFor reproduction: a: {:#?}\nb: {:#?}\ndir:{dir}",
                a, b
            )
        }

        res
    }

    pub fn to_vec(&self) -> Vec2D {
        self.a - self.b
    }

    pub fn is_corect(&self) -> bool {
        self.a.is_correct() && self.b.is_correct()
    }
}

fn get_barycentric_coord_origin(v1: Vec2D, v2: Vec2D) -> f64 {
    let l = v2 - v1;

    let l_sq = l * l;

    if l_sq.abs() < TOLERANCE {
        0.
    } else {
        let proj = -v1 * l;
        let mut t = proj / l_sq;

        if t < 0. {
            t = 0.;
        } else if t > 1. {
            t = 1.;
        }

        t
    }
}

#[derive(Debug, PartialEq)]
pub struct Simplex {
    pub points: Vec<CSOVertex>,
}

impl Simplex {
    pub fn new() -> Self {
        Self { points: vec![] }
    }

    pub fn add(&mut self, point: CSOVertex) {
        self.points.push(point);
    }

    pub fn insert(&mut self, index: usize, point: CSOVertex) {
        self.points.insert(index, point)
    }

    fn handle_line_case(&mut self, a: &RigidBody2D, b: &RigidBody2D, dir: &mut Vec2D) -> bool {
        let point_b = self.points[0].to_vec();
        let point_a = self.points[1].to_vec();

        let ab = point_b - point_a;
        let ao = -point_a;

        // println!("ab: {ab}, ao: {ao}");

        if (point_a.norm() + (-point_b).norm() - ab.norm()).abs() < TOLERANCE {
            println!("Origin lies on the edge.");
            let left_normal = Vec2D::new(ab.y, -ab.x).normalize();
            let right_normal = Vec2D::new(-ab.y, ab.x).normalize();

            let left_support = CSOVertex::get(a, b, left_normal);
            let right_support = CSOVertex::get(a, b, right_normal);

            if left_support != self.points[0] && left_support != self.points[1] {
                self.add(left_support);
                return true;
            } else if right_support != self.points[0] && right_support != self.points[1] {
                self.add(right_support);
                return true;
            } else {
                panic!("Denegerate case ?");
            }
        }

        let mut ab_perp = triple_product(ab, ao, ab);

        // println!("Triple product for ab_perp: {ab_perp}");
        // let ab_perp = ab.to_3d().cross(
        //     ao.to_3d()
        // ).cross(ab.to_3d()).to_2d();
        if ab_perp.near_zero() {
            // println!("It's near zero !");
            ab_perp = Vec2D::new(ab.y, -ab.x).normalize();
        }

        // println!("ab_perp: {ab_perp}");

        // clg!("ab: {ab}, ao: {ao}");
        // clg!("u_ab: {}, u_ao: {}", ab.normalize(), ao.normalize());
        if ab * ao > 0. {
            //     let ap_perp = triple_product(ab, ao, ab);
            //     // let ap_perp = ao * (ab * ab) - ab * (ao * ab);

            //     println!("ap_perp: {ap_perp}");

            *dir = ab_perp.normalize();
        //     // if ap_perp.near_zero() {
        //     //     // clg!("wsh");
        //     //     *dir = Vec2D::new(ab.y, -ab.x);
        //     // } else {
        //     //     *dir = ap_perp.normalize();
        //     // }
        } else {
            //     // clg!("Setting dir as ao: {ao}");
            self.points.remove(0);
            *dir = ao.normalize();
        }

        false
    }

    fn handle_triangle_case(&mut self, dir: &mut Vec2D) -> bool {
        let point_c = self.points[0].to_vec();
        let point_b = self.points[1].to_vec();
        let point_a = self.points[2].to_vec();

        // println!("point_a: {point_a}");

        let ab = point_b - point_a;
        let ac = point_c - point_a;
        let ao = -point_a;

        // println!("ab: {ab}, ac: {ac}, ao: {ao}");

        // let abc = ab.to_3d().cross(ac.to_3d());

        // if abc.cross(ac.to_3d()) * ao.to_3d() > 0. {
        //     if ac * ao > 0. {}
        // }

        // let ac_perp = Vec2D::new(-ac.y * dot, ac.y * dot);

        // // let ab_perp = Vec2D::new(ab.y * dot, -ab.x * dot);
        let ac_perp = triple_product(ab, ac, ac).normalize();
        let ab_perp = triple_product(ac, ab, ab).normalize();

        // println!("ac_perp: {ac_perp}, ab_perp: {ab_perp}");

        if ac_perp * ao > 0. {
            let _ = self.points.remove(1);
            *dir = ac_perp;
            false
        } else if ab_perp * ao > 0. {
            let _ = self.points.remove(0);
            *dir = ab_perp;
            false
        } else {
            true
        }
    }

    pub fn handle(&mut self, a: &RigidBody2D, b: &RigidBody2D, dir: &mut Vec2D) -> bool {
        if self.points.len() == 2 {
            self.handle_line_case(a, b, dir)
        } else {
            self.handle_triangle_case(dir)
        }
    }

    /// From: https://github.com/RayzRazko/gjk-js/blob/master/lib/gjk.js#L114
    pub fn closest_point_on_segment_to_origin(&self) -> Vec2D {
        let v1 = self.points[0].to_vec();
        let v2 = self.points[1].to_vec();
        let l = v2 - v1;
        let t = get_barycentric_coord_origin(v1, v2);

        l * t + v1
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }
}

impl Index<usize> for Simplex {
    type Output = CSOVertex;

    fn index(&self, index: usize) -> &Self::Output {
        &self.points[index]
    }
}

impl IndexMut<usize> for Simplex {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.points[index]
    }
}

const GJK_ITERATIONS: u32 = 1000;

/// Check if two rigid bodies intersects
///
/// From: See README.md for reference
pub fn gjk_collision(a: &RigidBody2D, b: &RigidBody2D) -> Option<Simplex> {
    let mut dir = (b.position - a.position).normalize();
    if dir.near_zero() || !dir.is_correct() {
        dir = Vec2D::new(1., 0.);
    }
    // let mut dir = Vec2D::new(1., -1.).normalize();

    // println!("Initial dir: {dir}");

    let support = CSOVertex::get(a, b, dir);
    let mut simplex = Simplex::new();
    simplex.add(support);

    // println!("Added {}", support.to_vec());

    dir = -support.to_vec().normalize();

    if dir.near_zero() || !dir.is_correct() {
        panic!(
            "Dir is zero or incorrect.\nFor reproduction: a: {:#?}\nb: {:#?}\nsimplex: {:#?}",
            a, b, simplex
        );
    }

    for _ in 0..GJK_ITERATIONS {
        // println!(
        // "Iteration: {i}------------------------------------------------------------------"
        // );
        // clg!("Le simplex: {:#?}", simplex);
        // clg!("La direction: {}", dir);
        if dir.near_zero() || !dir.is_correct() {
            panic!(
                "Dir is zero or incorrect.\nFor reproduction: a: {:#?}\nb: {:#?}\nsimplex: {:#?}",
                a, b, simplex
            );
        }

        let support = CSOVertex::get(a, b, dir);
        // println!("Got support {} with direction {}", support.to_vec(), dir);
        let point_a = support.to_vec();

        if point_a * dir < 0. {
            return None;
        }

        simplex.add(support);

        // clg!("Le simplex avant handle: {:#?}", simplex);

        if simplex.handle(a, b, &mut dir) {
            // panic!("attends un peu");
            return Some(simplex);
        }
    }

    // clg!("Runned out of iterations.");
    None
}

/// Compute the distance and the closest points between to rigid bodies.
/// *Warning*: The two bodies must not intersects, otherwise the result may be incorrect.
///
/// From: https://github.com/RayzRazko/gjk-js/blob/master/lib/gjk.js#L138
/// And: https://dyn4j.org/2010/04/gjk-distance-closest-points/
pub fn gjk_distance(a: &RigidBody2D, b: &RigidBody2D) -> Option<(f64, Vec2D, Vec2D)> {
    let mut dir = (a.position - b.position).normalize();

    let mut simplex = Simplex::new();

    simplex.add(CSOVertex::get(a, b, dir));

    simplex.add(CSOVertex::get(a, b, -dir));

    for _ in 0..GJK_ITERATIONS {
        let p = simplex.closest_point_on_segment_to_origin();

        if p.near_zero() {
            return None;
        }

        dir = (-p).normalize();

        let support = CSOVertex::get(a, b, dir);
        let point_a = simplex[0].to_vec();
        let point_b = simplex[1].to_vec();
        let point_c = support.to_vec();

        if ((point_c * dir) - (point_a * dir)).abs() < TOLERANCE {
            let t = get_barycentric_coord_origin(point_a, point_b);

            return Some((
                -point_c * dir,
                (1. - t) * simplex[0].a + t * simplex[1].a,
                (1. - t) * simplex[0].b + t * simplex[1].b,
            ));
        }

        if point_a.squared_norm() < point_b.squared_norm() {
            simplex[1] = support;
        } else {
            simplex[0] = support;
        }
    }

    // clg!("Runned out of iterations.");
    None
}

#[cfg(test)]
mod tests {
    use crate::{
        collision::algorithms::gjk::{gjk_collision, CSOVertex, Simplex},
        linalg::Vec2D,
        rigidbody2d::RigidBody2D,
    };

    /// Based on: https://dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
    #[test]
    fn example_one() {
        let a_vertices = vec![Vec2D::new(4., 5.), Vec2D::new(9., 9.), Vec2D::new(4., 11.)];
        let b_vertices = vec![
            Vec2D::new(7., 3.),
            Vec2D::new(10., 2.),
            Vec2D::new(12., 7.),
            Vec2D::new(5., 7.),
        ];

        let a = RigidBody2D::from_polygon(&a_vertices, 1.);

        // assert_eq!(a.position, Vec2D::new(5.5, 8.5));

        let b = RigidBody2D::from_polygon(&b_vertices, 1.);

        // assert_eq!(b.position, Vec2D::new(9., 5.));

        assert_eq!(
            CSOVertex::get(&a, &b, Vec2D::new(1., 0.)).to_vec(),
            Vec2D::new(4., 2.)
        );
        assert_eq!(
            CSOVertex::get(&a, &b, Vec2D::new(-1., 0.)).to_vec(),
            Vec2D::new(-8., -2.)
        );
        assert_eq!(
            CSOVertex::get(&a, &b, Vec2D::new(0., 1.)).to_vec(),
            Vec2D::new(-6., 9.)
        );

        let mut expected_result = Simplex::new();
        expected_result.add(CSOVertex::get(&a, &b, Vec2D::new(1., -1.)));
        expected_result.add(CSOVertex::get(&a, &b, Vec2D::new(-0.573, -0.819)));
        expected_result.add(CSOVertex::get(&a, &b, Vec2D::new(0.316, -0.948)));

        assert_eq!(gjk_collision(&a, &b), Some(expected_result))
    }

    #[test]
    fn example_two() {
        let a = RigidBody2D {
            position: Vec2D { x: 400.0, y: 250.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: crate::collider::Collider::CircleCollider { radius: 50.0 },
            mass: 1.0,
            force_generators: vec![],
        };

        let b = RigidBody2D {
            position: Vec2D { x: 422.0, y: 261.0 },
            velocity: Vec2D { x: 0.0, y: 0.0 },
            angle: 0.0,
            angular_velocity: 0.0,
            collider: crate::collider::Collider::PolygonCollider {
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

        assert_eq!(
            gjk_collision(&a, &b),
            Some(Simplex {
                points: vec![
                    CSOVertex {
                        a: Vec2D {
                            x: 444.7213595499958,
                            y: 272.3606797749979
                        },
                        b: Vec2D { x: 322.0, y: 211.0 }
                    },
                    CSOVertex {
                        a: Vec2D {
                            x: 355.2786404500042,
                            y: 227.6393202250021
                        },
                        b: Vec2D { x: 522.0, y: 311.0 }
                    },
                    CSOVertex {
                        a: Vec2D {
                            x: 422.3606797749979,
                            y: 205.27864045000422
                        },
                        b: Vec2D { x: 322.0, y: 311.0 }
                    }
                ]
            })
        )
    }
}
