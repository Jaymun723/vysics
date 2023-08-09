use crate::{
    collision::collider::Collider::{CircleCollider, PolygonCollider},
    linalg::Vec2D,
    rigidbody2d::RigidBody2D,
};

#[derive(Debug)]
struct ProjectedShape {
    mini: f64,
    maxi: f64,
}

impl ProjectedShape {
    fn new(axis: Vec2D, shape: &RigidBody2D) -> Self {
        match &shape.collider {
            CircleCollider { radius } => {
                let mid = axis * shape.position;
                Self {
                    maxi: mid + radius,
                    mini: mid - radius,
                }
            }
            PolygonCollider { vertices } => {
                let polygon: Vec<Vec2D> = vertices.iter().map(|v| shape.to_global(*v)).collect();

                let mut min = polygon[0] * axis;
                let mut max = min;

                for i in 1..polygon.len() {
                    let proj = polygon[i] * axis;

                    if proj > max {
                        max = proj;
                    }
                    if proj < min {
                        min = proj;
                    }
                }

                Self {
                    mini: min,
                    maxi: max,
                }
            }
        }
    }

    fn overlap(&self, other: &Self) -> bool {
        (self.maxi >= other.mini) && (self.mini <= other.maxi)
    }

    fn get_overlap(&self, other: &Self) -> f64 {
        let max = |a: f64, b: f64| {
            if a < b {
                b
            } else {
                a
            }
        };
        let min = |a: f64, b: f64| {
            if a < b {
                a
            } else {
                b
            }
        };

        min(self.maxi, other.maxi) - max(self.mini, other.mini)
    }

    fn contains_exclusive(&self, other: &Self) -> bool {
        (self.mini < other.mini) && (self.maxi > other.maxi)
    }
}

fn get_foci(body: &RigidBody2D) -> Vec<Vec2D> {
    match &body.collider {
        CircleCollider { .. } => vec![body.position],
        _ => vec![],
    }
}

fn get_normals(points: &Vec<Vec2D>) -> Vec<Vec2D> {
    let mut normals = vec![];
    let n = points.len();

    for i in 0..n {
        let j = (i + 1) % n;
        // println!("Computing the normal of the edge: {i} -> {j}");
        let edge = points[j] - points[i];
        // println!("The edge is {edge}: {} - {}", points[j], points[i]);
        normals.push(edge.right().normalize());
    }

    normals
}

fn get_axes(a: &RigidBody2D, foci: &Vec<Vec2D>) -> Vec<Vec2D> {
    match &a.collider {
        CircleCollider { .. } => vec![],
        PolygonCollider { vertices } => {
            let polygon: Vec<Vec2D> = vertices.iter().map(|v| a.to_global(*v)).collect();

            println!("Getting the axes of the polygon: {:#?}", polygon);

            let mut res = get_normals(&polygon);

            println!("Got normals: {:#?}", res);

            for focus in foci {
                let mut closest = polygon[0];
                let mut d = (*focus - closest).squared_norm();

                for i in 1..polygon.len() {
                    let vertex = polygon[i];
                    let distance = (*focus - vertex).squared_norm();

                    if distance < d {
                        d = distance;
                        closest = vertex;
                    }
                }

                println!("Adding: {}", *focus - closest);
                res.push((*focus - closest).normalize());
            }

            res
        }
    }
}

/// From: https://dyn4j.org/2010/01/sat/
pub fn sat(a: &RigidBody2D, b: &RigidBody2D) -> Option<Vec2D> {
    let mut overlap = std::f64::INFINITY;
    let mut smallest = None;

    let foci_a = get_foci(a);
    let foci_b = get_foci(b);

    let mut axes_a = get_axes(a, &foci_b);
    let mut axes_b = get_axes(b, &foci_a);

    axes_a.append(&mut axes_b);

    println!("Axes: {:#?}", axes_a);

    for mut axis in axes_a {
        if axis.near_zero() || !axis.is_correct() {
            continue;
        }

        println!("With axis: {axis} =====");

        let p_a = ProjectedShape::new(axis, a);

        let p_b = ProjectedShape::new(axis, b);

        println!("Got projections: p_a: {:#?} p_b: {:#?}", p_a, p_b);

        if !p_a.overlap(&p_b) {
            return None;
        } else {
            let mut o = p_a.get_overlap(&p_b);

            println!("The overlap is: {o}");

            if p_a.contains_exclusive(&p_b) || p_b.contains_exclusive(&p_a) {
                let max = (p_a.maxi - p_b.maxi).abs();
                let min = (p_a.mini - p_b.mini).abs();
                if max > min {
                    axis = -axis;
                    o += min;
                } else {
                    o += max;
                }
            }

            if o < overlap {
                overlap = o;
                smallest = Some(axis.normalize() * overlap);
            }
        }
    }

    if let Some(axis) = smallest {
        let a_to_b = b.position - a.position;
        if axis * a_to_b < 0. {
            smallest = Some(-axis);
        }
    }

    smallest
}

#[cfg(test)]
mod test {
    use crate::{
        collision::{
            algorithms::sat::sat,
            collider::Collider::{self, CircleCollider, PolygonCollider},
        },
        linalg::Vec2D,
        rigidbody2d::RigidBody2D,
    };

    #[test]
    fn example_one() {
        let a = RigidBody2D::new(Vec2D::new(2.5, 2.), Collider::rectangle(3., 2.), 1.);
        let b = RigidBody2D::new(Vec2D::new(4., 3.), Collider::rectangle(2., 2.), 1.);

        // let foci_a = get_foci(&a);
        // let foci_b = get_foci(&b);

        // let mut axes_a = get_axes(&a, &foci_b);
        // println!("axes_a: {:#?}", axes_a);

        // let mut axes_b = get_axes(&b, &foci_a);
        // println!("axes_b: {:#?}", axes_b);

        assert_eq!(sat(&a, &b), Some(Vec2D::new(0., 1.)));
        // assert!(false);
    }

    #[test]
    fn example_two() {
        let a = RigidBody2D::new(Vec2D::new(2.5, 2.), Collider::rectangle(3., 2.), 1.);
        let b = RigidBody2D::new(Vec2D { x: 5., y: 4. }, Collider::circle(1.), 1.);

        assert_eq!(sat(&a, &b), None);
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
            inertia: 1.,
        };

        let b = RigidBody2D {
            position: Vec2D { x: 572.0, y: 162.0 },
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
            inertia: 1.,
        };

        assert_eq!(sat(&a, &b), None);
    }
}
