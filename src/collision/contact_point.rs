use crate::{
    collider::Collider,
    linalg::{Vec2D, Vec3D},
};
use std::ops::Index;

// use wasm_bindgen::prelude::*;

// #[wasm_bindgen]
// extern "C" {
//     #[wasm_bindgen(js_namespace = console)]
//     fn log(s: &str);
// }

// macro_rules! clg {
//     ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
// }

#[derive(PartialEq, Debug)]
struct Edge {
    start: Vec2D,
    end: Vec2D,
    max: Vec2D,
}

impl Edge {
    fn to_vec(&self) -> Vec2D {
        self.end - self.start
    }

    fn dot(&self, vec: Vec2D) -> f64 {
        self.to_vec() * vec
    }
}

fn get_best_edge(verticies: &Vec<Vec2D>, n: Vec2D) -> Edge {
    let c = verticies.len();
    let mut max_projection = (verticies[0]) * n;
    let mut index = 0;

    for i in 1..c {
        let projection = verticies[i] * n;
        if projection > max_projection {
            max_projection = projection;
            index = i;
        }
    }

    let prev_index = (index + c - 1) % c;
    let next_index = (index + 1) % c;

    let l = (verticies[index] - verticies[next_index]).normalize();
    let r = (verticies[index] - verticies[prev_index]).normalize();

    if r * n <= l * n {
        Edge {
            max: verticies[index],
            start: verticies[prev_index],
            end: verticies[index],
        }
    } else {
        Edge {
            max: verticies[index],
            start: verticies[index],
            end: verticies[next_index],
        }
    }
}

#[derive(PartialEq, Debug)]
struct ClippedPoints {
    first: Option<Vec2D>,
    second: Option<Vec2D>,
    // points: Vec<Vec2D>,
}

impl ClippedPoints {
    fn clip(v1: Vec2D, v2: Vec2D, n: Vec2D, o: f64) -> Self {
        println!("Clipping: v1: {v1}, v2: {v2}, n: {n}, o: {o}");
        let d1 = n * v1 - o;
        let d2 = n * v2 - o;

        let mut first = None;
        let mut second = None;

        // println!("d1: {d1}, d2: {d2}");

        if d1 >= 0. {
            first = Some(v1);
        }
        if d2 >= 0. {
            match first {
                None => first = Some(v2),
                _ => second = Some(v2),
            }
        }

        if d1 * d2 < 0. {
            let mut e = v2 - v1;
            let u = d1 / (d1 - d2);
            e *= u;
            e += v1;

            match first {
                None => first = Some(e),
                _ => second = Some(e),
            }
        }

        ClippedPoints { first, second }
    }

    fn len(&self) -> usize {
        match (self.first, self.second) {
            (None, None) => 0,
            (Some(_), None) | (None, Some(_)) => 1,
            (Some(_), Some(_)) => 2,
        }
    }

    fn remove(&mut self, point: Vec2D) {
        match (self.first, self.second) {
            (Some(p), _) if p == point => self.first = None,
            (_, Some(p)) if p == point => self.second = None,
            _ => (),
        }
    }

    fn to_res(self) -> (Option<Vec2D>, Option<Vec2D>) {
        (self.first, self.second)
    }
}

impl Index<usize> for ClippedPoints {
    type Output = Vec2D;

    fn index(&self, index: usize) -> &Self::Output {
        match (index, &self.first, &self.second) {
            (0, Some(first), _) => first,
            (1, _, Some(second)) => second,
            _ => panic!("Error man"),
        }
    }
}

pub fn get_contact_points(
    a: &Vec<Vec2D>,
    b: &Vec<Vec2D>,
    n: Vec2D,
) -> (Option<Vec2D>, Option<Vec2D>) {
    let e1 = get_best_edge(a, n);
    let e2 = get_best_edge(b, -n);

    let mut refe = &e1;
    let mut inc = &e2;

    let mut flipped = false;

    if (e1.dot(n)).abs() > (e2.dot(n)).abs() {
        refe = &e2;
        inc = &e1;
        flipped = true;
    }

    // println!("refe: {:?}, inc: {:?}, flipped: {flipped}", refe, inc);
    // clg!("refe: {:#?}, inc: {:#?}", refe, inc);

    let refv = refe.to_vec().normalize();

    let o1 = refv * refe.start;

    println!("refv: {refv}, o1: {o1}");

    let cp = ClippedPoints::clip(inc.start, inc.end, refv, o1);

    // clg!("Premiers points: {:#?}", cp);

    if cp.len() < 2 {
        (None, None)
    } else {
        let o2 = refv * refe.end;

        // println!("o2: {o2}, cp[0]: {}, cp[1]: {}", cp[0], cp[1]);

        let mut cp = ClippedPoints::clip(cp[0], cp[1], -refv, -o2);

        // clg!("Seconds points: {:#?}", cp);

        if cp.len() < 2 {
            (None, None)
        } else {
            // let mut ref_norm = refv
            //     .to_3d()
            //     .cross(Vec3D::new(0., 0., -1.))
            //     .to_2d()
            //     .normalize();

            let ref_norm = Vec2D::new(-refv.y, refv.x);

            // let c = Collider::PolygonCollider {
            //     vertices: if !flipped { a.to_vec() } else { b.to_vec() },
            // }
            // .center();

            // println!("c: {c}, pre ref_norm: {ref_norm}");

            // if c * ref_norm > 0. {
            //     ref_norm = -ref_norm;
            // }

            // clg!("refv: {refv}, ref_norm: {ref_norm}");

            // println!("pre ref norm: {ref_norm}");

            // if flipped {
            //     ref_norm = -ref_norm;
            // }

            let max = ref_norm * refe.max;

            // clg!("ref_norm: {ref_norm}, max: {max}");

            // clg!("ref_norm * cp[0]: {}", ref_norm * cp[0]);
            if ref_norm * cp[0] - max < 0. {
                cp.remove(cp[0]);
            }

            // clg!("ref_norm * cp[1]: {}", ref_norm * cp[1]);
            if ref_norm * cp[1] - max < 0. {
                cp.remove(cp[1]);
            }

            cp.to_res()
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{
        collision::contact_point::{get_best_edge, get_contact_points, ClippedPoints, Edge},
        linalg::Vec2D,
    };

    #[test]
    fn example_one() {
        let a = vec![
            Vec2D::new(8., 4.),
            Vec2D::new(14., 4.),
            Vec2D::new(14., 8.),
            Vec2D::new(8., 8.),
        ];
        let b = vec![
            Vec2D::new(4., 2.),
            Vec2D::new(12., 2.),
            Vec2D::new(12., 5.),
            Vec2D::new(4., 5.),
        ];

        let n = Vec2D::new(0., -1.);

        assert_eq!(
            get_best_edge(&a, n),
            Edge {
                max: Vec2D::new(8., 4.),
                start: Vec2D::new(8., 4.),
                end: Vec2D::new(14., 4.)
            }
        );

        assert_eq!(
            ClippedPoints::clip(
                Vec2D::new(12., 5.),
                Vec2D::new(4., 5.),
                Vec2D::new(1., 0.),
                8.
            ),
            ClippedPoints {
                // points: vec![Vec2D::new(12., 5.), Vec2D::new(8., 5.)]
                first: Some(Vec2D::new(12., 5.)),
                second: Some(Vec2D::new(8., 5.)),
            }
        );

        assert_eq!(
            ClippedPoints::clip(
                Vec2D::new(12., 5.),
                Vec2D::new(8., 5.),
                Vec2D::new(-1., 0.),
                -14.
            ),
            ClippedPoints {
                first: Some(Vec2D::new(12., 5.)),
                second: Some(Vec2D::new(8., 5.))
            }
        );

        let cp = get_contact_points(&a, &b, n);

        assert_eq!(cp, (Some(Vec2D::new(12., 5.)), Some(Vec2D::new(8., 5.))))
    }

    #[test]
    fn example_two() {
        let a = vec![
            Vec2D::new(2., 8.),
            Vec2D::new(6., 4.),
            Vec2D::new(9., 7.),
            Vec2D::new(5., 11.),
        ];
        let b = vec![
            Vec2D::new(4., 2.),
            Vec2D::new(12., 2.),
            Vec2D::new(12., 5.),
            Vec2D::new(4., 5.),
        ];

        let n = Vec2D::new(0., -1.);

        assert_eq!(
            get_best_edge(&a, n),
            Edge {
                max: Vec2D::new(6., 4.),
                start: Vec2D::new(2., 8.),
                end: Vec2D::new(6., 4.),
            }
        );

        assert_eq!(
            get_best_edge(&b, -n),
            Edge {
                max: Vec2D::new(12., 5.),
                start: Vec2D::new(12., 5.),
                end: Vec2D::new(4., 5.)
            }
        );

        let cp = get_contact_points(&a, &b, n);

        assert_eq!(cp, (Some(Vec2D::new(6., 4.)), None));
    }

    #[test]
    fn example_three() {
        let a = vec![
            Vec2D::new(9., 4.),
            Vec2D::new(13., 3.),
            Vec2D::new(14., 7.),
            Vec2D::new(10., 8.),
        ];
        let b = vec![
            Vec2D::new(4., 2.),
            Vec2D::new(12., 2.),
            Vec2D::new(12., 5.),
            Vec2D::new(4., 5.),
        ];

        let n = Vec2D::new(-0.19, -0.98);

        let cp = get_contact_points(&a, &b, n);

        assert_eq!(cp, (Some(Vec2D::new(12., 5.)), Some(Vec2D::new(9.25, 5.))));
    }
}
