use crate::linalg::Vec2D;

#[derive(Debug)]
pub enum Collider {
    CircleCollider { radius: f64 },
    PolygonCollider { vertices: Vec<Vec2D> },
}

impl Collider {
    pub fn circle(radius: f64) -> Self {
        Self::CircleCollider { radius }
    }

    pub fn rectangle(width: f64, height: f64) -> Self {
        Self::PolygonCollider {
            vertices: vec![
                Vec2D::new(-width / 2., -height / 2.),
                Vec2D::new(-width / 2., height / 2.),
                Vec2D::new(width / 2., height / 2.),
                Vec2D::new(width / 2., -height / 2.),
            ],
        }
    }

    pub fn support(&self, direction: Vec2D) -> Vec2D {
        match self {
            Self::CircleCollider { radius } => direction * (radius / direction.norm()),
            Self::PolygonCollider { vertices } => {
                let mut furthest: Option<Vec2D> = None;

                for vertex in vertices {
                    match furthest {
                        None => furthest = Some(*vertex),
                        Some(point) => {
                            if (*vertex) * direction > point * direction {
                                furthest = Some(*vertex);
                            }
                        }
                    }
                }

                furthest.unwrap()
            }
        }
    }

    pub fn center(&self) -> Vec2D {
        match self {
            Self::CircleCollider { .. } => Vec2D::zero(),
            Self::PolygonCollider { vertices } => {
                let mut i = 0.;
                let mut center = Vec2D::zero();
                for vertex in vertices {
                    i += 1.;
                    center += *vertex;
                }
                center / i
            }
        }
    }
}
