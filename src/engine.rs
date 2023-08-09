use wasm_bindgen::prelude::*;
use web_sys::CanvasRenderingContext2d;

use crate::{
    collision::{
        collider::Collider::{self, CircleCollider,PolygonCollider},
        algorithms::
            gjk::gjk_collision
        ,
        manifold::ContactManifold,
    },
    linalg::Vec2D,
    render::{circle, line, polygon, rect},
    rigidbody2d::{force_generator::GravityGenerator, RigidBody2D},
};

// #[wasm_bindgen]
// extern "C" {
//     #[wasm_bindgen(js_namespace = console)]
//     fn log(s: &str);
// }

// macro_rules! clg {
//     ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
// }

pub struct Engine {
    pub width: f64,
    pub height: f64,
    pub bodies: Vec<RigidBody2D>,
}

impl Engine {
    pub fn demo_circle(width: f64, height: f64) -> Self {
        let circle_collider = CircleCollider { radius: 100. };
        let mut circle1 = RigidBody2D::new(Vec2D::new(width / 2., 0.), circle_collider, 1.);

        circle1.add_force_generator(Box::new(GravityGenerator {}));

        Engine {
            width,
            height,
            bodies: vec![circle1],
        }
    }

    pub fn demo_rectangle(width: f64, height: f64) -> Self {
        let rectangle_collider = Collider::rectangle(50., 50.);
        let mut rectangle1 =
            RigidBody2D::new(Vec2D::new(width / 2., height / 2.), rectangle_collider, 1.);

        rectangle1.angular_velocity = 0.1;

        Engine {
            width,
            height,
            bodies: vec![rectangle1],
        }
    }

    pub fn demo_collide(width: f64, height: f64) -> Self {
        let rectangle_collider1 = Collider::rectangle(100., 100.);
        // let circle_collider1 = Collider::circle(20.);
        let rectangle_collider2 = Collider::rectangle(200., 100.);

        let rectangle1 = 
        // thing
        RigidBody2D::new(Vec2D::new(400., 250.), rectangle_collider1, 1.);
        // RigidBody2D::new(Vec2D::new(400., 250.), circle_collider1, 1.);

        
        let rectangle2 = RigidBody2D::new(Vec2D::new(426., 301.), rectangle_collider2, 1.);

        // rectangle1.angle = std::f64::consts::PI;
        // rectangle1.angular_velocity = 0.3;

        Engine {
            width,
            height,
            bodies: vec![rectangle1, rectangle2],
        }
    }

    pub fn demo_collide_circle(width: f64, height: f64) -> Self {
        let circle_collider1 = CircleCollider { radius: 20. };
        let circle_collider2 = CircleCollider { radius: 30. };
        let circle1 = RigidBody2D::new(Vec2D::new(width / 2., height / 2.), circle_collider1, 1.);
        let circle2 = RigidBody2D::new(Vec2D::new(width / 2., height / 2.), circle_collider2, 1.);

        // circle1.add_force_generator(Box::new(GravityGenerator {}));

        Engine {
            width,
            height,
            bodies: vec![circle1, circle2],
        }
    }

    /// dt: in ms
    pub fn physics_step(&mut self, dt: f64) {
        for body in self.bodies.iter_mut() {
            body.step(dt / 1000.);
        }
    }

    pub fn draw(&self, ctx: &CanvasRenderingContext2d) -> Result<(), JsValue> {
        if self.bodies.len() == 2 {
            let a = &self.bodies[0];
            let b = &self.bodies[1];

            match gjk_collision(a, b) {
                Some(_) => {
                    rect(
                        &ctx,
                        Vec2D::zero(),
                        Vec2D::new(self.width, self.height),
                        "grey",
                    );
                }
                _ => (),
            }
        }

        for body in self.bodies.iter() {
            match &body.collider {
                CircleCollider { radius } => {
                    ctx.set_fill_style(&"red".into());
                    ctx.set_stroke_style(&"black".into());
                    ctx.begin_path();

                    ctx.arc(
                        body.position.x,
                        body.position.y,
                        *radius,
                        0.,
                        std::f64::consts::PI * 2.,
                    )?;

                    ctx.fill();
                    ctx.stroke();
                }
                PolygonCollider { vertices } => {
                    // let mut world_verticies = vec![];

                    // for vertex in vertices {
                    //     world_verticies.push(body.to_global(*vertex));
                    // }

                    let world_verticies: Vec<Vec2D> =
                        vertices.iter().map(|v| body.to_global(*v)).collect();

                    polygon(ctx, &world_verticies, "black");

                    circle(ctx, body.position, 10., "red")?;
                }
            }
        }

        if self.bodies.len() == 2 {
            let a = &self.bodies[0].clone();
            let b = &self.bodies[1].clone();

            let mut manifold = ContactManifold::new(a, b);
            manifold.compute();

            if let Some(normal) = manifold.normal {
                for (point, depth) in manifold.points {
                    circle(&ctx, point, 5., "green")?;
                    
                    line(&ctx, point, point + normal * depth, "green");
                }
            }



            // match sat(a, b) {
            //     Some(mtv) => {
            //         // if let None = gjk_collision(a, b) {
            //         //     panic!("Pas normal: a: {:#?}\nb: {:#?}",a,b);
            //         // }

            //         match gjk_collision(a, b) {
            //             None => panic!("Pas normal: a: {:#?}\nb: {:#?}",a,b),
            //             Some(s) => {
            //                 let CollisionResult {
            //                     depth,
            //                     normal,
            //                     point_a,
            //                     point_b,
            //                 } = epa(s, a, b);
            //                 circle(&ctx, point_a, 10., "purple")?;
            //                 circle(&ctx, point_b, 10., "purple")?;
            //                 line(&ctx, point_a, point_a + normal * depth, "green");
            //             }
            //         };

            //         // if (mtv_epa - mtv).near_zero() {
            //         //    rect(&ctx, Vec2D::zero(), Vec2D::new(30.,30.), "cyan"); 
            //         // }

            //         // clg!("Hey !");
            //         let center = Vec2D::new(self.width / 4., self.height / 4.);
            //         line(&ctx, center, center + mtv, "green");
            //     },
            //     None => ()
            // }

            // match gjk_collision(a, b) {
            //     None => {
            //         if let Some((dist, alpha, beta)) = gjk_distance(a, b) {
            //             circle(&ctx, alpha, 10., "green")?;
            //             circle(&ctx, beta, 10., "green")?;
            //             let n = (beta - alpha).normalize();
            //             line(&ctx, alpha, alpha + n * dist, "green");
            //         }
            //     }
            //     Some(simplex) => {
            //         // clg!("pre Epa");
            //         let CollisionResult {
            //             depth,
            //             normal,
            //             point_a,
            //             point_b,
            //         } = epa(simplex, a, b);
            //         // // clg!("post Epa");

            //         circle(&ctx, point_a, 10., "purple")?;
            //         circle(&ctx, point_b, 10., "purple")?;
            //         line(&ctx, point_a, point_a + normal * depth, "green");
            //     }
            // }

            // if let Some(_) = gjk(a, b) {
            //     ctx.set_fill_style(&"purple".into());
            //     ctx.fill_rect(0., 0., self.width, self.height);
            // }

            // let mut manifold = ContactManifold::new(a, b);

            // narrow_sphere_v_sphere(&mut manifold);

            // if let Some(n) = manifold.normal {
            //     let center = Vec2D::new(self.width / 2., self.height / 2.);

            //     line(&ctx, center, center + n * 10., "yellow");

            //     let p = manifold.points[0];

            //     circle(&ctx, p, 10., "green")?;
            // }
        }

        // if self.bodies.len() == 2 {
        //     let a = &self.bodies[0];
        //     let b = &self.bodies[1];

        //     match gjk(a, b) {
        //         None => (),
        //         Some(mut simplex) => {
        //             let CollisionResult {
        //                 point_a,
        //                 point_b,
        //                 normal,
        //                 depth,
        //             } = epa(&mut simplex, a, b);

        //             circle(ctx, point_a, 10., "cyan")?;
        //             circle(ctx, point_b, 10., "cyan")?;

        //             // match get_contact_points(a.collider., b, n)

        //             // let a_vertices = match &a.collider {
        //             //     CircleCollider { .. } => Err("wsh wsh"),
        //             //     Collider::PolygonCollider { vertices } => Ok(vertices
        //             //         .iter()
        //             //         .map(|v| a.to_global(*v))
        //             //         .collect::<Vec<Vec2D>>()),
        //             // }?;

        //             // let b_vertices = match &b.collider {
        //             //     CircleCollider { .. } => Err("wsh wsh"),
        //             //     Collider::PolygonCollider { vertices } => Ok(vertices
        //             //         .iter()
        //             //         .map(|v| b.to_global(*v))
        //             //         .collect::<Vec<Vec2D>>()),
        //             // }?;

        //             // clg!("{:#?}", a_vertices);
        //             // clg!("{:#?}", b_vertices);

        //             // let res = get_contact_points(&a_vertices, &b_vertices, normal);

        //             // clg!("{:#?}", res);

        //             // match res {
        //             //     (Some(p), None) | (None, Some(p)) => circle(ctx, p, 10., "grey")?,
        //             //     (Some(p), Some(q)) => {
        //             //         circle(ctx, p, 10., "grey")?;
        //             //         circle(ctx, q, 10., "grey")?
        //             //     }
        //             //     _ => (),
        //             // }

        //             // panic!("Panic at the disco !");
        //         }
        //     }
        // }

        Ok(())
    }
}
