pub mod collider;
pub mod collision;
pub mod engine;
pub mod linalg;
pub mod rigidbody2d;

pub mod render;

use crate::linalg::Vec2D;
use collision::contact_point::get_contact_points;
use engine::Engine;
use render::{circle, line, polygon};
use rigidbody2d::RigidBody2D;
use std::cell::{Cell, RefCell};
use std::f64::consts::PI;
use std::rc::Rc;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console)]
    fn log(s: &str);
}

macro_rules! clg {
    ($($t:tt)*) => (log(&format_args!($($t)*).to_string()))
}

fn window() -> web_sys::Window {
    web_sys::window().expect("no global `window` exists")
}

fn request_animation_frame(f: &Closure<dyn FnMut()>) {
    window()
        .request_animation_frame(f.as_ref().unchecked_ref())
        .expect("should register `requestAnimationFrame` OK");
}

fn document() -> web_sys::Document {
    window()
        .document()
        .expect("should have a document on window")
}

#[wasm_bindgen(start)]
fn run() -> Result<(), JsValue> {
    console_error_panic_hook::set_once();

    let f = Rc::new(RefCell::new(None));
    let g = f.clone();

    let performance = window().performance().unwrap();

    let canvas = document()
        .get_element_by_id("root")
        .unwrap()
        .dyn_into::<web_sys::HtmlCanvasElement>()?;

    let width = 800;
    let height = 500;

    canvas.set_width(width);
    canvas.set_height(height);

    let ctx = Rc::new(
        canvas
            .get_context("2d")?
            .unwrap()
            .dyn_into::<web_sys::CanvasRenderingContext2d>()?,
    );

    let mut date = performance.now();
    let mut accumulator = 0.;

    let mut engine = Engine::demo_collide(width as f64, height as f64);

    let pos = Rc::new(Cell::new(Vec2D::zero()));
    let clicked = Rc::new(Cell::new(false));

    {
        let pos = pos.clone();
        let closure = Closure::wrap(Box::new(move |event: web_sys::MouseEvent| {
            pos.set(Vec2D::new(event.offset_x() as f64, event.offset_y() as f64))
        }) as Box<dyn FnMut(_)>);
        canvas.add_event_listener_with_callback("mousemove", closure.as_ref().unchecked_ref())?;
        closure.forget();
    }

    {
        let clicked = clicked.clone();
        let closure = Closure::wrap(Box::new(move |_event: web_sys::MouseEvent| {
            clicked.set(true);
        }) as Box<dyn FnMut(_)>);
        canvas.add_event_listener_with_callback("mousedown", closure.as_ref().unchecked_ref())?;
        closure.forget();
    }

    {
        let clicked = clicked.clone();
        let closure = Closure::wrap(Box::new(move |_event: web_sys::MouseEvent| {
            clicked.set(false);
        }) as Box<dyn FnMut(_)>);
        canvas.add_event_listener_with_callback("mouseup", closure.as_ref().unchecked_ref())?;
        closure.forget();
    }

    {
        let pos = pos.clone();
        let clicked = clicked.clone();
        *g.borrow_mut() = Some(Closure::new(move || {
            let now = performance.now();

            let elapsed_time = now - date;

            accumulator += elapsed_time;

            date = now;

            let fixed_dt = 10.;

            while accumulator > fixed_dt {
                engine.physics_step(fixed_dt);

                accumulator -= fixed_dt;
            }

            engine.bodies[1].position = pos.get();
            if clicked.get() {
                engine.bodies[1].angular_velocity = 0.8;
            } else {
                engine.bodies[1].angular_velocity = 0.;
            }

            ctx.set_fill_style(&"white".into());
            ctx.fill_rect(0., 0., width as f64, height as f64);

            engine.draw(&ctx).unwrap();

            let _ = || {
                let a = vec![
                    Vec2D {
                        x: 400.98645446781956,
                        y: 196.97616660799673,
                    },
                    Vec2D {
                        x: 346.97616660799673,
                        y: 249.01354553218044,
                    },
                    Vec2D {
                        x: 399.01354553218044,
                        y: 303.02383339200327,
                    },
                    Vec2D {
                        x: 453.02383339200327,
                        y: 250.98645446781956,
                    },
                ];

                let b = vec![
                    Vec2D { x: 426.0, y: 176.0 },
                    Vec2D { x: 426.0, y: 226.0 },
                    Vec2D { x: 526.0, y: 226.0 },
                    Vec2D { x: 526.0, y: 176.0 },
                ];

                polygon(&ctx, &a, "blue");
                polygon(&ctx, &b, "red");

                let a_body = RigidBody2D::from_polygon(&a, 1.);
                let b_body = RigidBody2D::from_polygon(&b, 1.);

                // if let Some(mut simplex) = gjk(&a_body, &b_body) {
                //     let CollisionResult { normal: n, .. } = epa(&mut simplex, &a_body, &b_body);

                //     clg!("n: {n}");

                //     let center = Vec2D::new(width as f64 / 2., height as f64 / 2.);
                //     circle(&ctx, center, 10., "yellow").unwrap();

                //     line(&ctx, center, center + n * 50., "yellow");

                //     let cp = get_contact_points(&a, &b, n);

                //     // ref:
                //     circle(
                //         &ctx,
                //         Vec2D {
                //             x: 453.02383339200327,
                //             y: 250.98645446781956,
                //         },
                //         5.,
                //         "purple",
                //     )
                //     .unwrap();
                //     line(
                //         &ctx,
                //         Vec2D {
                //             x: 453.02383339200327,
                //             y: 250.98645446781956,
                //         },
                //         Vec2D {
                //             x: 400.98645446781956,
                //             y: 196.97616660799673,
                //         },
                //         "purple",
                //     );

                //     // ref_norm
                //     let ref_norm = Vec2D::new(0.7201, -0.6938);

                //     line(&ctx, center, center + ref_norm * 10., "green");

                //     clg!("{n}");

                //     match cp {
                //         (Some(p), _) | (_, Some(p)) => {
                //             clg!("{p}");
                //             circle(&ctx, p, 10., "green").unwrap()
                //         }
                //         _ => (),
                //     }

                //     panic!("wsh");

                //     // clg!("{:#?}", cp);
                // }
            };

            request_animation_frame(f.borrow().as_ref().unwrap());
        }));
    }

    request_animation_frame(g.borrow().as_ref().unwrap());
    Ok(())
}
