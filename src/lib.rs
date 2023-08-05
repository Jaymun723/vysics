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

    // crate::collision::algorithms::epa::tests::example_four();

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

            request_animation_frame(f.borrow().as_ref().unwrap());
        }));
    }

    request_animation_frame(g.borrow().as_ref().unwrap());
    Ok(())
}
