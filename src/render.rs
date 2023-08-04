// pub mod text;

use std::f64::consts::PI;

use wasm_bindgen::JsValue;
use web_sys::CanvasRenderingContext2d;

use crate::linalg::Vec2D;

pub fn circle(
    ctx: &CanvasRenderingContext2d,
    center: Vec2D,
    radius: f64,
    color: &str,
) -> Result<(), JsValue> {
    ctx.begin_path();
    ctx.set_fill_style(&color.into());
    ctx.arc(center.x, center.y, radius, 0., 2. * PI)?;
    ctx.fill();

    Ok(())
}

pub fn line(ctx: &CanvasRenderingContext2d, a: Vec2D, b: Vec2D, color: &str) {
    ctx.begin_path();
    ctx.set_stroke_style(&color.into());
    ctx.move_to(a.x, a.y);
    ctx.line_to(b.x, b.y);
    ctx.stroke();
}

pub fn polygon(ctx: &CanvasRenderingContext2d, vertices: &Vec<Vec2D>, color: &str) {
    ctx.set_fill_style(&color.into());
    ctx.begin_path();
    let last = *vertices.last().unwrap();
    ctx.move_to(last.x, last.y);

    for vertex in vertices {
        ctx.line_to(vertex.x, vertex.y);
    }

    ctx.stroke();
    ctx.fill();
}

pub fn rect(ctx: &CanvasRenderingContext2d, p1: Vec2D, p2: Vec2D, color: &str) {
    ctx.set_fill_style(&color.into());
    let size = p2 - p1;
    ctx.fill_rect(p1.x, p1.y, size.x, size.y)
}
