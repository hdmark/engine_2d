use std::panic;

use glm::{ abs, sqrt };
use pix_engine::prelude::*;
use nalgebra::*;
extern crate nalgebra as na;
extern crate nalgebra_glm as glm;
const SPEED: f32 = 1.0;

type Vec2 = na::Vector2<f32>;
type PointPE = pix_engine::prelude::Point<f32>;
type PointNA = na::Point2<f32>;
struct MyApp {
    objects: Vec<Body>,
    platform: Body,
}

impl MyApp {}
pub struct CircleE2d {
    radius: f32,
}
pub struct RectE2d {
    // pub min: Vec2,
    // pub max: Vec2,
    pub height: f32,
    pub width: f32,
}
pub enum Collider {
    CircleE2d(CircleE2d),
    RectE2d(RectE2d),
}
impl Collider {
    fn as_circle(&self) -> &CircleE2d {
        match self {
            Collider::CircleE2d(c) => c,
            _ => panic!("Expected CircleE2d"),
        }
    }
    fn as_rect(&self) -> &RectE2d {
        match self {
            Collider::RectE2d(c) => c,
            _ => panic!("Expected RectE2d"),
        }
    }
}

pub struct Body {
    pub position: Vec2,
    pub velocity: Vec2,
    pub inv_mass: f32,
    pub restitution: f32,
    pub collider: Collider,
}
impl CircleE2d {
    pub fn new(radius: f32) -> Self {
        CircleE2d { radius }
    }

    pub fn default() -> Self {
        CircleE2d { radius: 1.0 }
    }
}

impl RectE2d {
    pub fn new(width: f32, height: f32) -> Self {
        // RectE2d { half }
        // TODO - do pos + width /2 and pos + height /2
        RectE2d { width, height: height }
    }

    pub fn default() -> Self {
        todo!()
    }
}
impl Body {
    pub fn to_draw(&self) -> Vec<i32> {
        match &self.collider {
            Collider::CircleE2d(c) => {
                vec![self.position.x as i32, self.position.y as i32, c.radius as i32]
            }
            Collider::RectE2d(r) => {
                vec![
                    (self.position.x - r.width / 2.0) as i32,
                    (self.position.y - r.height / 2.0) as i32,

                    r.height as i32,
                    r.width as i32
                ]
            }
        }
    }
    // pub fn resolve_collision_with(&mut self, other: &mut Body) {
    //     use Collider::*;
    //     match (self.collider, other.collider) {
    //         (CircleE2d(a), CircleE2d(b)) => circle_circle(a, self, b, other),
    //         (CircleE2d(a), RectE2d(b)) => circle_square(a, self, b, other),
    //         (RectE2d(a), CircleE2d(b)) => circle_square(b, other, a, self),
    //         (RectE2d(a), RectE2d(b)) => square_square(a, self, b, other),
    //     }
    // }
}
// TODO - make manifold generation
// TODO - make resolve collision functions, combine?
// TODO - builder pattern?

pub struct Manifold<'a> {
    a: &'a Body,
    b: &'a Body,
    penetration: f32,
    normal: Vec2,
}
pub fn circle_vs_circle(m: &mut Manifold) -> bool {
    dbg!("c v c check");
    // let ca = m.a;
    // let cb = m.b;
    let sa = m.a.collider.as_circle();
    let sb = m.b.collider.as_circle();

    let n: Vec2 = m.b.position - m.a.position;
    let mut r = sa.radius + sb.radius;
    // r *= r;
    if n.magnitude_squared() > r * r {
        return false;
    }
    let d = n.magnitude();

    if d != 0.0 {
        m.penetration = r - d;
        m.normal = n.normalize();
    } else {
        m.penetration = sa.radius;
        m.normal = Vec2::new(1.0, 0.0);
    }

    return true;
}
pub fn aabb_vs_circle(m: &mut Manifold) -> bool {
    // dbg!("r v c check");

    // let col_a = m.a;
    // let col_b = m.b;
    let rect_a = m.a.collider.as_rect();
    let circ_b = m.b.collider.as_circle();

    // vec from a to b
    let n: Vec2 = m.a.position - m.b.position;
    // dbg!(col_b.position, col_a.position);
    // closest point on a to center of b
    let mut closest: Vec2 = n;

    // calc half extents
    let x_extent = rect_a.width / 2.0;
    let y_extent = rect_a.height / 2.0;

    // clamp point to edges of the aabb
    closest.x = clamp(closest.x, -x_extent, x_extent);
    closest.y = clamp(closest.y, -y_extent, y_extent);
    // dbg!(x_extent, y_extent);
    let mut inside = false;
    // dbg!(n, closest);

    // if circle inside, clamp center to closest edge
    if n == closest {
        dbg!("inside", n);
        inside = true;
        if n.x.abs() > n.y.abs() {
            // clamp to closest extent
            if closest.x > 0.0 {
                closest.x = x_extent;
            } else {
                closest.x = -x_extent;
            }
        } else {
            // y axis is shorter

            if closest.y > 0.0 {
                closest.y = y_extent;
            } else {
                closest.y = -y_extent;
            }
        }
    }
    let normal = n - closest;
    let mut d = normal.magnitude_squared();
    let r = circ_b.radius;

    // Early out of the radius is shorter than distance to closest point and
    // Circle not inside the AABB

    if d > r * r && !inside {
        // dbg!("early out");
        return false;
    }
    d = d.sqrt();

    // Collision normal needs to be flipped to point outside if circle was
    // inside the AABB
    // dbg!(inside);
    if inside {
        m.penetration = r - d;
        m.normal = normal.normalize();
    } else {
        m.penetration = r - d;
        m.normal = normal.normalize();
    }
    dbg!(closest, n, normal, d, r, m.penetration, m.normal, inside);

    return true;
}
pub fn circle_vs_aabb(m: &mut Manifold) -> bool {
    dbg!("c v a");
    let c = m.a;
    m.a = m.b;
    m.b = c;
    let col = aabb_vs_circle(m);
    m.normal *= -1.0;
    return col;
}
fn rc(a: &mut Body, b: &mut Body) {
    let mut m = Manifold {
        a,
        b,
        penetration: 0.0,
        normal: Vec2::new(0.0, 0.0),
    };
    let col: bool = match (&a.collider, &b.collider) {
        (Collider::CircleE2d(i), Collider::CircleE2d(j)) => {
            // let mut r = i.radius + j.radius;
            // r *= r;
            // r > (a.position.x - b.position.x).powf(2.0) + (a.position.y - b.position.y).powf(2.0)
            circle_vs_circle(&mut m)
        }
        (Collider::CircleE2d(circle_e2d), Collider::RectE2d(rect_e2d)) => { circle_vs_aabb(&mut m) }
        (Collider::RectE2d(rect_e2d), Collider::CircleE2d(circle_e2d)) => { aabb_vs_circle(&mut m) }
        (Collider::RectE2d(i), Collider::RectE2d(j)) => todo!(),
    };
    if !col {
        // dbg!("no col");
        return;
    }
    let penetration = m.penetration;
    let rv = b.velocity - a.velocity;
    // let mut n = b.position - a.position;
    // n /= glm::length(&n);
    let n = m.normal;

    let vel_along_normal = rv.dot(&n);
    dbg!(rv, n, rv.dot(&n));

    if vel_along_normal > 0.0 {
        dbg!("no collision");
        return;
    }
    dbg!(" collision");

    let e: f32 = f32::min(a.restitution, b.restitution);
    let mut j: f32 = -(1.0 + e) * vel_along_normal;
    j /= b.inv_mass + a.inv_mass;
    let impulse = j * n;
    dbg!(j, impulse.x, impulse.y, vel_along_normal, e);
    a.velocity -= a.inv_mass * impulse;
    b.velocity += b.inv_mass * impulse;

    // position correction
    let percent = 0.2;
    let slop = 0.01;
    let correction = ((penetration - slop).max(0.0) / (a.inv_mass + b.inv_mass)) * n * percent;
    dbg!(penetration, correction);
    a.position -= a.inv_mass * correction;
    b.position += b.inv_mass * correction;
}

impl PixEngine for MyApp {
    // Set up application state and initial settings. `PixState` contains
    // engine specific state and utility methods for actions like getting mouse
    // coordinates, drawing shapes, etc. (Optional)
    fn on_start(&mut self, s: &mut PixState) -> PixResult<()> {
        // Set the background to GRAY and clear the screen.
        s.background(Color::GRAY);

        // Change the font family to NOTO and size to 16 instead of using the
        // defaults.
        s.font_family(Font::NOTO)?;
        s.font_size(16)?;

        // Returning `Err` instead of `Ok` would indicate initialization failed,
        // and that the application should terminate immediately.
        Ok(())
    }

    fn on_mouse_clicked(
        &mut self,
        _s: &mut PixState,
        _btn: Mouse,
        pos: pix_engine::prelude::Point<i32>
    ) -> PixResult<bool> {
        let circle_body = Body {
            position: Vec2::new(pos.x() as f32, pos.y() as f32),
            velocity: Vec2::new(0.0, 5.0),
            inv_mass: 1.0,
            restitution: 0.8,
            collider: Collider::CircleE2d(CircleE2d { radius: 20.0 }),
        };
        self.objects.push(circle_body);

        Ok(true)
    }
    // Main update/render loop. Called as often as possible unless
    // `target_frame_rate` was set with a value. (Required)
    fn on_update(&mut self, s: &mut PixState) -> PixResult<()> {
        let max_height: f32 = s.window_height().unwrap() as f32;
        let max_width: f32 = s.window_width().unwrap() as f32;
        // reset image
        s.background(Color::GRAY);
        s.text("texting")?;
        // Set fill color to black if mouse is pressed, otherwise wite.
        s.fill(color!(0));

        // UPDATE PHYSICS
        if self.objects.len() >= 2 {
            // dbg!(self.objects.len());
            let i = self.objects.len() - 2;
            let j = self.objects.len() - 1;

            let (i, j) = if i < j { (i, j) } else { (j, i) };
            let (left, right) = self.objects.split_at_mut(j);

            rc(&mut left[i], &mut right[0]);
        }

        // Adjust positions
        let mut items_to_remove: Vec<usize> = Vec::new();
        for (idx, c) in self.objects.iter_mut().enumerate() {
            // grab to remove later
            if
                c.position.x > max_width ||
                c.position.x < 0.0 ||
                c.position.y > max_height ||
                c.position.y < 0.0
            {
                items_to_remove.push(idx);
            }
            {
                c.position.y += c.velocity.y;
                c.position.x += c.velocity.x;
                // dbg!(c.position.y);
            }
        }
        // remove items off screen
        for idx in items_to_remove.iter().rev() {
            self.objects.remove(*idx);
        }

        // draw all items
        let r = Rect::new(0, (max_height as i32) - 20, max_width as i32, max_height as i32);
        s.rect(r)?;
        for i in self.objects.iter() {
            let v = i.to_draw();
            match &i.collider {
                Collider::CircleE2d(c) => {
                    s.fill(Color::ALICE_BLUE);
                    s.circle([v[0], v[1], v[2]])?;
                    // s.circle([i.position.x as i32, i.position.y as i32, c.radius as i32])?;
                }
                Collider::RectE2d(r) => {
                    s.fill(color!(255));
                    s.rect([v[0], v[1], v[2], v[3]])?;
                    // s.rect(rect!([0, 0], 100, 100))?;
                }
            }
        }
        let m = s.mouse_pos();
        s.circle([m.x(), m.y(), 5])?;

        Ok(())
    }

    // Clean up any state or resources before exiting such as deleting temporary
    // files or saving game state. (Optional)
    fn on_stop(&mut self, s: &mut PixState) -> PixResult<()> {
        Ok(())
    }
}

fn main() -> PixResult<()> {
    let mut engine = Engine::builder()
        .dimensions(1200, 800)
        .title("PhysicsTest")
        .show_frame_rate()
        .target_frame_rate(60)
        .resizable()
        .build()?;
    let mut app = MyApp {
        objects: vec!(),
        platform: Body {
            position: Vec2::new(480.0, 500.0),
            velocity: Vec2::new(0.0, 0.0),
            inv_mass: 0.0,
            restitution: 0.8,
            collider: Collider::RectE2d(RectE2d {
                height: 200.0,
                width: 200.0,
            }),
        },
    };
    let rect_body = Body {
        position: Vec2::new(480.0, 500.0),
        velocity: Vec2::new(0.0, 0.0),
        inv_mass: 0.0,
        restitution: 0.8,
        collider: Collider::RectE2d(RectE2d {
            height: 100.0,
            width: 100.0,
        }),
    };
    let circle_body = Body {
        position: Vec2::new(480.0, 500.0),
        velocity: Vec2::new(0.0, 0.0),
        inv_mass: 0.0,
        restitution: 0.8,
        collider: Collider::CircleE2d(CircleE2d { radius: 200.0 }),
    };
    // app.objects.push(rect_body);

    app.objects.push(circle_body);

    engine.run(&mut app)
}
