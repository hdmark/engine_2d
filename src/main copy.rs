use std::panic;

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
    // position: Vec2,
    // // speed: f32,
    // velocity: Vec2,
    // inv_mass: f32,
    // restitution: f32,
}
pub struct RectE2d {
    pub min: Vec2,
    pub max: Vec2,
}
pub enum Collider {
    CircleE2d(CircleE2d),
    RectE2d(RectE2d),
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
    pub fn new(min: Vec2, max: Vec2) -> Self {
        // RectE2d { half }
        todo!()
    }

    pub fn default() -> Self {
        todo!()
    }
}
impl Body {
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
fn rc(a: &mut Body, b: &mut Body) {
    // let  a =  self.items[0];
    // let  b = self.items[1];
    let rv = b.velocity - a.velocity;
    let mut n = b.position - a.position;
    // let mut n =  Vec2::new(
    //      self.items[1].position.x - self.items[0].position.x,
    //      self.items[1].position.y - self.items[0].position.y,
    // );
    n /= glm::length(&n);

    let vel_along_normal = rv.dot(&n);
    dbg!(rv.dot(&n));

    if vel_along_normal > 0.0 {
        dbg!("no collision");
        return;
    }
    dbg!(" collision");

    let e: f32 = f32::min(a.restitution, b.restitution);
    let mut j: f32 = -(1.0 + e) * vel_along_normal;
    j /= b.inv_mass + a.inv_mass;
    let impulse = j * n;
    // let impulse = VecE2d { x: j * n.x, y: j * n.y };
    dbg!(j, impulse.x, impulse.y, vel_along_normal);
    a.velocity -= a.inv_mass * impulse;
    b.velocity += b.inv_mass * impulse;
    // self.objects[0].velocity.x -= self.objects[0].inv_mass * impulse.x;
    // self.objects[0].velocity.y -= self.objects[0].inv_mass * impulse.y;
    // self.objects[1].velocity.x += self.objects[1].inv_mass * impulse.x;
    // self.objects[1].velocity.y += self.objects[1].inv_mass * impulse.y;
}
// struct Shape {}
// struct Aabb {
//     min: PointNA,
//     max: Vec2,
// }
// impl Default for Aabb {
//     fn default() -> Self {
//         Aabb {
//             min: na::point!(20.0, 60.0),
//             max: Vec2::new(0.0, 1.0),
//         }
//     }
// }
// impl Aabb {
//     fn get_pix_point(&self) -> PointPE {
//         return pix_engine::point!(self.min.x, self.min.y);
//     }
// }
struct Manifold {
    a: Body,
    b: Body,
    penetration: f32,
    normal: Vec2,
}

// trait ObjectE2d

// fn circle_vs_circle(mut m : Manifold) -> bool{
//     let a : &CircleE2d = &m.a;
//     let b : &CircleE2d = &m.b;

//     let n: VecE2d = VecE2d{x: b.position.x - a.position.x, y: b.position.y - a.position.x};
//     let mut r : f32 = (a.radius + b.radius) as f32;
//     r*= r;
//     // no collision
//     if n.length_squared() > r {
//         dbg!("no collision");
//         return false;
//     }

//     dbg!("collision");
//     // collision
//     let d: f32 = n.length();
//     if(d != 0.){
//         m.penetration = r - d;``
//         m.normal.x /= d;
//         m.normal.y /= d;
//     }

//     return true;

// }
// fn circle_vs_circle_opt(a: &Collider<CircleE2d>, b: &Body) -> bool {
//     let mut r = a.radius + b.radius;
//     r *= r;
//     // dbg!(r, a.position.x, b.position.x, a.position.y, b.position.y);
//     // return r > (a - b).powf(2.0);
//     return r > (a.position.x - b.position.x).powf(2.0) + (a.position.y - b.position.y).powf(2.0);
// }
// fn aabb_vs_aabb(a: Aabb, b: Aabb) -> bool {
//     if a.max.x < b.min.x || a.min.x < b.max.x {
//         return false;
//     }
//     if a.max.y < b.min.y || a.min.y < b.max.y {
//         return false;
//     }
//     return true;
// }

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

        // s.text_area("Test", 100, 100, &mut self.text_area)?;
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
            velocity: Vec2::new(0.0, 10.0),
            inv_mass: 1.0,
            restitution: 0.8,
            collider: Collider::CircleE2d(CircleE2d { radius: 2.0 }),
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

        let r = Rect::new(0, (max_height as i32) - 20, max_width as i32, max_height as i32);
        s.rect(r)?;
        for i in self.objects.iter() {
            match &i.collider {
                Collider::CircleE2d(c) => {
                    s.circle([i.position.x as i32, i.position.y as i32, c.radius as i32])?;
                }
                Collider::RectE2d(rect_e2d) => todo!(),
            }
            //         (CircleE2d(a),
            // s.circle([i.position.x as i32, i.position.y as i32, i.radius as i32])?
            // s.circle([i.position.x as i32, i.position.y as i32, i.radius as i32])?;
        }
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
            // else if c.position.y + c.radius < max_height {
            //     c.position.y += c.speed;
            // } else if c.position.y + c.radius == max_height {
            //     c.position.x += -c.speed * ((idx as f32)+1.);
            // }
        }

        if self.objects.len() >= 2 {
            dbg!(self.objects.len());
            let i = self.objects.len() - 2;
            let j = self.objects.len() - 1;
            // if circle_vs_circle_opt(&self.objects[i], &self.objects[j]) {
            // s.text("Collision")?;
            // self.resolve_collision();

            let (i, j) = if i < j { (i, j) } else { (j, i) };
            let (left, right) = self.objects.split_at_mut(j);
            rc(&mut left[i], &mut right[0]);
            // rc(&mut self.objects[0], &mut self.objects[1]);
            // }
            // resolve_collision(&mut self.items[0], &mut self.items[1]);
            //  if circle_vs_circle_opt(&self.items[0], &self.items[1]){
            //             println!("Collision");
            //             self.items[1].speed *= -1;
            //        }
        }

        // remove items off screen
        for idx in items_to_remove.iter().rev() {
            self.objects.remove(*idx);
        }

        // Draw a circle with fill color at the mouse position with a radius of
        // 80.
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
                min: Vec2::new(0.0, 0.0),
                max: Vec2::new(0.0, 0.0),
            }),
        },
    };

    let circle_body = Body {
        position: Vec2::new(480.0, 500.0),
        velocity: Vec2::new(0.0, 0.0),
        inv_mass: 0.0,
        restitution: 0.8,
        collider: Collider::CircleE2d(CircleE2d { radius: 2.0 }),
    };
    app.objects.push(circle_body);
    // let platform = Body {
    //     position: Vec2::new(480.0, 500.0),
    //     velocity: Vec2::new(0.0, 0.0),
    //     inv_mass: 0.0,
    //     restitution: 0.8,
    //     collider: Collider::RectE2d(RectE2d { min: Vec2::new(0.0, 0.0), max: Vec2::new(0.0, 0.0) }),
    // };
    // app.platform = platform;
    // CircleE2d::default()

    //     app.items[1].position.x = 500.;
    //     app.items[1].position.y = 500.;
    // .
    // app.objects[0].position.x = 480.0;
    // app.objects[0].radius = 200.0;
    // app.objects[0].inv_mass = 0.0;
    // app.objects[0].position.y = 500.0;
    // app.objects[0].velocity.y = 0.0;

    // app.items[0].position.x -= 10.;
    // app.items[1].velocity.y  =0.;
    engine.run(&mut app)
}
