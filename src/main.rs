use pix_engine::prelude::*;
use nalgebra::*;
extern crate nalgebra as na;
extern crate nalgebra_glm as glm;
const SPEED: f32 = 1.0;
struct MyApp {
    objects: Vec<CircleE2d>,
}

impl MyApp {
    // fn rc(&mut self, i: usize, j: usize) {
    //     let a = self.objects.get_mut(i).unwrap();
    //     let b = self.objects.get_mut(j).unwrap();
    //     let rv = b.velocity - a.velocity;
    //     // let mut n = self.objects[1].position - self.objects[0].position;
    //     // // let mut n =  Vec2::new(
    //     // //      self.items[1].position.x - self.items[0].position.x,
    //     // //      self.items[1].position.y - self.items[0].position.y,
    //     // // );
    //     // n /= glm::length(&n);

    //     // let vel_along_normal = rv.dot(&n);
    //     // dbg!(rv.dot(&n));

    //     // if vel_along_normal > 0.0 {
    //     //     dbg!("no collision");
    //     //     return;
    //     // }
    //     // dbg!(" collision");

    //     // let e: f32 = f32::min(self.objects[0].restitution, self.objects[1].restitution);
    //     // let mut j: f32 = -(1.0 + e) * vel_along_normal;
    //     // j /= self.objects[1].inv_mass + self.objects[0].inv_mass;
    //     // let impulse = j * n;
    //     // // let impulse = VecE2d { x: j * n.x, y: j * n.y };
    //     // dbg!(j, impulse.x, impulse.y, vel_along_normal);
    //     a.velocity = a.velocity.add_scalar(1.0);
    //     b.velocity = b.velocity.add_scalar(1.0);
    // }
    fn resolve_collision(&mut self) {
        // let  a =  self.items[0];
        // let  b = self.items[1];
        let rv = Vec2::new(
            self.objects[1].velocity.x - self.objects[0].velocity.x,
            self.objects[1].velocity.y - self.objects[0].velocity.y
        );
        let mut n = self.objects[1].position - self.objects[0].position;
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

        let e: f32 = f32::min(self.objects[0].restitution, self.objects[1].restitution);
        let mut j: f32 = -(1.0 + e) * vel_along_normal;
        j /= self.objects[1].inv_mass + self.objects[0].inv_mass;
        let impulse = j * n;
        // let impulse = VecE2d { x: j * n.x, y: j * n.y };
        dbg!(j, impulse.x, impulse.y, vel_along_normal);

        self.objects[0].velocity.x -= self.objects[0].inv_mass * impulse.x;
        self.objects[0].velocity.y -= self.objects[0].inv_mass * impulse.y;
        self.objects[1].velocity.x += self.objects[1].inv_mass * impulse.x;
        self.objects[1].velocity.y += self.objects[1].inv_mass * impulse.y;
    }
}
fn rc(a: &mut CircleE2d, b: &mut CircleE2d) {
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
struct Shape {}
struct AABB {
    min: Vector2<f32>,
    max: Vector2<f32>,
}
struct Manifold {
    a: CircleE2d,
    b: CircleE2d,
    penetration: f32,
    normal: Vec2,
}
type Vec2 = na::Vector2<f32>;
// trait ObjectE2d
struct CircleE2d {
    radius: f32,
    position: Vec2,
    // speed: f32,
    velocity: Vec2,
    inv_mass: f32,
    restitution: f32,
}
impl CircleE2d {
    fn to_circle(&self) -> Vec<i32> {
        return vec![self.position.x as i32, self.position.y as i32, self.radius as i32];
    }
}
impl Default for CircleE2d {
    fn default() -> Self {
        CircleE2d {
            radius: 30.0,
            position: Vec2::new(20.0, 60.0),
            // speed: SPEED,
            velocity: Vec2::new(0.0, 1.0),
            inv_mass: 1.0,
            restitution: 0.55,
        }
    }
}
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
fn circle_vs_circle_opt(a: &CircleE2d, b: &CircleE2d) -> bool {
    let mut r = a.radius + b.radius;
    r *= r;
    // dbg!(r, a.position.x, b.position.x, a.position.y, b.position.y);
    // return r > (a - b).powf(2.0);
    return r > (a.position.x - b.position.x).powf(2.0) + (a.position.y - b.position.y).powf(2.0);
}
fn aabb_vs_aabb(a: AABB, b: AABB) -> bool {
    if a.max.x < b.min.x || a.min.x < b.max.x {
        return false;
    }
    if a.max.y < b.min.y || a.min.y < b.max.y {
        return false;
    }
    return true;
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
        let mut new_circle = CircleE2d::default();
        new_circle.position.x = pos.x() as f32;
        new_circle.position.y = pos.y() as f32;
        new_circle.inv_mass = 10.0;

        self.objects.push(new_circle);

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
        if s.mouse_pressed() {
            s.fill(color!(0));
        } else {
            s.fill(color!(255));
        }

        for i in self.objects.iter() {
            // s.circle([i.position.x as i32, i.position.y as i32, i.radius as i32])?
            s.circle([i.position.x as i32, i.position.y as i32, i.radius as i32])?;
        }
        let mut items_to_remove: Vec<usize> = Vec::new();
        for (idx, c) in self.objects.iter_mut().enumerate() {
            // grab to remove later
            // if c.position.x > max_width || c.position.x < 0.{
            //     items_to_remove.push(idx);
            // }
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
            if circle_vs_circle_opt(&self.objects[i], &self.objects[j]) {
                s.text("Collision")?;
                // self.resolve_collision();

                let (i, j) = if i < j { (i, j) } else { (j, i) };
                let (left, right) = self.objects.split_at_mut(j);
                rc(&mut left[i], &mut right[0]);
                // rc(&mut self.objects[0], &mut self.objects[1]);
            }
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
        objects: vec![
            CircleE2d::default()
            // CircleE2d::default()
        ],
    };

    //     app.items[1].position.x = 500.;
    //     app.items[1].position.y = 500.;
    // .
    app.objects[0].position.x = 480.0;
    app.objects[0].inv_mass = 1.0;
    app.objects[0].position.y = 500.0;
    app.objects[0].velocity.y = 0.0;

    // app.items[0].position.x -= 10.;
    // app.items[1].velocity.y  =0.;
    engine.run(&mut app)
}
