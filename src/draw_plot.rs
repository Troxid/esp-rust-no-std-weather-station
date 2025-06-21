use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle, StyledDrawable, Triangle};
use embedded_graphics::Drawable;
use embedded_layout::prelude::*;
use time::Time;

pub struct TimePlot {
    pub bar_width: u32,
    pub bar_margin: u32,
    pub bar_height: u32,
    pub pointer: u8,
    pub values: [f32; 24],
    pub top_left: Point,
}

impl TimePlot {
    pub fn new() -> Self {
        Self::default()
    }
}

impl Drawable for TimePlot {
    type Color = BinaryColor;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        let bar_width = self.bar_width;
        let bar_margin = self.bar_margin;
        let bar_height = self.bar_height;
        let graph_area = self.bounding_box();
        let style = PrimitiveStyle::with_fill(BinaryColor::On);

        for (ind, p) in self.values.iter().enumerate() {
            let bar_x = (ind as u32) * (bar_width + bar_margin);
            let bar_len = (bar_height as f32 * (1.0 - (*p as f32 / 100f32))) as i32;
            let corner1 =
                graph_area.top_left + Point::new((bar_x + bar_margin) as i32, bar_height as i32);
            let corner2 = graph_area.top_left + Point::new((bar_x + bar_width) as i32, bar_len);
            let bar = Rectangle::with_corners(corner1, corner2);
            bar.draw_styled(&style, target)?;

            if self.pointer == ind as u8 {
                let _ = Triangle::new(Point::new(-1, -2), Point::new(1, -2), Point::new(0, 1))
                    .align_to(&bar, horizontal::Center, vertical::BottomToTop)
                    .translate(Point::new(0, -1))
                    .draw_styled(&style, target);
            }
        }
        Ok(())
    }
}

impl Dimensions for TimePlot {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(
            self.top_left,
            Size::new(
                (self.bar_width + self.bar_margin) * 24 + 1,
                self.bar_height + 1,
            ),
        )
    }
}

impl Transform for TimePlot {
    fn translate(&self, by: Point) -> Self {
        Self {
            values: self.values.clone(),
            top_left: self.top_left + by,
            ..*self
        }
    }

    fn translate_mut(&mut self, by: Point) -> &mut Self {
        self.top_left += by;
        self
    }
}

impl Default for TimePlot {
    fn default() -> Self {
        Self {
            bar_width: 4,
            bar_margin: 1,
            bar_height: 20,
            pointer: 0,
            values: Default::default(),
            top_left: Default::default(),
        }
    }
}
