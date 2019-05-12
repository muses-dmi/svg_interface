use std::slice;

use lyon::math::Point;
use lyon::path::PathEvent;
use lyon::geom::{LineSegment, CubicBezierSegment};

use usvg;

// This module implements some glue between usvg and lyon.
// PathConvIter translate usvg's path data structure into an
// iterator of lyon's PathEvent.

fn point(x: &f64, y: &f64) -> Point {
    Point::new((*x) as f32, (*y) as f32)
}

pub struct PathConvIter<'a> {
    iter: slice::Iter<'a, usvg::PathSegment>,
    prev: Point,
    first: Point,
}

impl<'l> Iterator for PathConvIter<'l> {
    type Item = PathEvent;
    fn next(&mut self) -> Option<PathEvent> {
        match self.iter.next() {
            Some(usvg::PathSegment::MoveTo { x, y }) => {
                //println!("        builder.move_to(point({:.4},{:.4}));", x, y);
                self.prev = point(x, y);
                self.first = self.prev;
                Some(PathEvent::MoveTo(self.prev))
            }
            Some(usvg::PathSegment::LineTo { x, y }) => {
                //println!("        builder.line_to(point({:.4},{:.4}));", x, y);
                let from = self.prev;
                self.prev = point(x, y);
                Some(PathEvent::Line(LineSegment { from, to: self.prev }))
            }
            Some(usvg::PathSegment::CurveTo { x1, y1, x2, y2, x, y, }) => {
                let from = self.prev;
                self.prev = point(x, y);
                //println!("        builder.cubic_bezier_to(point({:.4}, {:.4}), point({:.4}, {:.4}), point({:.4}, {:.4}));", x1, y1, x2, y2, x, y);
                Some(PathEvent::Cubic(CubicBezierSegment {
                    from,
                    ctrl1: point(x1, y1),
                    ctrl2: point(x2, y2),
                    to: self.prev,
                }))
            }
            Some(usvg::PathSegment::ClosePath) => {
                self.prev = self.first;
                //println!("        builder.close();");
                Some(PathEvent::Close(LineSegment {
                    from: self.prev,
                    to: self.first,
                }))
            }
            None => None,
        }
    }
}

pub fn convert_path<'a>(p: &'a usvg::Path) -> PathConvIter<'a> {
    PathConvIter {
        iter: p.segments.iter(),
        first: Point::new(0.0, 0.0),
        prev: Point::new(0.0, 0.0),
    }
}
