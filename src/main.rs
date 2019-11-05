//! Utility to convert an interface for the Sensel, represented 
//! as a subset of SVG, to the Muses interface JSON format.
//! 
//! Pass it an interface.svg file (format described in docs/interface.md)
//! For a pretty printed version of the JSON pipe through something like jsonpp.
//! 
//! 
//! Copyright: Benedict R. Gaster (2019)

extern crate svg;
extern crate usvg;
extern crate serde_json;

extern crate getopts;
use getopts::Options;
use std::env;

#[macro_use]
extern crate log;
extern crate env_logger;

extern crate picto;
use picto::color::*;

use serde_json::json;

mod path_convert;
mod stroke_convert;
use path_convert::convert_path;
use stroke_convert::convert_stroke;
use usvg::Color;

use lyon::tessellation::{
    VertexBuffers, FillOptions, StrokeOptions, StrokeVertex, TessellationResult, LineJoin, LineCap};
use lyon::tessellation::basic_shapes::{fill_circle, fill_rectangle, stroke_polyline};
use lyon::math::{vector, point, Point};
use lyon::tessellation::geometry_builder::{BuffersBuilder};
use lyon::path::{Path};
use lyon::tessellation::{FillTessellator, StrokeTessellator};

use lyon::tessellation::geometry_builder::{GeometryBuilder};

use lyon::tessellation::geometry_builder::{VertexConstructor};
use lyon::tessellation;

use euclid::{Transform2D, TypedPoint2D, TypedPoint3D, TypedVector3D};

use svg::node::element::path::{Command, Data};
use svg::node::element::tag::{Path,Rectangle,Polygon,Circle};
use svg::node::{Value};
use svg::parser::Event;

use std::cmp::{max, min};


//-----------------------------------------------------------------------------
// constants

const SENSEL_WIDTH: u32    = 230;
const SENSEL_HEIGHT: u32   = 130;
const SENSEL_DEVICE: &'static str = "sensel"; 

const LIGHTPAD_WIDTH: u32  = 15;
const LIGHTPAD_HEIGHT: u32 = 15;
const LIGHTPAD_DEVICE: &'static str = "lightpad";

const UNSUPPORTED_DEVICE: &'static str = "unknown";

// SVG controller attributes 
const INTERFACE_OSC_ADDRESS_ATTR: &'static str ="interface_osc_address";
const INTERFACE_TYPE_ATTR: &'static str        = "interface_type";
const INTERFACE_OSC_ARGS_ATTR: &'static str    = "interface_osc_args";
const INTERFACE_MIN_ATTR: &'static str         = "min";
const INTERFACE_MAX_ATTR: &'static str         = "max";

//-----------------------------------------------------------------------------
// tessellation and rasterization

trait Buffer<T> {
    fn set(&mut self, x: i32, y: i32, v: &T);
}

// Rasterize to an image, used for creating visuals of the interface
// as we are working with SVGs this is not likely to be needed very often!
impl Buffer<picto::color::Rgb> for picto::buffer::Rgb {
    fn set(&mut self, x: i32, y: i32, v: &picto::color::Rgb) {
        self.set(x as u32, y as u32, v);
    }
}

type ID = u32;

struct Interface {
    /// buffer respresenting the sensel
    sensel: Vec<Vec<ID>>,
    
    /// unique id, 0 is reserved for non id
    next_id: ID,
}

impl Interface {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            sensel: vec![vec![0u32; height as usize]; width as usize],
            next_id: 1,
        }
    }

    pub fn id(&mut self) -> ID {
        let r = self.next_id;
        self.next_id = self.next_id + 1;
        r
    }

    pub fn number_ids(&self) -> u32 {
        self.next_id
    }

    pub fn to_json(&self) -> serde_json::Value {
        json!(self.sensel)
    }
}

impl Buffer<u32> for Interface {
    fn set(&mut self, x: i32, y: i32, v: &u32) {
        self.sensel[x as usize][y as usize] = *v;
    }
}

// first we implement a simple Bary centric rasterizer (if you want to know more about this
// stuff you could check out the excellent introduction here: 
// https://github.com/ssloy/tinyrenderer/wiki/Lesson-0:-getting-started).
// although it is important to note that we are not trying to implement an OpenGL rendering pipeline,
// we simply want to rasterise triangles.

type Point2i = TypedPoint2D<i32, i32>;
type Vec3f = TypedVector3D<f32, f32>;
type Triangle = [Point2i;3];

fn point2i(x: i32, y: i32) -> Point2i {
    Point2i::new(x,y)
}

fn point2i_to_point(p: Point2i) -> TypedPoint2D<f32, euclid::UnknownUnit> {
    point(p.x as f32, p.y as f32)
}

fn barycentric(points: Triangle, p: Point2i) -> Vec3f {
    let u: Vec3f = Vec3f::new((points[2].x - points[0].x) as f32, (points[1].x-points[0].x) as f32, (points[0].x - p.x) as f32).cross(
        Vec3f::new((points[2].y - points[0].y) as f32, (points[1].y-points[0].y) as f32, (points[0].y - p.y) as f32));

    // check to see if degenerate, if so return something with negative coordinates
    if u.z.abs() < 1.0 {
        Vec3f::new(-1.0, 1.0, 1.0)
    }
    else {
        Vec3f::new(1.0 - (u.x+u.y) / u.z, u.y/u.z, u.x/u.z)
    }
}

fn rasterize_triangle_old(
    width: u32, height: u32,
    points: Triangle, 
    buffer: &mut picto::buffer::Rgb) {
    // calulate min/max bounding box, cliping to inside the sensel viewport
    let bound_box_max = Point2i::new( 
        min(width as i32, max(points[0].x, max(points[1].x, points[2].x))),
        min(height as i32, max(points[0].y, max(points[1].y, points[2].y))));
    
    let bound_box_min = Point2i::new( 
        max(0, min(points[0].x, min(points[1].x, points[2].x))),
        max(0, min(points[0].y, min(points[1].y, points[2].y))));

    // now iterate over the bounding box
    for x in bound_box_min.x..bound_box_max.x {
        for y in bound_box_min.y..bound_box_max.y {
            let bc_screen = barycentric(points, Point2i::new(x,y));
            if bc_screen.x >= 0.0 && bc_screen.y >= 0.0 && bc_screen.z >= 0.0 {
                buffer.set(x as u32, y as u32, &Rgb::new(0.0, 0.0, 0.0));
            }
        }
    }
}

fn rasterize_triangle<T, B> (
    width: u32, height: u32,
    points: Triangle, buffer: &mut B, v: &T) 
    where B : Buffer<T> {
    // calulate min/max bounding box, cliping to inside the sensel viewport
    let bound_box_max = Point2i::new( 
        min(width as i32, max(points[0].x, max(points[1].x, points[2].x))),
        min(height as i32, max(points[0].y, max(points[1].y, points[2].y))));
    
    let bound_box_min = Point2i::new( 
        max(0, min(points[0].x, min(points[1].x, points[2].x))),
        max(0, min(points[0].y, min(points[1].y, points[2].y))));

    // now iterate over the bounding box
    for x in bound_box_min.x..bound_box_max.x {
        for y in bound_box_min.y..bound_box_max.y {
            let bc_screen = barycentric(points, Point2i::new(x,y));
            if bc_screen.x >= 0.0 && bc_screen.y >= 0.0 && bc_screen.z >= 0.0 {
                buffer.set(x, y, v);
            }
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Vertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

impl Vertex {
    pub fn new(position: [f32;3], colour: [f32;4]) -> Vertex {
        Vertex {
            position: position,
            color: colour,
        }
    }

    pub fn to_point(&self) -> Point2i {
        point2i(self.position[0] as i32, self.position[1] as i32)
    }
}

// A  simple vertex constructor
pub struct VertexCtor {
   pub color: [f32; 4],
   pub transform: Transform2D<f32>,
}

impl Default for VertexCtor {
    fn default() -> VertexCtor {
        VertexCtor {
            color : [0.0,0.0,0.0,0.0],
            transform: Transform2D::identity(),
        }
    }
}

impl VertexConstructor<tessellation::FillVertex, Vertex> for VertexCtor {
    fn new_vertex(&mut self, vertex: tessellation::FillVertex) -> Vertex {
        assert!(!vertex.position.x.is_nan());
        assert!(!vertex.position.y.is_nan());
        let tp = self.transform.transform_point(&vertex.position);
        let v = tp.to_array();
        Vertex {
            position: [v[0], v[1], 0.0 as f32],
            color: self.color,
        }
    }
}
impl VertexConstructor<tessellation::StrokeVertex, Vertex> for VertexCtor {
    fn new_vertex(&mut self, vertex: tessellation::StrokeVertex) -> Vertex {
        let tp = self.transform.transform_point(&vertex.position);
        let v = tp.to_array();
        let multi = 1.0;
        Vertex { 
            position: [v[0], v[1], 0.0 as f32], 
            color: self.color,
        }
    }
}

pub type Mesh = VertexBuffers<Vertex, u16>;

pub const FALLBACK_COLOR: Color = Color {
    red: 0,
    green: 0,
    blue: 0,
};

#[derive(Debug)]
pub enum Error {
    /// Only `svg` and `svgz` suffixes are supported.
    InvalidFileSuffix,

    /// Failed to open the provided file.
    FileOpenFailed,

    /// Only UTF-8 content are supported.
    NotAnUtf8Str,

    /// Compressed SVG must use the GZip algorithm.
    MalformedGZip,

    InvalidSize,
}

/// generates a set of verts and vert indices for a SVG path
// the implementation for this one is more compilated that the other shapes below, simply because it is a path
// and thus can contain arc, bezier curves, and so on. to avoid having to handle everything we use usvg to simpify
// first and then tesserlate the result.
fn path_tessellate(path: &str) -> Mesh {
    let mut mesh  = Mesh::new();

    // create a trivial SVG with our path
    // TODO: probably a less overkill way to do this, i.e. without having to create a complete SVG, but in
    // truth this seems like a nice easy and functional path and so not sure it's worth it :-)
    let svg = format!(r#"<?xml version="1.0" encoding="UTF-8"?>
      <!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
      <svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" height="140mm" width="240mm" 
           viewBox="0 0 240 140" version="1.11.1">
        <path d="{}" />
      </svg>"#, path);

    // use usvg to simplify our svg and Lyon to tesselate
    let mut fill_tess = FillTessellator::new();
    let mut stroke_tess = StrokeTessellator::new();
    let transform = Transform2D::identity();

    let rtree = usvg::Tree::from_str(&svg, &usvg::Options::default()).unwrap();
    for node in rtree.root().descendants() {
        if let usvg::NodeKind::Path(ref p) = *node.borrow() {
            if let Some(ref fill) = p.fill {
                fill_tess.tessellate_path(
                        convert_path(p),
                        &FillOptions::tolerance(0.01),
                        &mut BuffersBuilder::new(
                            &mut mesh,
                            VertexCtor { 
                                color : [0.0,0.0,0.0,0.0], 
                                transform  
                            }
                        ),
                ).expect("Error during tesselation!");
            }

            if let Some(ref stroke) = p.stroke {
                let (stroke_color, stroke_opts) = convert_stroke(stroke);

                let _ = stroke_tess.tessellate_path(
                    convert_path(p),
                    &stroke_opts.with_tolerance(0.01).with_line_width(1.0),
                    &mut BuffersBuilder::new(
                        &mut mesh,
                        VertexCtor { 
                                color : [0.0,0.0,0.0,0.0], 
                                transform  
                        },
                    ),
                );
            }
        }
    }

    mesh
}

/// generate a set of verts and vert indices for a SVG rect
/// (this will, of course, generate, just two triangles)
fn rect_tessellate(x: i32, y: i32, width: i32, height: i32) -> Mesh {

    let mut mesh  = Mesh::new();

    let fill_options = FillOptions::tolerance(0.01);
    
    let transform = Transform2D::identity();

    // tessellate onto the mesh
    fill_rectangle(
        &lyon::math::rect(x as f32, y as f32, width as f32, height as f32),
        &fill_options,
        &mut BuffersBuilder::new(
            &mut mesh, 
            VertexCtor { 
                color : [0.0,0.0,0.0,0.0], 
                transform 
            })
    ).unwrap();

    mesh
}

fn circle_tessellate(cpoint: Point2i, r: i32) -> Mesh {

    let mut mesh  = Mesh::new();

    let fill_options = FillOptions::tolerance(0.01);
    
    let transform = Transform2D::identity();

    // tessellate onto the mesh
    fill_circle(
        point2i_to_point(cpoint),
        r as f32,
        &fill_options,
        &mut BuffersBuilder::new(
            &mut mesh, 
            VertexCtor{ 
                color : [0.0,0.0,0.0,0.0], 
                transform 
            } ),
    ).unwrap();

    mesh
}

/// generate a set of verts and vert indices for a SVG polygon
/// (this will, of course, generate, just two triangles)
fn polygon_tessellate(points: Vec<Point2i>) -> Mesh {

    let mut mesh  = Mesh::new();

    let fill_options = FillOptions::tolerance(0.01);
    let mut tessellator_fill = FillTessellator::new(); 
    
    let transform = Transform2D::identity();
    let mut builder = Path::builder();
    builder.move_to(point2i_to_point(points[0]));

    // TODO might what add check that it is actually closed, it should be it is a SVG polygon

    for i in 1..points.len() {
        builder.line_to(point2i_to_point(points[i]));
    }

    let path = builder.build();
    tessellator_fill.tessellate_path(
        &path,
        &fill_options,
        &mut BuffersBuilder::new(
            &mut mesh, 
            VertexCtor{ 
                color : [0.0,0.0,0.0,0.0],
                transform: transform, 
            }),
    ).unwrap();

    mesh
}


/// rasterize a mesh to a given buffer
fn rasterize<T,B>(
     width: u32, height: u32,
     mesh: &Mesh, buffer: &mut B, v: &T)
    where B : Buffer<T> {
    
    for indices in mesh.indices.chunks(3) {
        
        let points = [
            mesh.vertices[indices[0] as usize].to_point(),
            mesh.vertices[indices[1] as usize].to_point(),
            mesh.vertices[indices[2] as usize].to_point()];
        rasterize_triangle(width, height, points, buffer, v);
    }
}

//-----------------------------------------------------------------------------

/// convert pixels to mm
/// assumption is that DPI = 72
fn px_to_mm(px: f64) -> f64 {
    px * 0.352777778
}

/// convert types to JSON names
fn handle_type(tag: &str) -> &str {
    match tag {
        "noteon" => {
            "noteon"
        },
        "noteoff" => {
            "noteoff"
        },
        "pad" => {
            "pad"
        },
        "endless" => {
            "endless"
        },
        "none" => {
            "none"
        },
        "border" => {
            "none"
        },
        "vert_slider" => {
            "vert_slider"
        },
        "horz_slider" => {
            "horz_slider"
        },
        _ => {
            warn!("unreconized type {}", tag);
            ""
        }
    }
}

/// convert an SVG int or float to i32
/// assume that these attributes are in px and convert to mm
/// the maths is all a little rough here and would need to be fixed for a more robust 
/// implementation
fn handle_int_attribute(v: &str, px: bool) -> i32 {
    if px {
        px_to_mm(v.parse::<f64>().unwrap()).round() as i32
    } 
    else {
        (v.parse::<f64>().unwrap()).round() as i32
    }
}

fn handle_argument(v: &str) -> serde_json::Value {
    match v.parse::<i32>() {
        Ok(i) => {
            json!(i)
        },
        Err(_) => {
            warn!("Unsupported argument type");
            serde_json::Value::Null
        }
    }
}

fn interface_device(d: &str) -> (u32, u32, &str) {
    match d {
        SENSEL_DEVICE   => (SENSEL_WIDTH, SENSEL_HEIGHT, SENSEL_DEVICE),
        LIGHTPAD_DEVICE => (LIGHTPAD_WIDTH, LIGHTPAD_HEIGHT, LIGHTPAD_DEVICE),
        _ => {
            error!("UNKNOWN interface type");
            (0,0, "unknown")
        }
    }
}

//-----------------------------------------------------------------------------
// command line utilities 

fn print_usage(program: &str, opts: Options) {
    let brief = format!("Usage: {} FILE [options]", program);
    print!("{}", opts.usage(&brief));
}

//-----------------------------------------------------------------------------
// entry point

fn main() {
    // handle command line arguments
    let args: Vec<String> = env::args().collect();
    let program = args[0].clone();

    let mut opts = Options::new();
    opts.optopt("", "png", "generate PNG output", "NAME");
    opts.optopt("", "json", "JSON file output", "NAME");
    opts.optflag("", "illustrator", "SVG used Adobe Illustrator attribute encoding");
    opts.optflag("h", "help", "print this help menu");

    let matches = match opts.parse(&args[1..]) {
        Ok(m) => { m }
        Err(f) => { panic!(f.to_string()) }
    };

    // display help and exit if necessary
    if matches.opt_present("h") {
        print_usage(&program, opts);
        return;
    }

    // input was exported from Adobe Illistrator, so utilises a different attribute encoding
    let illustrator = matches.opt_present("illustrator");

    // PNG path or ""
    let png_path = match matches.opt_str("png") {
        Some(x) => x,
        None => "".to_string(),
    };
    
    // must be an interface.svg
    let svg_path = if !matches.free.is_empty() {
        matches.free[0].clone()
    } else {
        print_usage(&program, opts);
        return;
    };

    let mut controllers = vec![];

    let mut device_width  = 0;
    let mut device_height = 0;
    let mut device_name: String = UNSUPPORTED_DEVICE.to_string();

    let doc = svg::open(svg_path.clone()).unwrap();
    for event in doc {
        match event {
            Event::Tag(SVG, _, attributes) => {
                
                match attributes.get("interface_device") {
                    Some(attr) => {
                        let (w,h,n)   = interface_device(attr);
                        device_width  = w;
                        device_height = h;
                        device_name   = n.to_string();
                    },
                    _ => {

                    }
                }
            },
             _ => {}
        };
    }

    // check if successfully found a valid interface device
    if device_name == UNSUPPORTED_DEVICE.to_string() {
        eprintln!("Unsupported Interface type");
        return;
    }

    // create an image mapping to the sensel, with white background 
    let mut image = picto::Buffer::<Rgb, u8, _>::from_pixel(
        device_width, 
        device_height, 
        &Rgb::new(1.0, 1.0, 1.0));

    let mut interface = Interface::new(device_width, device_height);

    let doc = svg::open(svg_path).unwrap();
    for event in doc {
        match event {
            Event::Tag(Path, _, attributes) => {
                let data = attributes.get("d").unwrap();
                let mesh = path_tessellate(data);
                
                // rasterize to PNG, if requested
                if png_path.len() > 0 {
                    rasterize(device_width, device_height, &mesh, &mut image, &Rgb::new(0.0, 0.0, 0.0));
                }

                // add to interface buffer
                let id = interface.id();
                rasterize(device_width, device_height, &mesh, &mut interface, &id);

                if illustrator {
                    // now handle the data-name attribute, which contains info about the type 
                    // of controller, OSC message, and so on.
                    // TODO: add some error checking, to avoid "seg-faulting" :-)
                    let data_name = attributes.get("data-name").unwrap();        
                    let need_to_live = (&*data_name).to_string();
                    let mut args : Vec<&str> = need_to_live.split(' ').collect();

                    let typ = handle_type(args[1]);

                    if typ != "none" { 
                        // osc address
                        let msg = args[2];

                        let mut args_json = vec![];
                        if args.len() > 3 {
                            args.drain(0..3);
                            // only osc arguments left
                            for a in args {
                                let arg = handle_argument(a);
                                if arg != serde_json::Value::Null {
                                    args_json.push(arg);
                                }
                            }
                        }

                        let path = json!({
                            "id": id,
                            "type_id": typ,
                            "args" : args_json,
                            "address" : msg
                        });

                        controllers.push(path);
                    }
                }
                else {  
                    let typ = handle_type(attributes.get(INTERFACE_TYPE_ATTR).unwrap());

                    if typ != "none" { 
                        let msg = attributes.get(INTERFACE_OSC_ADDRESS_ATTR).unwrap() as &str;

                        let mut args_json = vec![];
                        match attributes.get(INTERFACE_OSC_ARGS_ATTR) {
                            Some(osc_args) => {
                                let need_to_live = (&*osc_args).to_string();
                                let mut args : Vec<&str> = need_to_live.split(' ').collect();
                                
                                for a in args {
                                    let arg = handle_argument(a);
                                    if arg != serde_json::Value::Null {
                                        args_json.push(arg);
                                    }
                                }
                            },
                            _ => {

                            }
                        }

                        let path = json!({
                            "id": id,
                            "type_id": typ,
                            "args" : args_json,
                            "address" : msg
                        });

                        controllers.push(path);
                    }
                }  
            },
            Event::Tag(Rectangle, _, attributes) => {
                
                let x = handle_int_attribute(attributes.get("x").unwrap(), illustrator);
                let y = handle_int_attribute(attributes.get("y").unwrap(), illustrator);
                let width = handle_int_attribute(attributes.get("width").unwrap(), illustrator);
                let height = handle_int_attribute(attributes.get("height").unwrap(), illustrator);
                
                if illustrator {
                    // now handle the data-name attribute, which contains info about the type 
                    // of controller, OSC message, and so on.
                    // TODO: add some error checking, to avoid "seg-faulting" :-)
                    let data_name = attributes.get("data-name").unwrap();        
                    let need_to_live = (&*data_name).to_string();
                    let mut args : Vec<&str> = need_to_live.split(' ').collect();
                    
                    // controller type
                    let typ = handle_type(args[1]);

                    // none is a special case used for printing border for cutout, not included in interface
                    if typ != "none" { 
                        let mesh = rect_tessellate(x,y, width, height);
                        // rasterize to PNG, if requested
                        if png_path.len() > 0 {
                            rasterize(device_width, device_height, &mesh, &mut image, &Rgb::new(0.0, 0.0, 0.0));
                        }

                        // add to interface buffer
                        let id = interface.id();
                        rasterize(device_width, device_height, &mesh, &mut interface, &id);

                        // osc address
                        let msg = args[2];

                        let mut args_json = vec![];
                        if args.len() > 3 {
                            args.drain(0..3);
                            // only osc arguments left
                            for a in args {
                                let arg = handle_argument(a);
                                if arg != serde_json::Value::Null {
                                    args_json.push(arg);
                                }
                            }
                        }

                        let rect = json!({
                            "id": id,
                            "type_id": typ,
                            "args" : args_json,
                            "address" : msg
                        });

                        controllers.push(rect);
                    }
                }
                else {
                    let typ = handle_type(attributes.get(INTERFACE_TYPE_ATTR).unwrap());
                    // none is a special case used for printing border for cutout, not included in interface

                    if typ != "none" { 
                        let mesh = rect_tessellate(x,y, width, height);
                        // rasterize to PNG, if requested
                        if png_path.len() > 0 {
                            rasterize(device_width, device_height, &mesh, &mut image, &Rgb::new(0.0, 0.0, 0.0));
                        }

                        // add to interface buffer
                        let id = interface.id();
                        rasterize(device_width, device_height, &mesh, &mut interface, &id);

                        let msg = attributes.get(INTERFACE_OSC_ADDRESS_ATTR).unwrap() as &str;

                        let mut args_json = vec![];
                        match attributes.get(INTERFACE_OSC_ARGS_ATTR) {
                            Some(osc_args) => {
                                let need_to_live = (&*osc_args).to_string();
                                let mut args : Vec<&str> = need_to_live.split(' ').collect();
                                
                                for a in args {
                                    let arg = handle_argument(a);
                                    if arg != serde_json::Value::Null {
                                        args_json.push(arg);
                                    }
                                }
                            },
                            _ => {

                            }
                        }

                        let mut args_onoff = vec![];
                        match attributes.get("on") {
                            Some(value) => {
                                let arg = handle_argument(value);
                                if arg != serde_json::Value::Null {
                                    args_onoff.push(arg);
                                }
                                match attributes.get("off") {
                                    Some(value) => {
                                        let arg = handle_argument(value);
                                        if arg != serde_json::Value::Null {
                                            args_onoff.push(arg);
                                        }
                                    },
                                    _ => {
                                    }
                                };
                            },
                            _ => {
                            }
                        };


                        let rgb = match attributes.get("fill") {
                            Some(fill) => {
                                fill
                            },
                            _ => {
                                "rgb(0,0,0)"
                            }
                        };

                        let pressure = match attributes.get("pressure") {
                            Some(pressure) => {
                                pressure.to_string() == "True"
                            },
                            _ => {
                                false
                            }
                        };

                        let with_coords = match attributes.get("with_coords") {
                            Some(with_coords) => {
                                with_coords.to_string() == "True"
                            },
                            _ => {
                                false
                            }
                        };

                        if typ == "vert_slider" || typ == "horz_slider" {
                            let min = (attributes.get(INTERFACE_MIN_ATTR).unwrap() as &str).parse::<u32>().unwrap();
                            let max = (attributes.get(INTERFACE_MAX_ATTR).unwrap() as &str).parse::<u32>().unwrap();
                            let rect = json!({
                                "id": id,
                                "type_id": typ,
                                "args" : args_json,
                                "address" : msg,
                                "min" : min,
                                "max" : max,
                                "rgb" : rgb,
                                "pressure": pressure,
                                "generate_coords": with_coords,
                            });

                            controllers.push(rect);
                        } 
                        else { 
                            let generate_move = match attributes.get("with_move") {
                                Some(with_move) => {
                                    with_move.to_string() == "True"
                                },
                                _ => {
                                    false
                                }
                            };

                            let generate_end = match attributes.get("with_end") {
                                Some(with_end) => {
                                    with_end.to_string() == "True"
                                },
                                _ => {
                                    false
                                }
                            };

                            if args_onoff.len() == 2 && typ == "pad" {
                                let rect = json!({
                                    "id": id,
                                    "type_id": "dpad",
                                    "args" : args_json,
                                    "address" : msg,
                                    "rgb": rgb,
                                    "pressure": pressure,
                                    "generate_move": generate_move,
                                    "generate_end": generate_end,
                                    "generate_coords": with_coords,
                                    "on": args_onoff[0],
                                    "off": args_onoff[1],
                                });

                                controllers.push(rect);
                            }
                            else {
                                let rect = json!({
                                    "id": id,
                                    "type_id": typ,
                                    "args" : args_json,
                                    "address" : msg,
                                    "rgb": rgb,
                                    "pressure": pressure,
                                    "generate_move": generate_move,
                                    "generate_end": generate_end,
                                    "generate_coords": with_coords,
                                });

                                controllers.push(rect);
                            }

                            
                        }

                        
                    }
                }
            },
            Event::Tag(Circle, _, attributes) => {
                let cpoint = point2i(
                    handle_int_attribute(attributes.get("cx").unwrap(), illustrator),
                    handle_int_attribute(attributes.get("cy").unwrap(), illustrator));
                let r = handle_int_attribute(attributes.get("r").unwrap(), illustrator);

                let mesh = circle_tessellate(cpoint,r);
                // rasterize to PNG, if requested
                if png_path.len() > 0 {
                    rasterize(device_width, device_height, &mesh, &mut image, &Rgb::new(0.0, 0.0, 0.0));
                }

                // add to interface buffer
                let id = interface.id();
                rasterize(device_width, device_height, &mesh, &mut interface, &id); 

                if illustrator {

                    // now handle the data-name attribute, which contains info about the type 
                    // of controller, OSC message, and so on.
                    // TODO: add some error checking, to avoid "seg-faulting" :-)
                    let data_name = attributes.get("data-name").unwrap();        
                    let need_to_live = (&*data_name).to_string();
                    let mut args : Vec<&str> = need_to_live.split(' ').collect();

                    let typ = handle_type(args[1]);

                    if typ != "none" { 
                        // osc address
                        let msg = args[2];

                        let mut args_json = vec![];
                        if args.len() > 3 {
                            args.drain(0..3);
                            // only osc arguments left
                            for a in args {
                                let arg = handle_argument(a);
                                if arg != serde_json::Value::Null {
                                    args_json.push(arg);
                                }
                            }
                        }

                        let circle = json!({
                            "id": id,
                            "type_id": typ,
                            "args" : args_json,
                            "address" : msg
                        });

                        controllers.push(circle);
                    }
                }
                else {
                        let typ = handle_type(attributes.get(INTERFACE_TYPE_ATTR).unwrap());
                        // none is a special case used for printing border for cutout, not included in interface
                        
                        if typ != "none" { 
                            let msg = attributes.get(INTERFACE_OSC_ADDRESS_ATTR).unwrap() as &str;

                            let mut args_json = vec![];
                            match attributes.get(INTERFACE_OSC_ARGS_ATTR) {
                                Some(osc_args) => {
                                    let need_to_live = (&*osc_args).to_string();
                                    let mut args : Vec<&str> = need_to_live.split(' ').collect();
                                    
                                    for a in args {
                                        let arg = handle_argument(a);
                                        if arg != serde_json::Value::Null {
                                            args_json.push(arg);
                                        }
                                    }
                                },
                                _ => {

                                }
                            }

                            let mut args_onoff = vec![];
                            match attributes.get("on") {
                                Some(value) => {
                                    let arg = handle_argument(value);
                                    if arg != serde_json::Value::Null {
                                        args_onoff.push(arg);
                                    }
                                    match attributes.get("off") {
                                        Some(value) => {
                                            let arg = handle_argument(value);
                                            if arg != serde_json::Value::Null {
                                                args_onoff.push(arg);
                                            }
                                        },
                                        _ => {
                                        }
                                    };
                                },
                                _ => {
                                }
                            };

                            // TODO: all this code refactoring...
                            if args_onoff.len() == 2 && typ == "pad" {
                                let circle = json!({
                                    "id": id,
                                    "type_id": "dpad",
                                    "args" : args_json,
                                    "address" : msg,
                                    "on": args_onoff[0],
                                    "off": args_onoff[1],
                                });

                                controllers.push(circle);
                            }
                            else {

                                let circle = json!({
                                    "id": id,
                                    "type_id": typ,
                                    "args" : args_json,
                                    "address" : msg
                                });

                                controllers.push(circle);
                            }
                        }
                }
            },
            Event::Tag(Polygon, _, attributes) => { 
                // convert each elem of points to i32
                let values: Vec<i32> = attributes
                    .get("points")
                    .unwrap()
                    .split(' ')
                    .map(|s| handle_int_attribute(s, illustrator))
                    .collect();

                // pair them up into points
                let points: Vec<Point2i> = values
                    .chunks(2)
                    .map(|p| point2i(p[0], p[1]))
                    .collect();

                let mesh = polygon_tessellate(points);
                // rasterize to PNG, if requested
                if png_path.len() > 0 {
                    rasterize(device_width, device_height, &mesh, &mut image, &Rgb::new(0.0, 0.0, 0.0));
                }

                // add to interface buffer
                let id = interface.id();
                rasterize(device_width, device_height, &mesh, &mut interface, &id);

                if illustrator {
                    // now handle the data-name attribute, which contains info about the type 
                    // of controller, OSC message, and so on.
                    // TODO: add some error checking, to avoid "seg-faulting" :-)
                    let data_name = attributes.get("data-name").unwrap();        
                    let need_to_live = (&*data_name).to_string();
                    let mut args : Vec<&str> = need_to_live.split(' ').collect();

                    let typ = handle_type(args[1]);

                    // osc address
                    let msg = args[2];

                    let mut args_json = vec![];
                    if args.len() > 3 {
                        args.drain(0..3);
                        // only osc arguments left
                        for a in args {
                            let arg = handle_argument(a);
                            if arg != serde_json::Value::Null {
                                args_json.push(arg);
                            }
                        }
                    }

                    let poly = json!({
                        "id": id,
                        "type_id": typ,
                        "args" : args_json,
                        "address" : msg
                    });

                    controllers.push(poly);
                }
                else {
                    let typ = handle_type(attributes.get(INTERFACE_TYPE_ATTR).unwrap());
                    // none is a special case used for printing border for cutout, not included in interface
                    
                    let msg = attributes.get(INTERFACE_OSC_ADDRESS_ATTR).unwrap() as &str;

                    let mut args_json = vec![];
                    match attributes.get(INTERFACE_OSC_ARGS_ATTR) {
                        Some(osc_args) => {
                            let need_to_live = (&*osc_args).to_string();
                            let mut args : Vec<&str> = need_to_live.split(' ').collect();
                            
                            for a in args {
                                let arg = handle_argument(a);
                                if arg != serde_json::Value::Null {
                                    args_json.push(arg);
                                }
                            }
                        },
                        _ => {

                        }
                    }

                    let poly = json!({
                        "id": id,
                        "type_id": typ,
                        "args" : args_json,
                        "address" : msg
                    });

                    controllers.push(poly);
                }
            },
            _ => {}
        }
    }

    let interface_json = json!({
        "controllers": controllers,
        "buffer": interface.to_json(),
        "interface": device_name,
    });

    println!("{}", interface_json.to_string());

    // write PNG image of sensel interface
    if png_path.len() > 0 {
        picto::write::to_path(png_path, &image).unwrap();
    }
}
