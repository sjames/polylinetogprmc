#[macro_use]
extern crate clap;
extern crate geo;
extern crate num_traits;
extern crate chrono;
extern crate time;

use clap::*;
use std::fs::File;
use std::io::prelude::*;

use geo::algorithm::bearing::Bearing;
use geo::algorithm::haversine_destination::HaversineDestination;
use geo::prelude::*;
use geo::Line;
use geo::LineString;
use geo::Point;
use polyline;
use std::vec;
use chrono::prelude::*;
use time::Duration;

#[derive(Debug, Clone)]
struct Gprmc {
    location: geo::Point<f64>,
    speed_kn: f64,
    track: f64,
}

#[derive(Clap)]
struct Opts {
    file: String,
    #[clap(short = "s", long = "speed_kmph", default_value = "30")]
    speed_kmph: f32,
    #[clap(short = "i", long = "interval", default_value = "1.0")]
    interval_s: f32,
    #[clap(short = "o", long = "output")]
    output : Option<String>, //name of output file
}

fn main() {
    //    App::new("polylinetogprmc")
    //    .version("1.0")
    //    .about("Convert from polyline string to simulated GPRMC")
    //    .author("Sojan James")
    //    .get_matches();

    let opts: Opts = Opts::parse();

    println!("file {}", opts.file);
    println!("speed : {}", opts.speed_kmph);
    println!("interval: {}", opts.interval_s);
    println!("output:{:?}", opts.output);

    let mut file = File::open(&opts.file).expect("Unable to open file");
    let mut contents = String::new();
    file.read_to_string(&mut contents)
        .expect("Unable to read file");
    let contents = contents.trim_end();

    println!("{:?}", contents);

    let de_polyline =
        polyline::decode_polyline(&contents, 5).expect("Unable to decode polyline string");

    //println!("{:?}", de_polyline);

    //for point in de_polyline.points_iter() {
    //    println!("Point {:?}", point);
    //}

    let mut collection = Vec::new();


    for line in de_polyline.lines() {
      //  println!("Line {:?}", line);
        collection.push(line_to_points(&line, (opts.speed_kmph / 3.6) as f64, opts.interval_s as f64).unwrap());
    }

    generate_log(collection,opts.interval_s as f64,&opts);
}

fn bearing_to_heading(bearing: f64) -> f64 {
    assert!(bearing.abs() <= 180.0);
    if bearing > 0.0 {
        bearing
    } else {
        180.0 - bearing.abs() + 180.0
    }
}


//print the LOG
fn generate_log(collection: std::vec::Vec<std::vec::Vec<Gprmc>>, interval_s : f64, opts:&Opts) {
    let mut now = Utc::now();
    let duration = Duration::milliseconds( (interval_s * 1000.0) as i64 );
    
    let mut file : Option<File> = 

    if let Some(output) = &opts.output {
        Some(File::create(&output).expect("Unable to open file for writing"))
    } else {
        None
    };


    for segment in collection.iter() {
        for pos in segment.iter() {
            let (lat, ns) = lat_to_ns(pos.location.lat());
            let (lng, ew) = lng_to_ew(pos.location.lng());

            let content = format!(
                "GPRMC,{:02}{:02}{:02},A,{},{},{},{},{},{},{:02}{:02}{:02},0,W,*", 
                now.hour(),now.minute(), now.second(),
                lat_to_dm(lat),
                ns,
                lng_to_dm(lng),
                ew, 
                format_speed(pos.speed_kn ),
                format_speed(pos.track ),
                now.day(),now.month(),now.year() - 2000
            );
            now = now + duration;

            let checksum = 
            content.chars().fold(0, |sum, x|{
                        sum ^ x as u8
            });

            let content = format!("${}{:02X}",content,checksum);
            match &mut file {
                Some(file) => write!(file,"{}\n",content).expect("Unable to write to file"),
                None => println!("{}",content), 
            };
            
         
        }
    }
}

fn line_to_points(line: &Line<f64>, speed_m_per_s: f64, interval_s: f64) -> Option<Vec<Gprmc>> {
    // find the distance between the points

    let distance = line
        .start_point()
        .vincenty_distance(&line.end_point())
        .unwrap();

    let num_segments = (distance / (speed_m_per_s * interval_s)).trunc() as i32;

    let heading = bearing_to_heading(line.start_point().bearing(line.end_point()));

    let mut points = Vec::new();

    let speed_in_knots = speed_m_per_s * 1.944;

    // println!("Heading : {:?}", heading);

    println!(
        "Distance between {:?} and {:?} -> {:?} metres  number of segments: {:?}",
        line.start_point(),
        line.end_point(),
        distance,
        num_segments
    );

    //the first point
    let starting_point = Gprmc {
        location: line.start_point(),
        speed_kn: speed_in_knots,
        track: heading,
    };

    points.push(starting_point.clone());

    (0..num_segments).fold(starting_point, |point, _x| {
        let next = Gprmc {
            location: point
                .location
                .haversine_destination(heading, distance / num_segments as f64),
            speed_kn: speed_in_knots,
            track: heading,
         };
        //println!("next : {:?}", next);
        points.push(next.clone());
        next
    });
    //remove the last element to avoid repeated points when we combine all lines
    if points.len() > 1 {
        points.pop().unwrap();
    }
    //println!(
    //    "Length of points {} Last point: {:?}",
    //    &points.len(),
    //    &points[points.len() - 1]
    //);
    Some(points)
}

fn lat_to_ns(lat: f64) -> (f64, char) {
    (lat.abs(), if lat < 0.0 { 'S' } else { 'N' })
}

fn lng_to_ew(lat: f64) -> (f64, char) {
    (lat.abs(), if lat < 0.0 { 'W' } else { 'E' })
}

fn lat_to_dm(dec: f64) -> String {
    let deg = dec.trunc() as u32;
    let min = dec.fract() * 60.0;
    format!("{:02}{:02}.{}", deg, min.trunc() as u32, format_fract(min))
}

fn lng_to_dm(dec: f64) -> String {
    let deg = dec.trunc() as u32;
    let min = dec.fract() * 60.0;
    format!("{:03}{:02}.{}", deg, min.trunc() as u32, format_fract(min))
}

fn format_speed(spd:f64) -> String 
{
    let trunc = spd.trunc() as u32;
    let fract = format!("{:.*}",1,spd.fract());
    format!("{:03}.{}",trunc,&fract[2..])
}

fn format_fract(num : f64) -> String
{
    let fract = format!("{:.*}",5,num.fract());
    format!("{}",&fract[2..])
    
}