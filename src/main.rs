#[macro_use]
extern crate clap;
extern crate chrono;
extern crate geo;
extern crate num_traits;
extern crate time;

use clap::*;
use std::fs::File;
use std::io::prelude::*;

use chrono::prelude::*;
use geo::algorithm::bearing::Bearing;
use geo::algorithm::haversine_destination::HaversineDestination;
use geo::prelude::*;
use geo::Line;
//use geo::LineString;
//use geo::Point;
use polyline;
//use std::vec;
use geo::algorithm::rotate::RotatePoint;
use time::Duration;

#[derive(Debug, Clone)]
struct Gprmc {
    location: geo::Point<f64>,
    speed_kn: f64,
    track: f64,
    //pedestrian_pos: Option<geo::Point<f64>>,
    pedestrian : Option<Pedestrian>,
}

#[derive(Debug, Clone)]
struct Pedestrian {
    pos : geo::Point<f64>,
    track : f64,
    speed_kn : f64,
}

#[derive(Clap)]
struct Opts {
    file: String,
    #[clap(short = "s", long = "speed_kmph", default_value = "30")]
    speed_kmph: f32,
    #[clap(short = "i", long = "interval", default_value = "1.0")]
    interval_s: f32,
    #[clap(short = "o", long = "output")]
    output: Option<String>, //name of output file
    #[clap(short = "c", long = "collision_after_s")]
    collision_after_s: Option<u32>,
    #[clap(short = "a", long = "angle_of_collision_d", default_value = "90")]
    collision_angle: u32,
    #[clap(short = "p", long = "pedestrian_speed_kmph", default_value = "3")]
    pedestrian_speed: f64,
    #[clap(short = "l", long = "pedestrian_track_length_s", default_value = "10")]
    pedestrian_track_length: u32,
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
    println!("collision_after:{:?}", opts.collision_after_s);

    let mut file = File::open(&opts.file).expect("Unable to open file");
    let mut contents = String::new();
    file.read_to_string(&mut contents)
        .expect("Unable to read file");
    let contents = contents.trim_end();

    println!("{:?}", contents);

    let de_polyline =
        polyline::decode_polyline(&contents, 5).expect("Unable to decode polyline string");

    let mut collection = Vec::new();

    for line in de_polyline.lines() {
        //  println!("Line {:?}", line);
        collection.append(
            line_to_points(
                &line,
                (opts.speed_kmph / 3.6) as f64,
                opts.interval_s as f64,
            )
            .unwrap()
            .as_mut(),
        );
    }

    let collection = create_collision(collection, &opts);
    generate_log(collection, opts.interval_s as f64, &opts);
}

fn bearing_to_heading(bearing: f64) -> f64 {
    assert!(bearing.abs() <= 180.0);
    if bearing > 0.0 {
        bearing
    } else {
        180.0 - bearing.abs() + 180.0
    }
}

fn gprmc(time: DateTime<Utc>, pos: &Gprmc) -> String {
    let (lat, ns) = lat_to_ns(pos.location.lat());
    let (lng, ew) = lng_to_ew(pos.location.lng());

    let content = format!(
        "GPRMC,{:02}{:02}{:02},A,{},{},{},{},{},{},{:02}{:02}{:02},0,W,*",
        time.hour(),
        time.minute(),
        time.second(),
        lat_to_dm(lat),
        ns,
        lng_to_dm(lng),
        ew,
        format_speed(pos.speed_kn),
        format_speed(pos.track),
        time.day(),
        time.month(),
        time.year() - 2000
    );
    let checksum = content.chars().fold(0, |sum, x| sum ^ x as u8);

    if let Some(ped) = &pos.pedestrian {
        let (lat, ns) = lat_to_ns(ped.pos.lat());
        let (lng, ew) = lng_to_ew(ped.pos.lng());

        let pedestrian_content = format!(
        "GPRMC,{:02}{:02}{:02},A,{},{},{},{},{},{},{:02}{:02}{:02},0,W,*",
        time.hour(),
        time.minute(),
        time.second(),
        lat_to_dm(lat),
        ns,
        lng_to_dm(lng),
        ew,
        format_speed(ped.speed_kn),
        format_speed(ped.track),
        time.day(),
        time.month(),
        time.year() - 2000
    );
        let ped_checksum = pedestrian_content.chars().fold(0, |sum, x| sum ^ x as u8);
        format!("${}{:02X};${}{:02X}", content, checksum,pedestrian_content, ped_checksum)
    } else {
           format!("${}{:02X}", content, checksum)

    }
 }

//print the LOG
fn generate_log(collection: std::vec::Vec<Gprmc>, interval_s: f64, opts: &Opts) {
    let mut now = Utc::now();
    let duration = Duration::milliseconds((interval_s * 1000.0) as i64);

    let mut file: Option<File> = if let Some(output) = &opts.output {
        Some(File::create(&output).expect("Unable to open file for writing"))
    } else {
        None
    };

    for pos in collection.iter() {
        now = now + duration;

        let content = gprmc(now, &pos);

        match &mut file {
            Some(file) => write!(file, "{}\n", content).expect("Unable to write to file"),
            None => println!("{}", content),
        };
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
        pedestrian: None,
    };

    points.push(starting_point.clone());

    (0..num_segments).fold(starting_point, |point, _x| {
        let next = Gprmc {
            location: point
                .location
                .haversine_destination(heading, distance / num_segments as f64),
            speed_kn: speed_in_knots,
            track: heading,
            pedestrian: None,
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

//fn lat_to_dm(dec: f64) -> String {
//    let deg = dec.trunc() as u32;
//    let min = dec.fract() * 60.0;
//    format!("{:02}{:02}.{}", deg, min.trunc() as u32, format_fract(min))
//}

fn lat_to_dm(dec: f64) -> String {
    let deg = dec.trunc() as u32;
    let min = dec.fract() * 60.0;
    format!("{:02}{:08.5}", deg, min)
}

fn lng_to_dm(dec: f64) -> String {
    let deg = dec.trunc() as u32;
    let min = dec.fract() * 60.0;
    format!("{:03}{:08.5}", deg, min)
}


fn format_speed(spd: f64) -> String {
    format!("{:05.1}", spd)
}

//fn lng_to_dm(dec: f64) -> String {
//    let deg = dec.trunc() as u32;
//    let min = dec.fract() * 60.0;
//    format!("{:03}{:02}.{}", deg, min.trunc() as u32, format_fract(min))
//}

//fn format_speed(spd: f64) -> String {
//    let trunc = spd.trunc() as u32;
//    let fract = format!("{:.*}", 1, spd.fract());
//    format!("{:03}.{}", trunc, &fract[2..])
//}

fn format_fract(num: f64) -> String {
    let fract = format!("{:.*}", 5, num.fract());
    format!("{}", &fract[2..])
}

// Create a 10 seconds of GPS traces for a pedestrian who will intersect with the path of the vehicle
fn create_collision(mut points: Vec<Gprmc>, opts: &Opts) -> Vec<Gprmc> {
    // first get the collision point.
    let collision_point_index =
        (opts.collision_after_s.unwrap() as f32 / opts.interval_s).trunc() as usize;

    if collision_point_index < points.len() && collision_point_index > 0 {

    } else {
        // Nothing to do here, the collision point is beyond the data we have
        println!("Collision time is beyond the time for which we have the data. No collision");
    }

    let (col, precol) = (
        points[collision_point_index].location.clone(),
        points[collision_point_index - 1].location.clone(),
    );

    let rotated_point = precol.rotate_around_point(opts.collision_angle as f64, col);

    let pedestrian_track = create_pedestrian_track(&col, &rotated_point, opts);
    // The pedestrian track is reversed, starting at the point of potential collision.

    let mut ped_track_index = 0;

    for i in (1..collision_point_index+1).rev() {
        if ped_track_index < pedestrian_track.len() {
            points[i].pedestrian = Some(pedestrian_track[ped_track_index].clone());
            ped_track_index = ped_track_index + 1;
        } else {
            break;
        }
    }

    points
}

fn kmph_to_knots(kmph: f64) -> f64 {
    kmph / 1.852
}

fn kmph_to_ms(kmph: f64) -> f64 {
    kmph / 3.6
}

fn create_pedestrian_track(
    end: &geo::Point<f64>,
    previous: &geo::Point<f64>,
    opts: &Opts,
) -> Vec<Pedestrian> {
    // find the bearing from the collision point to the rotated point
    let bearing = end.bearing(*previous);

    let track = bearing_to_heading(previous.bearing(*end));

    let mut points = Vec::new();

    let ped = Pedestrian {
        pos : end.clone(),
        track : track,
        speed_kn : kmph_to_knots(opts.pedestrian_speed),
    };

    points.push(ped);

    let dist = kmph_to_ms(opts.pedestrian_speed) / opts.interval_s as f64;

    let pedestrian_track_length =
        (opts.pedestrian_track_length as f32 / opts.interval_s).trunc() as usize;
    for i in 1..pedestrian_track_length {
        let ped = Pedestrian {
            pos : end.haversine_destination(bearing, dist * i as f64),
            track : track,
            speed_kn : kmph_to_knots(opts.pedestrian_speed)
        };
        points.push(ped);
    }

    points
}
