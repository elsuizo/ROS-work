use env_logger;
use rosrust;
use rosrust::Publisher;
use static_math::V2;
use std::sync::mpsc;
use std::time::Duration;

use rosrust_msg::location_monitor::LandmarkDistance;

#[derive(Debug)]
struct Landmark {
    name: String,
    position: V2<f64>
}

impl Landmark {
    fn new(name: String, position: V2<f64>) -> Self {
        Self{name, position}
    }
}

struct LocationMonitor {
    landmarks: Vec<Landmark>,
    publisher: Publisher<LandmarkDistance>
}

impl LocationMonitor {
    fn init(publisher: Publisher<LandmarkDistance>) -> Self {
        let landmarks = vec![Landmark::new("unit_box".to_string(), V2::new_from(1.0, 1.0)),
                         Landmark::new("unit_cylinder".to_string(), V2::new_from(1.0, -1.0)),
                         Landmark::new("unit_sphere".to_string(), V2::new_from(-1.0, 0.0))];
        Self{landmarks, publisher}
    }

    fn get_distance(landmark: &Landmark, point: V2<f64>) -> f64 {
        let diff = landmark.position - point;
        f64::sqrt(diff[0] * diff[0] + diff[1] * diff[1])
    }

    fn find_closest(&self, point: V2<f64>) -> LandmarkDistance {
        let mut result = LandmarkDistance{name: "".to_string(), distance: -1.0};
        for landmark in &self.landmarks {
            let distance = Self::get_distance(landmark, point);
            if result.distance < 0.0 || distance < result.distance {
                result.name = landmark.name.clone();
                result.distance = distance;
            }
        }
        result
    }
}

fn main() {
    env_logger::init();

    // TODO(elsuizo:2021-07-21): esto hace algo o sirve para algo???
    rosrust::init("location_monitor_rust");
    let (tx, rx) = mpsc::channel();
    let landmark_publisher = rosrust::publish("landmark_distance", 2).unwrap();
    let monitor = LocationMonitor::init(landmark_publisher);

    // NOTE(elsuizo:2021-07-21): nos subscribimos al topic "/odom" y pasamos ese mensaje a traves
    // del thread
    let subscriber_info = rosrust::subscribe("odom", 2, move |msg: rosrust_msg::nav_msgs::Odometry| {
        // Callback for handling received messages
        // rosrust::ros_info!("Received: {:?}", msg.pose.pose.position);
        tx.send(msg).unwrap();
    })
    .unwrap();
    while rosrust::is_ok() {
        // NOTE(elsuizo:2021-07-21): esperamos la recepcion del mensaje
        if let Ok(msg) = rx.recv_timeout(Duration::from_millis(10)) {
            let x = msg.pose.pose.position.x;
            let y = msg.pose.pose.position.y;
            let position = V2::new_from(x, y);
            let landmark_distance = monitor.find_closest(position);
            monitor.publisher.send(landmark_distance);
        }
    }

    rosrust::spin();
}
