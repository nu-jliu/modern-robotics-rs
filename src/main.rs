use modern_robotics_rs;

fn main() {
    let result = modern_robotics_rs::modern_robotics::near_zero(0.03);
    assert!(!result);
    println!("Hello World");
}
