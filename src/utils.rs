use nalgebra::{Matrix4, Vector3, Rotation3};

pub fn translate(offset: Vector3<f32>) -> Matrix4<f32> {
    Matrix4::new_translation(&offset)
}

pub fn scale(scale: Vector3<f32>) -> Matrix4<f32> {
    Matrix4::new_nonuniform_scaling(&scale)
}

pub fn rotate_x(angle: f32) -> Matrix4<f32> {
    Rotation3::from_euler_angles(angle, 0.0, 0.0).to_homogeneous()
}

pub fn rotate_y(angle: f32) -> Matrix4<f32> {
    Rotation3::from_euler_angles(0.0, angle, 0.0).to_homogeneous()
}

pub fn rotate_z(angle: f32) -> Matrix4<f32> {
    Rotation3::from_euler_angles(0.0, 0.0, angle).to_homogeneous()
}
