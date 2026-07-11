use nalgebra::Vector3;

pub struct Ray {
    origin: Vector3<f32>,
    direction: Vector3<f32>,
}


pub struct Camera {
    position: Vector3<f32>,

    // Camera basis vectors
    forward: Vector3<f32>,
    right: Vector3<f32>,
    up: Vector3<f32>,

    fov_y: f32,
    aspect_ratio: f32,
}


impl Camera {
    pub fn look_at(
        eye: Vector3<f32>,
        target: Vector3<f32>,
        world_up: Vector3<f32>,
        fov_degrees: f32,
        width: u32,
        height: u32,
    ) -> Self {
        let forward = (target - eye).normalize();

        let right = forward.cross(&world_up).normalize();

        let up = right.cross(&forward).normalize();

        Self {
            position: eye,

            forward,
            right,
            up,

            fov_y: fov_degrees.to_radians(),
            aspect_ratio: width as f32 / height as f32,
        }
    }

    pub fn generate_ray(&self, x: u32, y: u32, width: u32, height: u32) -> Ray {
        // Convert pixel coordinate into normalized device coordinates
        // range: [-1, 1]
        let px = ((x as f32 + 0.5) / width as f32) * 2.0 - 1.0;

        let py = 1.0 - ((y as f32 + 0.5) / height as f32) * 2.0;

        // Size of the virtual image plane
        let half_height = (self.fov_y / 2.0).tan();

        let half_width = self.aspect_ratio * half_height;

        // Construct ray direction
        let direction =
            (self.forward + self.right * (px * half_width) + self.up * (py * half_height))
                .normalize();

        Ray {
            origin: self.position,
            direction,
        }
    }
}

