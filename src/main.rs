use image::{Rgb, RgbImage};
use nalgebra::Vector3;

struct Ray {
    origin: Vector3<f32>,
    direction: Vector3<f32>,
}

/** format
vim.keymap.set("n", "<leader>f", function()
    vim.lsp.buf.format()
end, { desc = "Format file" })
**/

struct Camera {
    position: Vector3<f32>,

    // Camera basis vectors
    forward: Vector3<f32>,
    right: Vector3<f32>,
    up: Vector3<f32>,

    fov_y: f32,
    aspect_ratio: f32,
}

impl Camera {
    fn look_at(
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

    fn generate_ray(&self, x: u32, y: u32, width: u32, height: u32) -> Ray {
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

// Placeholder for real geometry intersection.
// Later this will contain sphere/triangle tests.
fn intersect(_ray: &Ray) -> Vector3<f32> {
    // Orange
    Vector3::new(1.0, 0.5, 0.0)
}

fn render(width: u32, height: u32, camera: &Camera) -> RgbImage {
    let mut image = RgbImage::new(width, height);

    for y in 0..height {
        for x in 0..width {
            let ray = camera.generate_ray(x, y, width, height);

            let color = intersect(&ray);

            image.put_pixel(
                x,
                y,
                Rgb([
                    (color.x.clamp(0.0, 1.0) * 255.0) as u8,
                    (color.y.clamp(0.0, 1.0) * 255.0) as u8,
                    (color.z.clamp(0.0, 1.0) * 255.0) as u8,
                ]),
            );
        }
    }

    return image
}

fn main() {
    // Image parameters
    let width = 800;
    let height = 600;

    // Camera parameters
    let camera_position = Vector3::new(0.0, 2.0, 5.0);

    let camera_target = Vector3::new(0.0, 0.0, 0.0);

    let camera = Camera::look_at(
        camera_position,
        camera_target,
        Vector3::y(),
        60.0,
        width,
        height,
    );

    let image = render(width, height, &camera);

    image.save("render.png").expect("Failed to save PNG");

    println!("Rendered {}x{} image", width, height);
}
