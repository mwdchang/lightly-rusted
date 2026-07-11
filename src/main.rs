use image::{Rgb, RgbImage};
use nalgebra::Vector3;

mod camera;
use camera::Camera;
use camera::Ray;

/** format
vim.keymap.set("n", "<leader>f", function()
    vim.lsp.buf.format()
end, { desc = "Format file" })
**/


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

    image.save("render-result.png").expect("Failed to save PNG");

    println!("Rendered {}x{} image", width, height);
}
