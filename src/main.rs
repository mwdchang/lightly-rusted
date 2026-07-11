use image::{Rgb, RgbImage};
use nalgebra::Vector3;

mod camera;
use camera::Camera;
use camera::Ray;

mod models;
use models::Scene;
use models::Node;

mod collisions;
use collisions::intersect_unit_sphere;
use collisions::HitRecord;


/** format
vim.keymap.set("n", "<leader>f", function()
    vim.lsp.buf.format()
end, { desc = "Format file" })
**/


// Placeholder for real geometry intersection.
// Later this will contain sphere/triangle tests.
fn intersect(ray: &Ray, scene: &Scene) -> Vector3<f32> {

    fn visit(node: &Node, ray: &Ray, hits: &mut Vec<HitRecord>) {
        // println!("{:?}", node.get_transform_world());

        if node.get_mesh_id().as_deref() == Some("sphere") {
            // println!("checking sphere");
            if let Some(t) = intersect_unit_sphere(ray.origin, ray.direction) {
                let hit_point = ray.origin + ray.direction * t;
                // println!("Hit at {:?}", hit_point);
                hits.push( HitRecord {
                    t,
                    point: hit_point,
                    normal: Vector3::new(0.0, 0.0, 0.0),
                    material_id: 0,
                    front_face: true
                })
            }
        }


        for child in node.get_children() {
            visit(child, ray, hits);
        }
    }

    // Walk the scene
    let mut hits:Vec<HitRecord> = vec![];
    visit(scene.get_root(), ray, &mut hits);

    if hits.is_empty() {
        return Vector3::new(0.0, 0.0, 0.0)
    }

    // Orange
    return Vector3::new(1.0, 0.5, 0.0)
}

fn render(width: u32, height: u32, camera: &Camera, scene: &Scene) -> RgbImage {
    let mut image = RgbImage::new(width, height);

    for y in 0..height {
        for x in 0..width {
            let ray = camera.generate_ray(x, y, width, height);

            let color = intersect(&ray, &scene);

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
    let width = 400;
    let height = 300;

    // Camera parameters
    // let camera_position = Vector3::new(0.0, 2.0, 5.0);
    // let camera_target = Vector3::new(0.0, 0.0, 0.0);
    let camera_position = Vector3::new(0.0, 0.0, 4.0);
    let camera_target = Vector3::new(0.0, 0.0, 0.0);

    let camera = Camera::look_at(
        camera_position,
        camera_target,
        Vector3::y(),
        60.0,
        width,
        height,
    );

    // Scene building
    let mut root = Node::new(None);

    let child = Node::new(
        Some("sphere".to_string()),
    );
    root.add_child(child);

    let scene = Scene::new("test scene".to_string(), root);


    let image = render(width, height, &camera, &scene);
    image.save("render-result.png").expect("Failed to save PNG");

    println!("Rendered {}x{} image", width, height);
}
