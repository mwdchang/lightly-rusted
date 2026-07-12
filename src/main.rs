use image::{Rgb, RgbImage};
use nalgebra::Point3;
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

mod utils;
use utils::translate;

use std::process;

use crate::models::PointLight;


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

        let inv = (
            node.get_transform_local() * 
            node.get_transform_world()
        ).try_inverse().unwrap();

        let new_direction = inv.transform_vector(&ray.direction);
        let p = Point3::new(
            ray.origin[0],
            ray.origin[1],
            ray.origin[2]
        );
        let new_origin = inv.transform_point(&p);
        let new_origin_vec = Vector3::new(
            new_origin[0],
            new_origin[1],
            new_origin[2]
        );

        let n_ray = Ray {
            direction: new_direction,
            origin: new_origin_vec         
        };

        if node.get_mesh_id().as_deref() == Some("sphere") {
            // println!("{}", new_origin_vec);

            // println!("checking sphere");
            if let Some(t) = intersect_unit_sphere(n_ray.origin, n_ray.direction) {
                let hit_point = n_ray.origin + n_ray.direction * t;
                // println!("Hit at {:?}", hit_point);
                hits.push( HitRecord {
                    t,
                    point: hit_point,
                    // normal: Vector3::new(0.0, 0.0, 0.0),
                    normal: hit_point.normalize(),
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

    // TODO: Need to sort hits-buffers
    // TODO: All lights
    
    let light = scene.get_point_lights().get(0);
    let hit = hits.get(0);

    let to_light = light.unwrap().position - hit.unwrap().point;
    let distance = to_light.norm();
    let light_dir = to_light / distance;

    let attenuation = 1.0 / (distance * distance);

    let ndotl = hit.unwrap().normal.dot(&light_dir).max(1.0);

    let contribution =
        // material.albedo
        (Vector3::new(1.0, 0.5, 0.2)).component_mul(    
            &light.unwrap().intensity
        )
        * attenuation
        * ndotl;
    
    // println!("contribution {}", contribution);
    return contribution;
    // Orange
    // return Vector3::new(1.0, 0.5, 0.0)
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

    // process::exit(1);

    // Camera parameters
    // let camera_position = Vector3::new(0.0, 2.0, 5.0);
    // let camera_target = Vector3::new(0.0, 0.0, 0.0);
    let camera_position = Vector3::new(0.0, 0.0, 8.0);
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

    let mut group = Node::new(
        Some("group".to_string()),
    );
    let group_child = Node::new(
        Some("sphere".to_string()),
    );


    group.set_transform(
        translate(Vector3::new(3.0, 2.0, -5.0))
    );
    group.add_child(group_child);       

    root.add_child(child);
    root.add_child(group);


    let mut scene = Scene::new("test scene".to_string(), root);
    scene.add_point_light(PointLight {
        position: Vector3::new(8.0, 8.0, 8.0),
        intensity: Vector3::new(300.0, 300.0, 800.0)
    });

    // scene.print_tree();


    let image = render(width, height, &camera, &scene);
    image.save("render-result.png").expect("Failed to save PNG");

    println!("Rendered {}x{} image", width, height);
}
