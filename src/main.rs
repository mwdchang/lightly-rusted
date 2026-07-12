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
use crate::models::Material;
use crate::models::read_scene;
use crate::utils::scale;

use std::collections::HashMap;






/** format
vim.keymap.set("n", "<leader>f", function()
    vim.lsp.buf.format()
end, { desc = "Format file" })
**/


// Placeholder for real geometry intersection.
// Later this will contain sphere/triangle tests.
fn intersect(
    camera: &Camera, 
    ray: &Ray, 
    scene: &Scene
) -> Vector3<f32> {

    fn visit(node: &Node, ray: &Ray, hits: &mut Vec<HitRecord>) {
        // println!("{:?}", node.get_transform_world());
        // let inv = (
        //     node.get_transform_local() * 
        //     node.get_transform_world()
        // ).try_inverse().unwrap();
        
        // Transform ray to local coordinate space
        let inv = node.get_transform_inverse();
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
            if let Some(t) = intersect_unit_sphere(n_ray.origin, n_ray.direction) {
                let hit_point = n_ray.origin + n_ray.direction * t;
                hits.push( HitRecord {
                    t,
                    point: hit_point,
                    // normal: Vector3::new(0.0, 0.0, 0.0),
                    normal: hit_point.normalize(),
                    material_id: node.get_material_id(),
                    front_face: true
                })
            }
        }

        // Recurse
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

    // TODO: Need to sort hits/depth-buffers
    // TODO: All light sources
    // TODO: check shine material
    
    let light = scene.get_point_lights().get(0).unwrap();
    let hit = hits.get(0).unwrap();

    let to_light = light.position - hit.point;
    let distance = to_light.norm();
    let light_dir = to_light / distance;

    // let attenuation = 1.0 / (distance * distance);
    let attenuation = 2.5 / (distance * distance);

    let ndotl = hit.normal.dot(&light_dir).max(1.0);

    let material = scene.get_materials().get(hit.material_id as usize).unwrap();

    let contribution =
        material.albedo.component_mul(
            &light.intensity
        )   
        * attenuation
        * ndotl;
    
    /*
    let shine:f32 = 300.0;
    let view_dir = (camera.get_position() - hit.unwrap().point).normalize();
    let reflect_dir = -light_dir - 2.0 * (-light_dir).dot(&hit.unwrap().normal) * hit.unwrap().normal;
    let reflect_dir = reflect_dir.normalize();
    let spec = view_dir.dot(&reflect_dir).max(0.0).powf(shine);
    let specular = light.unwrap().intensity * spec;
    */
    
    let view_dir = (camera.get_position() - hit.point).normalize();
    let halfway = (light_dir + view_dir).normalize();
    let spec = hit.normal
        .dot(&halfway)
        .max(0.0)
        .powf(material.shine);
    let specular = light.intensity * spec;

    return contribution + specular;
}

fn render(
    width: u32, 
    height: u32, 
    camera: &Camera, 
    scene: &Scene
) -> RgbImage {
    let mut image = RgbImage::new(width, height);
    for y in 0..height {
        for x in 0..width {
            let ray = camera.generate_ray(x, y, width, height);

            let color = intersect(&camera, &ray, &scene);

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
    let camera_position = Vector3::new(0.0, 0.0, 6.0);
    let camera_target = Vector3::new(0.0, 0.0, 0.0);

    let camera = Camera::look_at(
        camera_position,
        camera_target,
        Vector3::y(),
        60.0,
        width,
        height,
    );

    let mut scene = read_scene("scene01.json");

    // // Scene building
    // let mut root = Node::new(None);

    // let mut child = Node::new(
    //     Some("sphere".to_string()),
    // );
    // child.set_material_id(0);

    // let mut child2 = Node::new(
    //     Some("sphere".to_string()),
    // );
    // child2.set_transform(
    //     translate(Vector3::new(0.0, -3.0, -12.0)) * scale(Vector3::new(6.0, 6.0, 6.0))
    // );
    // child2.set_material_id(2);



    // let mut group = Node::new(
    //     Some("group".to_string()),
    // );
    // let mut group_child = Node::new(
    //     Some("sphere".to_string()),
    // );
    // group_child.set_material_id(1);


    // group.set_transform(
    //     translate(Vector3::new(3.0, 2.0, -5.0))
    // );
    // group.add_child(group_child);       

    // root.add_child(child);
    // root.add_child(child2);
    // root.add_child(group);

    // scene.set_root(root);

    scene.print_tree();

    let image = render(width, height, &camera, &scene);
    image.save("render-result.png").expect("Failed to save PNG");
    println!("Rendered {}x{} image", width, height);
}
