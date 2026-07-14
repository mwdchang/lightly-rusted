use image::{Rgb, RgbImage};
use nalgebra::Point3;
use nalgebra::Vector3;

mod camera;
use camera::Camera;
use camera::Ray;

mod models;
use models::Scene;
use models::Node;
use models::read_scene;

mod collisions;
use collisions::intersect_unit_sphere;
use collisions::intersect_unit_cone;
use collisions::HitRecord;

mod utils;
use std::process;


mod argparser;
use argparser::Args;




/** format
vim.keymap.set("n", "<leader>f", function()
    vim.lsp.buf.format()
end, { desc = "Format file" })
**/


/** 
* typical:
* world_transform = translation * rotation * scale
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
        
        // Transform ray to local coordinate space
        let inv = node.get_transform_inverse();

        let n_ray = Ray {
            direction: inv.transform_vector(&ray.direction),
            origin: inv
                .transform_point(&Point3::from(ray.origin))
                .coords,
        };


        // let new_direction = inv.transform_vector(&ray.direction);
        // let p = Point3::new(
        //     ray.origin[0],
        //     ray.origin[1],
        //     ray.origin[2]
        // );
        // let new_origin = inv.transform_point(&p);
        // let new_origin_vec = Vector3::new(
        //     new_origin[0],
        //     new_origin[1],
        //     new_origin[2]
        // );

        // let n_ray = Ray {
        //     direction: new_direction,
        //     origin: new_origin_vec         
        // };

        let mesh_id = node.get_mesh_id().as_deref();

        let res = if mesh_id == Some("sphere") {
            intersect_unit_sphere(n_ray.origin, n_ray.direction)
        } else if mesh_id == Some("cone") {
            intersect_unit_cone(n_ray.origin, n_ray.direction)
        } else {
            None
        };

        if res.is_some() {
            let r = res.unwrap();
            let w_point = (
                node.get_transform_local() 
                * node.get_transform_world()
                * r.hit_point.push(1.0)
            ).xyz();

            let w_normal = (
                node.get_transform_inverse().transpose() * 
                r.normal.push(0.0)
            ).xyz().normalize();                

            let w_t = (w_point - ray.origin).norm();

            hits.push( HitRecord {
                t: w_t,
                point: w_point,
                normal: w_normal,
                material_id: node.get_material_id(),
                front_face: true
            })
        }

        /*
        if node.get_mesh_id().as_deref() == Some("sphere") {
            if let Some(res) = intersect_unit_sphere(n_ray.origin, n_ray.direction) {
                let w_point = (
                    node.get_transform_local() 
                    * node.get_transform_world()
                    * res.hit_point.push(1.0)
                ).xyz();

                // println!("{} ==> {}", hit_point.z, w_point.z);

                let w_normal = (
                    node.get_transform_inverse().transpose() * 
                    res.normal.push(0.0)
                ).xyz().normalize();                

                let w_t = (w_point - ray.origin).norm();


                // hits.push( HitRecord {
                //     t,
                //     point: hit_point,
                //     normal: hit_point.normalize(),
                //     material_id: node.get_material_id(),
                //     front_face: true
                // })
                
                hits.push( HitRecord {
                    t: w_t,
                    point: w_point,
                    normal: w_normal,
                    material_id: node.get_material_id(),
                    front_face: true
                })

            }
        }
        */

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

    let mut contribution: Vector3<f32> = Vector3::zeros();
    let mut specular: Vector3<f32> = Vector3::zeros();
    let ambient :Vector3<f32> = Vector3::new(0.1, 0.1, 0.1);
    let hit = hits
        .iter()
        .filter(|h| h.t > 0.001)
        .min_by(|a, b| a.t.partial_cmp(&b.t).unwrap())
        .unwrap();


    for light in scene.get_point_lights() {
        let material = scene.get_materials().get(hit.material_id as usize).unwrap();

        let to_light = light.position - hit.point;
        let distance = to_light.norm();
        let light_dir = to_light / distance;

        // Used to be 1.0, just making things look nice
        let attenuation = 2.5 / (distance * distance);
        let ndotl = hit.normal.dot(&light_dir).max(0.0);

        contribution +=
            material.albedo.component_mul(
                &light.intensity
            )   
            * attenuation
            * ndotl;

        let view_dir = (camera.get_position() - hit.point).normalize();
        let halfway = (light_dir + view_dir).normalize();
        let mut spec = hit.normal
            .dot(&halfway)
            .max(0.0)
            .powf(material.shine);

        
        if ndotl <= 0.0 {
            spec = 0.0
        }

        specular += 
            light.intensity 
            * spec
            * material.specular;
    }
    return contribution + specular + ambient;
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
    let args = Args::parse();

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
        args.width,
        args.height,
    );

    let scene = read_scene(&args.scene_file);
    scene.print_tree();

    let image = render(args.width, args.height, &camera, &scene);
    image.save("render-result.png").expect("Failed to save PNG");
    println!("Rendered {}x{} image", args.width, args.height);
}
