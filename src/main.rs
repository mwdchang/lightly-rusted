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
use collisions::intersect_unit_cube;
use collisions::HitRecord;

mod utils;

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
    scene: &Scene,
    depth: u8
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


        let mesh_id = node.get_mesh_id().as_deref();
        let res = if mesh_id == Some("sphere") {
            intersect_unit_sphere(n_ray.origin, n_ray.direction)
        } else if mesh_id == Some("cone") {
            intersect_unit_cone(n_ray.origin, n_ray.direction)
        } else if mesh_id == Some("cube") {
            intersect_unit_cube(n_ray.origin, n_ray.direction)
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

        // Recurse
        for child in node.get_children() {
            visit(child, ray, hits);
        }
    }

    // Walk the scene
    let mut hits:Vec<HitRecord> = vec![];
    visit(scene.get_root(), ray, &mut hits);

    if hits.is_empty() {
        // return Vector3::new(0.0, 0.0, 0.0)
        return scene.environment.background;
    }

    let mut contribution: Vector3<f32> = Vector3::zeros();
    let mut specular: Vector3<f32> = Vector3::zeros();
    let hit = hits
        .iter()
        .filter(|h| h.t > 0.001)
        .min_by(|a, b| a.t.partial_cmp(&b.t).unwrap());

    if hit.is_none() {
        return scene.environment.background;
    }

    let hit = hit.unwrap();


    for light in scene.get_point_lights() {
        // Cast shadow ray to check if the light has any contributions
        let shadow_ray = Ray {
            direction: hit.normal,
            origin: hit.point + hit.normal * 0.0001
        };
        let mut shadow_hits:Vec<HitRecord> = vec![];
        visit(scene.get_root(), &shadow_ray, &mut shadow_hits);
        if !shadow_hits.is_empty() {
            continue
        }


        // Light can reach, get material and compute diffuce and specular
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


    // Check for reflections
    let material = scene.get_materials().get(hit.material_id as usize).unwrap();
    let reflectivity = material.reflectivity;

    if reflectivity > 0.0 {
        let mut reflect_contribution: Vector3<f32> = Vector3::zeros();
        let reflect_direction = (
            ray.direction - 2.0 * ray.direction.dot(&hit.normal) * hit.normal
        ).normalize();

        let reflect_ray = Ray {
            direction: reflect_direction,
            origin: hit.point + 0.0001 * hit.normal
        };

        if depth < 3 {
            reflect_contribution = intersect(&camera, &reflect_ray, &scene, depth+1);
        }

        return 
            (1.0 - reflectivity) * (contribution + specular + scene.environment.ambient_light) + 
            reflectivity * reflect_contribution;
    }

    return contribution + specular + scene.environment.ambient_light; 

    // println!("({}):{} ==>  {}", depth, ray.direction, reflect_ray.direction);
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
            let color = intersect(&camera, &ray, &scene, 0);

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

    let scene = read_scene(&args.scene_file);
    scene.print_tree();

    // Camera parameters
    let camera_position = scene.environment.camera_position;
    let camera_target = scene.environment.camera_target;

    let camera = Camera::look_at(
        camera_position,
        camera_target,
        Vector3::y(),
        60.0,
        args.width,
        args.height,
    );

    let image = render(args.width, args.height, &camera, &scene);
    image.save("render-result.png").expect("Failed to save PNG");
    println!("Rendered {}x{} image", args.width, args.height);
}
