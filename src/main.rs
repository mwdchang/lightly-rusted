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
use collisions::intersect_unit_torus;
use collisions::intersect_unit_sphere;
use collisions::intersect_unit_cone;
use collisions::intersect_unit_cube;
use collisions::intersect_model;
use collisions::HitRecord;

mod utils;

mod argparser;
use argparser::Args;

mod obj;
use obj::load_obj_into_cache;
use crate::obj::ModelCache;

use std::fs;


/** format
vim.keymap.set("n", "<leader>f", function()
    vim.lsp.buf.format()
end, { desc = "Format file" })
**/


/** 
* typical:
* world_transform = translation * rotation * scale
**/

fn intersect(
    camera: &Camera, 
    ray: &Ray, 
    scene: &Scene,
    depth: u8
) -> Vector3<f32> {

    fn visit(node: &Node, ray: &Ray, hits: &mut Vec<HitRecord>, model_cache: &ModelCache) {
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
        } else if mesh_id == Some("torus") {
            intersect_unit_torus(n_ray.origin, n_ray.direction)
        } else if mesh_id == Some("bunny") {
            let m = &model_cache["bunny"];
            intersect_model(m, n_ray.origin, n_ray.direction)
        } else if mesh_id == Some("teapot") {
            let m = &model_cache["teapot"];
            intersect_model(m, n_ray.origin, n_ray.direction)
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

            let mut normal = w_normal;
            let front_face = ray.direction.dot(&normal) < 0.0;
            if !front_face {
                normal = -normal;
            }

            hits.push( HitRecord {
                t: w_t,
                point: w_point,
                normal: normal,
                material_id: node.get_material_id(),
                // front_face: true
                front_face: front_face
            })
        }

        // Recurse
        for child in node.get_children() {
            visit(child, ray, hits, model_cache);
        }
    }

    // Walk the scene
    let mut hits:Vec<HitRecord> = vec![];
    visit(scene.get_root(), ray, &mut hits, &scene.model_cache);

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
        let to_light = light.position - hit.point;
        let shadow_ray = Ray {
            direction: to_light.normalize(),
            origin: hit.point + hit.normal * 0.0001
        };
        let mut shadow_hits:Vec<HitRecord> = vec![];
        visit(scene.get_root(), &shadow_ray, &mut shadow_hits, &scene.model_cache);

        let dist_to_light = to_light.norm();

        let mut visibility = 1.0;
        for shadow_hit in shadow_hits {
            if shadow_hit.t > dist_to_light {
                break; // hit is behind the light
            }

            let s_material = scene.get_materials().get(shadow_hit.material_id as usize).unwrap();
            if s_material.transparency == 0.0 {
                visibility = 0.0;
                break;
            }

            visibility *= s_material.transparency;
            if visibility < 0.001 {
                break;
            }
        }

        // if !shadow_hits.is_empty() {
        //     continue
        // }


        // Light can reach, get material and compute diffuce and specular
        let material = scene.get_materials().get(hit.material_id as usize).unwrap();

        let to_light = light.position - hit.point;
        let distance = to_light.norm();
        let light_dir = to_light / distance;

        // Used to be 1.0, just making things look nice
        let attenuation = 1.0 / (distance * distance);
        let ndotl = hit.normal.dot(&light_dir).max(0.0);

        contribution +=
            material.albedo.component_mul(
                &light.intensity
            )   
            * attenuation
            * ndotl
            * visibility;

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
            * material.specular
            * visibility;
    }


    // Check for reflections
    let material = scene.get_materials().get(hit.material_id as usize).unwrap();
    let reflectivity = material.reflectivity;
    let transparency = material.transparency;


    let local_contrib = (contribution + specular + scene.environment.ambient_light);

    let mut reflect_contrib: Vector3<f32> = Vector3::zeros();
    let mut refract_contrib: Vector3<f32> = Vector3::zeros();

    if reflectivity > 0.0 {
        let reflect_direction = (
            ray.direction - 2.0 * ray.direction.dot(&hit.normal) * hit.normal
        ).normalize();

        let reflect_ray = Ray {
            direction: reflect_direction,
            origin: hit.point + 0.0001 * hit.normal
        };

        if depth < 4 {
            reflect_contrib = intersect(&camera, &reflect_ray, &scene, depth+1);
        }
    }

    // Calculate refraction
    if transparency > 0.0 {
        // Snells
        let (n1, n2) = if hit.front_face {
            (1.0, material.ior)
        } else {
            (material.ior, 1.0)
        };
        let eta = n1 / n2;
        let cos_theta = (-ray.direction)
            .dot(&hit.normal)
            .min(1.0);

        let sin2_theta = 1.0 - cos_theta * cos_theta;
        let k = 1.0 - eta * eta * sin2_theta;

        if k < 0.0 {
            // noop, total interal refraction
        } else {
            let refracted_dir =
                eta * ray.direction
                + (eta * cos_theta - k.sqrt()) * hit.normal;

            let offset =
                if hit.front_face {
                    -hit.normal
                } else {
                     hit.normal
                };

            let refract_ray = Ray {
                direction: refracted_dir.normalize(),
                origin: hit.point + offset * 0.0001
            };
            refract_contrib = intersect(&camera, &refract_ray, &scene, depth+1);
        }
    }

    return (1.0 - reflectivity - transparency) * local_contrib +
        reflectivity * reflect_contrib +
        transparency * refract_contrib;

    // return contribution + specular + scene.environment.ambient_light; 
    // println!("({}):{} ==>  {}", depth, ray.direction, reflect_ray.direction);
}


struct RenderPatch {
    x0: u32,
    y0: u32,
    x1: u32,
    y1: u32,
}

struct RenderedPixel {
    x: u32,
    y: u32,
    color: Rgb<u8>,
}

fn create_patches(width: u32, height: u32) -> Vec<RenderPatch> {
    const PATCH_SIZE: u32 = 96;

    let mut patches = Vec::new();

    let patches_x = (width + PATCH_SIZE - 1) / PATCH_SIZE;
    let patches_y = (height + PATCH_SIZE - 1) / PATCH_SIZE;

    for patch_y in 0..patches_y {
        for patch_x in 0..patches_x {
            let x0 = patch_x * PATCH_SIZE;
            let y0 = patch_y * PATCH_SIZE;

            let x1 = (x0 + PATCH_SIZE).min(width);
            let y1 = (y0 + PATCH_SIZE).min(height);

            patches.push(RenderPatch {
                x0,
                y0,
                x1,
                y1,
            });
        }
    }
    patches
}

fn render_patch(
    total_patches: usize,
    patch_idx: usize,
    patch: &RenderPatch,
    width: u32,
    height: u32,
    camera: &Camera,
    scene: &Scene,
) -> Vec<RenderedPixel> {
    let mut pixels = Vec::new();

    for y in patch.y0..patch.y1 {
        for x in patch.x0..patch.x1 {
            let ray = camera.generate_ray(x, y, width, height);
            let color = intersect(camera, &ray, scene, 0);

            pixels.push(RenderedPixel {
                x,
                y,
                color: Rgb([
                    (color.x.clamp(0.0, 1.0) * 255.0) as u8,
                    (color.y.clamp(0.0, 1.0) * 255.0) as u8,
                    (color.z.clamp(0.0, 1.0) * 255.0) as u8,
                ]),
            });
        }
    }

    println!("patch {}/{}", patch_idx + 1, total_patches);

    return pixels;
}


use rayon::prelude::*;
use rayon::ThreadPoolBuilder;


fn render(
    width: u32,
    height: u32,
    camera: &Camera,
    scene: &Scene,
) -> RgbImage {
    let mut image = RgbImage::new(width, height);
    let patches = create_patches(width, height);

    // Parallel rendering of patches using rayon
    let patch_results: Vec<Vec<RenderedPixel>> = patches
        .par_iter()
        .enumerate()
        .map(|(patch_idx, patch)| render_patch(patches.len(), patch_idx, patch, width, height, camera, scene))
        .collect();

    // let mut cnt: u32 = 0;
    // let num_patches = patch_results.len() as u32;

    println!("Assemble patches...");
    for pixels in patch_results {
        // cnt += 1;
        for pixel in pixels {
            image.put_pixel(pixel.x, pixel.y, pixel.color);
        }
        // println!("Done {}/{}", cnt, num_patches);
    }
    image
}


fn main() {
    let args = Args::parse();
    let mut scene = read_scene(&args.scene_file);
    scene.print_tree();

    println!("Loading models.....");
    for entry in fs::read_dir("./models").unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if let Some(name) = path.file_stem().and_then(|s| s.to_str()) {
            println!("{} name={}", path.to_str().unwrap(), name);
            load_obj_into_cache(
                &mut scene.model_cache,
                name,
                path.to_str().unwrap(),
                true,
            ).unwrap();
        }
    }
    println!("Done loading models.....");

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

    ThreadPoolBuilder::new()
        .num_threads(args.workers)
        .build_global()
        .unwrap();

    let image = render(args.width, args.height, &camera, &scene);
    image.save("render-result.png").expect("Failed to save PNG");
    println!("Rendered {}x{} image", args.width, args.height);
}
