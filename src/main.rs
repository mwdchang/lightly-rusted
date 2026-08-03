use image::{Rgb, RgbImage};

use nalgebra::Point3;
use nalgebra::Vector3;

use nalgebra::Vector2;

mod camera;
use camera::Camera;
use camera::Ray;

mod models;
use models::Scene;
use models::Node;
use models::read_scene;
use models::NodeType;

mod collisions;
use collisions::intersect_unit_torus;
use collisions::intersect_unit_sphere;
use collisions::intersect_unit_cone;
use collisions::intersect_unit_cylinder;
use collisions::intersect_unit_cube;
use collisions::intersect_model;
use collisions::HitRecord;

mod collisions_all;
use collisions_all::{
    intersect_all_unit_sphere,
    intersect_all_unit_cube,
    intersect_all_unit_cylinder,
    intersect_all_unit_cone,
    intersect_all_unit_torus,
};

mod utils;

mod argparser;
use argparser::Args;

mod obj;
use obj::load_obj_into_cache;
use crate::obj::ModelCache;


mod texture;
use texture::load_texture;
use texture::sample_texture;

use std::fs;


/** format
vim.keymap.set("n", "<leader>f", function()
    vim.lsp.buf.format()
end, { desc = "Format file" })
**/


struct WorldIntersectInterval {
    t_enter: f32,
    t_exit: f32,
    point_enter: Vector3<f32>,
    point_exit: Vector3<f32>,
    normal_enter: Vector3<f32>,
    normal_exit: Vector3<f32>,
    uv_enter: Vector2<f32>,
    uv_exit: Vector2<f32>,
    material_id_enter: u32,
    material_id_exit: u32,
}

fn get_primitive_interval(
    node: &Node,
    ray: &Ray,
) -> Option<WorldIntersectInterval> {
    let inv = node.get_transform_inverse();
    let local_ray = Ray {
        direction: inv.transform_vector(&ray.direction).normalize(),
        origin: inv.transform_point(&Point3::from(ray.origin)).coords,
    };

    let mesh_id = node.get_mesh_id().as_deref()?;
    let local_interval = if mesh_id == "sphere" {
        intersect_all_unit_sphere(local_ray.origin, local_ray.direction)
    } else if mesh_id == "cone" {
        intersect_all_unit_cone(local_ray.origin, local_ray.direction)
    } else if mesh_id == "cylinder" {
        intersect_all_unit_cylinder(local_ray.origin, local_ray.direction)
    } else if mesh_id == "cube" {
        intersect_all_unit_cube(local_ray.origin, local_ray.direction)
    } else if mesh_id == "torus" {
        intersect_all_unit_torus(local_ray.origin, local_ray.direction)
    } else {
        None
    }?;

    let point_enter_local = local_ray.origin + local_ray.direction * local_interval.t_enter;
    let point_exit_local = local_ray.origin + local_ray.direction * local_interval.t_exit;

    // Transform points to world space
    let transform = node.get_transform_world() * node.get_transform_local();
    let point_enter = (transform * point_enter_local.push(1.0)).xyz();
    let point_exit = (transform * point_exit_local.push(1.0)).xyz();

    // Transform normals to world space using inverse transpose
    let normal_enter = (inv.transpose() * local_interval.normal_enter.push(0.0)).xyz().normalize();
    let normal_exit = (inv.transpose() * local_interval.normal_exit.push(0.0)).xyz().normalize();

    // Calculate world t values
    let mut t_enter = (point_enter - ray.origin).dot(&ray.direction);
    let mut t_exit = (point_exit - ray.origin).dot(&ray.direction);

    // If for some reason transforms swapped the entry/exit order, swap them
    let (p_enter, p_exit, n_enter, n_exit, uv_e, uv_x) = if t_enter <= t_exit {
        (point_enter, point_exit, normal_enter, normal_exit, local_interval.uv_enter, local_interval.uv_exit)
    } else {
        std::mem::swap(&mut t_enter, &mut t_exit);
        (point_exit, point_enter, normal_exit, normal_enter, local_interval.uv_exit, local_interval.uv_enter)
    };

    Some(WorldIntersectInterval {
        t_enter,
        t_exit,
        point_enter: p_enter,
        point_exit: p_exit,
        normal_enter: n_enter,
        normal_exit: n_exit,
        uv_enter: uv_e,
        uv_exit: uv_x,
        material_id_enter: node.get_material_id(),
        material_id_exit: node.get_material_id(),
    })
}

fn make_csg_hit(
    t: f32,
    point: Vector3<f32>,
    outward_normal: Vector3<f32>,
    uv: Vector2<f32>,
    material_id: u32,
    ray_dir: &Vector3<f32>,
) -> HitRecord {
    let front_face = ray_dir.dot(&outward_normal) < 0.0;
    let normal = if front_face {
        outward_normal
    } else {
        -outward_normal
    };

    HitRecord {
        t,
        point,
        normal,
        material_id,
        front_face,
        uv,
    }
}

fn intersect_csg_difference(
    node: &Node,
    ray: &Ray,
    hits: &mut Vec<HitRecord>,
) {
    let children = node.get_children();
    if children.len() < 2 {
        return;
    }
    let child_a = &children[0];
    let child_b = &children[1];

    let interval_a = match get_primitive_interval(child_a, ray) {
        Some(i) => i,
        None => return,
    };
    let interval_b = get_primitive_interval(child_b, ray);


    #[allow(non_snake_case)]
    let tA_enter = interval_a.t_enter;

    #[allow(non_snake_case)]
    let tA_exit = interval_a.t_exit;

    if let Some(i_b) = interval_b {
        #[allow(non_snake_case)]
        let tB_enter = i_b.t_enter;

        #[allow(non_snake_case)]
        let tB_exit = i_b.t_exit;

        // Case 1: No overlap
        if tB_exit <= tA_enter || tB_enter >= tA_exit {
            hits.push(make_csg_hit(
                tA_enter, 
                interval_a.point_enter, 
                interval_a.normal_enter, 
                interval_a.uv_enter, 
                interval_a.material_id_enter, 
                &ray.direction)
            );
            hits.push(make_csg_hit(
                tA_exit, 
                interval_a.point_exit, 
                interval_a.normal_exit, 
                interval_a.uv_exit, 
                interval_a.material_id_exit, 
                &ray.direction)
            );
        }
        // Case 2: B completely covers A
        else if tB_enter <= tA_enter && tB_exit >= tA_exit {
            // No hits
        }
        // Case 3: B covers start of A
        else if tB_enter <= tA_enter && tB_exit > tA_enter && tB_exit < tA_exit {
            // Remaining interval is [tB_exit, tA_exit]
            hits.push(make_csg_hit(
                tB_exit, 
                i_b.point_exit, 
                -i_b.normal_exit, 
                i_b.uv_exit, 
                i_b.material_id_exit, 
                &ray.direction)
            );
            hits.push(make_csg_hit(
                tA_exit, 
                interval_a.point_exit, 
                interval_a.normal_exit, 
                interval_a.uv_exit, 
                interval_a.material_id_exit, 
                &ray.direction)
            );
        }
        // Case 4: B covers end of A
        else if tB_enter > tA_enter && tB_enter < tA_exit && tB_exit >= tA_exit {
            // Remaining interval is [tA_enter, tB_enter]
            hits.push(make_csg_hit(
                tA_enter, 
                interval_a.point_enter, 
                interval_a.normal_enter, 
                interval_a.uv_enter, 
                interval_a.material_id_enter, 
                &ray.direction)
            );
            hits.push(make_csg_hit(
                tB_enter, 
                i_b.point_enter, 
                -i_b.normal_enter, 
                i_b.uv_enter, 
                i_b.material_id_enter, 
                &ray.direction)
            );
        }
        // Case 5: B is inside A
        else if tB_enter > tA_enter && tB_exit < tA_exit {
            // Remaining intervals are [tA_enter, tB_enter] and [tB_exit, tA_exit]
            hits.push(make_csg_hit(
                tA_enter, 
                interval_a.point_enter, 
                interval_a.normal_enter, 
                interval_a.uv_enter, 
                interval_a.material_id_enter, 
                &ray.direction)
            );
            hits.push(make_csg_hit(
                tB_enter, 
                i_b.point_enter, 
                -i_b.normal_enter, 
                i_b.uv_enter, 
                i_b.material_id_enter, 
                &ray.direction)
            );
            hits.push(make_csg_hit(
                tB_exit, 
                i_b.point_exit, 
                -i_b.normal_exit, 
                i_b.uv_exit, 
                i_b.material_id_exit, 
                &ray.direction)
            );
            hits.push(make_csg_hit(
                tA_exit, 
                interval_a.point_exit, 
                interval_a.normal_exit, 
                interval_a.uv_exit, 
                interval_a.material_id_exit, 
                &ray.direction)
            );
        }
    } else {
        // B does not hit at all, just return A's interval
        hits.push(make_csg_hit(
            tA_enter, 
            interval_a.point_enter, 
            interval_a.normal_enter, 
            interval_a.uv_enter, 
            interval_a.material_id_enter, 
            &ray.direction)
        );
        hits.push(make_csg_hit(
            tA_exit, 
            interval_a.point_exit, 
            interval_a.normal_exit, 
            interval_a.uv_exit, 
            interval_a.material_id_exit, 
            &ray.direction)
        );
    }
}

fn intersect(
    camera: &Camera, 
    ray: &Ray, 
    scene: &Scene,
    depth: u8
) -> Vector3<f32> {

    fn visit(node: &Node, ray: &Ray, hits: &mut Vec<HitRecord>, model_cache: &ModelCache) {
        if node.get_node_type() == &NodeType::CsgDifference {
            intersect_csg_difference(node, ray, hits);
            return;
        }
        // println!("{:?}", node.get_transform_world());
        
        // Transform ray to local coordinate space
        let inv = node.get_transform_inverse();

        let n_ray = Ray {
            direction: inv.transform_vector(&ray.direction).normalize(),
            origin: inv
                .transform_point(&Point3::from(ray.origin))
                .coords,
        };


        let mesh_id = node.get_mesh_id().as_deref();
        let res = if mesh_id == Some("sphere") {
            intersect_unit_sphere(n_ray.origin, n_ray.direction)
        } else if mesh_id == Some("cone") {
            intersect_unit_cone(n_ray.origin, n_ray.direction)
        } else if mesh_id == Some("cylinder") {
            intersect_unit_cylinder(n_ray.origin, n_ray.direction)
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
                node.get_transform_world()
                * node.get_transform_local()
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
                front_face: front_face,
                uv: r.uv
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
            origin: hit.point + hit.normal * scene.environment.secondary_ray_eps
        };
        let mut visibility = 1.0;

        if scene.environment.shadows == true {
            let mut shadow_hits:Vec<HitRecord> = vec![];
            visit(scene.get_root(), &shadow_ray, &mut shadow_hits, &scene.model_cache);

            // Filter out self-intersections/negative t, and sort by t
            shadow_hits.retain(|h| h.t > 0.001);
            shadow_hits.sort_by(|a, b| a.t.partial_cmp(&b.t).unwrap());

            let dist_to_light = to_light.norm();

            for shadow_hit in shadow_hits {
                if shadow_hit.t > dist_to_light {
                    break; // hit is behind the light (all subsequent hits will also be behind since they are sorted)
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
        }

        // Light can reach, get material and compute diffuce and specular
        let material = scene.get_materials().get(hit.material_id as usize).unwrap();

        let to_light = light.position - hit.point;
        let distance = to_light.norm();
        let light_dir = to_light / distance;

        // Used to be 1.0, just making things look nice
        let attenuation = 1.0 / (distance * distance);
        let ndotl = hit.normal.dot(&light_dir).max(0.0);

        if material.texture.is_some() {
            let key = material.texture.as_ref().unwrap();
            let tex = &scene.texture_cache[key];

            let pixel = sample_texture(
                &tex, 
                hit.uv.x, 
                hit.uv.y
            );
            let texture_contrib = Vector3::new(
                pixel[0] as f32 / 255.0,
                pixel[1] as f32 / 255.0,
                pixel[2] as f32 / 255.0,
            );

            contribution +=
                texture_contrib.component_mul(
                    &light.intensity
                )   
                * attenuation
                * ndotl
                * visibility;
        } else {
            contribution +=
                material.albedo.unwrap().component_mul(
                    &light.intensity
                )   
                * attenuation
                * ndotl
                * visibility;
        }

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


    let local_contrib = contribution + specular + scene.environment.ambient_light;

    let mut reflect_contrib: Vector3<f32> = Vector3::zeros();
    let mut refract_contrib: Vector3<f32> = Vector3::zeros();

    if reflectivity > 0.0 && scene.environment.reflections == true {
        let reflect_direction = (
            ray.direction - 2.0 * ray.direction.dot(&hit.normal) * hit.normal
        ).normalize();

        let reflect_ray = Ray {
            direction: reflect_direction,
            origin: hit.point + scene.environment.secondary_ray_eps * hit.normal
        };

        if depth < 4 {
            reflect_contrib = intersect(&camera, &reflect_ray, &scene, depth+1);
        }
    }

    // Calculate refraction
    if transparency > 0.0 && scene.environment.refractions == true {
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
                origin: hit.point + offset * scene.environment.secondary_ray_eps
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
    println!("");


    println!("Loading textures.....");
    for entry in fs::read_dir("./textures").unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if let Some(name) = path.file_stem().and_then(|s| s.to_str()) {
            println!("{} name={}", path.to_str().unwrap(), name);
            load_texture(
                &mut scene.texture_cache,
                name,
                path.to_str().unwrap()
            ).unwrap();
        }
    }
    println!("Done loading textures");


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


