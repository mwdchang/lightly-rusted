use nalgebra::Vector3;

pub struct HitRecord {
    pub t: f32,
    pub point: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub material_id: u32,
    pub front_face: bool,
}


pub struct IntersectResult {
    pub t: f32,
    pub hit_point: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub front_face: bool,
}

/**
 * Checks sphere collision at (0, 0, 0)
 *
 * origin: origin of the ray
 * dir: direction of the ray
**/
pub fn intersect_unit_sphere(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectResult> {
    let a = dir.dot(&dir);
    let b = 2.0 * origin.dot(&dir);
    let c = origin.dot(&origin) - 1.0;

    let disc = b * b - 4.0 * a * c;

    if disc < 0.0 {
        return None;
    }

    let sqrt_disc = disc.sqrt();

    let t0 = (-b - sqrt_disc) / (2.0 * a);
    let t1 = (-b + sqrt_disc) / (2.0 * a);

    if t0 > 0.0 {
        let hitpoint = origin + dir * t0;
        let normal = hitpoint.normalize();
        return Some(IntersectResult {
            t: t0,
            hit_point: hitpoint,
            normal: normal,
            front_face: dir.dot(&normal) < 0.0,
        });
    } else if t1 > 0.0 {
        let hitpoint = origin + dir * t1;
        let normal = hitpoint.normalize();
        return Some(IntersectResult {
            t: t1,
            hit_point: hitpoint,
            normal: normal,
            front_face: dir.dot(&normal) < 0.0,
        });
    } else {
        None
    }
}



pub fn intersect_unit_cone(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<f32> {
    let mut hits = Vec::with_capacity(3);

    let eps = 1e-4;

    // --- Cone side ---
    let a = dir.x * dir.x
          + dir.z * dir.z
          - dir.y * dir.y;

    let b = 2.0 * (
        origin.x * dir.x +
        origin.z * dir.z -
        origin.y * dir.y
    );

    let c = origin.x * origin.x
          + origin.z * origin.z
          - origin.y * origin.y;

    if a.abs() > eps {
        let disc = b * b - 4.0 * a * c;

        if disc >= 0.0 {
            let sqrt_disc = disc.sqrt();

            hits.push((-b - sqrt_disc) / (2.0 * a));
            hits.push((-b + sqrt_disc) / (2.0 * a));
        }
    }

    // Keep only points on finite cone side
    hits.retain(|t| {
        if *t <= eps {
            return false;
        }

        let p = origin + dir * *t;

        p.y >= 0.0 &&
        p.y <= 1.0 &&
        p.x * p.x + p.z * p.z <= p.y * p.y + eps
    });


    // --- Base cap at y = 1 ---
    if dir.y.abs() > eps {
        let t = (1.0 - origin.y) / dir.y;

        if t > eps {
            let p = origin + dir * t;

            if p.x * p.x + p.z * p.z <= 1.0 {
                hits.push(t);
            }
        }
    }

    // Closest hit wins
    hits.into_iter()
        .min_by(|a, b| a.partial_cmp(b).unwrap())
}
