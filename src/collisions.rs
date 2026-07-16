use nalgebra::Vector3;

use crate::obj::{ObjModel, Triangle};

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



/**
 * Cone intersection, generated
**/
/*
pub fn intersect_unit_cone(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectResult> {
    let mut closest: Option<IntersectResult> = None;

    // ---- Cone side ----
    //
    // y^2 + z^2 = x^2
    //
    let a = dir.y * dir.y + dir.z * dir.z - dir.x * dir.x;
    let b = 2.0 * (origin.y * dir.y + origin.z * dir.z - origin.x * dir.x);
    let c = origin.y * origin.y + origin.z * origin.z - origin.x * origin.x;

    let disc = b * b - 4.0 * a * c;

    if disc >= 0.0 && a.abs() > 1e-6 {
        let sqrt_disc = disc.sqrt();

        let t0 = (-b - sqrt_disc) / (2.0 * a);
        let t1 = (-b + sqrt_disc) / (2.0 * a);

        for t in [t0, t1] {
            if t <= 0.0 {
                continue;
            }

            let hit = origin + dir * t;

            // finite cone bounds
            if hit.x < -1.0 || hit.x > 0.0 {
                continue;
            }

            // Gradient of x^2 - y^2 - z^2
            let mut normal = Vector3::new(
                -2.0 * hit.x,
                2.0 * hit.y,
                2.0 * hit.z,
            )
            .normalize();

            let front_face = dir.dot(&normal) < 0.0;

            if !front_face {
                normal = -normal;
            }

            let result = IntersectResult {
                t,
                hit_point: hit,
                normal,
                front_face,
            };

            if closest.is_none() || t < closest.as_ref().unwrap().t {
                closest = Some(result);
            }
        }
    }

    // ---- Base disk ----
    //
    // Plane: x = -1
    //
    if dir.x.abs() > 1e-6 {
        let t = (-1.0 - origin.x) / dir.x;

        if t > 0.0 {
            let hit = origin + dir * t;

            // Disk radius is 1
            if hit.y * hit.y + hit.z * hit.z <= 1.0 {
                let mut normal = Vector3::new(-1.0, 0.0, 0.0);

                let front_face = dir.dot(&normal) < 0.0;

                if !front_face {
                    normal = -normal;
                }

                let result = IntersectResult {
                    t,
                    hit_point: hit,
                    normal,
                    front_face,
                };

                if closest.is_none() || t < closest.as_ref().unwrap().t {
                    closest = Some(result);
                }
            }
        }
    }
    closest
}
*/

pub fn intersect_unit_cone(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectResult> {
    let mut closest: Option<IntersectResult> = None;

    // ---- Cone side ----
    //
    // x^2 + z^2 = (1 - y)^2
    //
    let a = dir.x * dir.x
        + dir.z * dir.z
        - dir.y * dir.y;

    let b = 2.0 * (
        origin.x * dir.x
        + origin.z * dir.z
        + (1.0 - origin.y) * dir.y
    );

    let c = origin.x * origin.x
        + origin.z * origin.z
        - (1.0 - origin.y) * (1.0 - origin.y);

    let disc = b * b - 4.0 * a * c;

    if disc >= 0.0 && a.abs() > 1e-6 {
        let sqrt_disc = disc.sqrt();

        let t0 = (-b - sqrt_disc) / (2.0 * a);
        let t1 = (-b + sqrt_disc) / (2.0 * a);

        for t in [t0, t1] {
            if t <= 0.0 {
                continue;
            }

            let hit = origin + dir * t;

            // finite cone bounds
            if hit.y < 0.0 || hit.y > 1.0 {
                continue;
            }

            // Gradient of x^2 + z^2 - (1-y)^2
            let mut normal = Vector3::new(
                2.0 * hit.x,
                2.0 * (1.0 - hit.y),
                2.0 * hit.z,
            )
            .normalize();

            let front_face = dir.dot(&normal) < 0.0;

            if !front_face {
                normal = -normal;
            }

            let result = IntersectResult {
                t,
                hit_point: hit,
                normal,
                front_face,
            };

            if closest.is_none() || t < closest.as_ref().unwrap().t {
                closest = Some(result);
            }
        }
    }

    // ---- Base disk ----
    //
    // Plane: y = 0
    //
    if dir.y.abs() > 1e-6 {
        let t = -origin.y / dir.y;

        if t > 0.0 {
            let hit = origin + dir * t;

            // Disk radius is 1
            if hit.x * hit.x + hit.z * hit.z <= 1.0 {
                let mut normal = Vector3::new(0.0, -1.0, 0.0);

                let front_face = dir.dot(&normal) < 0.0;

                if !front_face {
                    normal = -normal;
                }

                let result = IntersectResult {
                    t,
                    hit_point: hit,
                    normal,
                    front_face,
                };

                if closest.is_none() || t < closest.as_ref().unwrap().t {
                    closest = Some(result);
                }
            }
        }
    }

    closest
}


/**
 * Generated: slab intersection algorithm
**/
pub fn intersect_unit_cube(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectResult> {
    const EPSILON: f32 = 1e-6;

    let bounds_min = Vector3::new(-0.5, -0.5, -0.5);
    let bounds_max = Vector3::new( 0.5,  0.5,  0.5);

    let mut tmin = f32::NEG_INFINITY;
    let mut tmax = f32::INFINITY;

    // Keep track of which face produced tmin.
    let mut hit_normal = Vector3::zeros();

    for axis in 0..3 {
        let o = origin[axis];
        let d = dir[axis];

        if d.abs() < EPSILON {
            // Ray is parallel to this pair of planes.
            if o < bounds_min[axis] || o > bounds_max[axis] {
                return None;
            }
            continue;
        }

        let inv_d = 1.0 / d;

        let mut t0 = (bounds_min[axis] - o) * inv_d;
        let mut t1 = (bounds_max[axis] - o) * inv_d;

        let mut normal = Vector3::zeros();
        normal[axis] = -1.0;

        if t0 > t1 {
            std::mem::swap(&mut t0, &mut t1);
            normal[axis] = 1.0;
        }

        if t0 > tmin {
            tmin = t0;
            hit_normal = normal;
        }

        tmax = tmax.min(t1);

        if tmin > tmax {
            return None;
        }
    }

    // Choose entry point if outside, exit point if inside.
    let t = if tmin > 0.0 {
        tmin
    } else if tmax > 0.0 {
        tmax
    } else {
        return None;
    };

    let hit_point = origin + dir * t;

    let front_face = dir.dot(&hit_normal) < 0.0;
    let normal = if front_face {
        hit_normal
    } else {
        -hit_normal
    };

    Some(IntersectResult {
        t,
        hit_point,
        normal,
        front_face,
    })
}


const EPSILON: f32 = 1e-7;
fn intersect_triangle(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
    tri: &Triangle,
) -> Option<IntersectResult> {
    let edge1 = tri.p1 - tri.p0;
    let edge2 = tri.p2 - tri.p0;

    let h = dir.cross(&edge2);
    let a = edge1.dot(&h);

    if a.abs() < EPSILON {
        return None;
    }

    let f = 1.0 / a;

    let s = origin - tri.p0;

    let u = f * s.dot(&h);

    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&edge1);

    let v = f * dir.dot(&q);

    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(&q);

    if t <= EPSILON {
        return None;
    }

    let hit_point = origin + dir * t;

    let mut normal = match (&tri.n0, &tri.n1, &tri.n2) {
        (Some(n0), Some(n1), Some(n2)) => {
            let w = 1.0 - u - v;
            (n0 * w + n1 * u + n2 * v).normalize()
        }
        _ => tri.face_normal,
    };

    let front_face = dir.dot(&normal) < 0.0;

    if !front_face {
        normal = -normal;
    }

    Some(IntersectResult {
        t,
        hit_point,
        normal,
        front_face,
    })
}

pub fn intersect_model(
    model: &ObjModel,
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectResult> {
    let mut closest_hit = None;
    let mut closest_t = f32::INFINITY;

    if !model.bounds.intersect(origin, dir) {
        return None;
    }

    for tri in &model.triangles {
        if let Some(hit) = intersect_triangle(origin, dir, tri) {
            if hit.t < closest_t {
                closest_t = hit.t;
                closest_hit = Some(hit);
            }
        }
    }

    closest_hit
}
