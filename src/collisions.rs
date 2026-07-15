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
