use nalgebra::Vector3;
use nalgebra::Vector2;
use nalgebra::{DMatrix, linalg::Schur};


use crate::obj::{ObjModel, Triangle};


pub struct HitRecord {
    pub t: f32,
    pub point: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub material_id: u32,
    pub front_face: bool,
    pub uv: Vector2<f32>
}


pub struct IntersectResult {
    pub t: f32,
    pub hit_point: Vector3<f32>,
    pub normal: Vector3<f32>,
    pub front_face: bool,
    pub uv: Vector2<f32>,
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
        let (u, v) = sphere_uv(hitpoint);

        return Some(IntersectResult {
            t: t0,
            hit_point: hitpoint,
            normal: normal,
            front_face: dir.dot(&normal) < 0.0,
            uv: Vector2::new(u, v)
        });
    } else if t1 > 0.0 {
        let hitpoint = origin + dir * t1;
        let normal = hitpoint.normalize();
        let (u, v) = sphere_uv(hitpoint);

        return Some(IntersectResult {
            t: t1,
            hit_point: hitpoint,
            normal: normal,
            front_face: dir.dot(&normal) < 0.0,
            uv: Vector2::new(u, v)
        });
    } else {
        None
    }
}



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


            let theta = hit.z.atan2(hit.x);
            let u = 0.5 + theta / (2.0 * std::f32::consts::PI);
            let v = hit.y;


            let result = IntersectResult {
                t,
                hit_point: hit,
                normal,
                front_face,
                uv: Vector2::new(u, v)
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

                let u = hit.x * 0.5 + 0.5;
                let v = hit.z * 0.5 + 0.5;


                let result = IntersectResult {
                    t,
                    hit_point: hit,
                    normal,
                    front_face,
                    uv: Vector2::new(u, v)
                };

                if closest.is_none() || t < closest.as_ref().unwrap().t {
                    closest = Some(result);
                }
            }
        }
    }

    closest
}


pub fn intersect_unit_cylinder(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectResult> {
    let mut closest: Option<IntersectResult> = None;

    // ---- Cylinder side ----
    //
    // x^2 + z^2 = 1
    //
    let a = dir.x * dir.x + dir.z * dir.z;

    let b = 2.0 * (
        origin.x * dir.x +
        origin.z * dir.z
    );

    let c = origin.x * origin.x +
        origin.z * origin.z -
        1.0;

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

            // Finite cylinder bounds
            if hit.y < -1.0 || hit.y > 1.0 {
                continue;
            }

            let mut normal = Vector3::new(
                hit.x,
                0.0,
                hit.z,
            )
            .normalize();

            let front_face = dir.dot(&normal) < 0.0;

            if !front_face {
                normal = -normal;
            }

            let theta = hit.z.atan2(hit.x);
            let u = 0.5 + theta / (2.0 * std::f32::consts::PI);
            let v = (hit.y + 1.0) * 0.5;

            let result = IntersectResult {
                t,
                hit_point: hit,
                normal,
                front_face,
                uv: Vector2::new(u, v),
            };

            if closest.is_none() || t < closest.as_ref().unwrap().t {
                closest = Some(result);
            }
        }
    }

    // ---- Bottom disk (y = -1) ----
    if dir.y.abs() > 1e-6 {
        let t = (-1.0 - origin.y) / dir.y;

        if t > 0.0 {
            let hit = origin + dir * t;

            if hit.x * hit.x + hit.z * hit.z <= 1.0 {
                let mut normal = Vector3::new(0.0, -1.0, 0.0);

                let front_face = dir.dot(&normal) < 0.0;

                if !front_face {
                    normal = -normal;
                }

                let u = hit.x * 0.5 + 0.5;
                let v = hit.z * 0.5 + 0.5;

                let result = IntersectResult {
                    t,
                    hit_point: hit,
                    normal,
                    front_face,
                    uv: Vector2::new(u, v),
                };

                if closest.is_none() || t < closest.as_ref().unwrap().t {
                    closest = Some(result);
                }
            }
        }
    }

    // ---- Top disk (y = 1) ----
    if dir.y.abs() > 1e-6 {
        let t = (1.0 - origin.y) / dir.y;

        if t > 0.0 {
            let hit = origin + dir * t;

            if hit.x * hit.x + hit.z * hit.z <= 1.0 {
                let mut normal = Vector3::new(0.0, 1.0, 0.0);

                let front_face = dir.dot(&normal) < 0.0;

                if !front_face {
                    normal = -normal;
                }

                let u = hit.x * 0.5 + 0.5;
                let v = hit.z * 0.5 + 0.5;

                let result = IntersectResult {
                    t,
                    hit_point: hit,
                    normal,
                    front_face,
                    uv: Vector2::new(u, v),
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

    // let bounds_min = Vector3::new(-0.5, -0.5, -0.5);
    // let bounds_max = Vector3::new( 0.5,  0.5,  0.5);

    let bounds_min = Vector3::new(-1.0, -1.0, -1.0);
    let bounds_max = Vector3::new( 1.0,  1.0,  1.0);


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

    let (u, v) = cube_uv(hit_point, normal);

    Some(IntersectResult {
        t,
        hit_point,
        normal,
        front_face,
        uv: Vector2::new(u, v)
    })
}


const EPSILON: f32 = 1e-8;
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
        uv: Vector2::new(0.0, 0.0) // FIXME: Need to come from obj file
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


// use roots::{find_roots_quartic, Roots};
// fn solve_quartic_old(
//     a: f64,
//     b: f64,
//     c: f64,
//     d: f64,
//     e: f64,
// ) -> Vec<f64> {
//     match find_roots_quartic(a, b, c, d, e) {
//         Roots::No(_) => vec![],
//         Roots::One([x]) => vec![x],
//         Roots::Two([x0, x1]) => vec![x0, x1],
//         Roots::Three([x0, x1, x2]) => vec![x0, x1, x2],
//         Roots::Four([x0, x1, x2, x3]) => vec![x0, x1, x2, x3],
//     }
// }


pub fn solve_quartic(
    a: f64,
    b: f64,
    c: f64,
    d: f64,
    e: f64,
) -> Vec<f64> {
    if a.abs() < 1e-14 {
        return vec![];
    }

    // Normalize polynomial:
    // x^4 + Bx^3 + Cx^2 + Dx + E
    let b = b / a;
    let c = c / a;
    let d = d / a;
    let e = e / a;

    // Companion matrix:
    //
    // [ 0  0  0 -E ]
    // [ 1  0  0 -D ]
    // [ 0  1  0 -C ]
    // [ 0  0  1 -B ]
    //
    let mut m = DMatrix::<f64>::zeros(4, 4);

    m[(0, 3)] = -e;
    m[(1, 3)] = -d;
    m[(2, 3)] = -c;
    m[(3, 3)] = -b;

    m[(1, 0)] = 1.0;
    m[(2, 1)] = 1.0;
    m[(3, 2)] = 1.0;

    let schur = Schur::new(m);
    let eigenvalues = schur.complex_eigenvalues();

    eigenvalues
        .iter()
        .filter_map(|z| {
            if z.im.abs() < 1e-10 {
                Some(z.re)
            } else {
                None
            }
        })
        .collect()
}


pub fn intersect_unit_torus(
    origin: Vector3<f32>,
    dir: Vector3<f32>,
) -> Option<IntersectResult> {
    const MAJOR_R: f64 = 0.75;
    const MINOR_R: f64 = 0.25;
    const EPS: f64 = 1e-5;

    // Convert once to f64.
    let origin64 = origin.map(|x| x as f64);
    let dir64 = dir.map(|x| x as f64);

    let g = dir64.dot(&dir64);
    let h = 2.0 * origin64.dot(&dir64);
    let i = origin64.dot(&origin64);

    let j = dir64.x * dir64.x + dir64.y * dir64.y;
    let k = 2.0 * (origin64.x * dir64.x + origin64.y * dir64.y);
    let l = origin64.x * origin64.x + origin64.y * origin64.y;

    let s = MAJOR_R * MAJOR_R - MINOR_R * MINOR_R;

    let c4 = g * g;
    let c3 = 2.0 * g * h;
    let c2 = h * h + 2.0 * g * (i + s) - 4.0 * MAJOR_R * MAJOR_R * j;
    let c1 = 2.0 * h * (i + s) - 4.0 * MAJOR_R * MAJOR_R * k;
    let c0 = (i + s) * (i + s) - 4.0 * MAJOR_R * MAJOR_R * l;

    let roots = solve_quartic(c4, c3, c2, c1, c0);

    let best_t = roots
        .into_iter()
        .filter(|t| t.is_finite() && *t > EPS)
        .min_by(|a, b| a.partial_cmp(b).unwrap())?;

    let hit64 = origin64 + dir64 * best_t;

    let sum = hit64.dot(&hit64) + MAJOR_R * MAJOR_R - MINOR_R * MINOR_R;

    let normal64 = Vector3::new(
        4.0 * hit64.x * (sum - 2.0 * MAJOR_R * MAJOR_R),
        4.0 * hit64.y * (sum - 2.0 * MAJOR_R * MAJOR_R),
        4.0 * hit64.z * sum,
    );

    if normal64.norm_squared() < 1e-20 {
        return None;
    }

    let normal64 = normal64.normalize();

    // Convert back to f32 for the renderer.
    let hit = hit64.map(|x| x as f32);
    let normal = normal64.map(|x| x as f32);

    let (u, v) = torus_uv(hit, MAJOR_R as f32);

    Some(IntersectResult {
        t: best_t as f32,
        hit_point: hit,
        normal,
        front_face: dir.dot(&normal) < 0.0,
        uv: Vector2::new(u, v),
    })
}


use std::f32::consts::PI;
pub fn sphere_uv(p: Vector3<f32>) -> (f32, f32) {
    let theta = p.z.atan2(p.x); // longitude [-pi, pi]
    let phi = p.y.asin();       // latitude [-pi/2, pi/2]

    let u = 1.0 - (theta + PI) / (2.0 * PI);
    let v = 0.5 - phi / PI;
    (u, v)
}

pub fn cube_uv(hit: Vector3<f32>, normal: Vector3<f32>) -> (f32, f32) {
    let (u, v) = if normal.x > 0.5 {
        // +X
        (
            (hit.z + 1.0) * 0.5,
            (hit.y + 1.0) * 0.5,
        )
    } else if normal.x < -0.5 {
        // -X
        (
            (1.0 - hit.z) * 0.5,
            (hit.y + 1.0) * 0.5,
        )
    } else if normal.y > 0.5 {
        // +Y
        (
            (hit.x + 1.0) * 0.5,
            (1.0 - hit.z) * 0.5,
        )
    } else if normal.y < -0.5 {
        // -Y
        (
            (hit.x + 1.0) * 0.5,
            (hit.z + 1.0) * 0.5,
        )
    } else if normal.z > 0.5 {
        // +Z
        (
            (hit.x + 1.0) * 0.5,
            (hit.y + 1.0) * 0.5,
        )
    } else {
        // -Z
        (
            (1.0 - hit.x) * 0.5,
            (hit.y + 1.0) * 0.5,
        )
    };

    (u, v)
}



#[allow(non_snake_case)]
pub fn torus_uv(hit: Vector3<f32>, R: f32) -> (f32, f32) {
    let radial = (hit.x * hit.x + hit.y * hit.y).sqrt();

    let u_angle = hit.y.atan2(hit.x);
    let v_angle = hit.z.atan2(radial - R);

    let u = (u_angle + PI) / (2.0 * PI);
    let v = (v_angle + PI) / (2.0 * PI);

    (u, v)
}


