# lightly-rusted
lightly-rusted is a small raytracer that leverages a scene graph in JSON format.

lightly-rusted is written in Rust as demo project for me to learn the language. The code should be pretty easy to understand (though far from optimized) for people that want to start exploring raytracing, the main gist resides in the `intersect` function.


## Running 
This will create a `render-result.png`.
```
cargo run
```


## Scene graph spec
```json
{
  description: <str>,
  point_lights: [
    {
      positio: <vec3<float>>,
      intensity: <vec3<float>>
    },
    ...
  ],
  materials: [
    {
      albedo: <vec3<float>>
      shine: <float>
    },
    ...
  ],
  root: <Node> 
}

```

Where a `Node` is
```json
{
    mesh_id: <str | undefined>,
    material_id: <uint | undefined>,
    transforms: <Transform[] | undefiend>
    children: <Node[] | undefined>
  }
```

Where a `Transform` is one of
- translate: `{ type: translate, value: <vec3<float>>}`
- rotate_x: `{ type: rotate_x, value: <float>}`
- rotate_y: `{ type: rotate_y, value: <float>}`
- rotate_z: `{ type: rotate_z, value: <float>}`
- scale: `{ type: scale, value: <vec3<float>>}`

The transforms are specified in order you want to apply them, so if `transforms = [t1, t2, t3]` the local transformation matrix is `t3 * t2 * t1 * I`
