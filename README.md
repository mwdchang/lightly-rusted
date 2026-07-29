# lightly-rusted
lightly-rusted is a small raytracer that leverages a scene graph in JSON format.

lightly-rusted is written in Rust as demo project for me to learn the language. The code should be pretty easy to understand (though far from optimized) for people that want to start exploring raytracing. The main gist resides in the `intersect` function in `src/main.rs` which fires the primary ray.



## Running 
This will create a `render-result.png` file.

Available options are `--workers`, `--scene`, and `--size`.

```
# Use all defaults (scene01.json)
cargo run

# Custom scene only
cargo run -- --scene scene02.json

# Custom size only
cargo run -- --size 1920x1080

# Both scene and size
cargo run -- --scene examples/primitives.json --size 800x600


# Use worker 6 worker threads
cargo run -- --scene examples/primitives.json --workers 6
```

## Features
- Primitive shapes: sphere, cube, cone
- Reflection, refraction, shadows
- Hierarchical scene graph
- Model meshes (via tobj loader)
- Parallel patch rendering (via rayon)
- Texture support


## Scene graph spec
See `schema.json` and example scene `scene01.json`.


## Examples
[Primitives](examples/primitives.json)

<img src="examples/primitives.png" alt="Primitives example" width="50%">


[Texturing](examples/textures.json)

<img src="examples/textures.png" alt="Texturing example" width="50%">

[Obj model and Refraction](examples/obj.json)

<img src="examples/obj.png" alt="Texturing example" width="50%">
