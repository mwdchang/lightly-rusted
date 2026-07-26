use std::collections::HashMap;
use image::{DynamicImage, GenericImageView, Rgba};
use image::ImageReader;


pub type TextureCache = HashMap<String, DynamicImage>;


pub fn load_texture<'a>(
    cache: &'a mut TextureCache,
    key: &str,
    path: &str,
) -> image::ImageResult<&'a DynamicImage> {
    use std::collections::hash_map::Entry;

    match cache.entry(key.to_string()) {
        Entry::Occupied(e) => Ok(e.into_mut()),
        Entry::Vacant(e) => {
            let img = ImageReader::open(path)?.decode()?;
            Ok(e.insert(img))
        }
    }
}


pub fn sample_texture(
    texture: &DynamicImage,
    mut u: f32,
    mut v: f32,
) -> Rgba<u8> {
    // Wrap horizontally, clamp vertically.
    u = u.rem_euclid(1.0);
    v = v.clamp(0.0, 1.0);

    let (w, h) = texture.dimensions();

    let x = (u * (w - 1) as f32) as u32;
    let y = (v * (h - 1) as f32) as u32;

    texture.get_pixel(x, y)
}
