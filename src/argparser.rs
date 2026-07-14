use std::process;

pub struct Args {
    pub scene_file: String,
    pub width: u32,
    pub height: u32,
}

impl Args {
    // Usage: lightly-rusted [--scene <file>] [--size <width>x<height>]
    pub fn parse() -> Self {
        let mut scene_file = String::from("scene01.json");
        let mut width: u32 = 400;
        let mut height: u32 = 300;

        let args: Vec<String> = std::env::args().collect();
        let mut i = 1;
        while i < args.len() {
            match args[i].as_str() {
                "--scene" => {
                    i += 1;
                    if i >= args.len() {
                        eprintln!("Error: --scene requires a file argument");
                        process::exit(1);
                    }
                    scene_file = args[i].clone();
                }
                "--size" => {
                    i += 1;
                    if i >= args.len() {
                        eprintln!("Error: --size requires a WIDTHxHEIGHT argument (e.g. 800x600)");
                        process::exit(1);
                    }
                    let parts: Vec<&str> = args[i].splitn(2, 'x').collect();
                    if parts.len() != 2 {
                        eprintln!("Error: --size format must be WIDTHxHEIGHT (e.g. 800x600)");
                        process::exit(1);
                    }
                    width = parts[0].parse().unwrap_or_else(|_| {
                        eprintln!("Error: invalid width '{}'", parts[0]);
                        process::exit(1);
                    });
                    height = parts[1].parse().unwrap_or_else(|_| {
                        eprintln!("Error: invalid height '{}'", parts[1]);
                        process::exit(1);
                    });
                }
                unknown => {
                    eprintln!("Error: unknown argument '{}'", unknown);
                    eprintln!("Usage: lightly-rusted [--scene <file>] [--size <width>x<height>]");
                    process::exit(1);
                }
            }
            i += 1;
        }

        Args { scene_file, width, height }
    }
}

