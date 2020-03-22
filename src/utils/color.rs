use log::warn;

#[derive(Debug, Default, Clone)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    pub fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Color { r, g, b, a }
    }

    pub fn from_str(color: &str) -> Self {
        return match color {
            "r"     => Color::new(1., 0., 0., 1.),
            "g"     => Color::new(0., 1., 0., 1.),
            "b"     => Color::new(0., 0., 1., 1.),
            "y"     => Color::new(1., 1., 0., 1.),
            "m"     => Color::new(1., 0., 1., 1.),
            "c"     => Color::new(0., 1., 1., 1.),
            "w"     => Color::new(1., 1., 1., 1.),
            "k"     => Color::new(0., 0., 0., 1.),
            "red"   => Color::new(0.6350, 0.0780, 0.1840, 1.0000),
            "green" => Color::new(0.4660, 0.6740, 0.1880, 1.0000),
            "blue"  => Color::new(0.0000, 0.4470, 0.7410, 1.0000),
            _ => {
                warn!("Color short name {} not supported. Use default color.", color);
                Color::new(1., 0., 0., 1.)
            }
        };
    }
}