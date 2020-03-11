use crate::geometry::DoubleEdgeList;
use crate::math::Vector2f;

#[test]
fn test_dcel_init() {
    let mut dcel = DoubleEdgeList::<String, f32, String>::new();

    let vertices = vec![
        String::from("v1"),
        String::from("v2"),
        String::from("v3"),
    ];

    let edges = vec![
        1., -1.,
        2., -2.,
        3., -3.,
    ];

    let face = String::from("Face");

    dcel.initialize(&vertices, &edges, &face);

    print!("Hello");
}