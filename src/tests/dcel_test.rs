use crate::geometry::DoubleEdgeList;
use crate::math::Vector2f;

#[test]
fn test_dcel_init() {
    let mut dcel = DoubleEdgeList::<Vector2f, f32, String>::new();

    let vertices = vec![
        Vector2f::new(400., 150.),
        Vector2f::new(350., 350.),
        Vector2f::new(300., 250.),
    ];

    let edges = vec![
        1., -1.,
        2., -2.,
        3., -3.,
    ];

    let face = String::from("Face");

    dcel.initialize(vertices, edges, face);
}