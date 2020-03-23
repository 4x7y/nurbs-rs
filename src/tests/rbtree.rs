use crate::robotics::RigidBodyTree;

fn setup() -> RigidBodyTree {
    let file = "resource/sample.urdf";
    RigidBodyTree::from_urdf_file(file).expect("urdf file not found.")
}

fn is_all_same<T: Eq>(slice: &[T]) -> bool {
    slice
        .get(0)
        .map(|first| slice.iter().all(|x| x == first))
        .unwrap_or(true)
}

#[test]
fn test_nums() {
    let rbtree = setup();

    assert_eq!(rbtree.num_joint(), 42);
    assert_eq!(rbtree.num_fixed_body(), 12);
    assert_eq!(rbtree.num_non_fixed_body(), 30);
    assert_eq!(rbtree.num_body(), 42);
    assert_eq!(rbtree.num_dof(), 30);
}

#[test]
fn test_kinematics_tree_path () {
    let rbtree = setup();

    let path = rbtree.kinematics_tree_path("world", "rh_mfdistal");
    let path_true = vec![0, 1, 3, 4, 5, 6, 7, 8, 10, 11, 12, 16, 34, 35, 36];
    assert_eq!(path, path_true);

    let path = rbtree.kinematics_tree_path("rh_mfdistal", "world");
    let path_true = vec![36, 35, 34, 16, 12, 11, 10, 8, 7, 6, 5, 4, 3, 1, 0];
    assert_eq!(path, path_true);

    let path = rbtree.kinematics_tree_path("rh_mfdistal", "rh_rfdistal");
    let path_true = Vec::<usize>::new();
    assert_eq!(path, path_true);

    let path = rbtree.kinematics_tree_path("rh_mfdistal", "rh_mfdistal");
    let path_true = vec![36];
    assert_eq!(path, path_true);

    let path = rbtree.kinematics_tree_path("rh_mfdistal", "rh_palm");
    let path_true = vec![36, 35, 34, 16, 12];
    assert_eq!(path, path_true);

    let path = rbtree.kinematics_tree_path("rh_palm", "rh_mfdistal");
    let path_true = vec![12, 16, 34, 35, 36];
    assert_eq!(path, path_true);
}