use core::ptr::NonNull;
use crate::geometry::*;

pub struct DoubleEdgeList<V, E, F> {
    pub vertices: Vec<DcelVertexPtr<V, E, F>>,
    pub edges: Vec<DcelHalfEdgePtr<V, E, F>>,
    pub faces: Vec<DcelFacePtr<V, E, F>>,
}

impl<V, E, F> DoubleEdgeList<V, E, F>
    where V: Copy, E: Copy, {
    pub fn new() -> Self {
        DoubleEdgeList{
            vertices: Default::default(),
            edges: Default::default(),
            faces: Default::default(),
        }
    }

    pub fn initialize(&mut self, vertex_element: Vec<V>, edge_element: Vec<E>, face_element: F) {
        let face = DcelFacePtr::new(face_element);
        self.faces.push(face);

        let mut prev_left_edge  = DcelHalfEdgePtr::null();
        let mut prev_right_edge = DcelHalfEdgePtr::null();

        for i in 0..vertex_element.len() {
            let vertex = DcelVertexPtr::new(vertex_element[i]);
            let mut left   = DcelHalfEdgePtr::new(edge_element[2 * i]);
            let mut right  = DcelHalfEdgePtr::new(edge_element[2 * i + 1]);

            left.set_face(face);
            left.set_next(DcelHalfEdgePtr::null());
            left.set_origin(vertex);
            left.set_twin(right);

            right.set_face(DcelFacePtr::null());
            right.set_next(prev_right_edge);
            right.set_origin(DcelVertexPtr::null());
            right.set_twin(left);

            self.edges.push(left);
            self.edges.push(right);

            vertex.set_leaving(left);
            self.vertices.push(vertex);

            if ! prev_left_edge.is_null() {
                prev_left_edge.set_next(left);
            }

            if ! prev_right_edge.is_null() {
                prev_right_edge.set_origin(vertex);
            }

            prev_left_edge = left;
            prev_right_edge = right;
        }

        let first_left_edge = self.edges[0];
        prev_left_edge.set_next(first_left_edge);
        let mut first_right_edge = self.edges[1];
        first_right_edge.set_next(prev_right_edge);

        prev_right_edge.set_origin(self.vertices[0]);

        face.set_edge(first_left_edge);
    }
}
