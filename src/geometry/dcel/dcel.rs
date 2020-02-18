use core::ptr::NonNull;
use std::collections::LinkedList;
use crate::geometry::*;

/// Why implement DCEL in rust is tricky?
///
/// Because of Rust’s affine type system / ownership, it’s actually tricky to implement
/// DCEL. The main reason is it seems a edge needs to have two owners from adjacent edge
/// and vertex. However, that’s possible with NonNull<T>.

pub struct DoubleEdgeList<V, E, F> {
    pub vertices: LinkedList<DoubleEdgeListVertex<V, E, F>>,
    pub edges: LinkedList<DoubleEdgeListHalfEdge<V, E, F>>,
    pub faces: LinkedList<DoubleEdgeListFace<V, E, F>>,
}

impl<V, E, F> DoubleEdgeList<V, E, F> {
    pub fn new() -> Self {
        DoubleEdgeList{
            vertices: Default::default(),
            edges: Default::default(),
            faces: Default::default(),
        }
    }
}
