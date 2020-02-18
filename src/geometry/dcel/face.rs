use crate::geometry::*;
use std::ptr::NonNull;

pub struct DoubleEdgeListFace<V, E, F> {
    pub element: F,
    pub edge: *mut DoubleEdgeListHalfEdge<V, E, F>,
}