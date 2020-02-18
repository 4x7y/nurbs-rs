use crate::geometry::*;
use std::ptr::NonNull;

pub struct DoubleEdgeListFace<V, E, F> {
    pub element: F,
    pub edge: Option<NonNull<DoubleEdgeListHalfEdge<V, E, F>>>,
}