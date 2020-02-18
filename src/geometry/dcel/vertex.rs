use crate::geometry::*;
use std::ptr::NonNull;

pub struct DoubleEdgeListVertex<V, E, F> {
    pub element: V,
    pub leaving: Option<NonNull<DoubleEdgeListHalfEdge<V, E, F>>>,
}