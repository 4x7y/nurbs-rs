use crate::geometry::*;
use std::ptr::NonNull;

/// Why not Option<Box<Node<T>>> ?
///
/// It’s probably a good idea to see what would be the difference if we use
/// Box<Node<T>> instead. We discussed what Unique<T> is and how it’s different
/// from NonNull<T> previously, but as a quick reminder, Unique<T> owns the
/// referent whereas NonNull<T> does not and in fact Box<T> (a pointer type for
/// heap allocation) just wraps Unique<T> and provides new interface for
/// interacting with Unique<T>.

pub struct DoubleEdgeListHalfEdge<V, E, F> {
    pub element: E,
    
    pub origin: Option<NonNull<DoubleEdgeListVertex<V, E, F>>>,
    // The incident half edge in the list having the same face
    pub next: Option<NonNull<DoubleEdgeListHalfEdge<V, E, F>>>,
    pub twin: Option<NonNull<DoubleEdgeListHalfEdge<V, E, F>>>,

    // The incident face of this half edge
    pub face: Option<NonNull<DoubleEdgeListFace<V, E, F>>>,

    pub visited: bool,
}