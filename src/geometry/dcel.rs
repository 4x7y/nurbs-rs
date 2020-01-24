use na::{DVector, Vector3};
use core::mem;
use core::ptr::NonNull;
use std::boxed::Box;

pub struct DoubleEdgeListVertex<V, E> {
    pub element: V,
    pub leaving: Option<NonNull<DoubleEdgeListHalfEdge<V, E>>>,
}

pub struct DoubleEdgeListHalfEdge<V, E> {
    pub element: E,
    pub origin: Option<NonNull<DoubleEdgeListVertex<V, E>>>,
    // The incident half edge in the list having the same face
    pub next: Option<NonNull<DoubleEdgeListHalfEdge<V, E>>>,
    pub twin: Option<NonNull<DoubleEdgeListHalfEdge<V, E>>>,

    // The incident face of this half edge
    pub face: Option<NonNull<DoubleEdgeListFace<V, E>>>,

    pub visited: bool,
}

pub struct DoubleEdgeListFace<V, E> {
    pub edge: Option<NonNull<DoubleEdgeListHalfEdge<V, E>>>,
}

impl<V, E> DoubleEdgeListVertex<V, E> {

    pub fn new(element: V, leaving: HalfEdgePtr<V, E>) -> Self {
        DoubleEdgeListVertex { element, leaving, }
    }

    pub fn into_element(self: Box<Self>) -> V {
        self.element
    }

    pub fn get_edge_to(&self, vertex: VertexPtr<V, E>) -> Option<Box<DoubleEdgeListHalfEdge<V, E>>> {
        if !self.leaving.is_none() {
            return None;
        }

        match self.leaving {
            None => return Box
            Some(leaving) => {
                if leaving.next.twin == vertex {
                    return leaving;
                } else {
                    let mut edge = self.leaving.twin.next;
                    while edge != self.leaving {
                        if edge.twin.origin == vertex {
                            return edge;
                        } else {
                            edge = edge.twin.next;
                        }
                    }
                }
            }
        }
    }
}