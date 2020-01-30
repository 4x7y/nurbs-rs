use core::ptr::NonNull;
use std::collections::LinkedList;

pub struct DoubleEdgeListVertex<V, E, F> {
    pub element: V,
    pub leaving: Option<NonNull<DoubleEdgeListHalfEdge<V, E, F>>>,
}

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

pub struct DoubleEdgeListFace<V, E, F> {
    pub element: F,
    pub edge: Option<NonNull<DoubleEdgeListHalfEdge<V, E, F>>>,
}

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

//impl<V, E> DoubleEdgeListVertex<V, E> {
//
//    pub fn new(element: V, leaving: HalfEdgePtr<V, E>) -> Self {
//        DoubleEdgeListVertex { element, leaving, }
//    }
//
//    pub fn into_element(self: Box<Self>) -> V {
//        self.element
//    }
//
//    pub fn get_edge_to(&self, vertex: VertexPtr<V, E>) -> Option<Box<DoubleEdgeListHalfEdge<V, E>>> {
//        if !self.leaving.is_none() {
//            return None;
//        }
//
//        match self.leaving {
//            None => return Box
//            Some(leaving) => {
//                if leaving.next.twin == vertex {
//                    return leaving;
//                } else {
//                    let mut edge = self.leaving.twin.next;
//                    while edge != self.leaving {
//                        if edge.twin.origin == vertex {
//                            return edge;
//                        } else {
//                            edge = edge.twin.next;
//                        }
//                    }
//                }
//            }
//        }
//    }
//}