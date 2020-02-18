use crate::geometry::*;
use std::ptr;

pub struct DoubleEdgeListHalfEdge<V, E, F> {
    pub element: E,

    pub origin: DcelVertexPtr<V, E, F>,
    // The incident half edge in the list having the same face
    pub next: DcelHalfEdgePtr<V, E, F>,
    pub twin: DcelHalfEdgePtr<V, E, F>,

    // The incident face of this half edge
    pub face: *mut DoubleEdgeListFace<V, E, F>,

    pub visited: bool,
}

pub struct DcelHalfEdgePtr<V, E, F> {
    pub ptr: *mut DoubleEdgeListHalfEdge<V, E, F>,
}

impl<V, E, F> DcelHalfEdgePtr<V, E, F> {
    #[inline]
    pub fn origin(&self) -> DcelVertexPtr<V, E, F> {
        if self.is_null() {
            return DcelVertexPtr::null();
        }
        unsafe {
            (*self.ptr).origin.clone()
        }
    }

    pub fn next(&self) -> DcelHalfEdgePtr<V, E, F> {
        if self.is_null() {
            return DcelHalfEdgePtr::null();
        }
        unsafe {
            (*self.ptr).next.clone()
        }
    }

    pub fn twin(&self) -> DcelHalfEdgePtr<V, E, F> {
        if self.is_null() {
            return DcelHalfEdgePtr::null();
        }
        unsafe {
            (*self.ptr).twin.clone()
        }
    }

    #[inline]
    pub fn null() -> DcelHalfEdgePtr<V, E, F> {
        DcelHalfEdgePtr {
            ptr: ptr::null_mut(),
        }
    }

    #[inline]
    pub fn is_null(&self) -> bool {
        self.ptr.is_null()
    }
}

impl<V,E,F> Clone for DcelHalfEdgePtr<V,E,F> {
    fn clone(&self) -> DcelHalfEdgePtr<V,E,F> {
        DcelHalfEdgePtr {
            ptr: self.ptr
        }
    }
}

impl<V,E,F> PartialEq for DcelHalfEdgePtr<V,E,F> {
    fn eq(&self, other: &DcelHalfEdgePtr<V,E,F>) -> bool {
        self.ptr == other.ptr
    }
}