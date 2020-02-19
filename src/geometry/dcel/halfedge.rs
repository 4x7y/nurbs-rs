use crate::geometry::*;
use std::ptr;

pub struct DoubleEdgeListHalfEdge<V, E, F> {
    pub element: E,

    pub origin: DcelVertexPtr<V, E, F>,
    // The incident half edge in the list having the same face
    pub next: DcelHalfEdgePtr<V, E, F>,
    pub twin: DcelHalfEdgePtr<V, E, F>,

    // The incident face of this half edge
    pub face: DcelFacePtr<V, E, F>,

    pub visited: bool,
}

pub struct DcelHalfEdgePtr<V, E, F> {
    pub ptr: *mut DoubleEdgeListHalfEdge<V, E, F>,
}

impl<V, E, F> DcelHalfEdgePtr<V, E, F> {

    pub fn new(element: E) -> Self {
        let edge = DoubleEdgeListHalfEdge {
            element,
            origin: DcelVertexPtr::null(),
            next: DcelHalfEdgePtr::null(),
            twin: DcelHalfEdgePtr::null(),
            face: DcelFacePtr::null(),
            visited: false,
        };
        DcelHalfEdgePtr {
            ptr: Box::into_raw(Box::new(edge)),
        }
    }

    #[inline]
    pub fn origin(&self) -> DcelVertexPtr<V, E, F> {
        if self.is_null() {
            return DcelVertexPtr::null();
        }
        unsafe {
            (*self.ptr).origin.clone()
        }
    }

    #[inline]
    pub fn next(&self) -> DcelHalfEdgePtr<V, E, F> {
        if self.is_null() {
            return DcelHalfEdgePtr::null();
        }
        unsafe {
            (*self.ptr).next.clone()
        }
    }

    #[inline]
    pub fn twin(&self) -> DcelHalfEdgePtr<V, E, F> {
        if self.is_null() {
            return DcelHalfEdgePtr::null();
        }
        unsafe {
            (*self.ptr).twin.clone()
        }
    }

    #[inline]
    pub fn face(&self) -> DcelFacePtr<V, E, F> {
        if self.is_null() {
            return DcelFacePtr::null();
        }
        unsafe {
            (*self.ptr).face.clone()
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
    fn clone(&self) -> Self {
        DcelHalfEdgePtr {
            ptr: self.ptr,
        }
    }
}

impl<V,E,F> Copy for DcelHalfEdgePtr<V,E,F> { }

impl<V,E,F> PartialEq for DcelHalfEdgePtr<V,E,F> {
    fn eq(&self, other: &DcelHalfEdgePtr<V,E,F>) -> bool {
        self.ptr == other.ptr
    }
}

impl<V,E,F> DcelHalfEdgePtr<V,E,F> {

    /// Get destination vertex
    pub fn get_destination(&self) -> DcelVertexPtr<V,E,F> {
        self.next().origin()
    }

    /// Get previous vertex
    pub fn get_previous(&self) -> DcelHalfEdgePtr<V,E,F> {
        let mut edge = self.twin().next().twin();
        // walk around the face
        while *self != edge.next() {
            edge = edge.next().twin();
        }
        return edge;
    }

    /// Get adjacent face
    pub fn get_face(&self) -> DcelFacePtr<V,E,F> {
        self.face()
    }

    /// Set adjacent face
    pub fn set_face(&mut self, face: DcelFacePtr<V, E, F>) {
        if self.is_null() {
            return;
        }
        unsafe {
            (*self.ptr).face = face;
        }
    }

    /// Set next edge
    pub fn set_next(&mut self, next: DcelHalfEdgePtr<V, E, F>) {
        if self.is_null() {
            return;
        }
        unsafe {
            (*self.ptr).next = next;
        }
    }

    /// Set origin vertex
    pub fn set_origin(&mut self, origin: DcelVertexPtr<V, E, F>) {
        if self.is_null() {
            return;
        }
        unsafe {
            (*self.ptr).origin = origin;
        }
    }

    /// Set twin edge
    pub fn set_twin(&mut self, twin: DcelHalfEdgePtr<V, E, F>) {
        if self.is_null() {
            return;
        }
        unsafe {
            (*self.ptr).twin = twin;
        }
    }
}
