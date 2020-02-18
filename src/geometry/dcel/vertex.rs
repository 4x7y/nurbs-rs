use crate::geometry::*;
use std::ptr;

pub struct DoubleEdgeListVertex<V,E,F> {
    pub element: V,
    pub leaving: DcelHalfEdgePtr<V,E,F>,
}

pub struct DcelVertexPtr<V,E,F> {
    pub ptr: *mut DoubleEdgeListVertex<V,E,F>,
}

impl<V,E,F> DcelVertexPtr<V,E,F> {

    #[inline]
    pub fn leaving(&self) -> DcelHalfEdgePtr<V,E,F> {
        if self.is_null() {
            return DcelHalfEdgePtr::null();
        }
        unsafe {
            (*self.ptr).leaving.clone()
        }
    }

    #[inline]
    pub fn null() -> DcelVertexPtr<V,E,F> {
        DcelVertexPtr {
            ptr: ptr::null_mut(),
        }
    }

    #[inline]
    pub fn is_null(&self) -> bool {
        self.ptr.is_null()
    }
}

impl<V,E,F> Clone for DcelVertexPtr<V,E,F> {
    fn clone(&self) -> DcelVertexPtr<V,E,F> {
        DcelVertexPtr {
            ptr: self.ptr
        }
    }
}

impl<V,E,F> PartialEq for DcelVertexPtr<V,E,F> {
    fn eq(&self, other: &DcelVertexPtr<V,E,F>) -> bool {
        self.ptr == other.ptr
    }
}

impl<V,E,F> DoubleEdgeListVertex<V,E,F> {

    /// Create an empty DCEL vertex
    pub fn new(element: V) -> Self {
        DoubleEdgeListVertex {
            element,
            leaving: DcelHalfEdgePtr::null(),
        }
    }

    /// Get edge to a given vertex
    pub unsafe fn get_edge_to(&self, node: DcelVertexPtr<V,E,F>) -> DcelHalfEdgePtr<V,E,F> {
        if ! self.leaving.is_null() {
            if self.leaving.twin().origin() == node {
                return self.leaving.clone();
            } else {
                let mut edge = self.leaving.twin().next();
                while edge != self.leaving {
                    if edge.twin().origin() == node {
                        return edge;
                    } else {
                        edge = edge.twin().next();
                    }
                }
            }
        }

        return DcelHalfEdgePtr::null();
    }

}