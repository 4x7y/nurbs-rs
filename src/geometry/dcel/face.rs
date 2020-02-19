use crate::geometry::*;
use std::ptr;

pub struct DoubleEdgeListFace<V, E, F> {
    pub element: F,
    pub edge: DcelHalfEdgePtr<V, E, F>,
}

pub struct DcelFacePtr<V, E, F> {
    pub ptr: *mut DoubleEdgeListFace<V, E, F>,
}

impl<V, E, F> DcelFacePtr<V, E, F> {

    pub fn new(element: F) -> Self {
        let face = DoubleEdgeListFace {
            element,
            edge: DcelHalfEdgePtr::null(),
        };
        DcelFacePtr {
            ptr: Box::into_raw(Box::new(face)),
        }
    }

    #[inline]
    pub fn edge(&self) -> DcelHalfEdgePtr<V, E, F> {
        if self.is_null() {
            return DcelHalfEdgePtr::null();
        }
        unsafe {
            (*self.ptr).edge.clone()
        }
    }

    #[inline]
    pub fn set_edge(&self, edge: DcelHalfEdgePtr<V, E, F>) {
        if self.is_null() {
            return;
        }
        unsafe {
            (*self.ptr).edge = edge;
        }
    }

    #[inline]
    pub fn null() -> DcelFacePtr<V, E, F> {
        DcelFacePtr {
            ptr: ptr::null_mut(),
        }
    }

    #[inline]
    pub fn is_null(&self) -> bool {
        self.ptr.is_null()
    }
}

impl<V,E,F> Clone for DcelFacePtr<V,E,F> {
    fn clone(&self) -> Self {
        DcelFacePtr {
            ptr: self.ptr,
        }
    }
}

impl<V,E,F> Copy for DcelFacePtr<V,E,F> {
}

impl<V,E,F> PartialEq for DcelFacePtr<V,E,F> {
    fn eq(&self, other: &DcelFacePtr<V,E,F>) -> bool {
        self.ptr == other.ptr
    }
}

impl<V,E,F> DcelFacePtr<V,E,F> {
    pub fn get_edge_count(&self) -> usize {
        let mut edge = self.edge();

        let mut count: usize = 0;
        if ! edge.is_null() {
            count += 1;
            while edge.next() != self.edge() {
                count += 1;
                edge = edge.next();
            }
        }
        return count;
    }
}
