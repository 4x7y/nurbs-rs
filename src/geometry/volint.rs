/// Fast and Accurate Computation of Polyhedral Mass Propertie
///
/// https://people.eecs.berkeley.edu/~jfc/mirtich/massProps.html
///
/// The location of a body's center of mass, and its moments and products
/// of inertia about various axes are important physical quantities needed
/// for any type of dynamic simulation or physical based modeling. We
/// present an algorithm for automatically computing these quantities for
/// a general class of rigid bodies: those composed of uniform density polyhedra.
///
/// Our algorithm is based on a three step reduction of the volume integrals
/// to successively simpler integrals. The algorithm is designed to minimize
/// the numerical errors that can result from poorly conditioned alignment of
/// polyhedral faces. It is also designed for efficiency. All required volume
/// integrals of a polyhedron are computed together during a single walk over
/// the boundary of the polyhedron; exploiting common subexpressions reduces
/// floating point operations.
pub fn comp_projection_integrals() {
    unimplemented!()
}

