##
# @package spatialdyn
# SpatialDyn Python bindings

class ArticulatedBody:

    ## Constructor that sets the name of the articulated body.
    # @param name str
    # @see C++: SpatialDyn::ArticulatedBody::ArticulatedBody()
    def __init__(self, name: str = ""):
        pass

    ## Name of the articulated body.
    # @see C++: SpatialDyn::ArticulatedBody::name
    name = ""

    ## Graphics struct parsed from the URDF file.
    # @see C++: SpatialDyn::ArticulatedBody::graphics
    graphics = spatialdyn.Graphics

    ## Degrees of freedom of the articulated body.
    # @see C++: SpatialDyn::ArticulatedBody::dof()
    dof = 0

    ## Transform from the articulated body's base frame to the world frame.
    # @see C++: SpatialDyn::ArticulatedBody::T_base_to_world()
    T_base_to_world = eigen.Isometry3d()

    ## Add a rigid body to the articulated body and return its assigned ID.
    # @param rb spatialdyn.RigidBody
    # @param id_parent int
    # @return int
    # @see C++: SpatialDyn::ArticulatedBody::AddRigidBody(RigidBody&&, int)
    def add_rigid_body(self, rb: spatialdyn.RigidBody, id_parent: int = -1):
        pass

    ## List of rigid bodies, indexed by IDs assigned in add_rigid_body().
    # @see C++: SpatialDyn::ArticulatedBody::rigid_bodies()
    rigid_bodies = [spatialdyn.RigidBody()]

    ## Joint positions as a numpy array of size dof.
    # @see C++: SpatialDyn::ArticulatedBody::q()
    q = numpy.zeros(dof, dtype=numpy.float64)

    ## Joint velocities as a numpy array of size dof.
    # @see C++: SpatialDyn::ArticulatedBody::dq()
    dq = numpy.zeros(dof, dtype=numpy.float64)

    ## Joint accelerations as a numpy array of size dof.
    # @see C++: SpatialDyn::ArticulatedBody::ddq()
    ddq = numpy.zeros(dof, dtype=numpy.float64)

    ## Gravity vector as a numpy array of size 3.
    # @see C++: SpatialDyn::ArticulatedBody::g()
    g = numpy.array([0., 0., -9.81], dtype=numpy.float64)

    ## Get the transform from rigid body `i`'s frame to its parent's frame
    # @param i int
    # @return spatialdyn.eigen.Isometry3d
    # @see C++: SpatialDyn::ArticulatedBody::T_to_parent()
    def T_to_parent(self, i: int):
        pass

    ## Inverse transform of T_to_parent()
    # @param i int
    # @return spatialdyn.eigen.Isometry3d
    # @see C++: SpatialDyn::ArticulatedBody::T_from_parent()
    def T_from_parent(self, i: int):
        pass

    ## Get the transform from rigid body `i`'s frame to the world frame
    # @param i int
    # @return spatialdyn.eigen.Isometry3d
    # @see C++: SpatialDyn::ArticulatedBody::T_to_world()
    def T_to_world(self, i: int):
        pass

    ## Get the list of ancestors of rigid body `i` (inclusive).
    # @param i int
    # @return List[int]
    # @see C++: SpatialDyn::ArticulatedBody::ancestors()
    def ancestors(self, i: int):
        pass

    ## Get the list of IDs in the subtree of rigid body `i` (inclusive).
    # @param i int
    # @return List[int]
    # @see C++: SpatialDyn::ArticulatedBody::subtree()
    def subtree(self, i):
        pass

    ## Map the given function across the articulated body structure
    # @param rb_function Callable[[spatialdyn.RigidBody], float]
    # @return numpy.ndarray[numpy.Float64]
    # @see C++: SpatialDyn::ArticulatedBody::Map()
    def map(self, rb_function):
        pass

    ## Convert the articulated body to a JSON string
    # @see spatialdyn.urdf.Serialize()
    def __str__(self):
        pass
