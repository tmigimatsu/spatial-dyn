##
# @package spatialdyn
# SpatialDyn Python bindings

##
# Python bindings for the articulated body struct generated by Pybind11.
# @see C++: SpatialDyn::ArticulatedBody
class ArticulatedBody:

    ## Constructor that sets the name of the articulated body.
    # @param name str
    # @see C++: SpatialDyn::ArticulatedBody::ArticulatedBody(const std::string&)
    def __init__(self, name = ""):
        pass

    ## Name of the articulated body for debugging purposes.
    # @see C++: SpatialDyn::ArticulatedBody::name
    name = ""

    ## %Graphics for the base.
    # @see C++: SpatialDyn::ArticulatedBody::graphics
    graphics = Graphics()

    ## Degrees of freedom of the articulated body.
    # @see C++: SpatialDyn::ArticulatedBody::dof()
    dof = 0

    ## Add a rigid body to the articulated body and return its assigned ID.
    # @param rb spatialdyn.RigidBody
    # @param id_parent int
    # @return int
    # @see C++: SpatialDyn::ArticulatedBody::AddRigidBody()
    def add_rigid_body(self, rb: spatialdyn.RigidBody, id_parent = -1):
        pass

    ## List of rigid bodies, indexed by IDs assigned in add_rigid_body().
    # @see C++: SpatialDyn::ArticulatedBody::rigid_bodies()
    rigid_bodies = [RigidBody()]

    ## %Joint positions as a numpy array of size dof.
    # @see C++: SpatialDyn::ArticulatedBody::q()
    q = numpy.zeros(dof, dtype=numpy.float64)

    ## %Joint velocities as a numpy array of size dof.
    # @see C++: SpatialDyn::ArticulatedBody::dq()
    dq = numpy.zeros(dof, dtype=numpy.float64)

    ## %Joint accelerations as a numpy array of size dof.
    # @see C++: SpatialDyn::ArticulatedBody::ddq()
    ddq = numpy.zeros(dof, dtype=numpy.float64)

    ## Gravity vector as a numpy array of size 3.
    # @see C++: SpatialDyn::ArticulatedBody::g()
    g = numpy.array([0., 0., -9.81], dtype=numpy.float64)

    ## Transform from the articulated body's base frame to the world frame.
    # @see C++: SpatialDyn::ArticulatedBody::T_base_to_world()
    T_base_to_world = eigen.Isometry3d()

    ## Get the transform from rigid body `i`'s frame to its parent's frame
    # @param i int
    # @return spatialdyn.eigen.Isometry3d
    # @see C++: SpatialDyn::ArticulatedBody::T_to_parent()
    def T_to_parent(self, i):
        pass

    ## Inverse transform of T_to_parent()
    # @param i int
    # @return spatialdyn.eigen.Isometry3d
    # @see C++: SpatialDyn::ArticulatedBody::T_from_parent()
    def T_from_parent(self, i):
        pass

    ## Get the transform from rigid body `i`'s frame to the world frame
    # @param i int
    # @return spatialdyn.eigen.Isometry3d
    # @see C++: SpatialDyn::ArticulatedBody::T_to_world()
    def T_to_world(self, i):
        pass

    ## Get the list of ancestors of rigid body `i` (inclusive).
    # @param i int
    # @return List[int]
    # @see C++: SpatialDyn::ArticulatedBody::ancestors()
    def ancestors(self, i):
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

    # TODO: C++?
    ## Convert the articulated body to a JSON string.
    # @see spatialdyn.urdf.Serialize()
    def __str__(self):
        pass

    ## Print the articulated body for debugging.
    # @see C++: SpatialDyn::operator<<(std::ostream&, const ArticulatedBody&)
    def __repr__(self):
        pass

##
# Python bindings for the rigid body struct generated by Pybind11.
# @see C++: SpatialDyn::RigidBody
class RigidBody:

    ## Constructor that sets the name of the rigid body.
    # @param name str
    # @see C++: SpatialDyn::RigidBody::RigidBody(const std::string&)
    def __init__(self, name = ""):
        pass

    ## Name of the rigid body.
    # @see C++: SpatialDyn::RigidBody::name
    name = ""

    ## %Graphics for the rigid body.
    # @see C++: SpatialDyn::RigidBody::graphics
    graphics = Graphics()

    ## ID of the rigid body as assigned by ArticulatedBody.add_rigid_body().
    # @see C++: SpatialDyn::RigidBody::id()
    id = -1

    ## ID of the rigid body's parent as assigned by ArticulatedBody.add_rigid_body().
    # @see C++: SpatialDyn::RigidBody::id_parent()
    id_parent = -1

    ## Fixed transform from the rigid body's frame to its parent's frame when
    #  the joint position is 0.
    # @see C++: SpatialDyn::RigidBody::T_to_parent()
    T_to_parent = eigen.Isometry3d()

    ## Spatial inertia of the rigid body.
    # @see C++: SpatialDyn::RigidBody::inertia()
    inertia = SpatialInertiad()

    ## %Joint attached to the rigid body.
    # @see C++: SpatialDyn::RigidBody::joint()
    joint = Joint()

    # TODO: C++?
    ## Convert the rigid body to a JSON string.
    # @see spatialdyn.urdf.Serialize()
    def __str__(self):
        pass

    ## Print the rigid body for debugging.
    # @see C++: SpatialDyn::operator<<(std::ostream&, const RigidBody&)
    def __repr__(self):
        pass

##
# Python bindings for the joint struct generated by Pybind11.
# @see C++: SpatialDyn::Joint
class Joint:

    ## Constructor that sets the joint type.
    # @param type str
    # @see C++: SpatialDyn::Joint::Joint(const std::string&)
    def __init__(self, type = "UNDEFINED"):
        pass

    ## %Joint type.
    #
    # Valid values: `"UNDEFINED"`, `"RX"`, `"RY"`, `"RZ"`, `"PX"`, `"PY"`, `"PZ"`.
    #
    # @see C++: SpatialDyn::Joint::type()
    type = "UNDEFINED"

    ## Boolean indicating whether the joint is prismatic.
    # @see C++: SpatialDyn::Joint::is_prismatic()
    is_prismatic = False

    ## Boolean indicating whether the joint is revolute.
    # @see C++: SpatialDyn::Joint::is_revolute()
    is_revolute = False

    ## Lower joint limit.
    # @see C++: SpatialDyn::Joint::q_min()
    q_min = -float("inf")

    ## Upper joint limit.
    # @see C++: SpatialDyn::Joint::q_max()
    q_max = float("inf")

    ## Set the lower and upper joint limits.
    # @see C++: SpatialDyn::Joint::set_q_limits()
    def set_q_limits(q_min, q_max):
        pass

    ## Maximum joint velocity.
    # @see C++: SpatialDyn::Joint::dq_max
    dq_max = float("inf")

    ## Maximum joint torque.
    # @see C++: SpatialDyn::Joint::fq_max
    fq_max = float("inf")

    ## Coulomb friction coefficient
    # @see C++: SpatialDyn::Joint::f_coulomb
    f_coulomb = 0.0

    ## Viscous friction coefficient
    # @see C++: SpatialDyn::Joint::f_viscous
    f_viscous = 0.0

    ## Stiction friction coefficient
    # @see C++: SpatialDyn::Joint::f_stiction
    f_stiction = 0.0

    ## Compute the transform from the joint frame to its parent rigid body's frame.
    # @see C++: SpatialDyn::Joint::T_joint()
    def T_joint(q):
        pass

    # TODO: C++?
    ## Convert the rigid body to a JSON string.
    # @see spatialdyn.urdf.Serialize()
    def __str__(self):
        pass

    ## Print the rigid body for debugging.
    # @see C++: SpatialDyn::operator<<(std::ostream&, const RigidBody&)
    def __repr__(self):
        pass

