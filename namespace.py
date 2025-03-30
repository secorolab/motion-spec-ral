# SPDX-License-Identifier: MPL-2.0
from rdflib import URIRef
from rdflib.namespace import DefinedNamespace, Namespace

class APP(DefinedNamespace):
    constraints: URIRef
    path: URIRef

    _extras = [
        "import",
        "entry-point",
        "reasoning-rules",
        "iri-map"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/application/")


class GEOM_ENT(DefinedNamespace):
    Point: URIRef
    Frame: URIRef
    SimplicialComplex: URIRef

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/structural-entities#")

class QUDT_SCHEMA(DefinedNamespace):
    Quantity: URIRef
    hasQuantityKind: URIRef
    unit: URIRef
    value: URIRef

    _NS = Namespace("http://qudt.org/schema/qudt/")

class QUDT_QKIND(DefinedNamespace):
    Angle: URIRef
    Length: URIRef
    AngularVelocity: URIRef
    LinearVelocity: URIRef
    AngularAcceleration: URIRef
    LinearAcceleration: URIRef
    Torque: URIRef
    Force: URIRef

    _NS = Namespace("http://qudt.org/vocab/quantitykind/")

class QUDT_UNIT(DefinedNamespace):
    UNITLESS: URIRef
    M: URIRef
    N: URIRef

    _extras = [
        "M-PER-SEC",
        "M-PER-SEC2",
        "N-M",
        "RAD-PER-SEC",
        "RAD-PER-SEC2"
    ]

    _NS = Namespace("http://qudt.org/vocab/unit/")

class GEOM_REL(DefinedNamespace):
    Pose: URIRef
    VelocityTwist: URIRef
    AccelerationTwist: URIRef

    of: URIRef

    _extras = [ 
        "with-respect-to",
        "reference-point"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/spatial-relations#")

class GEOM_COORD(DefinedNamespace):
    DirectionCoordinate: URIRef
    PositionCoordinate: URIRef
    PoseCoordinate: URIRef
    VelocityTwistCoordinate: URIRef
    AccelerationTwistCoordinate: URIRef
    DirectionCosineXYZ: URIRef
    VectorXYZ: URIRef

    x: URIRef
    y: URIRef
    z: URIRef

    _extras = [ 
        "of-pose",
        "of-velocity",
        "of-acceleration",
        "as-seen-by",
        "direction-cosine-x",
        "direction-cosine-y",
        "direction-cosine-z",
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/coordinates#")

class GEOM_OP(DefinedNamespace):
    RotateDirectionDistalToProximalWithPose: URIRef
    AddVelocityTwist: URIRef
    AddAccelerationTwist: URIRef
    ComposePose: URIRef
    TransformVelocityTwistToDistal: URIRef
    RotateVelocityTwistToProximalWithPose: URIRef
    TransformAccelerationTwistToDistal: URIRef
    PoseToAngleAroundAxis: URIRef
    PoseToLinearDistance: URIRef
    PoseToDirection: URIRef
    PlanarAngleFromDirections: URIRef
    InvertAngle: URIRef

    in1: URIRef
    in2: URIRef
    composite: URIRef
    pose: URIRef
    to: URIRef
    distance: URIRef
    direction: URIRef
    angle: URIRef
    out: URIRef
    axis: URIRef

    _extras = [
        "from",
        "absolute-velocity",
        "relative-velocity",
        "from-directions",
        "in"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/geometry/spatial-operators#")

class RBDYN_ENT(DefinedNamespace):
    Wrench: URIRef

    _extras = [
        "reference-point"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/structural-entities#")

class RBDYN_COORD(DefinedNamespace):
    WrenchCoordinate: URIRef
    #VectorXYZ: URIRef

    _extras = [
        "as-seen-by"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/coordinates#")

class RBDYN_OP(DefinedNamespace):
    TransformWrenchToProximal: URIRef
    RotateWrenchToDistalWithPose: URIRef
    RotateWrenchToProximalWithPose: URIRef
    WrenchFromPositionDirectionAndMagnitude: URIRef
    AddWrench: URIRef

    position: URIRef
    pose: URIRef
    to: URIRef
    magnitude: URIRef
    direction: URIRef
    wrench: URIRef
    in1: URIRef
    in2: URIRef
    out: URIRef

    _extras = [
        "from"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/newtonian-rigid-body-dynamics/operators#")

class MAP(DefinedNamespace):
    View: URIRef
    DirectionCoordinateView: URIRef
    PoseCoordinateView: URIRef
    VelocityTwistCoordinateView: URIRef
    AccelerationTwistCoordinateView: URIRef
    WrenchCoordinateView: URIRef

    superobject: URIRef
    subobject: URIRef
    subspace: URIRef
    position: URIRef
    torque: URIRef
    force: URIRef
    axis: URIRef
    x: URIRef
    y: URIRef
    z: URIRef

    _extras = [
        "angular-velocity",
        "linear-velocity",
        "angular-acceleration",
        "linear-acceleration"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/task/map#")

class CSTR(DefinedNamespace):
    Constraint: URIRef
    EqualityConstraint: URIRef
    UnilateralConstraint: URIRef
    GreaterThanConstraint: URIRef
    LessThanConstraint: URIRef
    BilateralConstraint: URIRef

    quantity: URIRef
    threshold: URIRef

    _extras = [
        "reference-value",
        "lower-threshold",
        "upper-threshold"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/task/constraint#")

class MOT(DefinedNamespace):
    GuardedMotion: URIRef

    when: URIRef
    until: URIRef

    _extras = [
        "while"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/task/motion-specification#")

class CSTR_HDL(DefinedNamespace):
    ConstraintHandler: URIRef
    ConstraintEvaluator: URIRef
    AssignmentEvaluator: URIRef
    ErrorEvaluator: URIRef
    Controller: URIRef
    ProportionalIntegralDerivative: URIRef
    DecayingIntegralTerm: URIRef

    motion: URIRef
    evaluators: URIRef
    monitors: URIRef
    controllers: URIRef
    constraint: URIRef
    error: URIRef

    _extras = [
        "error-signal",
        "control-signal",
        "proportional-gain",
        "integral-gain",
        "derivative-gain",
        "decay-rate"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/task/constraint-handler#")

class SLV(DefinedNamespace):
    VelocityCompositionSolver: URIRef
    ForceDistributionSolver: URIRef
    SolverWithInputAndOutput: URIRef
    MotionDrivers: URIRef
    AccelerationConstraintSpecification: URIRef
    CartesianForceSpecification: URIRef
    AccelerationConstraint: URIRef
    AxisAligned: URIRef

    constraints: URIRef
    force: URIRef
    subspace: URIRef
    axis: URIRef
    x: URIRef
    y: URIRef
    z: URIRef
    configuration: URIRef
    velocity: URIRef
    output: URIRef

    _extras = [
        "motion-drivers",
        "cartesian-force",
        "acceleration-constraint",
        "acceleration-energy",
        "angular-acceleration",
        "linear-acceleration",
        "attached-to"
    ]

    _NS = Namespace("https://comp-rob2b.github.io/metamodels/task/solver-specification#")