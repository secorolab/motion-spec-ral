# SPDX-License-Identifier: MPL-2.0
import sys
from dataclasses import dataclass, field, is_dataclass, asdict
from enum import Enum
import collections
import re
import json
import rdflib
import resolver
from functools import wraps
from rdflib.namespace import RDF
from rdflib import URIRef
from namespace import APP, QUDT_SCHEMA, QUDT_QKIND, QUDT_UNIT, \
    GEOM_ENT, GEOM_REL, GEOM_COORD, GEOM_OP, RBDYN_ENT, RBDYN_COORD, RBDYN_OP, \
    MAP, CSTR, MOT, CSTR_HDL, SLV


class JSONEncoder(json.JSONEncoder):
    def default(self, o):
        if is_dataclass(o):
            return asdict(o)
        return super().default(o)



def parse_argument(g, closure_id, argument, to_id):
    # For each of the key differentiate if there is one or more associated value
    entry = list(g[closure_id : argument])
    if len(entry) == 0:
        return None
    elif len(entry) == 1:
        return to_id(entry[0])
    else:
        return [to_id(e) for e in entry]


@dataclass
class Operator:
    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)

    def closure_step(self, g, to_id, closure_id):
        closure = {
            "id": to_id(closure_id),
            "type": to_id(self.type_)
        }

        for input in self.input:
            closure[to_id(input)] = parse_argument(g, closure_id, input, to_id)
        for output in self.output:
            closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
        for param in self.parameters:
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id)

        return closure

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        for out in self.output:
            return [op for op in g.subjects(out, data_out) if g[op : RDF["type"] : self.type_]]

    def is_schedulable(self):
        return True

    def scheduler_step(self, g, data_out):
        data_structures = set()
        schedule = []

        for out in self.output:
            for call in g[: out : data_out]:
                if not g[call : RDF["type"] : self.type_]:
                    continue

                # We will only record this call if it has any input
                has_any_input = False

                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)
                        has_any_input = True

                if has_any_input:
                    schedule.append(call)

        return {
            "data_structures": data_structures,
            "schedule": schedule
        }


@dataclass
class Specification:
    """
    A specification is not a computation and, hence, has neither a closure nor
    an entry in a schedule but contributes in finding further computations by
    propagating from outputs to inputs.
    """
    type_: URIRef
    input: list[URIRef]
    output: list[URIRef]
    parameters: list = field(default_factory=list)

    def closure_step(self, g, to_id, closure_id):
        return None

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for in_ in self.input:
            for data_in in g.objects(operator_id, in_):
                data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        for out in self.output:
            return [op for op in g.subjects(out, data_out)
                    if g[op : RDF["type"] : self.type_]]

    def is_schedulable(self):
        """
        A specification is not callable.
        """
        return False

    def scheduler_step(self, g, data_out):
        data_structures = set()
        for out in self.output:
            for call in g.subjects(out, data_out):
                for in_ in self.input:
                    for data_in in g.objects(call, in_):
                        data_structures.add(data_in)

        return {
            "data_structures": data_structures,
            "schedule": []
        }



class ErrorEvaluator:
    def __init__(self):
        self.type_ = CSTR_HDL["ErrorEvaluator"]
        self.cstr_op = [
            Operator(
                type_=CSTR["EqualityConstraint"],
                input=[CSTR["quantity"], CSTR["reference-value"]],
                output=[CSTR_HDL["error"]]),
            Operator(
                type_=CSTR["GreaterThanConstraint"],
                input=[CSTR["quantity"], CSTR["threshold"]],
                output=[CSTR_HDL["error"]]),
            Operator(
                type_=CSTR["LessThanConstraint"],
                input=[CSTR["quantity"], CSTR["threshold"]],
                output=[CSTR_HDL["error"]]),
            Operator(
                type_=CSTR["BilateralConstraint"],
                input=[CSTR["quantity"], CSTR["lower-threshold"], CSTR["upper-threshold"]],
                output=[CSTR_HDL["error"]]),
        ]

    def closure_step(self, g, to_id, closure_id):
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        for operator in self.cstr_op:
            if not g[constraint_id : RDF["type"] : operator.type_]:
                continue

            closure = {
                "id": to_id(closure_id),
                "type": "ErrorEvaluator",
                "constraint": to_id(operator.type_)
            }

            for input in operator.input:
                closure[to_id(input)] = parse_argument(g, constraint_id, input, to_id)
            for output in operator.output:
                closure[to_id(output)] = parse_argument(g, closure_id, output, to_id)
            for param in operator.parameters:
                closure[to_id(param)] = parse_argument(g, closure_id, param, to_id)

            # Only return the first matching type
            return closure

        # Should never happen
        return None

    def from_operator_to_input(self, g, operator_id):
        data_structures = set()

        for op in self.cstr_op:
            if op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
                continue

            for in_ in op.input:
                for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                    data_structures.add(data_in)

        return data_structures

    def from_output_to_operator(self, g, data_out):
        for operator in self.cstr_op:
            for out in operator.output:
                return [op for op in g.subjects(out, data_out)
                        if g[op : RDF["type"] : self.type_]]

    def is_schedulable(self):
        """
        An error evaluator is not callable (but it will be added separately to
        the schedule!).
        """
        return False

    def scheduler_step(self, g, data_out):
        data_structures = set()
        schedule = []

        for op in self.cstr_op:
            for out in op.output:
                for call in g.subjects(out, data_out):
                    if op.type_ not in g[call : CSTR_HDL["constraint"] / RDF["type"]]:
                        continue

                    # We will only record this call if it has any input
                    has_any_input = False

                    for in_ in op.input:
                        for data_in in g.objects(call, CSTR_HDL["constraint"] / in_):
                            data_structures.add(data_in)
                            has_any_input = True

                    if has_any_input:
                        schedule.append(call)

        return {
            "data_structures": data_structures,
            "schedule": schedule
        }


class AssignmentEvaluator:
    def __init__(self):
        self.type_ = CSTR_HDL["AssignmentEvaluator"]
        self.cstr_op = Operator(
                type_=CSTR["EqualityConstraint"],
                input=[CSTR["quantity"], CSTR["reference-value"]],
                output=[])

    def closure_step(self, g, to_id, closure_id):
        constraint_id = g.value(closure_id, CSTR_HDL["constraint"])

        if not g[constraint_id : RDF["type"] : self.cstr_op.type_]:
            return None

        closure = {
            "id": to_id(closure_id),
            "type": "AssignmentEvaluator",
            "constraint": to_id(self.cstr_op.type_)
        }

        for input in self.cstr_op.input:
            closure[to_id(input)] = parse_argument(g, constraint_id, input, to_id)
        for param in self.cstr_op.parameters:
            closure[to_id(param)] = parse_argument(g, closure_id, param, to_id)

        # Only return the first matching type
        return closure

    def from_operator_to_input(self, g, operator_id):
        if self.cstr_op.type_ not in g[operator_id : CSTR_HDL["constraint"] / RDF["type"]]:
            return set()

        data_structures = set()
        for in_ in self.cstr_op.input:
            for data_in in g.objects(operator_id, CSTR_HDL["constraint"] / in_):
                data_structures.add(data_in)

        return data_structures

    def is_schedulable(self):
        return True

    def scheduler_step(self, g, data_out):
        return {
            "data_structures": [],
            "schedule": []
        }


ops_generic = [
    Operator(
        type_=GEOM_OP["RotateDirectionDistalToProximalWithPose"],
        input=[GEOM_OP["pose"], GEOM_OP["from"]],
        output=[GEOM_OP["to"]]),
    Operator(
        type_=GEOM_OP["ComposePose"],
        input=[GEOM_OP["in1"], GEOM_OP["in2"]],
        output=[GEOM_OP["composite"]]),
    Operator(
        type_=GEOM_OP["RotateVelocityTwistToProximalWithPose"],
        input=[GEOM_OP["pose"], GEOM_OP["from"]],
        output=[GEOM_OP["to"]]),
    Operator(
        type_=GEOM_OP["PoseToAngleAroundAxis"],
        input=[GEOM_OP["pose"]],
        output=[GEOM_OP["angle"]],
        parameters=[GEOM_OP["axis"]]),
    Operator(
        type_=GEOM_OP["PoseToLinearDistance"],
        input=[GEOM_OP["pose"]],
        output=[GEOM_OP["distance"]]),
    Operator(
        type_=GEOM_OP["PoseToDirection"],
        input=[GEOM_OP["pose"]],
        output=[GEOM_OP["direction"]]),
    Operator(
        type_=GEOM_OP["PlanarAngleFromDirections"],
        input=[GEOM_OP["from-directions"]],
        output=[GEOM_OP["angle"]]),
    Operator(
        type_=GEOM_OP["InvertAngle"],
        input=[GEOM_OP["in"]],
        output=[GEOM_OP["out"]]),
    Operator(
        type_=RBDYN_OP["AddWrench"],
        input=[RBDYN_OP["in1"], RBDYN_OP["in2"]],
        output=[RBDYN_OP["out"]]),
    Operator(
        type_=RBDYN_OP["RotateWrenchToDistalWithPose"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]]),
    Operator(
        type_=RBDYN_OP["RotateWrenchToProximalWithPose"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]]),
    Operator(
        type_=RBDYN_OP["TransformWrenchToProximal"],
        input=[RBDYN_OP["pose"], RBDYN_OP["from"]],
        output=[RBDYN_OP["to"]]),
    Operator(
        type_=RBDYN_OP["WrenchFromPositionDirectionAndMagnitude"],
        input=[RBDYN_OP["magnitude"], RBDYN_OP["direction"], RBDYN_OP["position"]],
        output=[RBDYN_OP["wrench"]]),
    Specification(
        type_=MAP["View"],
        input=[MAP["superobject"], MAP["subobject"]],
        output=[MAP["superobject"], MAP["subobject"]])
]

ops_cstr_hdl = [
    Operator(
        type_=CSTR_HDL["Controller"],
        input=[CSTR_HDL["error-signal"]],
        output=[CSTR_HDL["control-signal"]],
        parameters=[CSTR_HDL["proportional-gain"], CSTR_HDL["integral-gain"],
                    CSTR_HDL["derivative-gain"], CSTR_HDL["decay-rate"]]),
    AssignmentEvaluator(),
    ErrorEvaluator()
]

ops_slv = [
    Specification(
        type_=SLV["CartesianForceSpecification"],
        input=[SLV["force"]],
        output=[]),
    Specification(
        type_=SLV["AccelerationConstraint"],
        input=[SLV["acceleration-energy"]],
        output=[]),
    Specification(
        type_=SLV["ForceDistributionSolver"],
        input=[SLV["force"]],
        output=[])
]


class Subspace(str, Enum):
    Position = "Position"
    AngularVelocity = "AngularVelocity"
    LinearVelocity = "LinearVelocity"
    AngularAcceleration = "AngularAcceleration"
    LinearAcceleration = "LinearAcceleration"
    Torque = "Torque"
    Force = "Force"

class Axis(str, Enum):
    X = "X"
    Y = "Y"
    Z = "Z"

class UnilateralConstraintType(str, Enum):
    GreaterThan = "GreaterThan"
    LessThan = "LessThan"

class EvaluatorType(str, Enum):
    AssignmentEvaluator = "AssignmentEvaluator"
    ErrorEvaluator = "ErrorEvaluator"

# Forward declaration
class View:
    pass

@dataclass
class Point:
    id: str
    type: str = field(default="Point")

@dataclass
class Frame:
    id: str
    type: str = field(default="Frame")

@dataclass
class QuantityKind:
    id: str
    type: str = field(default="QuantityKind")

@dataclass
class Unit:
    id: str
    type: str = field(default="Unit")

@dataclass
class Quantity:
    id: str
    quantity_kind: QuantityKind
    unit: Unit
    value: float
    has_view: bool
    type: str = field(default="Quantity")

@dataclass
class SimplicialComplex:
    id: str
    type: str = field(default="SimplicialComplex")

@dataclass
class Direction:
    id: str
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame
    unit: list[Unit]
    type: str = field(default="Direction")

@dataclass
class Position:
    id: str
    of: SimplicialComplex
    with_respect_to: SimplicialComplex
    quantity_kind: QuantityKind
    as_seen_by: Frame
    unit: Unit
    position: list[float] | None
    type: str = field(default="Position")

@dataclass
class Pose:
    id: str
    of: SimplicialComplex
    with_respect_to: SimplicialComplex
    quantity_kind: list[QuantityKind]
    as_seen_by: Frame
    unit: list[Unit]
    direction_cosine_x: list[float] | None
    direction_cosine_y: list[float] | None
    direction_cosine_z: list[float] | None
    position: list[float] | None
    type: str = field(default="Pose")

@dataclass
class VelocityTwist:
    id: str
    of: SimplicialComplex
    with_respect_to: SimplicialComplex
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    type: str = field(default="VelocityTwist")

@dataclass
class AccelerationTwist:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    type: str = field(default="AccelerationTwist")

@dataclass
class Wrench:
    id: str
    quantity_kind: list[QuantityKind]
    reference_point: Point
    as_seen_by: Frame
    unit: list[Unit]
    type: str = field(default="Wrench")


@dataclass
class View:
    id: str
    superobject: VelocityTwist | AccelerationTwist | Wrench
    subobject: Quantity
    subspace: Subspace
    axis: Axis
    type: str = field(default="View")

@dataclass
class EqualityConstraint:
    reference_value: Quantity
    type: str = field(default="EqualityConstraint")

@dataclass
class UnilateralConstraint:
    type_: UnilateralConstraintType
    threshold: Quantity
    type: str = field(default="UnilateralConstraint")

@dataclass
class BilateralConstraint:
    lower_threshold: Quantity
    upper_threshold: Quantity
    type: str = field(default="BilateralConstraint")

@dataclass
class Constraint:
    id: str
    quantity: Quantity
    parameter: EqualityConstraint | UnilateralConstraint | BilateralConstraint
    type: str = field(default="Constraint")

@dataclass
class GuardedMotion:
    id: str
    when: list[Constraint]
    while_: list[Constraint]
    until: list[Constraint]
    type: str = field(default="GuardedMotion")


@dataclass
class ConstraintEvaluator:
    id: str
    type_: EvaluatorType
    constraint: Constraint
    error: Quantity | None
    type: str = field(default="ConstraintEvaluator")

@dataclass
class Controller:
    id: str
    error_signal: Quantity
    control_signal: Quantity
    proportional_gain: float
    integral_gain: float
    derivative_gain: float
    decay_rate: float | None
    type: str = field(default="Controller")

@dataclass
class ConstraintHandler:
    id: str
    motion: GuardedMotion
    evaluators: list[ConstraintEvaluator]
    controllers: list[Controller]
    type: str = field(default="ConstraintHandler")


@dataclass
class AccelerationConstraint:
    id: str
    subspace: Subspace
    axis: Axis
    acceleration_energy: Quantity
    type: str = field(default="AccelerationConstraint")

@dataclass
class AccelerationConstraintSpecification:
    id: str
    constraints: list[AccelerationConstraint]
    attached_to: SimplicialComplex
    type: str = field(default="AccelerationConstraintSpecification")

@dataclass
class CartesianForceSpecification:
    id: str
    force: Wrench
    attached_to: SimplicialComplex
    type: str = field(default="CartesianForceSpecification")

@dataclass
class MotionDrivers:
    id: str
    acceleration_constraint: list[AccelerationConstraintSpecification]
    cartesian_force: list[CartesianForceSpecification]
    type: str = field(default="MotionDrivers")

@dataclass
class SolverWithInputAndOutput:
    id: str
    motion_drivers: MotionDrivers
    output: list
    type: str = field(default="SolverWithInputAndOutput")

@dataclass
class VelocityCompositionSolver:
    id: str
    configuration: str
    velocity: VelocityTwist
    type: str = field(default="VelocityCompositionSolver")

@dataclass
class ForceDistributionSolver:
    id: str
    configuration: str
    force: Wrench
    type: str = field(default="ForceDistributionSolver")


def memoize(func):
    @wraps(func)
    def decorator(self, *args, **kwargs):
        key = args + tuple(kwargs.items())
        if key not in self.cache:
            self.cache[key] = func(self, *args, **kwargs)
        return self.cache[key]
    return decorator

def escape(s):
    s = re.sub("[:-]", "_", s)
    s = re.sub("[<>]", "", s)
    return s


class Parser:
    def __init__(self, g):
        self.cache = dict()
        self.g = g
        self.sched = set()

    def id(self, x):
        try:
            q = self.g.compute_qname(x)
            return escape(q[2])
        except:
            return x

    @memoize
    def velocity_composition_solver(self, id_):
        assert(SLV["VelocityCompositionSolver"] in self.g[id_ : RDF["type"]])

        conf = self.id(self.g.value(id_, SLV["configuration"]))
        velocity = self.velocity_twist(self.g.value(id_, SLV["velocity"]))

        return VelocityCompositionSolver(self.id(id_), conf, velocity)

    @memoize
    def force_distribution_solver(self, id_):
        assert(SLV["ForceDistributionSolver"] in self.g[id_ : RDF["type"]])

        conf = self.id(self.g.value(id_, SLV["configuration"]))
        force = self.wrench(self.g.value(id_, SLV["force"]))

        return ForceDistributionSolver(self.id(id_), conf, force)

    @memoize
    def solver_with_input_and_output(self, id_):
        assert(SLV["SolverWithInputAndOutput"] in self.g[id_ : RDF["type"]])

        io_dispatcher = [
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist)
        ]

        drv = self.motion_drivers(self.g.value(id_, SLV["motion-drivers"]))
        out = []
        for o in self.g[id_ : SLV["output"]]:
            for type_, func in io_dispatcher:
                if self.g[o : RDF["type"] : type_ ]:
                    out.append(func(o))

        return SolverWithInputAndOutput(self.id(id_), drv, out)

    @memoize
    def motion_drivers(self, id_):
        assert(SLV["MotionDrivers"] in self.g[id_ : RDF["type"]])

        spec_acc = []
        spec_frc = []

        for a in self.g[id_ : SLV["acceleration-constraint"]]:
            spec_acc.append(self.acceleration_constraint_specification(a))

        for f in self.g[id_ : SLV["cartesian-force"]]:
            spec_frc.append(self.cartesian_force_specification(f))

        return MotionDrivers(self.id(id_), spec_acc, spec_frc)

    @memoize
    def cartesian_force_specification(self, id_):
        assert(SLV["CartesianForceSpecification"] in self.g[id_ : RDF["type"]])

        force = self.wrench(self.g.value(id_, SLV["force"]))
        attached_to = self.simplicial_complex(self.g.value(id_, SLV["attached-to"]))

        return CartesianForceSpecification(self.id(id_), force, attached_to)

    @memoize
    def acceleration_constraint_specification(self, id_):
        assert(SLV["AccelerationConstraintSpecification"] in self.g[id_ : RDF["type"]])

        constraints = []
        for c in self.g[id_ : SLV["constraints"]]:
            constraints.append(self.acceleration_constraint(c))
        attached_to = self.simplicial_complex(self.g.value(id_, SLV["attached-to"]))

        return AccelerationConstraintSpecification(self.id(id_), constraints, attached_to)

    @memoize
    def acceleration_constraint(self, id_):
        assert(SLV["AccelerationConstraint"] in self.g[id_ : RDF["type"]])
        assert(SLV["AxisAligned"] in self.g[id_ : RDF["type"]])

        subspace = self.subspace(self.g.value(id_, SLV["subspace"]))
        axis = self.axis(self.g.value(id_, SLV["axis"]))
        e_acc = self.quantity(self.g.value(id_, SLV["acceleration-energy"]))

        return AccelerationConstraint(self.id(id_), subspace, axis, e_acc)

    @memoize
    def subspace(self, id_):
        d = {
            MAP["position"]: Subspace.Position,
            MAP["angular-velocity"]: Subspace.AngularVelocity,
            MAP["linear-velocity"]: Subspace.LinearVelocity,
            MAP["angular-acceleration"]: Subspace.AngularAcceleration,
            MAP["linear-acceleration"]: Subspace.LinearAcceleration,
            MAP["torque"]: Subspace.Torque,
            MAP["force"]: Subspace.Force,
            SLV["angular-acceleration"]: Subspace.AngularAcceleration,
            SLV["linear-acceleration"]: Subspace.LinearAcceleration
        }
        assert(id_ in d.keys())

        return d[id_]

    @memoize
    def axis(self, id_):
        d = {
            MAP["x"]: Axis.X,
            MAP["y"]: Axis.Y,
            MAP["z"]: Axis.Z,
            SLV["x"]: Axis.X,
            SLV["y"]: Axis.Y,
            SLV["z"]: Axis.Z
        }
        assert(id_ in d.keys())

        return d[id_]


    @memoize
    def constraint_handler(self, id_):
        assert(CSTR_HDL["ConstraintHandler"] in self.g[id_ : RDF["type"]])

        motion = self.guarded_motion(self.g.value(id_, CSTR_HDL["motion"]))

        evaluators = []
        for e in self.g[id_ : CSTR_HDL["evaluators"]]:
            evaluators.append(self.constraint_evaluator(e))

        controllers = []
        for c in self.g[id_ : CSTR_HDL["controllers"]]:
            controllers.append(self.controller(c))

        return ConstraintHandler(self.id(id_), motion, evaluators, controllers)

    @memoize
    def constraint_evaluator(self, id_):
        assert(CSTR_HDL["ConstraintEvaluator"] in self.g[id_ : RDF["type"]])

        constraint = self.constraint(self.g.value(id_, CSTR_HDL["constraint"]))

        if CSTR_HDL["AssignmentEvaluator"] in self.g[id_ : RDF["type"]]:
            t = EvaluatorType.AssignmentEvaluator
            error = None
        else:
            t = EvaluatorType.ErrorEvaluator
            error = self.quantity(self.g.value(id_, CSTR_HDL["error"]))

        return ConstraintEvaluator(self.id(id_), t, constraint, error)

    @memoize
    def controller(self, id_):
        assert(CSTR_HDL["Controller"] in self.g[id_ : RDF["type"]])
        assert(CSTR_HDL["ProportionalIntegralDerivative"] in self.g[id_ : RDF["type"]])

        error_signal = self.quantity(self.g.value(id_, CSTR_HDL["error-signal"]))
        control_signal = self.quantity(self.g.value(id_, CSTR_HDL["control-signal"]))

        decay_rate = None
        if CSTR_HDL["DecayingIntegralTerm"] in self.g[id_ : RDF["type"]]:
            decay_rate = self.g.value(id_, CSTR_HDL["decay-rate"]).value
        
        p = self.g.value(id_, CSTR_HDL["proportional-gain"]).value
        i = self.g.value(id_, CSTR_HDL["integral-gain"]).value
        d = self.g.value(id_, CSTR_HDL["derivative-gain"]).value

        return Controller(self.id(id_), error_signal, control_signal, p, i, d, decay_rate)


    @memoize
    def guarded_motion(self, id_):
        assert(MOT["GuardedMotion"] in self.g[id_ : RDF["type"]])

        when = []
        for c in self.g[id_ : MOT["when"]]:
            when.append(self.constraint(c))

        while_ = []
        for c in self.g[id_ : MOT["while"]]:
            while_.append(self.constraint(c))

        until = []
        for c in self.g[id_ : MOT["until"]]:
            until.append(self.constraint(c))

        return GuardedMotion(self.id(id_), when, while_, until)


    @memoize
    def constraint(self, id_):
        assert(CSTR["Constraint"] in self.g[id_ : RDF["type"]])

        quantity = self.quantity(self.g.value(id_, CSTR["quantity"]))

        parameter = None
        if CSTR["EqualityConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.equality_constraint(id_)
        elif CSTR["UnilateralConstraint"] in self.g[id_ : RDF["type"]]:
            parameter = self.unilateral_constraint(id_)
        else:
            parameter = self.bilateral_constraint(id_)

        return Constraint(self.id(id_), quantity, parameter)

    @memoize
    def equality_constraint(self, id_):
        assert(CSTR["EqualityConstraint"] in self.g[id_ : RDF["type"]])

        reference_value = self.quantity(self.g.value(id_, CSTR["reference-value"]))

        return EqualityConstraint(reference_value)

    @memoize
    def unilateral_constraint(self, id_):
        assert(CSTR["UnilateralConstraint"] in self.g[id_ : RDF["type"]])

        threshold = self.quantity(self.g.value(id_, CSTR["threshold"]))
        type_ = UnilateralConstraintType.LessThan
        if CSTR["GreaterThanConstraint"] in self.g[id_ : RDF["type"]]:
            type_ = UnilateralConstraintType.GreaterThan

        return UnilateralConstraint(type_, threshold)

    @memoize
    def bilateral_constraint(self, id_):
        assert(CSTR["BilateralConstraint"] in self.g[id_ : RDF["type"]])

        lower_threshold = self.quantity(self.g.value(id_, CSTR["lower-threshold"]))
        upper_threshold = self.quantity(self.g.value(id_, CSTR["upper-threshold"]))

        return BilateralConstraint(lower_threshold, upper_threshold)


    @memoize
    def direction(self, id_):
        assert(GEOM_COORD["DirectionCoordinate"] in self.g[id_ : RDF["type"]])
        assert(GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        quantity_kind = []
        for k in g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))

        return Direction(self.id(id_), quantity_kind, as_seen_by, unit)

    def parse_vector3(self, node):
        from rdflib import collection
        l = list(collection.Collection(self.g, node))
        if len(l) != 3:
            return None

        # Get the Python representation of the associated RDF literal
        return list(map(lambda v: v.value, l))

    def parse_direction_cosine_xyz(self, node):
        cos_x = self.parse_vector3(self.g.value(node, GEOM_COORD["direction-cosine-x"]))
        cos_y = self.parse_vector3(self.g.value(node, GEOM_COORD["direction-cosine-y"]))
        cos_z = self.parse_vector3(self.g.value(node, GEOM_COORD["direction-cosine-z"]))

        if cos_x is None or cos_y is None or cos_z is None:
            return None

        mat_col_major = []
        for col in [cos_x, cos_y, cos_z]:
            mat_col_major.extend(col)

        return mat_col_major

    def parse_xyz(self, node):
        x = self.g.value(node, GEOM_COORD["x"])
        y = self.g.value(node, GEOM_COORD["y"])
        z = self.g.value(node, GEOM_COORD["z"])

        if x is None or y is None or z is None:
            return None

        return [x.value, y.value, z.value]

    @memoize
    def position(self, id_):
        assert(GEOM_COORD["PositionCoordinate"] in self.g[id_ : RDF["type"]])
        assert(GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        of = self.point(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.point(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = self.quantity_kind(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        pos = self.parse_xyz(id_)

        return Position(self.id(id_), of, wrt, quantity_kind, as_seen_by, unit, pos)

    @memoize
    def pose(self, id_):
        assert(GEOM_COORD["PoseCoordinate"] in self.g[id_ : RDF["type"]])
        assert(GEOM_COORD["DirectionCosineXYZ"] in self.g[id_ : RDF["type"]])
        assert(GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        of = self.frame(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.frame(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = []
        for k in g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))
        dc_x = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-x"]))
        dc_y = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-y"]))
        dc_z = self.parse_vector3(self.g.value(id_, GEOM_COORD["direction-cosine-z"]))
        pos = self.parse_xyz(id_)

        return Pose(self.id(id_), of, wrt, quantity_kind, as_seen_by, unit, dc_x, dc_y, dc_z, pos)

    @memoize
    def velocity_twist(self, id_):
        assert(GEOM_COORD["VelocityTwistCoordinate"] in self.g[id_ : RDF["type"]])
        assert(GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        of = self.simplicial_complex(self.g.value(id_, GEOM_REL["of"]))
        wrt = self.simplicial_complex(self.g.value(id_, GEOM_REL["with-respect-to"]))
        quantity_kind = []
        for k in g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        return VelocityTwist(self.id(id_), of, wrt, quantity_kind, reference_point, as_seen_by, unit)

    @memoize
    def acceleration_twist(self, id_):
        assert(GEOM_COORD["AccelerationTwistCoordinate"] in self.g[id_ : RDF["type"]])
        assert(GEOM_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        quantity_kind = []
        for k in g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, GEOM_REL["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, GEOM_COORD["as-seen-by"]))
        unit = []
        for u in g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        return AccelerationTwist(self.id(id_), quantity_kind, reference_point, as_seen_by, unit)

    @memoize
    def wrench(self, id_):
        assert(RBDYN_COORD["WrenchCoordinate"] in self.g[id_ : RDF["type"]])
        #assert(RBDYN_COORD["VectorXYZ"] in self.g[id_ : RDF["type"]])

        quantity_kind = []
        for k in g[id_ : QUDT_SCHEMA["hasQuantityKind"]]:
            quantity_kind.append(self.quantity_kind(k))
        reference_point = self.point(self.g.value(id_, RBDYN_ENT["reference-point"]))
        as_seen_by = self.frame(self.g.value(id_, RBDYN_COORD["as-seen-by"]))
        unit = []
        for u in g[id_ : QUDT_SCHEMA["unit"]]:
            unit.append(self.unit(u))

        return Wrench(self.id(id_), quantity_kind, reference_point, as_seen_by, unit)

    @memoize
    def quantity(self, id_):
        assert(QUDT_SCHEMA["Quantity"] in self.g[id_ : RDF["type"]])

        quantity_kind = self.quantity_kind(self.g.value(id_, QUDT_SCHEMA["hasQuantityKind"]))

        unit = self.unit(self.g.value(id_, QUDT_SCHEMA["unit"]))
        value = None
        if (id_, QUDT_SCHEMA["value"], None) in self.g:
            value = float(self.g.value(id_, QUDT_SCHEMA["value"]))
        has_view = (id_, ~MAP["subobject"], None) in self.g

        return Quantity(self.id(id_), quantity_kind, unit, value, has_view)

    @memoize
    def quantity_kind(self, id_):
        return self.id(id_)

    @memoize
    def unit(self, id_):
        return self.id(id_)

    @memoize
    def simplicial_complex(self, id_):
        assert(GEOM_ENT["SimplicialComplex"] in self.g[id_ : RDF["type"]])

        return SimplicialComplex(self.id(id_))

    @memoize
    def frame(self, id_):
        assert(GEOM_ENT["Frame"] in self.g[id_ : RDF["type"]])

        return Frame(self.id(id_))

    @memoize
    def point(self, id_):
        assert(GEOM_ENT["Point"] in self.g[id_ : RDF["type"]])

        return Point(self.id(id_))

    def view(self):
        dispatcher = [
            (MAP["DirectionCoordinateView"], self.direction),
            (MAP["PoseCoordinateView"], self.pose),
            (MAP["VelocityTwistCoordinateView"], self.velocity_twist),
            (MAP["AccelerationTwistCoordinateView"], self.acceleration_twist),
            (MAP["WrenchCoordinateView"], self.wrench)
        ]

        view_map = {}
        for view in self.g[: RDF["type"] : MAP["View"]]:
            for type_, func in dispatcher:
                if not self.g[view : RDF["type"] : type_]:
                    continue

                superobject_id = self.g.value(view, MAP["superobject"])
                superobject = func(superobject_id)
                break

            subobject = self.quantity(self.g.value(view, MAP["subobject"]))
            subspace = self.subspace(self.g.value(view, MAP["subspace"]))
            axis = self.axis(self.g.value(view, MAP["axis"]))

            view_map[self.id(subobject.id)] = View(self.id(view), superobject, subobject, subspace, axis)

        return view_map

    def data_structures(self):
        dispatcher = [
            (GEOM_COORD["DirectionCoordinate"], self.direction),
            (GEOM_COORD["PositionCoordinate"], self.position),
            (GEOM_COORD["PoseCoordinate"], self.pose),
            (GEOM_COORD["VelocityTwistCoordinate"], self.velocity_twist),
            (GEOM_COORD["AccelerationTwistCoordinate"], self.acceleration_twist),
            (RBDYN_COORD["WrenchCoordinate"], self.wrench),
            (QUDT_SCHEMA["Quantity"], self.quantity),
        ]

        data_structures = []
        for type_, func in dispatcher:
            for dstruct in self.g[: RDF["type"] : type_]:
                data_structures.append(func(dstruct))

        return data_structures

    def closures(self, operators):
        closures = {}
        for operator in operators:
            for closure in self.g.subjects(RDF["type"], operator.type_):
                cl = operator.closure_step(self.g, self.id, closure)
                if cl:
                    closures[self.id(closure)] = cl

        return closures

    def schedule(self, start, ops):
        # Start at a SolverWithInputAndOutput
        # Then traverse along data structure and collect function blocks
        q = collections.deque()
        data_structures = set()
        sched = []
        for v in start:
            for op in ops:
                if not self.g[v : RDF["type"] : op.type_]:
                    continue

                call = self.id(v)
                if op.is_schedulable() and call not in set(self.sched):
                    sched.append(call)
                    self.sched.add(call)

                for data_in in op.from_operator_to_input(self.g, v):
                    q.append(data_in)
                    data_structures.add(data_in)

        # This is how an operator/call looks in the "forward" direction:
        #   data_in --(in)--> operator/call --(out)--> data_out
        # But it will be traversed in the "backward" direction.
        # Note that there may exist multiple inputs and outputs.
        while len(q) > 0:
            data_out = q.pop()
            # Iterate through all allowed operators and their outputs
            for op in ops:
                res = op.scheduler_step(self.g, data_out)

                for call in res["schedule"]:
                    call = self.id(call)
                    if call and call not in set(self.sched):
                        sched.append(call)
                        self.sched.add(call)
                
                for data_in in res["data_structures"]:
                    # We have already visited this data structure,
                    # so skip it
                    if data_in in data_structures:
                        continue

                    q.append(data_in)
                    data_structures.add(data_in)

        sched.reverse()
        return sched

    def get_schedule(self):
        s = self.sched
        s.reverse()
        return s


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(0)

    app_model = sys.argv[1]

    # Load top-level, application model
    g = rdflib.ConjunctiveGraph()
    g.parse(app_model, format="json-ld")

    # Load IRI map
    url_map = {}
    for key in g.objects(predicate=APP["iri-map"]):
        value = g.value(key, APP["path"]).value
        url_map[str(key)] = value

    resolver.install(resolver.IriToFileResolver(url_map))

    # Load/import the referenced models
    models = list(g.objects(predicate=APP["import"]))
    for o in models:
        g.parse(location=o, format="json-ld")

    p = Parser(g)

    sched1 = []
    slv_base_vel = []
    sched2 = []
    hdl = []
    sched3 = []
    slv_arm = []
    sched4 = []
    slv_base_frc = []


    # Construct the computational graph that feeds into the mobile base's
    # velocity composition solver.
    slv_id = g.subjects(RDF.type, SLV["VelocityCompositionSolver"])
    for s in slv_id:
        slv_base_vel.append(p.velocity_composition_solver(s))
        start = [s]
        sched1.extend(p.schedule(start, ops_generic + ops_slv))

    # Traverse backward from the "proximal" constraint handler.
    hdl_id = g.subjects(RDF.type, CSTR_HDL["ConstraintHandler"])
    for h in hdl_id:
        hdl.append(p.constraint_handler(h))

        start = g[h : CSTR_HDL["evaluators"] | CSTR_HDL["controllers"]]
        sched2.extend(p.schedule(start, ops_generic + ops_cstr_hdl))

    # Traverse backward from the "distal" solver configuration.
    # The "parser" keeps track of the previously visited closures/computations
    # so that they are visited only once.
    slv_id = g.subjects(RDF.type, SLV["SolverWithInputAndOutput"])
    for s in slv_id:
        slv_arm.append(p.solver_with_input_and_output(s))

        start = g[s : SLV["motion-drivers"]
                        / ((SLV["acceleration-constraint"] / SLV["constraints"])
                        | (SLV["cartesian-force"]))]
        sched3.extend(p.schedule(start, ops_generic + ops_slv))

    # Construct the computational graph that feeds into the mobile base's force
    # distribution solver.
    slv_id = g.subjects(RDF.type, SLV["ForceDistributionSolver"])
    for s in slv_id:
        slv_base_frc.append(p.force_distribution_solver(s))
        start = [s]
        sched4.extend(p.schedule(start, ops_generic + ops_slv))

    # Compose the overall schedule via concatenation
    sched = sched1 + sched2 + sched3 + sched4

    # Extract all views, data structures and closures
    view_map = p.view()
    data_structures = p.data_structures()
    closures = p.closures(ops_generic + ops_slv + ops_cstr_hdl)

    #print(json.dumps(slv, cls=JSONEncoder, indent=4))
    #print(json.dumps(sched, indent=4))
    #print(json.dumps(view_map, cls=JSONEncoder, indent=4))
    #print(json.dumps(data_structures, cls=JSONEncoder, indent=4))
    #print(json.dumps(closures, indent=4))

    ir = {
        "slv_arm": slv_arm,
        "slv_base_vel": slv_base_vel,
        "slv_base_frc": slv_base_frc,
        "cstr_hdl": hdl,
        "data": data_structures,
        "closures": closures,
        "schedule": sched,
        "views": view_map
    }

    print(json.dumps(ir, cls=JSONEncoder, indent=4))