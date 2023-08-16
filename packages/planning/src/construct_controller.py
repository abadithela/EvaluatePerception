# This script invokes TuLiP to construct a controller for the system with respect to a
# temporal logic specification based on the observed outputs of the perception algorithm
# Inputs to the controller synthesis function are: discrete_dynamics (disc_dynamics),
# cell/set of env. variables (env_vars), cell/set of system variables (sys_vars),
# cell/set of initial env. variables (env_init), cell/set of initial system variables (sys_init),
# enviornment and system safety and progress specifications: env_safe/env_prog and sys_safe/sys_prog.

from __future__ import print_function
import logging
from tulip import spec, synth
from tulip.dumpsmach import write_python_case
import pdb

class RoadLayout:
    def __init__(self):
        self.N=6
        
def design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog):
    logging.basicConfig(level=logging.WARNING)
    show = False

    # Constructing GR1spec from environment and systems specifications:
    specs = spec.GRSpec(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    specs.moore = True
    specs.qinit = "\E \A"

    # Synthesize
    ctrl = synth.synthesize(specs)
    assert ctrl is not None, "unrealizable"
    return ctrl


# Function constructing transition system, specification variables for pedestrian/car example:
# Takes as input the geometry of the sidewalk:
# Ncar: No. of cells of the car, Nped: No. of cells of the crosswalk, xcw: initial cell number of pedestrian, xc: initial cell of car, vc: initial velocity of car: Cell number of road at which the crosswalk starts
# vmin: Lower integer speed bound for car, vmax: Upper integer speed bound for car
def not_pedestrianK(Ncar, xc, vc, vmin, vmax, xcw):
    sys_vars = {}
    sys_vars["xc"] = (1, Ncar)
    sys_vars["vc"] = (vmin, vmax)
    env_vars = {}
    env_vars["xobj"] = (0, 1)  # Difficult to have a set of just 1

    sys_init = {"xc=" + str(xc), "vc=" + str(vc)}
    env_init = {"xobj=" + str(1)}

    # Test lines:
    sys_init = {"xc=" + str(xc), "vc=" + str(vc)}
    env_init = {"xobj=" + str(1)}

    sys_prog = set()  # For now, no need to have progress
    env_prog = set()

    sys_safe = set()
    env_safe = {"xobj=1 -> X(xobj=1)"}

    # Object controllers: If you see an object that is not a pedestrian, then keep moving:
    # spec_k = {'(xobj=1)->X(vc=1)'}
    # sys_safe |= spec_k

    for vi in range(vmax, 1, -1):
        spec_k = {"(xobj=1 && vc=" + str(vi) + ")->X(vc=" + str(vi - 1) + ")"}
        sys_safe |= spec_k
    spec_k = {"(xobj=1 && vc=1) -> X(vc=1)"}
    sys_safe |= spec_k
    # Add system dynamics to safety specs:
    for ii in range(1, Ncar + 1):
        for vi in range(vmin, vmax + 1):
            if vi == 0:
                spec_ii = {"((xc=" + str(ii) + ") && (vc=0))-> X((vc=1) && xc=" + str(ii) + ")"}
                sys_safe |= spec_ii
            elif vi == vmax:
                xf_ii = min(ii + vi, Ncar)
                spec_ii = {
                    "((xc="
                    + str(ii)
                    + ") && (vc="
                    + str(vi)
                    + "))-> X((vc="
                    + str(vi)
                    + "|| vc="
                    + str(vi - 1)
                    + ") && xc="
                    + str(xf_ii)
                    + ")"
                }
                sys_safe |= spec_ii
            else:
                xf_ii = min(ii + vi, Ncar)
                spec_ii = {
                    "((xc="
                    + str(ii)
                    + ") && (vc="
                    + str(vi)
                    + "))-> X((vc="
                    + str(vi)
                    + "|| vc="
                    + str(vi - 1)
                    + "|| vc="
                    + str(vi + 1)
                    + ") && xc="
                    + str(xf_ii)
                    + ")"
                }
                sys_safe |= spec_ii
    return env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog


def pedestrianK(Ncar, xc, vc, vmin, vmax, xcw):
    '''
    Pedestrian controller synthesis. The car needs to come to a stop 
    by xcw. That is, synthesize a controller from (xc,vc) to (xcw, 0).
    '''
    sys_vars = {}
    sys_vars["xc"] = (1, Ncar)
    sys_vars["vc"] = (vmin, vmax)
    env_vars = {}
    env_vars["xcw"] = (0, 1)  # Pedestrian is present or absent

    sys_init = {"xc=" + str(xc), "vc=" + str(vc)}
    env_init = {"xcw=" + str(xcw)}

    # Test lines:
    # sys_init = {'xc='+str(xc)}
    env_init = set()
    # ========================= #
    sys_prog = set()  # For now, no need to have progress
    env_prog = set()
    xc_jj = xcw - 1  # eventual goal location for car
    # sys_prog = set({'xc = '+str(xc_jj)})

    sys_safe = set()
    env_safe = set()

    # Environment safety specs: Static pedestrian
    for xi in range(0, 2):
        env_safe |= {"xcw=" + str(xi) + "-> X(xcw=" + str(xi) + ")"}

    # Safety specifications to specify that car must stop before pedestrian:
    sys_safe |= {"((xcw = 1) ||!(xc = " + str(xc_jj) + " && vc = 0))"}
    car_states = ""
    for xc_ii in range(xc_jj, Ncar + 1):
        if car_states == "":
            car_states = "xc = " + str(xc_ii)
        else:
            car_states = car_states + " || xc = " + str(xc_ii)
    sys_safe |= {"(!(xcw = 1)||!(" + car_states + ")||(vc = 0 && xc = " + str(xc_jj) + "))"}

    # Safety specs for car to not stop before car reaching pedestrian sidewalk
    for xi in range(1, xc_jj):
        sys_safe |= {"!(xc = " + str(xi) + " && vc = 0)"}

    # Add system dynamics to safety specs:
    for ii in range(1, Ncar + 1):
        for vi in range(vmin, vmax + 1):
            if vi == 0:
                spec_ii = {"((xc=" + str(ii) + ") && (vc=0))-> X((vc=0 || vc=1) && xc=" + str(ii) + ")"}
                sys_safe |= spec_ii
            elif vi == vmax:
                xf_ii = min(ii + vi, Ncar)
                spec_ii = {
                    "((xc="+ str(ii)+ ") && (vc="+ str(vi)+"))-> X((vc="+ str(vi)
                    + "|| vc="+ str(vi - 1)+ ") && xc="+ str(xf_ii)+ ")"
                }
                sys_safe |= spec_ii
            else:
                xf_ii = min(ii + vi, Ncar)
                spec_ii = {
                    "((xc="+ str(ii)+ ") && (vc="+ str(vi)+ "))-> X((vc=" + str(vi)+ "|| vc="
                    + str(vi - 1)+ "|| vc="+ str(vi + 1)+ ") && xc="+ str(xf_ii)+ ")"
                }
                sys_safe |= spec_ii
    return env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog


# Controller for empty observation:
def emptyK(Ncar, xc, vc, vmin, vmax, xcw):
    sys_vars = {}
    sys_vars["xc"] = (1, Ncar)
    sys_vars["vc"] = (vmin, vmax)
    env_vars = {}
    env_vars["xempty"] = (0, 1)  # Pavement is empty

    sys_init = {"xc=" + str(xc), "vc=" + str(vc)}
    env_init = {"xempty=" + str(1)}
    # Test lines:
    sys_init = {"xc=" + str(xc), "vc=" + str(vc)}
    env_init = {"xempty=" + str(1)}

    sys_prog = set()  # For now, no need to have progress
    env_prog = set()

    sys_safe = set()

    # Env safe spec: If env is empty, it always remains empty
    env_spec = {"xempty=1 -> X(xempty=1)"}
    env_safe = set()
    env_safe |= env_spec

    # Environment safety specs: Static pedestrian
    # env_safe |= {'xcw='+str(xcw)+'-> X(xcw='+str(xcw)+')'}
    # Spec: If you don't see an object, keep moving:
    spec_k = {"(xempty=1 && vc=0)->X(vc=1)"}
    sys_safe |= spec_k
    for vi in range(1, vmax):
        spec_k = {"(xempty=1 && vc=" + str(vi) + ")->X(vc=" + str(vi + 1) + ")"}
        sys_safe |= spec_k
    spec_k = {"(xempty=1 && vc=" + str(vmax) + ")->X(vc=" + str(vmax) + ")"}
    sys_safe |= spec_k

    # Add system dynamics to safety specs:
    for ii in range(1, Ncar + 1):
        for vi in range(vmin, vmax + 1):
            if vi == 0:
                spec_ii = {"((xc=" + str(ii) + ") && (vc=0))-> X((vc=1) && xc=" + str(ii) + ")"}
                sys_safe |= spec_ii
            elif vi == vmax:
                xf_ii = min(ii + vi, Ncar)
                spec_ii = {
                    "((xc="
                    + str(ii)
                    + ") && (vc="
                    + str(vi)
                    + "))-> X((vc="
                    + str(vi)
                    + "|| vc="
                    + str(vi - 1)
                    + ") && xc="
                    + str(xf_ii)
                    + ")"
                }
                sys_safe |= spec_ii
            else:
                xf_ii = min(ii + vi, Ncar)
                spec_ii = {
                    "((xc="
                    + str(ii)
                    + ") && (vc="
                    + str(vi)
                    + "))-> X((vc="
                    + str(vi)
                    + "|| vc="
                    + str(vi - 1)
                    + "|| vc="
                    + str(vi + 1)
                    + ") && xc="
                    + str(xf_ii)
                    + ")"
                }
                sys_safe |= spec_ii
    return env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog


def pretty_print_specs(spec_set, spec_name):
    print("=============================================")
    print(spec_name)
    for spec in spec_set:
        print(spec)


# +
# Design controller:
def construct_controllers(Ncar, vmin, vmax, xcw, vc):
    # Simple example of pedestrian crossing street:
    xc = 1
    # When a pedestrian is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = pedestrianK(
        Ncar, xc, vc, vmin, vmax, xcw
    )
    # pretty_print_specs(sys_safe, "sys safe")
    Kped = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("ped_controller.py", Kped)

    # When something other than a pedestrian is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = not_pedestrianK(
        Ncar, xc, vc, vmin, vmax, xcw
    )
    Knot_ped = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("obj_controller.py", Knot_ped)

    # When nothing is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = emptyK(
        Ncar, xc, vc, vmin, vmax, xcw
    )
    Kempty = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("empty_controller.py", Kempty)

    K = dict()
    K["ped"] = Kped
    K["obj"] = Knot_ped
    K["empty"] = Kempty
    return K


if __name__ == "__main__":
    # Simple example of pedestrian crossing street:
    Ncar = 6
    vmax = 2
    vmin = 0
    xc = 1
    vc = vmax
    xcw = 5

    # When a pedestrian is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = pedestrianK(
        Ncar, xc, vc, vmin, vmax, xcw
    )
    Kped = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("ped_controller.py", Kped)

    # When something other than a pedestrian is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = not_pedestrianK(
        Ncar, xc, vc, vmin, vmax, xcw
    )
    Knot_ped = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("not_ped_controller.py", Knot_ped)

    # When nothing is observed:
    env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog = emptyK(
        Ncar, xc, vc, vmin, vmax, xcw
    )
    Kempty = design_C(env_vars, sys_vars, env_init, sys_init, env_safe, sys_safe, env_prog, sys_prog)
    write_python_case("empty_controller.py", Kempty)