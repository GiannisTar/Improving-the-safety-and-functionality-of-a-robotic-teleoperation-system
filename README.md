# Improving the Safety and Functionality of a Robotic Teleoperation System

Adding redundancy-resolution optimization, soft safety constraints, and a "learn by demonstration" capability to an existing mobile-manipulator teleoperation system built for offshore wind turbine blade maintenance. MSc thesis, Robot Systems (Advanced Robotics), University of Southern Denmark, supervised by Dr. Cheng Fang, June 2026. Part of the SENSIBLE project (Symbiotic Teleoperation for Wind Turbine Blade Maintenance).

**Note:** the source code for this project is under an NDA tied to the SENSIBLE project and isn't included in this repository. This repo covers the thesis itself and demo videos of the system in action.

## At a glance

| | |
|---|---|
| **System** | A grounded Franka Emika Panda arm as the master (local, operator-controlled), teleoperating a second Panda arm mounted on a Summit XL Steel omnidirectional mobile base as the slave (remote), fitted with a Dremel tool for sanding and a force/torque sensor at the flange |
| **Base controller** | Cartesian impedance control on the arm + velocity admittance control on the mobile base, unified through a dynamically consistent nullspace projector |
| **Added functionality** | A Quadratic Programming (QP) layer projected into the controller's nullspace, optimizing manipulability, minimizing unnecessary mobile-base motion, and enforcing joint-limit and collision-avoidance constraints — all without interfering with the operator's primary task |
| **Added capability** | Dynamic Movement Primitives (DMPs) to teach the system a sanding task from a single teleoperated demonstration and reproduce it autonomously, including a custom rhythmic DMP for the repetitive sanding motion |
| **Validated on** | The real robotic system: isolated QP optimization tests, then a full recorded sanding demonstration and DMP-based reenactment, including replay at a different speed |
| **Honest limitation** | Because the QP solution lives in the controller's nullspace, the safety constraints behave as *soft* constraints, not hard guarantees — this is measured and discussed explicitly, not glossed over |

## The problem

Teleoperated robots are a natural fit for tasks that are dangerous, remote, or expensive for humans to do directly — like inspecting and sanding offshore wind turbine blades. But a mobile-manipulator teleoperation system has more moving parts (literally) than a fixed-base one: the mobile base's own kinematic redundancy is either wasted or, worse, moves unpredictably as a side effect of the arm's impedance control, and there's no built-in mechanism to keep the arm away from its joint limits or the base away from collisions. On top of that, repetitive tasks (like sanding the same patch of blade in a circular pattern) are tedious to have a human operator repeat over and over by hand.

## Quadratic Programming for redundancy resolution

The combined arm + mobile base system has 10 degrees of freedom, of which the Cartesian impedance controller only needs 6 to fully specify the end-effector's motion — leaving 4 redundant DOF free to be used for something useful. A QP optimization problem is solved at every control step over those redundant DOF, with the solution projected into the controller's nullspace via a dynamically consistent projector (so it never fights the primary task), and added as an auxiliary joint torque.

The optimization combines several objectives and constraints:
- **Manipulability maximization** — following the manipulability gradient keeps the arm in a dexterous, singularity-avoiding configuration, moving the base as needed to support it.
- **Regularization and velocity damping** — penalizing large or jerky QP solutions for smoother, more stable behavior.
- **Mobile-base motion minimization** — the impedance controller alone tends to move the mobile base more than necessary to realize a given end-effector motion; this objective damps that out while still allowing the base to move when it's actually needed.
- **Joint limits and collision avoidance** — formulated as inequality constraints with tunable influence/safety distances, producing a velocity bound that ramps up smoothly as the arm nears a joint limit or the base nears an obstacle.

## Learning a sanding task by demonstration

A "teaching" script preprocesses a recorded teleoperation demonstration (end-effector trajectory, orientation, and force/torque data from a ROS bag) and fits it to a set of Dynamic Movement Primitives: two discrete DMPs for the approach and retreat phases, and a custom rhythmic DMP — implemented from scratch, since the library used didn't provide one — for the repetitive circular sanding motion itself, with its period estimated automatically via FFT. A fourth discrete DMP learns the demonstrated force profile so contact forces can be reproduced too. A "replay" controller then steps these DMPs online, feeding their output into the same Cartesian impedance controller used for teleoperation, and can even reproduce the task at a different speed by scaling the DMPs' temporal parameter.

## Results

The QP layer was validated in isolation first: under external disturbance, a system with the QP optimization active kept its manipulability high and its mobile base far calmer than the same system with the optimization switched off. The collision-avoidance constraint successfully stopped the mobile base short of a virtual wall — though, as discussed openly in the thesis, the stopping distance wasn't perfectly consistent between runs, because the nullspace-projected solution can't always be honored exactly; it's a soft constraint by construction, not a hard one.

The full sanding task was then demonstrated once via teleoperation and reproduced with DMPs. Position and orientation tracking reproduced the demonstration closely, including at half the original speed. Force tracking was noticeably less precise — traced back to a hysteresis effect from incompletely compensated joint friction in the arm, which let small unintended end-effector movements creep in during base motion. That diagnosis, and not just the symptom, is laid out in the thesis's discussion chapter.

## Demo videos

[QP optimization: manipulability + mobile-base motion minimization under external force](assets/qp-optimization-demo.mp4) — the isolated validation experiment comparing the controller with the QP module on vs. off.

[DMP sanding demonstration and replay](assets/dmp-sanding-replay-demo.mp4) — the teleoperated demonstration of the sanding task followed by its autonomous DMP reenactment.

## Repository structure

```
Improving the Safety and Functionality of a Robotic Teleoperation System/
├── README.md
├── docs/
│   └── Improving_the_Safety_and_Functionality_of_a_Robotic_Teleoperation_System_Thesis.pdf
└── assets/
    ├── qp-optimization-demo.mp4
    └── dmp-sanding-replay-demo.mp4
```

## Tools & technology

ROS, Franka Emika Panda (Cartesian impedance control), Summit XL Steel omnidirectional mobile base (velocity admittance control), Quadratic Programming (qpOASES-style solver, nullspace projection), Dynamic Movement Primitives (`movement_primitives` library plus a custom rhythmic DMP implementation), FFT-based period estimation, ridge regression for DMP weight fitting.

## Skills demonstrated

Whole-body redundancy resolution for a mobile manipulator via nullspace-projected QP optimization, formulating manipulability, smoothness, and safety objectives as tractable QP terms, learning-from-demonstration with Dynamic Movement Primitives (including deriving and implementing a rhythmic DMP variant not available off the shelf), systematic experimental validation on real hardware, and honest root-cause diagnosis of a real limitation (joint-friction-induced hysteresis) rather than papering over it.
