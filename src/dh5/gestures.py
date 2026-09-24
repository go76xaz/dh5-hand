"""Hand poses as data, and one runner that executes any of them.

Every gesture in the original library was the same algorithm written out
again by hand: check that some axes are far enough open, interpolate a
width, then send one simultaneous move. Here that algorithm lives once, in
`perform()`, and each gesture is a table entry.

Adding a gesture means adding an entry to `GESTURES` - no new function, no
new CLI branch, no new ROS2 service handler. Callers that enumerate
`GESTURES` pick it up automatically.
"""

import logging
import time
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Pose:
    """One simultaneous move, plus the preconditions it needs.

    fixed
        Axes that always go to the same percentage.
    ramp
        Axes whose target slides with `width`: {axis: (at_width_0,
        at_width_max)}, linearly interpolated.
    require_above
        Axes that must CURRENTLY sit strictly above their target before the
        move is allowed. This is the guard that stops a finger from being
        driven into an object it is already resting against.
    prepare
        Axes to move individually first, each only if it currently sits
        below the given target. Used where a pose needs one finger out of
        the way before the rest can move together.
    stagger
        Axes moved individually with staggered start times rather than as
        one batch: (axis, target_percent, delay) tuples, `delay` being
        seconds after this pose starts. Each move is issued non-blocking,
        so a later entry can start while an earlier one is still
        travelling. Mutually exclusive with `fixed`/`ramp` on the same
        pose.
    pause_after
        Seconds to sleep once the move completes.
    """

    fixed: Mapping[int, float] = field(default_factory=dict)
    ramp: Mapping[int, Tuple[float, float]] = field(default_factory=dict)
    require_above: Tuple[int, ...] = ()
    prepare: Tuple[Tuple[int, float], ...] = ()
    stagger: Tuple[Tuple[int, float, float], ...] = ()
    pause_after: float = 0.0

    def targets(self, width: float = 0.0, max_width: float = 0.0) -> Dict[int, float]:
        """Resolve this pose to {axis: percent} for a given width."""
        targets = dict(self.fixed)
        if self.ramp:
            fraction = 0.0 if max_width <= 0 else min(1.0, max(0.0, width / max_width))
            for axis, (low, high) in self.ramp.items():
                targets[axis] = low + (high - low) * fraction
        return targets


@dataclass(frozen=True)
class Gesture:
    """A named sequence of poses, optionally in several variants.

    ros_service
        Whether the ROS2 controller node should expose this gesture as a
        service. False keeps it terminal/CLI-only - `dh5.cli` registers
        every entry in `GESTURES` regardless of this flag.
    """

    name: str
    summary: str
    variants: Mapping[str, Tuple[Pose, ...]]
    max_width: float = 0.0
    default_variant: str = "default"
    ros_service: bool = True

    @property
    def is_scalable(self) -> bool:
        return self.max_width > 0

    @property
    def variant_names(self) -> Tuple[str, ...]:
        return tuple(self.variants)

    def poses(self, variant: Optional[str] = None) -> Tuple[Pose, ...]:
        variant = variant or self.default_variant
        if variant not in self.variants:
            raise ValueError(
                f"Unknown variant {variant!r} for {self.name!r}; "
                f"available: {', '.join(self.variant_names)}"
            )
        return self.variants[variant]


GESTURES: Dict[str, Gesture] = {
    "open_hand": Gesture(
        name="open_hand",
        summary="Move all six axes to 99% (just short of the end stop) simultaneously.",
        variants={"default": (Pose(fixed={axis: 99 for axis in range(1, 7)}),)},
    ),
    "wink": Gesture(
        name="wink",
        summary="Thumb axes out, then the four finger axes dip to 30% and return.",
        variants={"default": (
            Pose(fixed={1: 99, 6: 99}, pause_after=1.0),
            Pose(fixed={axis: 30 for axis in (2, 3, 4, 5)}),
            Pose(fixed={axis: 99 for axis in (2, 3, 4, 5)}),
        )},
    ),
    "wink2": Gesture(
        name="wink2",
        summary=(
            "Thumb axes held at 99%; axes 2-5 dip to 20% and back, each "
            "starting 0.25s after the previous one, overlapping in flight. "
            "Terminal/CLI only, not exposed as a ROS2 service."
        ),
        ros_service=False,
        variants={"default": (
            Pose(fixed={1:99, 2:99, 3:99, 4:99, 5:99, 6:99}),
            Pose(stagger=(
                            (2, 20, 0.0), (3, 20, 0.25), (4, 20, 0.5), (5, 20, 0.75),
                            (2, 99, 1.0), (3, 99, 1.25), (4, 99, 1.5), (5, 99, 1.75),
            )),
        )},
    ),
    "point": Gesture(
        name="point",
        summary="Index finger extended, the rest closed.",
        variants={"default": (
            Pose(
                fixed={1: 40, 2: 99, 3: 0, 4: 0, 5: 0, 6: 0},
                # Axis 1 is cleared out of the way first, then the index
                # finger is fully extended, before everything moves as one.
                # The (6, 0) entry never fires - nothing can sit below 0% -
                # but it is kept because the original sequence listed it and
                # removing it would silently change the documented order.
                prepare=((1, 40), (6, 0), (2, 100)),
            ),
        )},
    ),
    "round_grip": Gesture(
        name="round_grip",
        summary="Rounded power grip; every axis must already be more open than its target.",
        variants={"default": (
            Pose(
                fixed={1: 20, 2: 58, 3: 58, 4: 58, 5: 58, 6: 20},
                require_above=(1, 2, 3, 4, 5, 6),
            ),
        )},
    ),
    "two_finger_pinch": Gesture(
        name="two_finger_pinch",
        summary="Pinch between the thumb and one finger; width 0-25 sets the opening.",
        max_width=25,
        default_variant="axis2",
        variants={
            # Thumb against the index finger.
            "axis2": (Pose(
                fixed={1: 50, 3: 100, 4: 100, 5: 100},
                ramp={2: (58, 83), 6: (25, 50)},
                require_above=(2, 6),
            ),),
            # Thumb against the middle finger.
            "axis3": (Pose(
                fixed={1: 0, 2: 100, 4: 100, 5: 100},
                ramp={3: (54, 79), 6: (18, 43)},
                require_above=(3, 6),
            ),),
        },
    ),
}


def get(name: str) -> Gesture:
    """Look up a gesture by name, with a helpful error if it is unknown."""
    try:
        return GESTURES[name]
    except KeyError:
        raise ValueError(
            f"Unknown gesture {name!r}; available: {', '.join(sorted(GESTURES))}"
        ) from None


def perform(
    hand,
    name: str,
    width: float = 0.0,
    variant: Optional[str] = None,
    wait: bool = True,
    poll_interval: Optional[float] = None,
) -> Optional[List]:
    """Run a gesture on `hand`.

    Returns the list of `MoveResult`s, one per pose, or None if a pose's
    `require_above` precondition was not met - matching how the original
    gesture functions refused to move and returned None. After
    `hand.stop()` the remaining poses are skipped and the results so far
    are returned; check `hand.stop_requested` to tell the two apart.
    """
    gesture = get(name)
    poses = gesture.poses(variant)

    if width and not gesture.is_scalable:
        raise ValueError(f"Gesture {name!r} takes no width.")
    if not (0 <= width <= max(gesture.max_width, 0)):
        raise ValueError(f"width must be 0-{gesture.max_width:g} for {name!r}, got {width}.")

    results = []
    for index, pose in enumerate(poses, start=1):
        if hand.stop_requested:
            logger.info("%s: stopped before pose %d/%d.", name, index, len(poses))
            return results
        targets = pose.targets(width, gesture.max_width)

        if not _preconditions_met(hand, name, pose, targets):
            return None

        for axis, target in pose.prepare:
            if hand.position_percent(axis) < target:
                hand.move_axis(axis, target, wait=True, poll_interval=poll_interval)

        if pose.stagger:
            logger.info("%s: pose %d/%d -> staggered %s", name, index, len(poses), pose.stagger)
            results.append(_run_stagger(hand, pose.stagger, wait=wait, poll_interval=poll_interval))
        else:
            logger.info("%s: pose %d/%d -> %s", name, index, len(poses),
                        {axis: round(value, 1) for axis, value in targets.items()})
            results.append(hand.move(targets, wait=wait, poll_interval=poll_interval))

        if pose.pause_after:
            time.sleep(pose.pause_after)

    return results


def _run_stagger(
    hand,
    stagger: Tuple[Tuple[int, float, float], ...],
    wait: bool,
    poll_interval: Optional[float],
) -> List:
    """Issue each (axis, target, delay) at its scheduled offset from now,
    without waiting for earlier moves to finish - so later axes start
    while earlier ones are still travelling."""
    start = time.monotonic()
    results = []
    for axis, target, delay in stagger:
        remaining = delay - (time.monotonic() - start)
        if remaining > 0:
            time.sleep(remaining)
        if hand.stop_requested:
            return results
        results.append(hand.move_axis(axis, target, wait=False, poll_interval=poll_interval))

    if wait:
        # Later entries for the same axis overwrite earlier ones, leaving
        # each axis's final target.
        targets = {axis: position for move in results for axis, position in move.positions.items()}
        hand.wait_for_axes(set(targets), poll_interval, targets=targets)

    return results


def _preconditions_met(hand, name: str, pose: Pose, targets: Mapping[int, float]) -> bool:
    for axis in pose.require_above:
        target = targets[axis]
        current = hand.position_percent(axis)
        if current <= target:
            logger.error(
                "Cannot execute %s: axis %d is at %.1f%% and must be above %.1f%% first.",
                name, axis, current, target,
            )
            return False
    return True


def describe(gesture: Gesture) -> str:
    """One CLI-ready help line for a gesture."""
    extras = []
    if gesture.is_scalable:
        extras.append(f"width 0-{gesture.max_width:g}")
    if len(gesture.variants) > 1:
        extras.append(f"variants: {', '.join(gesture.variant_names)} (default {gesture.default_variant})")
    suffix = f" [{'; '.join(extras)}]" if extras else ""
    return f"{gesture.summary}{suffix}"
