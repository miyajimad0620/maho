#!/usr/bin/env python3

import argparse
import math
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    import matplotlib.pyplot as plt
    import yaml
    from matplotlib.lines import Line2D
    from matplotlib.widgets import Button, Slider, TextBox
except ImportError as error:
    package = getattr(error, "name", "required package")
    sys.exit(
        f"Missing {package}. Install dependencies with "
        "'python3 -m pip install -r script/requirements.txt'."
    )


Frame = Dict[str, Any]
Point = Tuple[float, float]
COLLISION_FREE_COLOR = "#0072B2"
COLLIDING_COLOR = "#D55E00"
OBSTACLE_COLOR = "#4D4D4D"


def has_numeric_components(value: Any, components: Sequence[str]) -> bool:
    return isinstance(value, dict) and all(
        isinstance(value.get(component), (int, float)) for component in components
    )


def load_data(
    path: Path,
) -> Tuple[List[Frame], List[Point], Optional[Point]]:
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML in {path}: {error}") from error

    if not isinstance(document, dict) or not isinstance(
        document.get("position_updates"), list
    ):
        raise ValueError("YAML root must contain a 'position_updates' list")
    if not document["position_updates"]:
        raise ValueError("'position_updates' must not be empty")

    obstacle_data = document.get("obstacles", [])
    if not isinstance(obstacle_data, list):
        raise ValueError("'obstacles' must be a list")
    obstacles = []
    for obstacle in obstacle_data:
        if not has_numeric_components(obstacle, ("x", "y")):
            raise ValueError("every obstacle must have numeric x and y values")
        obstacles.append((float(obstacle["x"]), float(obstacle["y"])))

    goal_data = document.get("goal")
    goal = None
    if goal_data is not None:
        if not has_numeric_components(goal_data, ("x", "y")):
            raise ValueError("'goal' must have numeric x and y values")
        goal = float(goal_data["x"]), float(goal_data["y"])

    initialization_steps = document.get("initialization_steps", [])
    if not isinstance(initialization_steps, list):
        raise ValueError("'initialization_steps' must be a list")

    frames = []
    frame_groups = (
        (initialization_steps, "initialization_step", "initialization"),
        (
            document["position_updates"],
            "position_update_count",
            "position update",
        ),
    )
    for group, count_key, frame_type in frame_groups:
        seen_counts = set()
        for frame in group:
            if not isinstance(frame, dict) or not isinstance(
                frame.get(count_key), int
            ):
                raise ValueError(
                    f"each {frame_type} must have an integer "
                    f"'{count_key}' value"
                )
            count = frame[count_key]
            if count in seen_counts:
                raise ValueError(f"duplicate {count_key} value: {count}")
            seen_counts.add(count)

            if not has_numeric_components(
                frame.get("current_pose"), ("x", "y", "theta")
            ):
                raise ValueError(
                    f"current_pose at {frame_type} {count} must have "
                    "numeric x, y, and theta values"
                )
            if not has_numeric_components(
                frame.get("command_velocity"), ("x", "y", "theta")
            ):
                raise ValueError(
                    f"command_velocity at {frame_type} {count} must have "
                    "numeric x, y, and theta values"
                )
            if not isinstance(frame.get("node_sequences"), list):
                raise ValueError(
                    f"{frame_type} {count} must contain a "
                    "'node_sequences' list"
                )
            for node_sequence in frame["node_sequences"]:
                if (
                    not isinstance(node_sequence, dict)
                    or not isinstance(node_sequence.get("rank"), int)
                    or not isinstance(node_sequence.get("collides"), bool)
                    or not isinstance(node_sequence.get("nodes"), list)
                ):
                    raise ValueError(
                        f"every node sequence at {frame_type} {count} must "
                        "have an integer rank, a boolean collides value, "
                        "and a 'nodes' list"
                    )
                for node in node_sequence["nodes"]:
                    pose = node.get("pose") if isinstance(node, dict) else None
                    velocity = (
                        node.get("velocity")
                        if isinstance(node, dict)
                        else None
                    )
                    if not has_numeric_components(
                        pose, ("x", "y", "theta")
                    ):
                        raise ValueError(
                            f"every node at {frame_type} {count} must have "
                            "numeric pose x, y, and theta values"
                        )
                    if not has_numeric_components(
                        velocity, ("x", "y", "theta")
                    ):
                        raise ValueError(
                            f"every node at {frame_type} {count} must have "
                            "numeric velocity x, y, and theta values"
                        )
            frame["_frame_type"] = frame_type
            frame["_frame_count"] = count
            frames.append(frame)
    return frames, obstacles, goal


def frame_reference(frame: Frame) -> str:
    prefix = "init" if frame["_frame_type"] == "initialization" else "update"
    return f"{prefix}:{frame['_frame_count']}"


def current_position(frame: Frame) -> Point:
    pose = frame["current_pose"]
    return float(pose["x"]), float(pose["y"])


def axis_limits(
    frames: Sequence[Frame],
    obstacles: Sequence[Point],
    goal: Optional[Point],
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    points = [
        (node["pose"]["x"], node["pose"]["y"])
        for frame in frames
        for node_sequence in frame["node_sequences"]
        for node in node_sequence["nodes"]
        if math.isfinite(node["pose"]["x"])
        and math.isfinite(node["pose"]["y"])
    ]
    points.extend(
        point
        for point in (
            current_position(frame)
            for frame in frames
        )
        if math.isfinite(point[0]) and math.isfinite(point[1])
    )
    points.extend(
        point
        for point in obstacles
        if math.isfinite(point[0]) and math.isfinite(point[1])
    )
    if goal is not None and math.isfinite(goal[0]) and math.isfinite(goal[1]):
        points.append(goal)
    if not points:
        return (-1.0, 1.0), (-1.0, 1.0)

    xs, ys = zip(*points)
    x_padding = max((max(xs) - min(xs)) * 0.05, 0.5)
    y_padding = max((max(ys) - min(ys)) * 0.05, 0.5)
    return (
        (min(xs) - x_padding, max(xs) + x_padding),
        (min(ys) - y_padding, max(ys) + y_padding),
    )


class NodeSequenceViewer:
    def __init__(
        self,
        frames: Sequence[Frame],
        obstacles: Sequence[Point],
        goal: Optional[Point],
        source_name: str,
    ) -> None:
        self.frames = frames
        self.obstacles = obstacles
        self.goal = goal
        self.source_name = source_name
        self.frame_index_by_reference = {
            frame_reference(frame): index
            for index, frame in enumerate(frames)
        }
        self.index = 0
        self.updating_controls = False
        self.x_limits, self.y_limits = axis_limits(
            frames, obstacles, goal
        )

        self.figure, self.axes = plt.subplots(figsize=(10, 7))
        self.figure.subplots_adjust(bottom=0.22)

        previous_axes = self.figure.add_axes([0.08, 0.07, 0.08, 0.06])
        next_axes = self.figure.add_axes([0.17, 0.07, 0.08, 0.06])
        slider_axes = self.figure.add_axes([0.30, 0.085, 0.43, 0.03])
        input_axes = self.figure.add_axes([0.80, 0.07, 0.12, 0.06])

        self.previous_button = Button(previous_axes, r"$\leftarrow$")
        self.next_button = Button(next_axes, r"$\rightarrow$")
        self.slider = Slider(
            slider_axes,
            "Frame",
            0,
            max(len(frames) - 1, 1),
            valinit=0,
            valstep=1,
        )
        self.frame_input = TextBox(
            input_axes, "Go to ", initial=frame_reference(frames[0])
        )

        self.previous_button.on_clicked(
            lambda _: self.change_frame(self.index - 1)
        )
        self.next_button.on_clicked(
            lambda _: self.change_frame(self.index + 1)
        )
        self.slider.on_changed(
            lambda value: self.change_frame(round(value))
        )
        self.frame_input.on_submit(self.submit_frame)
        self.figure.canvas.mpl_connect("key_press_event", self.on_key_press)

        self.draw()

    def change_frame(self, index: int) -> None:
        if self.updating_controls:
            return
        bounded_index = min(max(index, 0), len(self.frames) - 1)
        if bounded_index == self.index:
            self.sync_controls()
            return
        self.index = bounded_index
        self.sync_controls()
        self.draw()

    def sync_controls(self) -> None:
        self.updating_controls = True
        try:
            if round(self.slider.val) != self.index:
                self.slider.set_val(self.index)
            expected_text = frame_reference(self.frames[self.index])
            if self.frame_input.text != expected_text:
                self.frame_input.set_val(expected_text)
        finally:
            self.updating_controls = False

    def submit_frame(self, text: str) -> None:
        if self.updating_controls:
            return
        try:
            reference = text.strip().lower()
            if ":" not in reference:
                reference = f"update:{int(reference)}"
            index = self.frame_index_by_reference[reference]
        except (ValueError, KeyError):
            self.sync_controls()
            return
        self.change_frame(index)

    def on_key_press(self, event: Any) -> None:
        if event.key == "left":
            self.change_frame(self.index - 1)
        elif event.key == "right":
            self.change_frame(self.index + 1)

    def draw(self) -> None:
        self.axes.clear()
        frame = self.frames[self.index]
        node_sequences = frame["node_sequences"]
        if self.obstacles:
            obstacle_xs, obstacle_ys = zip(*self.obstacles)
            self.axes.scatter(
                obstacle_xs,
                obstacle_ys,
                color=OBSTACLE_COLOR,
                s=55,
                zorder=4,
            )
        if self.goal is not None:
            self.axes.scatter(
                [self.goal[0]],
                [self.goal[1]],
                color="orange",
                marker="*",
                s=180,
                zorder=5,
            )
        drawable_indices = [
            index
            for index, node_sequence in enumerate(node_sequences)
            if node_sequence["nodes"]
        ]
        best_index = next(
            (
                index
                for index in drawable_indices
                if not node_sequences[index]["collides"]
            ),
            None,
        )

        draw_order = [index for index in drawable_indices if index != best_index]
        if best_index is not None:
            draw_order.append(best_index)
        current_x, current_y = current_position(frame)
        for index in draw_order:
            node_sequence = node_sequences[index]
            xs = [current_x] + [
                node["pose"]["x"] for node in node_sequence["nodes"]
            ]
            ys = [current_y] + [
                node["pose"]["y"] for node in node_sequence["nodes"]
            ]
            is_best = index == best_index
            self.axes.plot(
                xs,
                ys,
                color=(
                    COLLIDING_COLOR
                    if node_sequence["collides"]
                    else COLLISION_FREE_COLOR
                ),
                linewidth=2.5 if is_best else 1.0,
                alpha=1.0 if is_best else 0.55,
                marker="o" if is_best else None,
                markersize=3,
                zorder=2 if is_best else 1,
            )

        history = [
            current_position(previous_frame)
            for previous_frame in self.frames[: self.index + 1]
            if previous_frame["_frame_type"] == "position update"
        ]
        if history:
            history_xs, history_ys = zip(*history)
            self.axes.plot(
                history_xs,
                history_ys,
                color="green",
                linewidth=2.5,
                marker="o",
                markersize=4,
                zorder=3,
            )

        title = (
            f"{self.source_name} — {frame['_frame_type']} "
            f"{frame['_frame_count']}"
        )
        self.axes.set_title(title)
        self.axes.set_xlabel("x")
        self.axes.set_ylabel("y")
        self.axes.set_xlim(*self.x_limits)
        self.axes.set_ylim(*self.y_limits)
        self.axes.set_aspect("equal", adjustable="box")
        self.axes.grid(True, alpha=0.3)
        legend_handles = [
            Line2D(
                [0],
                [0],
                color=COLLISION_FREE_COLOR,
                linewidth=2.5,
                marker="o",
                markersize=3,
                label="Best collision-free node sequence",
            ),
            Line2D(
                [0],
                [0],
                color=COLLISION_FREE_COLOR,
                linewidth=1.0,
                alpha=0.55,
                label="Other collision-free node sequences",
            ),
            Line2D(
                [0],
                [0],
                color=COLLIDING_COLOR,
                linewidth=1.0,
                alpha=0.55,
                label="Colliding node sequences",
            ),
        ]
        if history:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    color="green",
                    linewidth=2.5,
                    label="Position history",
                )
            )
        if self.obstacles:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    color=OBSTACLE_COLOR,
                    marker="o",
                    linestyle="none",
                    markersize=7,
                    label="Obstacles",
                )
            )
        if self.goal is not None:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    color="orange",
                    marker="*",
                    linestyle="none",
                    markersize=12,
                    label="Goal",
                )
            )
        self.axes.legend(handles=legend_handles, loc="best")
        self.figure.canvas.draw_idle()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Interactively display node sequences from maho_example YAML output."
        )
    )
    parser.add_argument(
        "yaml_file",
        nargs="?",
        type=Path,
        default=Path("node_sequences.yaml"),
        help="YAML file to display (default: node_sequences.yaml)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        frames, obstacles, goal = load_data(args.yaml_file)
    except ValueError as error:
        sys.exit(f"Error: {error}")
    viewer = NodeSequenceViewer(
        frames, obstacles, goal, args.yaml_file.name
    )
    plt.show()
    del viewer


if __name__ == "__main__":
    main()
