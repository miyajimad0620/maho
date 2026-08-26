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


PositionUpdate = Dict[str, Any]
Point = Tuple[float, float]


def has_numeric_components(value: Any, components: Sequence[str]) -> bool:
    return isinstance(value, dict) and all(
        isinstance(value.get(component), (int, float)) for component in components
    )


def load_data(
    path: Path,
) -> Tuple[List[PositionUpdate], List[Point], Optional[Point]]:
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

    position_updates = document["position_updates"]
    seen_position_update_counts = set()
    for position_update in position_updates:
        if not isinstance(position_update, dict) or not isinstance(
            position_update.get("position_update_count"), int
        ):
            raise ValueError(
                "each position update must have an integer "
                "'position_update_count' value"
            )
        position_update_count = position_update["position_update_count"]
        if position_update_count in seen_position_update_counts:
            raise ValueError(
                f"duplicate position_update_count value: {position_update_count}"
            )
        seen_position_update_counts.add(position_update_count)

        if not has_numeric_components(
            position_update.get("current_pose"), ("x", "y", "theta")
        ):
            raise ValueError(
                f"current_pose at position update {position_update_count} must "
                "have numeric x, y, and theta values"
            )
        if not has_numeric_components(
            position_update.get("command_velocity"), ("x", "y", "theta")
        ):
            raise ValueError(
                f"command_velocity at position update {position_update_count} "
                "must have numeric x, y, and theta values"
            )
        if not isinstance(position_update.get("node_sequences"), list):
            raise ValueError(
                f"position update {position_update_count} must contain a "
                "'node_sequences' list"
            )
        for node_sequence in position_update["node_sequences"]:
            if (
                not isinstance(node_sequence, dict)
                or not isinstance(node_sequence.get("rank"), int)
                or not isinstance(node_sequence.get("nodes"), list)
            ):
                raise ValueError(
                    f"every node sequence at position update "
                    f"{position_update_count} must have an integer rank and a "
                    "'nodes' list"
                )
            for node in node_sequence["nodes"]:
                pose = node.get("pose") if isinstance(node, dict) else None
                velocity = (
                    node.get("velocity") if isinstance(node, dict) else None
                )
                if not has_numeric_components(pose, ("x", "y", "theta")):
                    raise ValueError(
                        f"every node at position update {position_update_count} "
                        "must have numeric pose x, y, and theta values"
                    )
                if not has_numeric_components(velocity, ("x", "y", "theta")):
                    raise ValueError(
                        f"every node at position update {position_update_count} "
                        "must have numeric velocity x, y, and theta values"
                    )
    return position_updates, obstacles, goal


def current_position(position_update: PositionUpdate) -> Point:
    pose = position_update["current_pose"]
    return float(pose["x"]), float(pose["y"])


def axis_limits(
    position_updates: Sequence[PositionUpdate],
    obstacles: Sequence[Point],
    goal: Optional[Point],
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    points = [
        (node["pose"]["x"], node["pose"]["y"])
        for position_update in position_updates
        for node_sequence in position_update["node_sequences"]
        for node in node_sequence["nodes"]
        if math.isfinite(node["pose"]["x"])
        and math.isfinite(node["pose"]["y"])
    ]
    points.extend(
        point
        for point in (
            current_position(position_update)
            for position_update in position_updates
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
        position_updates: Sequence[PositionUpdate],
        obstacles: Sequence[Point],
        goal: Optional[Point],
        source_name: str,
    ) -> None:
        self.position_updates = position_updates
        self.obstacles = obstacles
        self.goal = goal
        self.source_name = source_name
        self.position_update_index_by_count = {
            position_update["position_update_count"]: index
            for index, position_update in enumerate(position_updates)
        }
        self.index = 0
        self.updating_controls = False
        self.x_limits, self.y_limits = axis_limits(
            position_updates, obstacles, goal
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
            "Position update",
            0,
            max(len(position_updates) - 1, 1),
            valinit=0,
            valstep=1,
        )
        initial_count = position_updates[0]["position_update_count"]
        self.position_update_input = TextBox(
            input_axes, "Count ", initial=str(initial_count)
        )

        self.previous_button.on_clicked(
            lambda _: self.change_position_update(self.index - 1)
        )
        self.next_button.on_clicked(
            lambda _: self.change_position_update(self.index + 1)
        )
        self.slider.on_changed(
            lambda value: self.change_position_update(round(value))
        )
        self.position_update_input.on_submit(self.submit_position_update)
        self.figure.canvas.mpl_connect("key_press_event", self.on_key_press)

        self.draw()

    def change_position_update(self, index: int) -> None:
        if self.updating_controls:
            return
        bounded_index = min(max(index, 0), len(self.position_updates) - 1)
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
            expected_text = str(
                self.position_updates[self.index]["position_update_count"]
            )
            if self.position_update_input.text != expected_text:
                self.position_update_input.set_val(expected_text)
        finally:
            self.updating_controls = False

    def submit_position_update(self, text: str) -> None:
        if self.updating_controls:
            return
        try:
            position_update_count = int(text)
            index = self.position_update_index_by_count[position_update_count]
        except (ValueError, KeyError):
            self.sync_controls()
            return
        self.change_position_update(index)

    def on_key_press(self, event: Any) -> None:
        if event.key == "left":
            self.change_position_update(self.index - 1)
        elif event.key == "right":
            self.change_position_update(self.index + 1)

    def draw(self) -> None:
        self.axes.clear()
        position_update = self.position_updates[self.index]
        node_sequences = position_update["node_sequences"]
        if self.obstacles:
            obstacle_xs, obstacle_ys = zip(*self.obstacles)
            self.axes.scatter(
                obstacle_xs,
                obstacle_ys,
                color="blue",
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
        best_index = drawable_indices[0] if drawable_indices else None

        draw_order = [index for index in drawable_indices if index != best_index]
        if best_index is not None:
            draw_order.append(best_index)
        current_x, current_y = current_position(position_update)
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
                color="red" if is_best else "black",
                linewidth=2.5 if is_best else 1.0,
                alpha=1.0 if is_best else 0.55,
                marker="o" if is_best else None,
                markersize=3,
                zorder=2 if is_best else 1,
            )

        history = [
            current_position(previous_position_update)
            for previous_position_update in self.position_updates[
                : self.index + 1
            ]
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

        position_update_count = position_update["position_update_count"]
        title = (
            f"{self.source_name} — position update {position_update_count}"
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
                color="red",
                linewidth=2.5,
                label="Best node sequence",
            ),
            Line2D(
                [0],
                [0],
                color="black",
                linewidth=1.0,
                alpha=0.55,
                label="Other node sequences",
            ),
            Line2D(
                [0],
                [0],
                color="green",
                linewidth=2.5,
                label="Position history",
            ),
        ]
        if self.obstacles:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    color="blue",
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
        position_updates, obstacles, goal = load_data(args.yaml_file)
    except ValueError as error:
        sys.exit(f"Error: {error}")
    viewer = NodeSequenceViewer(
        position_updates, obstacles, goal, args.yaml_file.name
    )
    plt.show()
    del viewer


if __name__ == "__main__":
    main()
