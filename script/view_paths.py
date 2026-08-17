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


Step = Dict[str, Any]
Point = Tuple[float, float]


def load_data(path: Path) -> Tuple[List[Step], List[Point], Optional[Point]]:
    try:
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML in {path}: {error}") from error

    if not isinstance(document, dict) or not isinstance(document.get("steps"), list):
        raise ValueError("YAML root must contain a 'steps' list")
    if not document["steps"]:
        raise ValueError("'steps' must not be empty")

    obstacle_data = document.get("obstacles", [])
    if not isinstance(obstacle_data, list):
        raise ValueError("'obstacles' must be a list")
    obstacles = []
    for obstacle in obstacle_data:
        if not isinstance(obstacle, dict) or not all(
            isinstance(obstacle.get(axis), (int, float)) for axis in ("x", "y")
        ):
            raise ValueError("every obstacle must have numeric x and y values")
        obstacles.append((float(obstacle["x"]), float(obstacle["y"])))

    goal_data = document.get("goal")
    goal = None
    if goal_data is not None:
        if not isinstance(goal_data, dict) or not all(
            isinstance(goal_data.get(axis), (int, float)) for axis in ("x", "y")
        ):
            raise ValueError("'goal' must have numeric x and y values")
        goal = float(goal_data["x"]), float(goal_data["y"])

    steps = document["steps"]
    seen_step_numbers = set()
    for step in steps:
        if not isinstance(step, dict) or not isinstance(step.get("step"), int):
            raise ValueError("each step must have an integer 'step' value")
        if step["step"] in seen_step_numbers:
            raise ValueError(f"duplicate step value: {step['step']}")
        seen_step_numbers.add(step["step"])
        current_node = step.get("current_node")
        if current_node is not None:
            current_pose = (
                current_node.get("pose") if isinstance(current_node, dict) else None
            )
            if not isinstance(current_pose, dict) or not all(
                isinstance(current_pose.get(axis), (int, float))
                for axis in ("x", "y")
            ):
                raise ValueError(
                    f"current_node in step {step['step']} must have numeric pose.x and pose.y"
                )
        if not isinstance(step.get("paths"), list):
            raise ValueError(f"step {step['step']} must contain a 'paths' list")
        for path_data in step["paths"]:
            if not isinstance(path_data, dict) or not isinstance(
                path_data.get("nodes"), list
            ):
                raise ValueError(
                    f"every path in step {step['step']} must contain a 'nodes' list"
                )
            for node in path_data["nodes"]:
                pose = node.get("pose") if isinstance(node, dict) else None
                if not isinstance(pose, dict) or not all(
                    isinstance(pose.get(axis), (int, float)) for axis in ("x", "y")
                ):
                    raise ValueError(
                        f"every node in step {step['step']} must have numeric pose.x and pose.y"
                    )
    return steps, obstacles, goal


def current_position(step: Step) -> Optional[Point]:
    current_node = step.get("current_node")
    if isinstance(current_node, dict) and isinstance(current_node.get("pose"), dict):
        pose = current_node["pose"]
        return float(pose["x"]), float(pose["y"])
    for path_data in step["paths"]:
        if path_data["nodes"]:
            pose = path_data["nodes"][0]["pose"]
            return float(pose["x"]), float(pose["y"])
    return None


def axis_limits(
    steps: Sequence[Step], obstacles: Sequence[Point], goal: Optional[Point]
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    points = [
        (node["pose"]["x"], node["pose"]["y"])
        for step in steps
        for path_data in step["paths"]
        for node in path_data["nodes"]
        if math.isfinite(node["pose"]["x"]) and math.isfinite(node["pose"]["y"])
    ]
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


def cost(path_data: Dict[str, Any]) -> float:
    value = path_data.get("cost")
    return float(value) if isinstance(value, (int, float)) else math.inf


class PathViewer:
    def __init__(
        self,
        steps: Sequence[Step],
        obstacles: Sequence[Point],
        goal: Optional[Point],
        source_name: str,
    ) -> None:
        self.steps = steps
        self.obstacles = obstacles
        self.goal = goal
        self.source_name = source_name
        self.step_index_by_number = {
            step["step"]: index for index, step in enumerate(steps)
        }
        self.index = 0
        self.updating_controls = False
        self.x_limits, self.y_limits = axis_limits(steps, obstacles, goal)

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
            "Step",
            0,
            max(len(steps) - 1, 1),
            valinit=0,
            valstep=1,
        )
        self.step_input = TextBox(input_axes, "Step ", initial=str(steps[0]["step"]))

        self.previous_button.on_clicked(lambda _: self.change_step(self.index - 1))
        self.next_button.on_clicked(lambda _: self.change_step(self.index + 1))
        self.slider.on_changed(lambda value: self.change_step(round(value)))
        self.step_input.on_submit(self.submit_step)
        self.figure.canvas.mpl_connect("key_press_event", self.on_key_press)

        self.draw()

    def change_step(self, index: int) -> None:
        if self.updating_controls:
            return
        bounded_index = min(max(index, 0), len(self.steps) - 1)
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
            expected_text = str(self.steps[self.index]["step"])
            if self.step_input.text != expected_text:
                self.step_input.set_val(expected_text)
        finally:
            self.updating_controls = False

    def submit_step(self, text: str) -> None:
        if self.updating_controls:
            return
        try:
            step_number = int(text)
            index = self.step_index_by_number[step_number]
        except (ValueError, KeyError):
            self.sync_controls()
            return
        self.change_step(index)

    def on_key_press(self, event: Any) -> None:
        if event.key == "left":
            self.change_step(self.index - 1)
        elif event.key == "right":
            self.change_step(self.index + 1)

    def draw(self) -> None:
        self.axes.clear()
        step = self.steps[self.index]
        paths = step["paths"]
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
            index for index, path_data in enumerate(paths) if path_data["nodes"]
        ]
        best_index = (
            min(drawable_indices, key=lambda index: cost(paths[index]))
            if drawable_indices
            else None
        )

        draw_order = [index for index in drawable_indices if index != best_index]
        if best_index is not None:
            draw_order.append(best_index)
        for index in draw_order:
            path_data = paths[index]
            xs = [node["pose"]["x"] for node in path_data["nodes"]]
            ys = [node["pose"]["y"] for node in path_data["nodes"]]
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
            position
            for position in (
                current_position(previous_step)
                for previous_step in self.steps[: self.index + 1]
            )
            if position is not None
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

        title = f"{self.source_name} — step {step['step']}"
        if best_index is not None:
            title += f" — best cost: {cost(paths[best_index]):.6g}"
        self.axes.set_title(title)
        self.axes.set_xlabel("x")
        self.axes.set_ylabel("y")
        self.axes.set_xlim(*self.x_limits)
        self.axes.set_ylim(*self.y_limits)
        self.axes.set_aspect("equal", adjustable="box")
        self.axes.grid(True, alpha=0.3)
        legend_handles = [
            Line2D([0], [0], color="red", linewidth=2.5, label="Best path"),
            Line2D(
                [0],
                [0],
                color="black",
                linewidth=1.0,
                alpha=0.55,
                label="Other paths",
            ),
            Line2D([0], [0], color="green", linewidth=2.5, label="History"),
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
        description="Interactively display paths from maho_example YAML output."
    )
    parser.add_argument(
        "yaml_file",
        nargs="?",
        type=Path,
        default=Path("paths.yaml"),
        help="YAML file to display (default: paths.yaml)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        steps, obstacles, goal = load_data(args.yaml_file)
    except ValueError as error:
        sys.exit(f"Error: {error}")
    viewer = PathViewer(steps, obstacles, goal, args.yaml_file.name)
    plt.show()
    del viewer


if __name__ == "__main__":
    main()
