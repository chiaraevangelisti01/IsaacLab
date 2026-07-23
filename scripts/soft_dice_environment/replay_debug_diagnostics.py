from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np
import torch


def _to_numpy(value):
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    return np.asarray(value)


def _quat_abs_dot_xyzw(q1, q2):
    q1 = np.asarray(q1, dtype=np.float64).reshape(4)
    q2 = np.asarray(q2, dtype=np.float64).reshape(4)
    n1 = np.linalg.norm(q1)
    n2 = np.linalg.norm(q2)
    if n1 < 1e-12 or n2 < 1e-12:
        return np.nan
    return float(abs(np.dot(q1 / n1, q2 / n2)))


class ReplayDiagnostics:
    """All-or-nothing replay diagnostics.

    When disabled, no plot arrays are allocated and all methods return
    immediately. When enabled, tracking, periodic prints, geometry checks,
    and all plots are active.
    """

    def __init__(self, enabled: bool, print_every: int = 60):
        self.enabled = bool(enabled)
        self.print_every = int(print_every)
        if self.print_every <= 0:
            raise ValueError("print_every must be greater than zero")

        if not self.enabled:
            self.times = None
            self.frames = None
            self.q_raw = None
            self.q_desired = None
            self.q_actual = None
            self.left_ee_commanded = None
            self.left_ee_actual = None
            self.right_ee_commanded = None
            self.right_ee_actual = None
            self.cube_rows = None
            return

        self.times = []
        self.frames = []
        self.q_raw = []
        self.q_desired = []
        self.q_actual = []
        self.left_ee_commanded = []
        self.left_ee_actual = []
        self.right_ee_commanded = []
        self.right_ee_actual = []
        self.cube_rows = []

    def should_print(self, step: int) -> bool:
        return self.enabled and step % self.print_every == 0

    def print_deformable_geometry(
        self,
        cube,
        label: str,
        desired_center_np=None,
        table_top_z=None,
        cube_size=None,
        cube_scale_z=1.0,
    ):
        if not self.enabled:
            return

        nodal_pos = cube.data.nodal_pos_w.torch[0].detach()
        root_pos = cube.data.root_pos_w.torch[0].detach()
        bbox_min = nodal_pos.amin(dim=0)
        bbox_max = nodal_pos.amax(dim=0)
        bbox_center = 0.5 * (bbox_min + bbox_max)
        bbox_extent = bbox_max - bbox_min

        print(f"\n[DeformableCubeGeomDebug] {label}")
        print(f"  root_pos_w mean nodes: {root_pos.cpu().numpy()}")
        print(f"  bbox_min_w:            {bbox_min.cpu().numpy()}")
        print(f"  bbox_max_w:            {bbox_max.cpu().numpy()}")
        print(f"  bbox_center_w:         {bbox_center.cpu().numpy()}")
        print(f"  bbox_extent_xyz:       {bbox_extent.cpu().numpy()}")

        if desired_center_np is not None:
            desired_center = torch.as_tensor(
                desired_center_np,
                dtype=torch.float32,
                device=nodal_pos.device,
            )
            print(f"  desired_center_w:      {desired_center.cpu().numpy()}")
            print(f"  root_minus_desired:    {(root_pos - desired_center).cpu().numpy()}")
            print(f"  bbox_center-desired:   {(bbox_center - desired_center).cpu().numpy()}")

        if table_top_z is not None:
            gap = float(bbox_min[2].cpu().item() - table_top_z)
            print(f"  table_top_z:           {table_top_z:.6f}")
            if cube_size is not None:
                expected_bottom = float(root_pos[2].cpu().item()) - float(cube_size) * float(cube_scale_z) / 2.0
                print(f"  expected_cube_bottom_z:{expected_bottom: .6f}")
            print(f"  bbox_min_z-table_top:  {gap:.6f} m")

    def record_tracking(
        self,
        sim_time,
        frame,
        q_raw,
        q_des,
        q_actual,
        left_ee_commanded,
        left_ee_actual,
        right_ee_commanded,
        right_ee_actual,
    ):
        if not self.enabled:
            return

        self.times.append(float(sim_time))
        self.frames.append(int(frame))
        self.q_raw.append(_to_numpy(q_raw).copy())
        self.q_desired.append(_to_numpy(q_des).copy())
        self.q_actual.append(_to_numpy(q_actual).copy())
        self.left_ee_commanded.append(_to_numpy(left_ee_commanded).copy())
        self.left_ee_actual.append(_to_numpy(left_ee_actual).copy())
        self.right_ee_commanded.append(_to_numpy(right_ee_commanded).copy())
        self.right_ee_actual.append(_to_numpy(right_ee_actual).copy())

    def record_cube(
        self,
        step,
        frame,
        desired_cube_pos,
        actual_cube_pos,
        desired_cube_quat_xyzw,
        reference_cube_quat_xyzw,
    ):
        if not self.enabled:
            return

        desired = np.asarray(desired_cube_pos, dtype=np.float64)
        actual = _to_numpy(actual_cube_pos).astype(np.float64)
        reference_quat = _to_numpy(reference_cube_quat_xyzw).astype(np.float64)
        desired_quat = np.asarray(desired_cube_quat_xyzw, dtype=np.float64)
        error = actual - desired
        error_norm = float(np.linalg.norm(error))

        self.cube_rows.append(
            {
                "frame": int(frame),
                "desired_cube_x": float(desired[0]),
                "desired_cube_y": float(desired[1]),
                "desired_cube_z": float(desired[2]),
                "actual_cube_x": float(actual[0]),
                "actual_cube_y": float(actual[1]),
                "actual_cube_z": float(actual[2]),
                "cube_pos_err_norm": error_norm,
            }
        )

        if self.should_print(step):
            quat_dot = _quat_abs_dot_xyzw(reference_quat, desired_quat)
            print(
                f"[CubeReplayDebug] step={step}, frame={frame} | "
                f"desired={desired} actual={actual} error={error} | "
                f"norm={error_norm:.6f} m | reference_quat_dot={quat_dot:.6f}"
            )

    def finalize(self, robot_joint_names):
        if not self.enabled or not self.times:
            return

        self._show_upper_body_tracking_plot(robot_joint_names)
        self._show_fake_ee_plot()
        self._show_cube_plot()
        plt.show()

    @staticmethod
    def _upper_body_joint_ids():
        holosoma_names = [
            "left_hip_yaw_joint", "left_hip_roll_joint", "left_hip_pitch_joint",
            "left_knee_joint", "left_ankle_joint", "right_hip_yaw_joint",
            "right_hip_roll_joint", "right_hip_pitch_joint", "right_knee_joint",
            "right_ankle_joint", "torso_joint", "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
        ]
        mapping = [0, 5, 10, 1, 6, 11, 15, 2, 7, 12, 16, 3, 8, 13, 17, 4, 9, 14, 18]
        upper_start = holosoma_names.index("torso_joint")
        ids, names = [], []
        for isaac_id, holosoma_id in enumerate(mapping):
            if holosoma_id >= upper_start:
                ids.append(isaac_id)
                names.append(holosoma_names[holosoma_id])
        return ids, names

    def _show_upper_body_tracking_plot(self, robot_joint_names):
        times = np.asarray(self.times)
        q_raw = np.asarray(self.q_raw)
        q_des = np.asarray(self.q_desired)
        q_actual = np.asarray(self.q_actual)
        upper_ids, upper_names = self._upper_body_joint_ids()
        error = q_actual[:, upper_ids] - q_des[:, upper_ids]

        print("\n[Upper-body tracking error]")
        for plot_id, isaac_id in enumerate(upper_ids):
            print(
                f"  {robot_joint_names[isaac_id]:30s} "
                f"mean_abs_error={np.abs(error[:, plot_id]).mean():.6f} rad, "
                f"max_abs_error={np.abs(error[:, plot_id]).max():.6f} rad"
            )

        ncols = 3
        nrows = int(np.ceil(len(upper_ids) / ncols))
        fig, axes = plt.subplots(nrows, ncols, figsize=(5.5 * ncols, 3.2 * nrows), sharex=True)
        axes = np.asarray(axes).reshape(-1)

        for plot_id, isaac_id in enumerate(upper_ids):
            ax = axes[plot_id]
            ax.plot(times, q_des[:, isaac_id], label="desired after clamp")
            ax.plot(times, q_actual[:, isaac_id], "--", label="actual")
            ax.plot(times, q_raw[:, isaac_id], ":", alpha=0.8, label="desired before clamp")
            ax.set_title(f"{robot_joint_names[isaac_id]}\nfrom {upper_names[plot_id]}")
            ax.set_xlabel("time [s]")
            ax.set_ylabel("joint position [rad]")
            ax.grid(True)

        for index in range(len(upper_ids), len(axes)):
            axes[index].axis("off")

        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc="upper center", ncol=3)
        fig.suptitle("Upper-body joint tracking: desired vs actual", y=0.995)
        fig.tight_layout(rect=[0, 0, 1, 0.93])

    def _show_fake_ee_plot(self):
        frames = np.asarray(self.frames)
        times = np.asarray(self.times)
        left_cmd = np.asarray(self.left_ee_commanded)
        left_actual = np.asarray(self.left_ee_actual)
        right_cmd = np.asarray(self.right_ee_commanded)
        right_actual = np.asarray(self.right_ee_actual)
        left_error = left_actual - left_cmd
        right_error = right_actual - right_cmd
        left_norm = np.linalg.norm(left_error, axis=1)
        right_norm = np.linalg.norm(right_error, axis=1)
        left_peak = int(np.argmax(left_norm))
        right_peak = int(np.argmax(right_norm))

        print("\n[Fake EE commanded-vs-actual task-space error]")
        print(
            f"  left fake EE:  mean={left_norm.mean():.6f} m, max={left_norm.max():.6f} m "
            f"at frame={frames[left_peak]}, time={times[left_peak]:.6f} s"
        )
        print(
            f"  right fake EE: mean={right_norm.mean():.6f} m, max={right_norm.max():.6f} m "
            f"at frame={frames[right_peak]}, time={times[right_peak]:.6f} s"
        )

        fig, axes = plt.subplots(3, 3, figsize=(16, 9), sharex=True)
        for index, label in enumerate(("x", "y", "z")):
            axes[0, index].plot(frames, left_cmd[:, index], label="commanded/reference")
            axes[0, index].plot(frames, left_actual[:, index], "--", label="actual/executed")
            axes[0, index].axvline(frames[left_peak], color="red", linestyle=":")
            axes[0, index].set_title(f"left fake EE {label}")
            axes[0, index].grid(True)
            axes[0, index].legend()

            axes[1, index].plot(frames, right_cmd[:, index], label="commanded/reference")
            axes[1, index].plot(frames, right_actual[:, index], "--", label="actual/executed")
            axes[1, index].axvline(frames[right_peak], color="red", linestyle=":")
            axes[1, index].set_title(f"right fake EE {label}")
            axes[1, index].grid(True)
            axes[1, index].legend()

            axes[2, index].plot(frames, left_error[:, index], label="left actual - commanded")
            axes[2, index].plot(frames, right_error[:, index], "--", label="right actual - commanded")
            axes[2, index].set_title(f"fake EE tracking error {label}")
            axes[2, index].set_xlabel("frame number")
            axes[2, index].grid(True)
            axes[2, index].legend()

        fig.suptitle(
            "Fake MuJoCo end-effector tracking: commanded/reference vs actual/executed",
            y=0.995,
        )
        fig.tight_layout(rect=[0, 0, 1, 0.94])

    def _show_cube_plot(self):
        if not self.cube_rows:
            return

        frames = np.asarray([row["frame"] for row in self.cube_rows])
        fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)

        for index, label in enumerate(("x", "y", "z")):
            axes[index].plot(
                frames,
                [row[f"desired_cube_{label}"] for row in self.cube_rows],
                label=f"Holosoma desired cube {label}",
            )
            axes[index].plot(
                frames,
                [row[f"actual_cube_{label}"] for row in self.cube_rows],
                "--",
                label=f"Isaac actual cube {label}",
            )
            axes[index].set_ylabel(f"{label} [m]")
            axes[index].grid(True)
            axes[index].legend()

        axes[3].plot(
            frames,
            [row["cube_pos_err_norm"] for row in self.cube_rows],
            label="actual - desired norm",
        )
        axes[3].set_ylabel("error [m]")
        axes[3].set_xlabel("frame number")
        axes[3].grid(True)
        axes[3].legend()
        fig.suptitle("Cube trajectory: Holosoma desired vs Isaac deformable cube")
        fig.tight_layout()

