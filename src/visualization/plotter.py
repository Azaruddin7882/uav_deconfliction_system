import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required even if not directly used
from matplotlib.animation import FuncAnimation, FFMpegWriter
from typing import List, Dict
import numpy as np
from datetime import datetime, timedelta

from core.mission import DroneMission


class MissionVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.time_text = self.ax.text2D(0.02, 0.95, '', transform=self.ax.transAxes)

    def plot_missions(self, primary: DroneMission, others: List[DroneMission], conflicts: List[Dict] = None):
        """Plot all missions and highlight conflicts"""
        # Plot primary mission
        self._plot_single_mission(primary, color='blue', label='Primary Mission')

        # Plot other missions
        for i, mission in enumerate(others):
            self._plot_single_mission(mission, color='red', label=f'Other Drone {i+1}')

        # Highlight conflicts
        if conflicts:
            conflict_points = [c['location'] for c in conflicts]
            xs, ys, zs = zip(*conflict_points)
            self.ax.scatter(xs, ys, zs, color='yellow', s=100, label='Conflict Points', alpha=0.7)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Altitude (m)')
        self.ax.set_title('UAV Mission Deconfliction Visualization')
        self.ax.legend()
        plt.tight_layout()

    def _plot_single_mission(self, mission: DroneMission, color: str, label: str):
        """Plot a single mission trajectory"""
        if not mission.trajectory:
            mission.generate_trajectory(speed=10.0)

        xs = [wp.x for wp in mission.trajectory]
        ys = [wp.y for wp in mission.trajectory]
        zs = [wp.z for wp in mission.trajectory]

        self.ax.plot(xs, ys, zs, color=color, label=label, marker='o', markersize=4)

    def animate_missions(self, primary: DroneMission, others: List[DroneMission],
                         conflicts: List[Dict], output_file: str = None, fps: int = 10):
        """
        Create an animated visualization of the missions with time progression.
        """
        if not primary.trajectory:
            primary.generate_trajectory(speed=10.0)
        for mission in others:
            if not mission.trajectory:
                mission.generate_trajectory(speed=10.0)

        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.time_text = self.ax.text2D(0.02, 0.95, '', transform=self.ax.transAxes)

        all_missions = [primary] + others
        min_time = min(m.start_time for m in all_missions)
        max_time = max(m.end_time for m in all_missions)
        total_duration = (max_time - min_time).total_seconds()

        num_frames = int(total_duration * fps)
        time_points = [min_time + timedelta(seconds=i / fps) for i in range(num_frames)]

        # Prepare plot elements
        primary_line, = self.ax.plot([], [], [], 'b-', label='Primary Mission', linewidth=2)
        primary_dot = self.ax.scatter([], [], [], c='blue', s=100, alpha=0.8)

        other_lines = []
        other_dots = []
        for i in range(len(others)):
            line, = self.ax.plot([], [], [], 'r-', label=f'Other Drone {i + 1}', alpha=0.7)
            dot = self.ax.scatter([], [], [], c='red', s=80, alpha=0.7)
            other_lines.append(line)
            other_dots.append(dot)

        conflict_markers = self.ax.scatter([], [], [], c='yellow', s=200,
                                           label='Conflict', marker='X', alpha=0.9)

        # Axis limits
        all_waypoints = []
        for mission in all_missions:
            all_waypoints.extend([(wp.x, wp.y, wp.z) for wp in mission.waypoints])
        xs, ys, zs = zip(*all_waypoints)
        padding = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)) * 0.2
        self.ax.set_xlim(min(xs) - padding, max(xs) + padding)
        self.ax.set_ylim(min(ys) - padding, max(ys) + padding)
        self.ax.set_zlim(max(0, min(zs) - padding), max(zs) + padding)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Altitude (m)')
        self.ax.set_title('4D UAV Mission Deconfliction Animation')
        self.ax.legend()

        # Precompute drone positions
        positions = {
            'primary': [primary.get_position_at_time(t) for t in time_points]
        }
        for i, mission in enumerate(others):
            positions[f'other_{i}'] = [mission.get_position_at_time(t) for t in time_points]

        # Precompute conflict markers per frame
        active_conflicts = []
        for t in time_points:
            current_conflicts = [
                c['location'] for c in conflicts
                if abs((c['time'] - t).total_seconds()) <= 1.0 / fps
            ]
            active_conflicts.append(current_conflicts)

        def update(frame):
            current_time = time_points[frame]
            self.time_text.set_text(f'Time: {current_time.strftime("%H:%M:%S")}')

            # Primary drone
            primary_pos = positions['primary'][frame]
            if primary_pos:
                hx = [p.x for p in positions['primary'][:frame + 1] if p]
                hy = [p.y for p in positions['primary'][:frame + 1] if p]
                hz = [p.z for p in positions['primary'][:frame + 1] if p]
                primary_line.set_data(hx, hy)
                primary_line.set_3d_properties(hz)
                primary_dot._offsets3d = ([primary_pos.x], [primary_pos.y], [primary_pos.z])

            # Other drones
            for i in range(len(others)):
                pos = positions[f'other_{i}'][frame]
                if pos:
                    hx = [p.x for p in positions[f'other_{i}'][:frame + 1] if p]
                    hy = [p.y for p in positions[f'other_{i}'][:frame + 1] if p]
                    hz = [p.z for p in positions[f'other_{i}'][:frame + 1] if p]
                    other_lines[i].set_data(hx, hy)
                    other_lines[i].set_3d_properties(hz)
                    other_dots[i]._offsets3d = ([pos.x], [pos.y], [pos.z])

            # Conflicts
            conflicts_now = active_conflicts[frame]
            if conflicts_now:
                cx, cy, cz = zip(*conflicts_now)
                conflict_markers._offsets3d = (cx, cy, cz)
            else:
                conflict_markers._offsets3d = ([], [], [])

            return [primary_line, primary_dot] + other_lines + other_dots + [conflict_markers, self.time_text]

        ani = FuncAnimation(self.fig, update, frames=len(time_points), interval=1000 / fps, blit=True)

        if output_file:
            writer = FFMpegWriter(fps=fps, metadata=dict(artist='UAV Simulation'), bitrate=1800)
            ani.save(output_file, writer=writer, dpi=300)
            plt.close()
            print(f"\n🎥 Video generated successfully and saved to {output_file}")
        else:
            plt.tight_layout()
            plt.show()

        return ani
