#!/usr/bin/env python3
import json
from pathlib import Path
from datetime import datetime, timedelta
import matplotlib.pyplot as plt
from core.mission import DroneMission, MissionLoader
from core.conflict import ConflictDetector
from visualization.plotter import MissionVisualizer
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

def main():
    # Configuration
    SAFETY_BUFFER = 5.0  # meters
    DRONE_SPEED = 10.0    # m/s
    ANIMATION_FPS = 10     # frames per second for animation
    
    print("=== UAV Strategic Deconfliction System ===")
    
    # Load mission data
    data_dir = Path(__file__).parent.parent / 'data' / 'input'
    output_dir = Path(__file__).parent.parent / 'data' / 'output'
    output_dir.mkdir(exist_ok=True)
    
    try:
        print("\nLoading mission data...")
        primary = MissionLoader.load_primary_mission(data_dir / 'primary_mission.json')
        others = MissionLoader.load_simulated_flights(data_dir / 'simulated_flights.json')
        
        print(f"Primary mission loaded: {len(primary.waypoints)} waypoints")
        print(f"Simulated flights loaded: {len(others)} drones")
        
        # Generate trajectories
        print("\nGenerating trajectories...")
        primary.generate_trajectory(speed=DRONE_SPEED)
        for i, drone in enumerate(others):
            drone.generate_trajectory(speed=DRONE_SPEED)
            print(f"  Drone {i+1} trajectory generated")
        
        # Detect conflicts
        print("\nDetecting conflicts...")
        detector = ConflictDetector(safety_buffer=SAFETY_BUFFER)
        conflicts = detector.detect_conflicts(primary, others)
        
        # Report results
        if conflicts:
            print(f"\n🚨 Conflict detected! Found {len(conflicts)} potential conflicts.")
            for i, conflict in enumerate(conflicts[:3]):  # Show first 3 conflicts
                print(f"\nConflict {i+1}:")
                print(f"  Time: {conflict['time'].strftime('%H:%M:%S')}")
                print(f"  Location: (X: {conflict['location'][0]:.1f}m, Y: {conflict['location'][1]:.1f}m, Z: {conflict['location'][2]:.1f}m)")
                print(f"  Distance: {conflict['distance']:.2f}m")
                print(f"  With: {conflict['conflicting_drone']}")
        else:
            print("\n✅ No conflicts detected. Mission is safe to proceed.")
        
        # Generate static visualization
        print("\nGenerating visualization...")
        visualizer = MissionVisualizer()
        visualizer.plot_missions(primary, others, conflicts)
        plot_path = output_dir / 'mission_plot.png'
        plt.savefig(plot_path, dpi=300)
        print(f"Static plot saved to {plot_path}")
        
        # Generate animation
        print("\nGenerating animation... (this may take a moment)")
        animation_path = output_dir / 'mission_animation.mp4'
        visualizer.animate_missions(
            primary,
            others,
            conflicts,
            output_file=str(animation_path),
            fps=ANIMATION_FPS
        )
        print(f"Animation saved to {animation_path}")
        
        # Save conflict report
        if conflicts:
            report_path = output_dir / 'conflict_report.json'
            with open(report_path, 'w') as f:
                json.dump([{
                    'time': c['time'].isoformat(),
                    'location': c['location'],
                    'distance': c['distance'],
                    'conflicting_drone': c['conflicting_drone']
                } for c in conflicts], f, indent=2)
            print(f"\nConflict report saved to {report_path}")
        
        print("\n=== Analysis complete ===")
        
    except FileNotFoundError as e:
        print(f"\nError: {e}")
        print("Please ensure the input files exist in the data/input directory")
    except ValueError as e:
        print(f"\nError: {e}")
    except Exception as e:
        print(f"\nUnexpected error: {e}")

if __name__ == "__main__":
    main()