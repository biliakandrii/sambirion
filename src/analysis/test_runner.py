#!/usr/bin/env python3
"""
Test runner for comparing Nav2 configurations.
Supports: default, prediction_kf, prediction_ekf, prediction_ukf
Saves trajectories in separate folders for each config
"""

import subprocess
import json
import time
import signal
import sys
from datetime import datetime
from pathlib import Path
import argparse
import threading
import os

class TestRunner:
    def __init__(self, output_dir="test_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.current_processes = []
        self.results = []
        self.json_file = self.output_dir / "results.json"
    
    def cleanup_processes(self, keep_shell=True):
        """Kill all processes in current TTY except shell"""
        tty = os.ttyname(sys.stdin.fileno())
        tty_short = tty.replace("/dev/", "")
        self_pid = os.getpid()

        ps = subprocess.run(
            ["ps", "-t", tty_short, "-o", "pid=,comm="],
            capture_output=True,
            text=True
        )

        for line in ps.stdout.strip().splitlines():
            pid_str, cmd = line.strip().split(maxsplit=1)
            pid = int(pid_str)

            if pid == self_pid or cmd in ("bash", "zsh", "fish", "sh"):
                continue

            try:
                os.kill(pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass

    def run_test(self, test_name, nav2_config, estimator_type, goals_str, goal_tolerance="0.3", obstacles=None):
        """
        Run a single test
        
        Args:
            test_name: name of the test
            nav2_config: "default" or "prediction"
            estimator_type: "kf", "ekf", or "ukf" (only used if nav2_config == "prediction")
            goals_str: comma-separated goals
            goal_tolerance: goal tolerance in meters
            obstacles: list of obstacle configurations
        """
        # Build config name
        if nav2_config == "default":
            config_name = "default"
        else:
            config_name = f"prediction_{estimator_type}"
        
        print(f"\n{'='*60}")
        print(f"Test: {test_name} | Config: {config_name}")
        print(f"{'='*60}\n")
        
        test_result = {
            'test_name': test_name,
            'nav2_config': nav2_config,
            'estimator_type': estimator_type if nav2_config == "prediction" else None,
            'config_name': config_name,
            'time': None
        }
        
        try:
            # Launch Gazebo
            print("Starting Gazebo...")
            gazebo = subprocess.Popen(
                ['ros2', 'launch', 'sambirion_bringup', 'sambirion_gazebo.launch.py'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.current_processes.append(gazebo)

            # Launch Nav2
            print(f"Starting Nav2 ({nav2_config})...")
            nav2_params = f"/root/sambirion/src/sambirion_navigation/params/nav2_params{'_default' if nav2_config == 'default' else ''}.yaml"
            nav2 = subprocess.Popen(
                ['ros2', 'launch', 'sambirion_navigation', 'nav2_bringup.launch.py', f'params:={nav2_params}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            self.current_processes.append(nav2)
            
            # Launch state estimator only if using prediction config
            if nav2_config == "prediction":
                print(f"Starting state estimator ({estimator_type})...")
                state_est = subprocess.Popen(
                    ['ros2', 'launch', 'state_estimator', 'state_estimator_launch.py', f'estimator_type:={estimator_type}'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1
                )
                # self._stream_output(state_est, "state_est")
                self.current_processes.append(state_est)
            
            # Initial pose
            subprocess.run([
                'ros2', 'topic', 'pub', '--once', '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped',
                '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'
            ], capture_output=True)
            
            # Launch obstacle detector
            obs_det = subprocess.Popen(
                ['ros2', 'launch', 'obstacle_detector', 'nodes.launch.xml'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            self.current_processes.append(obs_det)
            
            subprocess.run([
                'ros2', 'topic', 'pub', '--once', '/initialpose',
                'geometry_msgs/PoseWithCovarianceStamped',
                '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'
            ], capture_output=True)
            time.sleep(1)
            
            # Launch trajectory plotter - save to config-specific folder
            trajectory_output_dir = f'/root/sambirion/plots/trajectory/{config_name}'
            print(f"Starting trajectory plotter (output: {trajectory_output_dir})...")
            traj_plotter = subprocess.Popen(
                ['ros2', 'run', 'sambirion_application', 'trajectory_plotter.py', '--ros-args',
                 '-p', 'use_sim_time:=True',
                 '-p', 'topic_name:=/odom',
                 '-p', 'max_points:=1000',
                 '-p', 'map_size:=10.0',
                 '-p', f'output_dir:={trajectory_output_dir}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            # self._stream_output(traj_plotter, "traj_plotter")
            self.current_processes.append(traj_plotter)
            
            # Wait for initialpose topic
            print("Waiting for /initialpose topic...")
            while True:
                topic_result = subprocess.run(['ros2', 'topic', 'list'], 
                                      capture_output=True, text=True)
                if '/initialpose' in topic_result.stdout:
                    break
                time.sleep(1)
            
            # Publish initial pose again
            print("Publishing initial pose...")
            for _ in range(2):
                subprocess.run([
                    'ros2', 'topic', 'pub', '--once', '/initialpose',
                    'geometry_msgs/PoseWithCovarianceStamped',
                    '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}'
                ], capture_output=True)
                time.sleep(1)

            time.sleep(5)
            
            # Launch obstacles
            if obstacles:
                print(f"Launching {len(obstacles)} obstacle(s)...")
                for obs in obstacles:
                    obs_cmd = ['ros2', 'run', 'sambirion_application', 'moving_obstacle_node.py', '--ros-args',
                               '-p', f'model_name:={obs["model_name"]}',
                               '-p', f'trajectory:={obs["trajectory"]}']
                    
                    if obs.get("linear_axis"):
                        obs_cmd.extend(['-p', f'linear_axis:={obs["linear_axis"]}'])
                    
                    obs_cmd.extend([
                        '-p', f'start_x:={obs["start_x"]}',
                        '-p', f'start_y:={obs["start_y"]}',
                        '-p', f'speed:={obs["speed"]}',
                        '-p', f'radius:={obs["radius"]}'
                    ])
                    
                    if obs.get("obstacle_radius"):
                        obs_cmd.extend(['-p', f'obstacle_radius:={obs["obstacle_radius"]}'])
                    
                    print(f"  - {obs['model_name']}: {obs['trajectory']} trajectory")
                    obs_proc = subprocess.Popen(obs_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    self.current_processes.append(obs_proc)
                
            # Run navigation
            print("Running navigation...")
            goal_proc = subprocess.Popen(
                ['ros2', 'run', 'sambirion_application', 'goal_publisher.py', '--ros-args',
                 '-p', f'goals:={goals_str}',
                 '-p', f'goal_tolerance:={goal_tolerance}'],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
            self.current_processes.append(goal_proc)
            
            # Monitor for completion
            timeout = 100
            start = time.time()
            completed = False
            output_lines = []
            output_lock = threading.Lock()

            def read_output():
                try:
                    for line in iter(goal_proc.stdout.readline, ''):
                        if not line:
                            break
                        with output_lock:
                            output_lines.append(line)
                except Exception as e:
                    print(f"Output reader error: {e}")

            reader_thread = threading.Thread(target=read_output, daemon=True)
            reader_thread.start()

            while time.time() - start < timeout:
                if goal_proc.poll() is not None:
                    time.sleep(0.5)
                    break
                
                with output_lock:
                    for line in output_lines:
                        if "Total time:" in line and test_result['time'] is None:
                            try:
                                time_str = line.split("Total time:")[1].split("seconds")[0].strip()
                                test_result['time'] = float(time_str)
                                print(f"✓ Completed in {test_result['time']:.2f}s")
                                completed = True
                                break
                            except Exception as parse_error:
                                print(f"Error parsing time: {parse_error}")
                
                if completed:
                    break
                    
                time.sleep(0.1)

            if not completed:
                with output_lock:
                    for line in output_lines:
                        if "Total time:" in line:
                            try:
                                time_str = line.split("Total time:")[1].split("seconds")[0].strip()
                                test_result['time'] = float(time_str)
                                print(f"✓ Completed in {test_result['time']:.2f}s")
                                completed = True
                                break
                            except Exception:
                                pass

            if not completed:
                print(f"✗ Test timed out after {timeout}s")
                try:
                    goal_proc.kill()
                    goal_proc.wait(timeout=2)
                except Exception:
                    pass
                
        except Exception as e:
            print(f"Error: {e}")
        
        finally:
            self.cleanup_processes()
            
        return test_result
    
    def run_test_suite(self, tests, iterations=1):
        """
        Run multiple tests with different configs
        
        Configs tested:
        - default (no prediction)
        - prediction_kf
        - prediction_ekf
        - prediction_ukf
        """
        # Define all configurations to test
        configs = [
            ('default', None),
            # ('prediction', 'kf'),
            # ('prediction', 'ekf'),
            # ('prediction', 'ukf')
        ]
        
        for test in tests:
            test_name = test['name']
            goals = test['goals']
            tolerance = test.get('tolerance', '0.4')
            obstacles = test.get('obstacles', None)
            
            for nav2_config, estimator_type in configs:
                for i in range(iterations):
                    print(f"\n>>> Iteration {i+1}/{iterations}")
                    test_result = self.run_test(
                        test_name, 
                        nav2_config, 
                        estimator_type, 
                        goals, 
                        tolerance, 
                        obstacles
                    )
                    self.results.append(test_result)
                    self._save_results()
                    
                    print("Waiting before next iteration...")
                    time.sleep(5)
        
        return self.results
    
    def _save_results(self):
        """Save results to JSON"""
        with open(self.json_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"Saved: {self.json_file}")

    def _stream_output(self, proc, name):
        """Continuously stream subprocess output to terminal"""
        def _reader():
            try:
                for line in proc.stdout:
                    print(f"[{name}] {line.rstrip()}")
            except Exception:
                pass

        t = threading.Thread(target=_reader, daemon=True)
        t.start()


def main():
    parser = argparse.ArgumentParser(description='Run Nav2 comparison tests')
    parser.add_argument('--iterations', type=int, default=3, help='Iterations per test')
    parser.add_argument('--output-dir', default='test_results', help='Output directory')
    args = parser.parse_args()
    
    # Define your tests here
    linear_tests = [
        {
            'name': 'test1',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'x',
                    'start_x': '0.0',
                    'start_y': '-2.5',
                    'speed': '0.3',
                    'radius': '0.5',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test2',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'x',
                    'start_x': '0.0',
                    'start_y': '-2.5',
                    'speed': '0.3',
                    'radius': '0.7',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test3',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'x',
                    'start_x': '0.0',
                    'start_y': '-2.5',
                    'speed': '0.3',
                    'radius': '1.0',
                    'obstacle_radius': '0.2'
                }
            ]
        }

    ]
    tests = [
        {
            'name': 'test1_linear',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'x',
                    'start_x': '0.0',
                    'start_y': '-2.5',
                    'speed': '0.3',
                    'radius': '0.5',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test2_linear',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'x',
                    'start_x': '0.0',
                    'start_y': '-2.5',
                    'speed': '0.3',
                    'radius': '0.7',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test3_linear',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'x',
                    'start_x': '0.0',
                    'start_y': '-2.5',
                    'speed': '0.3',
                    'radius': '1.0',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test1_circular_slow',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'circular',
                    'start_x': '0.5',
                    'start_y': '-2.5',
                    'speed': '0.1',
                    'radius': '1.0',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test2_circular_medium',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'circular',
                    'start_x': '0.5',
                    'start_y': '-2.5',
                    'speed': '0.3',
                    'radius': '1.0',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test3_circular_fast',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'circular',
                    'start_x': '0.8',
                    'start_y': '-2.5',
                    'speed': '0.1',
                    'radius': '2.0',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test_linear_head_on',
            'goals': '0.06,-3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'not_x',
                    'start_x': '0.0',
                    'start_y': '-4.5',
                    'speed': '0.05',
                    'radius': '10.0',
                    'obstacle_radius': '0.2'
                }
            ]
        },
         {
            'name': 'test_linear_overtake',
            'goals': '0.06,3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'linear',
                    'linear_axis': 'not_x',
                    'start_x': '0.0',
                    'start_y': '0.7',
                    'speed': '0.008',
                    'radius': '10.0',
                    'obstacle_radius': '0.2'
                }
            ]
        },
        {
            'name': 'test_nonlinear_arch',
            'goals': '0.06,3.38,0.0',
            'tolerance': '0.3',
            'obstacles': [
                {
                    'model_name': 'obs1',
                    'trajectory': 'circular',
                    'start_x': '1.8',
                    'start_y': '1.0',
                    'speed': '0.075',
                    'radius': '4.0',
                    'obstacle_radius': '0.2'
                }
            ]
        }
    ]
    
    runner = TestRunner(output_dir=args.output_dir)
    
    def signal_handler(sig, frame):
        print("\n\nInterrupted! Cleaning up...")
        runner.cleanup_processes()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f"\n{'='*60}")
    print(f"Starting test suite:")
    print(f"  Tests: {len(tests)}")
    print(f"  Configs: 4 (default, prediction_kf, prediction_ekf, prediction_ukf)")
    print(f"  Iterations: {args.iterations}")
    print(f"  Total runs: {len(tests) * 4 * args.iterations}")
    print(f"{'='*60}\n")
    
    try:
        runner.run_test_suite(tests, iterations=args.iterations)
        print("\n✓ Test suite completed!")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        runner.cleanup_processes()


if __name__ == '__main__':
    main()