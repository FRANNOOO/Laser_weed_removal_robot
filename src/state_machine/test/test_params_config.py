"""Tests for centralized configuration file params.yaml and launch integration."""

import importlib.util
import os

from launch import LaunchDescription
import yaml


def _load_launch_module(relative_path: str):
    base_dir = os.path.dirname(os.path.dirname(__file__))
    full_path = os.path.join(base_dir, relative_path)
    spec = importlib.util.spec_from_file_location('launch_module', full_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_params_yaml_exists_and_parses():
    """Verify that params.yaml exists and contains required sections and parameters."""
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'config'
    )
    params_file = os.path.join(config_dir, 'params.yaml')
    assert os.path.isfile(params_file), f'params.yaml not found at {params_file}'

    with open(params_file, 'r') as f:
        data = yaml.safe_load(f)

    # Check top-level namespaces
    assert '/**' in data, 'Shared /** namespace missing in params.yaml'
    assert 'state_machine_node' in data, 'state_machine_node namespace missing in params.yaml'
    assert 'nav_integration_node' in data, 'nav_integration_node namespace missing in params.yaml'

    # Check shared parameters
    shared = data['/**']['ros__parameters']
    assert 'workspace_min_x' in shared
    assert 'workspace_max_x' in shared
    assert 'workspace_min_y' in shared
    assert 'workspace_max_y' in shared
    assert 'dedup_radius' in shared
    assert 'robot_base_frame' in shared

    # Check state_machine_node parameters (specifically arm movement time requested by user)
    sm_params = data['state_machine_node']['ros__parameters']
    assert 'duration_sec' in sm_params, 'Arm movement duration_sec parameter missing'
    assert sm_params['duration_sec'] == 2.0
    assert 'laser_duration_us' in sm_params
    assert 'workspace_min_z' in sm_params
    assert 'workspace_max_z' in sm_params
    assert 'target_z_default' in sm_params

    # Check nav_integration_node parameters (specifically back edge margin requested by user)
    nav_params = data['nav_integration_node']['ros__parameters']
    assert 'back_edge_margin_x' in nav_params, 'back_edge_margin_x parameter missing'
    assert nav_params['back_edge_margin_x'] == 0.040
    assert 'stop_delay_sec' in nav_params
    assert 'min_resume_distance_m' in nav_params
    assert 'service_timeout_sec' in nav_params


def test_launch_descriptions_generate_successfully():
    """Verify that both launch files instantiate LaunchDescription properly."""
    sm_mod = _load_launch_module(os.path.join('launch', 'state_machine.launch.py'))
    sm_ld = sm_mod.generate_launch_description()
    assert isinstance(sm_ld, LaunchDescription)
    assert len(sm_ld.entities) == 4

    nav_mod = _load_launch_module(os.path.join('launch', 'nav_integration.launch.py'))
    nav_ld = nav_mod.generate_launch_description()
    assert isinstance(nav_ld, LaunchDescription)
    assert len(nav_ld.entities) == 2

