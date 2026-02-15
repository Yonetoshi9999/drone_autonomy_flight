# Drone Assets

This directory contains 3D models and URDF files for drone simulation.

## Files

### cf2x.urdf
Unified Robot Description Format (URDF) file for the Bitcraze Crazyflie 2.x quadcopter in X-configuration.

**Source**: https://github.com/utiasDSL/gym-pybullet-drones

**Key Properties**:
- Mass: 0.027 kg
- Arm length: 0.0397 m
- Thrust coefficient (kf): 3.16e-10
- Torque coefficient (km): 7.94e-12
- Thrust-to-weight ratio: 2.25

**Structure**:
- `base_link`: Main body (cylinder collision shape)
- `prop0_link` to `prop3_link`: Four propeller attachment points
- `center_of_mass_link`: Center of mass reference frame

### cf2.dae
COLLADA 3D mesh file for visual representation of the Crazyflie quadcopter.

**Source**: https://github.com/utiasDSL/gym-pybullet-drones

**Format**: COLLADA (.dae)
**Usage**: Referenced by cf2x.urdf for visual geometry

## License

These models are from the gym-pybullet-drones project and are used for educational and research purposes. Please refer to the original repository for licensing information:
https://github.com/utiasDSL/gym-pybullet-drones

## Citation

If you use these models in your research, please cite:

```bibtex
@INPROCEEDINGS{panerati2021learning,
      title={Learning to Fly---a Gym Environment with PyBullet Physics for Reinforcement Learning of Multi-agent Quadcopter Control},
      author={Jacopo Panerati and Hehui Zheng and SiQi Zhou and James Xu and Amanda Prorok and Angela P. Schoellig},
      booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
      year={2021},
      pages={},
      doi={}
}
```

## Adding New Models

To add a new drone model:

1. Place the URDF file in this directory
2. Place any mesh files (`.dae`, `.stl`, `.obj`) in this directory
3. Update the mesh file paths in the URDF to be relative (e.g., `./mesh_file.dae`)
4. Ensure the URDF includes proper inertial properties
5. Test loading with `test_crazyflie_headless.py`

Example URDF structure:
```xml
<?xml version="1.0" ?>
<robot name="my_drone">
  <link name="base_link">
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.002"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="./my_drone.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>
</robot>
```
