# File Tree: warebot_ws

**Generated:** 2/8/2026, 9:30:18 AM
**Root Path:** `/workspaces/warebot_ws`

```
├── 📁 .devcontainer
│   └── ⚙️ devcontainer.json
├── 📁 install
│   ├── 📁 warebot
│   │   └── 📁 share
│   │       ├── 📁 ament_index
│   │       │   └── 📁 resource_index
│   │       │       ├── 📁 package_run_dependencies
│   │       │       │   └── 📄 warebot
│   │       │       ├── 📁 packages
│   │       │       │   └── 📄 warebot
│   │       │       └── 📁 parent_prefix_path
│   │       │           └── 📄 warebot
│   │       ├── 📁 colcon-core
│   │       │   └── 📁 packages
│   │       │       └── 📄 warebot
│   │       └── 📁 warebot
│   │           ├── 📁 cmake
│   │           │   ├── 📄 warebotConfig-version.cmake
│   │           │   └── 📄 warebotConfig.cmake
│   │           ├── 📁 config
│   │           ├── 📁 description
│   │           │   ├── 📁 meshes
│   │           │   │   ├── 📁 accessories
│   │           │   │   │   ├── 📄 lidar_mount.stl
│   │           │   │   │   ├── 📄 lms1xx_mount.dae
│   │           │   │   │   └── 📄 lms1xx_mount.stl
│   │           │   │   ├── 📁 attachments
│   │           │   │   │   ├── 📄 300_mm_sensor_arch.dae
│   │           │   │   │   ├── 📄 300_mm_sensor_arch.stl
│   │           │   │   │   ├── 📄 510_mm_sensor_arch.dae
│   │           │   │   │   ├── 📄 510_mm_sensor_arch.stl
│   │           │   │   │   ├── 📄 bumper.dae
│   │           │   │   │   ├── 📄 bumper2.dae
│   │           │   │   │   ├── 📄 bumper3.dae
│   │           │   │   │   ├── 📄 bumper_extension.dae
│   │           │   │   │   ├── 📄 large_top_plate.dae
│   │           │   │   │   ├── 📄 large_top_plate_collision.stl
│   │           │   │   │   ├── 📄 observer_backpack_masts.stl
│   │           │   │   │   ├── 📄 observer_backpack_shell.stl
│   │           │   │   │   ├── 📄 pacs_top_plate.stl
│   │           │   │   │   ├── 📄 top_plate.dae
│   │           │   │   │   ├── 📄 top_plate.stl
│   │           │   │   │   ├── 📄 user_rail.dae
│   │           │   │   │   └── 📄 user_rail.stl
│   │           │   │   ├── 📁 wheels
│   │           │   │   │   ├── 📄 indoor.dae
│   │           │   │   │   ├── 📄 outdoor.dae
│   │           │   │   │   └── 📄 outdoor.stl
│   │           │   │   ├── 📄 base_link.dae
│   │           │   │   ├── 📄 base_link.stl
│   │           │   │   ├── 📄 top_chassis.dae
│   │           │   │   └── 📄 top_chassis.stl
│   │           │   └── 📁 urdf
│   │           │       ├── 📁 attachments
│   │           │       │   ├── 📄 bumper.urdf.xacro
│   │           │       │   ├── 📄 observer_backpack.urdf.xacro
│   │           │       │   ├── 📄 sensor_arch.urdf.xacro
│   │           │       │   └── 📄 top_plate.urdf.xacro
│   │           │       ├── 📁 drivetrain
│   │           │       │   ├── 📁 wheels
│   │           │       │   │   ├── 📄 indoor.urdf.xacro
│   │           │       │   │   └── 📄 outdoor.urdf.xacro
│   │           │       │   └── 📄 wheels.urdf.xacro
│   │           │       └── 📄 a200.urdf.xacro
│   │           ├── 📁 environment
│   │           │   ├── 📄 ament_prefix_path.dsv
│   │           │   ├── 📄 ament_prefix_path.sh
│   │           │   ├── 📄 path.dsv
│   │           │   └── 📄 path.sh
│   │           ├── 📁 hook
│   │           │   ├── 📄 cmake_prefix_path.dsv
│   │           │   ├── 📄 cmake_prefix_path.ps1
│   │           │   └── 📄 cmake_prefix_path.sh
│   │           ├── 📁 launch
│   │           │   ├── 📁 install
│   │           │   │   ├── ⚙️ .colcon_install_layout
│   │           │   │   ├── 📄 COLCON_IGNORE
│   │           │   │   ├── 🐍 _local_setup_util_ps1.py
│   │           │   │   ├── 🐍 _local_setup_util_sh.py
│   │           │   │   ├── 📄 local_setup.bash
│   │           │   │   ├── 📄 local_setup.ps1
│   │           │   │   ├── 📄 local_setup.sh
│   │           │   │   ├── 📄 local_setup.zsh
│   │           │   │   ├── 📄 setup.bash
│   │           │   │   ├── 📄 setup.ps1
│   │           │   │   ├── 📄 setup.sh
│   │           │   │   └── 📄 setup.zsh
│   │           │   ├── 📁 log
│   │           │   │   ├── 📁 build_2026-02-08_09-25-41
│   │           │   │   ├── 📄 COLCON_IGNORE
│   │           │   │   ├── 📄 latest
│   │           │   │   └── 📄 latest_build
│   │           │   └── 🐍 display.launch.py
│   │           ├── 📄 local_setup.bash
│   │           ├── 📄 local_setup.dsv
│   │           ├── 📄 local_setup.sh
│   │           ├── 📄 local_setup.zsh
│   │           ├── 📄 package.bash
│   │           ├── 📄 package.dsv
│   │           ├── 📄 package.ps1
│   │           ├── 📄 package.sh
│   │           ├── ⚙️ package.xml
│   │           └── 📄 package.zsh
│   ├── ⚙️ .colcon_install_layout
│   ├── 📄 COLCON_IGNORE
│   ├── 🐍 _local_setup_util_ps1.py
│   ├── 🐍 _local_setup_util_sh.py
│   ├── 📄 local_setup.bash
│   ├── 📄 local_setup.ps1
│   ├── 📄 local_setup.sh
│   ├── 📄 local_setup.zsh
│   ├── 📄 setup.bash
│   ├── 📄 setup.ps1
│   ├── 📄 setup.sh
│   └── 📄 setup.zsh
├── 📁 log
│   ├── 📁 build_2026-02-08_09-14-27
│   │   └── 📁 warebot
│   ├── 📁 build_2026-02-08_09-20-43
│   │   └── 📁 warebot
│   ├── 📁 build_2026-02-08_09-27-26
│   │   └── 📁 warebot
│   ├── 📄 COLCON_IGNORE
│   ├── 📄 latest
│   └── 📄 latest_build
├── 📁 src
│   ├── 📁 install
│   │   ├── 📁 warebot
│   │   │   └── 📁 share
│   │   │       ├── 📁 ament_index
│   │   │       │   └── 📁 resource_index
│   │   │       │       ├── 📁 package_run_dependencies
│   │   │       │       │   └── 📄 warebot
│   │   │       │       ├── 📁 packages
│   │   │       │       │   └── 📄 warebot
│   │   │       │       └── 📁 parent_prefix_path
│   │   │       │           └── 📄 warebot
│   │   │       ├── 📁 colcon-core
│   │   │       │   └── 📁 packages
│   │   │       │       └── 📄 warebot
│   │   │       └── 📁 warebot
│   │   │           ├── 📁 cmake
│   │   │           │   ├── 📄 warebotConfig-version.cmake
│   │   │           │   └── 📄 warebotConfig.cmake
│   │   │           ├── 📁 config
│   │   │           ├── 📁 description
│   │   │           │   ├── 📁 meshes
│   │   │           │   │   ├── 📁 accessories
│   │   │           │   │   │   ├── 📄 lidar_mount.stl
│   │   │           │   │   │   ├── 📄 lms1xx_mount.dae
│   │   │           │   │   │   └── 📄 lms1xx_mount.stl
│   │   │           │   │   ├── 📁 attachments
│   │   │           │   │   │   ├── 📄 300_mm_sensor_arch.dae
│   │   │           │   │   │   ├── 📄 300_mm_sensor_arch.stl
│   │   │           │   │   │   ├── 📄 510_mm_sensor_arch.dae
│   │   │           │   │   │   ├── 📄 510_mm_sensor_arch.stl
│   │   │           │   │   │   ├── 📄 bumper.dae
│   │   │           │   │   │   ├── 📄 bumper2.dae
│   │   │           │   │   │   ├── 📄 bumper3.dae
│   │   │           │   │   │   ├── 📄 bumper_extension.dae
│   │   │           │   │   │   ├── 📄 large_top_plate.dae
│   │   │           │   │   │   ├── 📄 large_top_plate_collision.stl
│   │   │           │   │   │   ├── 📄 observer_backpack_masts.stl
│   │   │           │   │   │   ├── 📄 observer_backpack_shell.stl
│   │   │           │   │   │   ├── 📄 pacs_top_plate.stl
│   │   │           │   │   │   ├── 📄 top_plate.dae
│   │   │           │   │   │   ├── 📄 top_plate.stl
│   │   │           │   │   │   ├── 📄 user_rail.dae
│   │   │           │   │   │   └── 📄 user_rail.stl
│   │   │           │   │   ├── 📁 wheels
│   │   │           │   │   │   ├── 📄 indoor.dae
│   │   │           │   │   │   ├── 📄 outdoor.dae
│   │   │           │   │   │   └── 📄 outdoor.stl
│   │   │           │   │   ├── 📄 base_link.dae
│   │   │           │   │   ├── 📄 base_link.stl
│   │   │           │   │   ├── 📄 top_chassis.dae
│   │   │           │   │   └── 📄 top_chassis.stl
│   │   │           │   └── 📁 urdf
│   │   │           │       ├── 📁 attachments
│   │   │           │       │   ├── 📄 bumper.urdf.xacro
│   │   │           │       │   ├── 📄 observer_backpack.urdf.xacro
│   │   │           │       │   ├── 📄 sensor_arch.urdf.xacro
│   │   │           │       │   └── 📄 top_plate.urdf.xacro
│   │   │           │       ├── 📁 drivetrain
│   │   │           │       │   ├── 📁 wheels
│   │   │           │       │   │   ├── 📄 indoor.urdf.xacro
│   │   │           │       │   │   └── 📄 outdoor.urdf.xacro
│   │   │           │       │   └── 📄 wheels.urdf.xacro
│   │   │           │       └── 📄 a200.urdf.xacro
│   │   │           ├── 📁 environment
│   │   │           │   ├── 📄 ament_prefix_path.dsv
│   │   │           │   ├── 📄 ament_prefix_path.sh
│   │   │           │   ├── 📄 path.dsv
│   │   │           │   └── 📄 path.sh
│   │   │           ├── 📁 hook
│   │   │           │   ├── 📄 cmake_prefix_path.dsv
│   │   │           │   ├── 📄 cmake_prefix_path.ps1
│   │   │           │   └── 📄 cmake_prefix_path.sh
│   │   │           ├── 📁 launch
│   │   │           │   ├── 📁 install
│   │   │           │   │   ├── ⚙️ .colcon_install_layout
│   │   │           │   │   ├── 📄 COLCON_IGNORE
│   │   │           │   │   ├── 🐍 _local_setup_util_ps1.py
│   │   │           │   │   ├── 🐍 _local_setup_util_sh.py
│   │   │           │   │   ├── 📄 local_setup.bash
│   │   │           │   │   ├── 📄 local_setup.ps1
│   │   │           │   │   ├── 📄 local_setup.sh
│   │   │           │   │   ├── 📄 local_setup.zsh
│   │   │           │   │   ├── 📄 setup.bash
│   │   │           │   │   ├── 📄 setup.ps1
│   │   │           │   │   ├── 📄 setup.sh
│   │   │           │   │   └── 📄 setup.zsh
│   │   │           │   ├── 📁 log
│   │   │           │   │   ├── 📁 build_2026-02-08_09-25-41
│   │   │           │   │   ├── 📄 COLCON_IGNORE
│   │   │           │   │   ├── 📄 latest
│   │   │           │   │   └── 📄 latest_build
│   │   │           │   └── 🐍 display.launch.py
│   │   │           ├── 📄 local_setup.bash
│   │   │           ├── 📄 local_setup.dsv
│   │   │           ├── 📄 local_setup.sh
│   │   │           ├── 📄 local_setup.zsh
│   │   │           ├── 📄 package.bash
│   │   │           ├── 📄 package.dsv
│   │   │           ├── 📄 package.ps1
│   │   │           ├── 📄 package.sh
│   │   │           ├── ⚙️ package.xml
│   │   │           └── 📄 package.zsh
│   │   ├── ⚙️ .colcon_install_layout
│   │   ├── 📄 COLCON_IGNORE
│   │   ├── 🐍 _local_setup_util_ps1.py
│   │   ├── 🐍 _local_setup_util_sh.py
│   │   ├── 📄 local_setup.bash
│   │   ├── 📄 local_setup.ps1
│   │   ├── 📄 local_setup.sh
│   │   ├── 📄 local_setup.zsh
│   │   ├── 📄 setup.bash
│   │   ├── 📄 setup.ps1
│   │   ├── 📄 setup.sh
│   │   └── 📄 setup.zsh
│   ├── 📁 log
│   │   ├── 📁 build_2026-02-08_09-25-51
│   │   │   └── 📁 warebot
│   │   ├── 📄 COLCON_IGNORE
│   │   ├── 📄 latest
│   │   └── 📄 latest_build
│   └── 📁 warebot
│       ├── 📁 config
│       ├── 📁 description
│       │   ├── 📁 meshes
│       │   │   ├── 📁 accessories
│       │   │   │   ├── 📄 lidar_mount.stl
│       │   │   │   ├── 📄 lms1xx_mount.dae
│       │   │   │   └── 📄 lms1xx_mount.stl
│       │   │   ├── 📁 attachments
│       │   │   │   ├── 📄 300_mm_sensor_arch.dae
│       │   │   │   ├── 📄 300_mm_sensor_arch.stl
│       │   │   │   ├── 📄 510_mm_sensor_arch.dae
│       │   │   │   ├── 📄 510_mm_sensor_arch.stl
│       │   │   │   ├── 📄 bumper.dae
│       │   │   │   ├── 📄 bumper2.dae
│       │   │   │   ├── 📄 bumper3.dae
│       │   │   │   ├── 📄 bumper_extension.dae
│       │   │   │   ├── 📄 large_top_plate.dae
│       │   │   │   ├── 📄 large_top_plate_collision.stl
│       │   │   │   ├── 📄 observer_backpack_masts.stl
│       │   │   │   ├── 📄 observer_backpack_shell.stl
│       │   │   │   ├── 📄 pacs_top_plate.stl
│       │   │   │   ├── 📄 top_plate.dae
│       │   │   │   ├── 📄 top_plate.stl
│       │   │   │   ├── 📄 user_rail.dae
│       │   │   │   └── 📄 user_rail.stl
│       │   │   ├── 📁 wheels
│       │   │   │   ├── 📄 indoor.dae
│       │   │   │   ├── 📄 outdoor.dae
│       │   │   │   └── 📄 outdoor.stl
│       │   │   ├── 📄 base_link.dae
│       │   │   ├── 📄 base_link.stl
│       │   │   ├── 📄 top_chassis.dae
│       │   │   └── 📄 top_chassis.stl
│       │   └── 📁 urdf
│       │       ├── 📁 attachments
│       │       │   ├── 📄 bumper.urdf.xacro
│       │       │   ├── 📄 observer_backpack.urdf.xacro
│       │       │   ├── 📄 sensor_arch.urdf.xacro
│       │       │   └── 📄 top_plate.urdf.xacro
│       │       ├── 📁 drivetrain
│       │       │   ├── 📁 wheels
│       │       │   │   ├── 📄 indoor.urdf.xacro
│       │       │   │   └── 📄 outdoor.urdf.xacro
│       │       │   └── 📄 wheels.urdf.xacro
│       │       └── 📄 a200.urdf.xacro
│       ├── 📁 include
│       │   └── 📁 warebot
│       ├── 📁 launch
│       │   ├── 📁 install
│       │   │   ├── ⚙️ .colcon_install_layout
│       │   │   ├── 📄 COLCON_IGNORE
│       │   │   ├── 🐍 _local_setup_util_ps1.py
│       │   │   ├── 🐍 _local_setup_util_sh.py
│       │   │   ├── 📄 local_setup.bash
│       │   │   ├── 📄 local_setup.ps1
│       │   │   ├── 📄 local_setup.sh
│       │   │   ├── 📄 local_setup.zsh
│       │   │   ├── 📄 setup.bash
│       │   │   ├── 📄 setup.ps1
│       │   │   ├── 📄 setup.sh
│       │   │   └── 📄 setup.zsh
│       │   ├── 📁 log
│       │   │   ├── 📁 build_2026-02-08_09-25-41
│       │   │   ├── 📄 COLCON_IGNORE
│       │   │   ├── 📄 latest
│       │   │   └── 📄 latest_build
│       │   └── 🐍 display.launch.py
│       ├── 📁 sensors_description
│       │   ├── 📁 meshes
│       │   │   ├── 📁 ouster
│       │   │   │   ├── 📄 os1_base.dae
│       │   │   │   ├── 📄 os1_fins.dae
│       │   │   │   ├── 📄 os1_halo.dae
│       │   │   │   └── 📄 os1_lidar.dae
│       │   │   ├── 📄 axis_dome.stl
│       │   │   ├── 📄 axis_q62_base.stl
│       │   │   ├── 📄 axis_q62_top.stl
│       │   │   ├── 📄 fixposition.stl
│       │   │   ├── 📄 flir_blackfly.stl
│       │   │   ├── 📄 gnss_helical.stl
│       │   │   ├── 📄 gnss_patch.stl
│       │   │   ├── 📄 gnss_spherical.stl
│       │   │   ├── 📄 hokuyo_ust.stl
│       │   │   ├── 📄 hokuyo_utm30.stl
│       │   │   ├── 📄 novatel_smart6.stl
│       │   │   ├── 📄 novatel_smart7.stl
│       │   │   ├── 📄 oakd_lite.dae
│       │   │   ├── 📄 oakd_pro.dae
│       │   │   ├── 📄 oakd_pro_w_poe.dae
│       │   │   ├── 📄 seyond_robin_w.stl
│       │   │   ├── 📄 sick_lms1xx_collision.stl
│       │   │   └── 📄 sick_lms1xx_small.dae
│       │   ├── 📁 urdf
│       │   │   ├── 📁 axis
│       │   │   │   ├── 📄 axis_dome_fixed.urdf.xacro
│       │   │   │   ├── 📄 axis_dome_ptz.urdf.xacro
│       │   │   │   └── 📄 axis_q62.urdf.xacro
│       │   │   ├── 📁 intel
│       │   │   │   ├── 📄 d415.urdf.xacro
│       │   │   │   ├── 📄 d435.urdf.xacro
│       │   │   │   ├── 📄 d435i.urdf.xacro
│       │   │   │   ├── 📄 d455.urdf.xacro
│       │   │   │   └── 📄 d456.urdf.xacro
│       │   │   ├── 📁 meshes
│       │   │   │   ├── 📁 ouster
│       │   │   │   │   ├── 📄 os1_base.dae
│       │   │   │   │   ├── 📄 os1_fins.dae
│       │   │   │   │   ├── 📄 os1_halo.dae
│       │   │   │   │   └── 📄 os1_lidar.dae
│       │   │   │   ├── 📄 axis_dome.stl
│       │   │   │   ├── 📄 axis_q62_base.stl
│       │   │   │   ├── 📄 axis_q62_top.stl
│       │   │   │   ├── 📄 fixposition.stl
│       │   │   │   ├── 📄 flir_blackfly.stl
│       │   │   │   ├── 📄 gnss_helical.stl
│       │   │   │   ├── 📄 gnss_patch.stl
│       │   │   │   ├── 📄 gnss_spherical.stl
│       │   │   │   ├── 📄 hokuyo_ust.stl
│       │   │   │   ├── 📄 hokuyo_utm30.stl
│       │   │   │   ├── 📄 novatel_smart6.stl
│       │   │   │   ├── 📄 novatel_smart7.stl
│       │   │   │   ├── 📄 oakd_lite.dae
│       │   │   │   ├── 📄 oakd_pro.dae
│       │   │   │   ├── 📄 oakd_pro_w_poe.dae
│       │   │   │   ├── 📄 seyond_robin_w.stl
│       │   │   │   ├── 📄 sick_lms1xx_collision.stl
│       │   │   │   └── 📄 sick_lms1xx_small.dae
│       │   │   ├── 📄 CHANGELOG.rst
│       │   │   ├── 📄 CMakeLists.txt
│       │   │   ├── 📄 axis_camera.urdf.xacro
│       │   │   ├── 📄 chrobotics_um6.urdf.xacro
│       │   │   ├── 📄 fixposition.urdf.xacro
│       │   │   ├── 📄 flir_blackfly.urdf.xacro
│       │   │   ├── 📄 garmin_18x.urdf.xacro
│       │   │   ├── 📄 gnss_antenna.urdf.xacro
│       │   │   ├── 📄 hokuyo_ust.urdf.xacro
│       │   │   ├── 📄 intel_realsense.urdf.xacro
│       │   │   ├── 📄 luxonis_oakd.urdf.xacro
│       │   │   ├── 📄 microstrain_gq7.urdf.xacro
│       │   │   ├── 📄 microstrain_imu.urdf.xacro
│       │   │   ├── 📄 novatel_smart6.urdf.xacro
│       │   │   ├── 📄 novatel_smart7.urdf.xacro
│       │   │   ├── 📄 ouster_os1.urdf.xacro
│       │   │   ├── ⚙️ package.xml
│       │   │   ├── 📄 phidgets_spatial.urdf.xacro
│       │   │   ├── 📄 redshift_um7.urdf.xacro
│       │   │   ├── 📄 seyond_lidar.urdf.xacro
│       │   │   ├── 📄 sick_lms1xx.urdf.xacro
│       │   │   ├── 📄 stereolabs_zed.urdf.xacro
│       │   │   ├── 📄 swiftnav_duro.urdf.xacro
│       │   │   ├── 📄 velodyne_lidar.urdf.xacro
│       │   │   └── 📄 wiferion.urdf.xacro
│       │   ├── 📄 CHANGELOG.rst
│       │   ├── 📄 CMakeLists.txt
│       │   └── ⚙️ package.xml
│       ├── 📄 CMakeLists.txt
│       └── ⚙️ package.xml
├── 🐳 Dockerfile
├── 📄 start.sh
└── 📄 warebot_ws
```

---
*Generated by FileTree Pro Extension*