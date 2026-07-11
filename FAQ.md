# SG-SLAM Frequently Asked Questions (FAQ)

> **English** · [简体中文](./FAQ-zh.md)

This document is compiled from all of the project's GitHub Issues, covering common problems with build & dependencies, runtime crashes, map visualization, object detection, custom cameras, algorithm details, and evaluation accuracy.

## Table of Contents

- [1. 🔧 Build & Dependencies](#1-build--dependencies)
- [2. 💥 Runtime Crashes & Abnormal Exits](#2-runtime-crashes--abnormal-exits)
- [3. 🗺️ Octomap / Point Cloud Map Visualization](#3-octomap--point-cloud-map-visualization)
- [4. 🎯 Object Detection Network & Models](#4-object-detection-network--models)
- [5. 📷 Custom Cameras & Datasets](#5-custom-cameras--datasets)
- [6. 🧩 Algorithm & Code Details](#6-algorithm--code-details)
- [7. 📊 Evaluation & Accuracy](#7-evaluation--accuracy)
- [8. 💡 Other Conceptual Questions](#8-other-conceptual-questions)

---

## 1. 🔧 Build & Dependencies

### Q: Third-party libraries fail to build / executable not found (`libg2o.so` missing, `Sim3Solver` undefined reference, `sg_slam_tum` not found)

The common root cause of these errors is that **the sg-slam main project was not actually linked/built successfully**, not surface-level issues like the PCL version. Troubleshoot in this order:

1. **First make sure the third-party libraries build.** Run the third-party build script in the project root:

   ```bash
   ./ThirdpartyBuild.sh
   ```

   This script builds `DBoW2`, `g2o`, and `ncnn`. The most common failure for `g2o` is not finding `eigen3` — once `eigen3` is installed, `Thirdparty/g2o/lib/libg2o.so` is generated. Otherwise `catkin_make` will report:

   ```
   No rule to make target '.../Thirdparty/g2o/lib/libg2o.so', needed by 'libsg-slam.so'. Stop.
   ```

   Note: the PCL `std::auto_ptr` message is only a **deprecated warning**, not an error, and is unrelated to the failure.

2. **A batch of `ORB_SLAM2::Sim3Solver` undefined reference errors at link time** (e.g. `GetEstimatedTranslation`, `SetRansacParameters`, `iterate`, the constructor, etc.). `Sim3Solver.cc` is already compiled into `libsg-slam.so`, so **you do not need to manually edit CMake**. This is usually caused by an inconsistent `build`/`devel` incremental cache; clear the cache and rebuild:

   ```bash
   rm -rf build/ devel/
   catkin_make
   ```

   You can first run `ldd libsg-slam.so` to check for any `not found` dependencies.

3. **The runtime error `./Examples/sg_slam_tum: not found`** means the target was never actually built. Re-run the build command a few times, or carefully re-check for errors that were overlooked in that step, until the executable is generated:

   ```bash
   catkin_make --pkg sg-slam
   ```

Author's reference environment: `libpcl-dev 1.8.1`, `eigen 3.3.4`.

> Related issues: [#10](https://github.com/silencht/SG-SLAM/issues/10), [#19](https://github.com/silencht/SG-SLAM/issues/19), [#32](https://github.com/silencht/SG-SLAM/issues/32)

### Q: ncnn build problems (can't find `ncnnConfig.cmake` / library path; gcc version causes `_mm256_set_m128` not declared)

ncnn build failures usually have two causes: **CMake cannot locate the config file**, or **the compiler version is too old**.

1. **Path problem** — CMake can't find `ncnnConfig.cmake` (or `ncnn-config.cmake`). Confirm the file actually exists at the path you set, then choose one of:
   - Write the absolute path of that config file directly in CMake;
   - `export` the ncnn path in the terminal;
   - Copy the ncnn build artifacts into a standard system search path, e.g. copy the `bin`, `lib`, `include` directories under `Thirdparty/ncnn/build/install/` into `/usr/local/lib/ncnn`:

     ```bash
     sudo cp -r .../Thirdparty/ncnn/build/install/{bin,lib,include} /usr/local/lib/ncnn/
     ```

     `find_package` can then locate it.

2. **The error `'_mm256_set_m128' was not declared in this scope`** (commonly in x86 AVX layers such as `shufflechannel_x86_avx512.cpp`). This intrinsic was introduced in gcc 8, and `gcc 7.5.0` does not support it. Upgrade to gcc/g++ 8 or later and rebuild `Thirdparty/ncnn`:

   ```bash
   sudo apt install gcc-9 g++-9
   sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
   sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
   ```

**Troubleshooting tip:** first get the official ncnn `examples`/`benchmark` running to confirm the library itself works.

> Related issues: [#11](https://github.com/silencht/SG-SLAM/issues/11), [#29](https://github.com/silencht/SG-SLAM/issues/29), [#38](https://github.com/silencht/SG-SLAM/issues/38)

### Q: Static assertion under newer GCC that `std::map` and `Eigen::aligned_allocator` types don't match (`KeyFrameAndPose`)

The error `std::map must have the same value_type as its allocator` occurs at the `KeyFrameAndPose` typedef in `LoopClosing` and `Optimizer`. The root cause is that newer GCC (9.x/11.x) libstdc++ added a `static_assert` requiring the allocator's `value_type` to exactly match the map's `value_type` (i.e. `std::pair<const Key, T>`), while the original code's `Eigen::aligned_allocator` was missing `const` on the first template argument of the pair.

**Fix:** add `const` to the first type of the pair. Use `src/sg-slam/include/LoopClosing.h` lines 49-50 as the reference:

```cpp
typedef map<KeyFrame*, g2o::Sim3, std::less<KeyFrame*>,
            Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```

The equivalent form is `std::pair<KeyFrame* const, g2o::Sim3>`. Modify the identically-named definition in `Optimizer.cc` the same way. See this [Zhihu article](https://zhuanlan.zhihu.com/p/218019316). Projects based on the old ORB-SLAM2 (such as DynaSLAM) that hit the same error are handled identically.

⚠️ Note: after fixing the build, #40 and #41 also reported a **runtime segfault** (map construction repeatedly resetting, the system being reset and unable to track, empty octree), but the discussion never pinned down the exact root cause (the user's debug screenshots don't render on GitHub and the discussion trailed off). That runtime-crash portion has no final conclusion.

> Related issues: [#12](https://github.com/silencht/SG-SLAM/issues/12), [#40](https://github.com/silencht/SG-SLAM/issues/40), [#41](https://github.com/silencht/SG-SLAM/issues/41)

### Q: Smart-pointer mismatch with newer PCL: `boost::make_shared` cannot be assigned to `pcl::Ptr`

Newer PCL (e.g. 1.12) defines `pcl::PointCloud::Ptr` as `std::shared_ptr`, while `PointcloudMapping.cc` uses `boost::make_shared`, which returns a `boost::shared_ptr`. The two are not assignable, producing `no match for 'operator='`.

**Solution:** change all `boost::make_shared` in the constructor of `src/PointcloudMapping.cc` to `std::make_shared` (around lines 23-29: `semantic_objMap`, `localMap`, `temp_globalMap`, `globalMap`, etc.):

```cpp
semantic_objMap = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
localMap        = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
temp_globalMap  = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
globalMap       = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
```

After this change it builds (confirmed working under PCL 1.12).

> Related issues: [#16](https://github.com/silencht/SG-SLAM/issues/16)

### Q: OpenCV and cv_bridge version conflict causes a `cv::FileStorage::open` segfault

The machine has both OpenCV 4.2 and 3.3.1 installed; SG-SLAM links against 3.3.1 while `cv_bridge` links against 4.2. It crashes at `System.cc` line 62, `cv::FileStorage::open`, while reading the yaml (the error looks like the file can't be opened, but the yaml path/content are actually fine — this is a false symptom caused by the version conflict).

**Solution:** unify SG-SLAM and `cv_bridge` to the same OpenCV version (3.3.1 in this case), after which it runs.

Debugging approach:

```bash
gdb ./Examples/sg_slam_tum
run <path_to_vocabulary> <path_to_settings> <path_to_sequence> <path_to_association>
bt
```

Rebuilding with Debug mode enabled in `CMakeLists.txt` gives a clearer stack trace.

(The other half of this issue — rviz not showing the map — is covered by the `/map` TF problem in the Octomap / Point Cloud section.)

> Related issues: [#20](https://github.com/silencht/SG-SLAM/issues/20)

### Q: `catkin_make --pkg octomap_server` reports `OctomapServerConfig has not been declared`

`dynamic_reconfigure` failed to generate the `OctomapServerConfig` header. The root cause is that the `.cfg` file lacks executable permission, and/or a non-system Python is in use.

**Solution:**

```bash
chmod +x src/octomap_server/cfg/*.cfg
```

If the default `python` has been taken over by conda, switch back to the system Python (`deactivate` the conda environment, or make the first-line shebang of the `.cfg` point to the system python), then clean and rebuild:

```bash
rm -rf build devel
catkin_make
```

Once `OctomapServerConfig.h` is generated, the build passes.

> Related issues: [#18](https://github.com/silencht/SG-SLAM/issues/18)

### Q: Pangolin v0.5 build fails: `AVFMT_RAWPICTURE` not declared

Pangolin v0.5's ffmpeg driver (`src/video/drivers/ffmpeg.cpp:564`) references the `AVFMT_RAWPICTURE` macro, which has been removed in newer FFmpeg.

**Two solutions (choose either):**

1. **Build the Pangolin master/latest branch instead** (verified to work; SG-SLAM runs fine, v0.5 is not mandatory);
2. If you insist on v0.5, follow [this blog post](https://blog.csdn.net/Robert_Q/article/details/121690089) to work around the FFmpeg error.

> Related issues: [#43](https://github.com/silencht/SG-SLAM/issues/43)

### Q: VSCode Debug throws `std::bad_alloc` (not an actual out-of-memory)

After rebuilding in Debug mode, `std::bad_alloc` is thrown at `new ORBVocabulary()` in `System.cc` while loading the ORB vocabulary, even though the machine still has plenty of free memory — this is a **misconfigured VSCode C++/debug environment**, not insufficient memory.

**Solution:** re-check your `launch.json`/`tasks.json`, and be careful to distinguish VSCode's native debug from the CMake Tools plugin's debug (two separate mechanisms) — don't mix them. See [VSCode C++ environment setup](https://zhuanlan.zhihu.com/p/554248108) and [explanation of the two debug mechanisms](https://www.zhihu.com/question/471176365/answer/2386359912).

Note: the author generally runs in Release (Debug is too slow) and inspects intermediate variables mostly with prints.

> Related issues: [#26](https://github.com/silencht/SG-SLAM/issues/26)

---

## 2. 💥 Runtime Crashes & Abnormal Exits

### Q: Segfault / window crash when running a dataset, gdb points to g2o `linearizeOplus` (inconsistent `-march=native` flags / Eigen memory alignment)

This is a classic problem for the ORB-SLAM2 family: the `-march=native` optimization flag is inconsistent between the main project and the various libraries (`g2o`, `DBoW2`, `Eigen`, `Pangolin`, `PCL`), causing a memory-alignment/instruction-set mismatch that segfaults in `g2o::EdgeSE3ProjectXYZOnlyPose::linearizeOplus()` (often right after `New map created with ... points`).

**Solution:** keep `-march=native` consistent across all relevant `CMakeLists.txt` files (**add it everywhere or remove it everywhere**). The key gotcha is that the apt-installed `libpcl-dev` **does not** use `-march=native` by default, so the safest approach is to **remove** `-march=native` from all of the following `CMakeLists.txt` files:

- `./eigen/CMakeLists.txt`
- `./Pangolin/CMakeLists.txt`
- `./SG-SLAM/src/sg-slam/CMakeLists.txt`
- `./SG-SLAM/src/sg-slam/Thirdparty/g2o/CMakeLists.txt`
- `./SG-SLAM/src/sg-slam/Thirdparty/DBoW2/CMakeLists.txt`

Use `ldd libsg-slam.so | grep pcl` to confirm which PCL is actually linked. After the change you **must clear the cmake cache in each `build` directory and do a full rebuild**. See [ORB_SLAM2 PR #844](https://github.com/raulmur/ORB_SLAM2/pull/844), g2o PR #314, and rtabmap issues #637/#685.

> Related issues: [#21](https://github.com/silencht/SG-SLAM/issues/21), [#24](https://github.com/silencht/SG-SLAM/issues/24), [#31](https://github.com/silencht/SG-SLAM/issues/31)

### Q: Segfault when running a different dataset (locate it with gdb)

SG-SLAM is not limited to `walking_xyz`; other dynamic sequences are supported too. After switching sequences, a segfault is usually caused by an **incorrect dataset path / associate file configuration** or an out-of-bounds access on a particular frame.

General troubleshooting:

```bash
gdb --args ./your_executable args...
run
# after the crash:
bt
```

Use `bt` to inspect the call stack and pinpoint the offending line. The reporter resolved it after this (the specific change was not made public).

> Related issues: [#13](https://github.com/silencht/SG-SLAM/issues/13)

### Q: Tracking lost / no window and no response / missing tf reporting `A is singular`

Once tracking is lost, the system stops publishing the pose tf, and octomap mapping fails due to the missing tf, accompanied by `A is singular, this shouldn't happen`. In theory tracking should not lose feature points; you can first adjust the parameters that affect tracking robustness in `TUM3.yaml` as described in the README.

Common causes of tracking loss:

- Insufficient device compute — a slow backend drags down the frontend;
- Highly dynamic scenes with too few correctly matched feature points;
- Weak texture / small FOV / repetitive texture;
- Lighting too bright, too dark, or changing abruptly;
- Camera motion too violent (large rotation, shaking, occlusion);
- Depth cameras scan water, glass, etc. poorly;
- The early map is too small, so later frames can't correct in time before it dies.

**Advice for custom datasets:** move the camera slowly, ensure ample texture and stable lighting, and avoid rapid large rotations and occlusion. For "no window / no response", a more complete runtime log is needed to diagnose.

> Related issues: [#7](https://github.com/silencht/SG-SLAM/issues/7), [#34](https://github.com/silencht/SG-SLAM/issues/34)

### Q: Program hangs with no error at the PCL statistical filter / Euclidean clustering step

It hangs without any error while executing `mSor_Detect3D.filter(...)` (statistical filtering) or `ec.extract(...)` (Euclidean clustering) in `Detector3D::DetectOne`. The author did not confirm the root cause; the following are discussion-level troubleshooting suggestions:

1. **Confirm the real blocking location.** The program is multi-threaded, so the apparent hang point may be another thread's problem stalling the current thread. Add prints/breakpoints to confirm where it truly blocks;
2. **Tune the PCL parameters in the yaml:**
   - Statistical filter: `setMeanK` (`Detect3D_Sor_MeanK_`), `setStddevMulThresh` (`Detect3D_Sor_StddevMulThresh_`);
   - Voxel filter: `setLeafSize` (`Detect3D_Voxel_LeafSize_`);
   - Euclidean clustering: `setClusterTolerance`, `setMinClusterSize`, `setMaxClusterSize`.

   Poor parameters (too small a cluster tolerance, too large a max cluster size) can make it extremely slow, appearing as a "hang".

> ⚠️ This issue has no definitive root-cause conclusion; you'll need to troubleshoot yourself.
>
> Related issues: [#15](https://github.com/silencht/SG-SLAM/issues/15)

---

## 3. 🗺️ Octomap / Point Cloud Map Visualization

### Q: Nothing shows in rviz / octomap reports `Nothing to publish, octree is empty`

An empty octree / nothing displayed is usually one of two causes: **the display side isn't subscribed to the topics**, or **the main program isn't actually publishing point clouds**. Steps:

1. **Load the project's rviz config instead of opening `rviz` directly:**

   ```bash
   rviz -d .../src/sg-slam/Examples/rvizconfig.rviz
   ```

   Or load that file manually via rviz's `File` menu. The topics to subscribe to include `/occupied_cells_vis_array`, `/SG_SLAM/Semantic_Objects`, `/octomap_point_cloud_centers`, etc.; for the octree use `Add → By topic → occupied_cells_vis_array`'s `MarkerArray`.

2. **An empty octree usually means SG-SLAM isn't running.** Check the dataset, `ORBvoc`, and yaml paths in `run_tum_walking_xyz.sh` (if a relative path is problematic, switch to an absolute path). The TUM dataset **must first be synchronized with `associate.py` for RGB/depth timestamps**, otherwise no paired frames are read and the octree stays empty.

3. **Verify the data one item at a time:**

   ```bash
   rostopic list
   rostopic echo <topic_name>
   ```

   If the point cloud topic can be `echo`'d but the octree topic can't, the point cloud isn't being accumulated into the octree by `octomap_server`.

4. **A `/map` vs `map` transform failure** is covered by the TF problem in this section. The new version **does not need `transform.launch`** (the tf transform logic is built into the program).

WARNs like `Uninitialized quaternion, assuming identity` can be ignored for now. The author ultimately suggested adding prints in the octomap code to help locate the problem (#17 was not fully resolved).

> Related issues: [#1](https://github.com/silencht/SG-SLAM/issues/1), [#4](https://github.com/silencht/SG-SLAM/issues/4), [#17](https://github.com/silencht/SG-SLAM/issues/17)

### Q: Octomap has no color / reports `Colored map requested... please define COLOR_OCTOMAP_SERVER`

The launch file sets `colored_map=true` but starts the color-less `octomap_server_node`; or it loads the apt system package instead of the project's own color version (with the `COLOR_OCTOMAP_SERVER` macro enabled).

**Solution:**

1. Change the node type on line 11 of `src/octomap_server/launch/octomap.launch` to the color version, and keep `colored_map=true`:

   ```xml
   <node pkg="octomap_server" type="octomap_color_server_node" name="octomap_server">
       <param name="colored_map" type="bool" value="true"/>
   ```

   `octomap_color_server_node` is already defined in `CMakeLists.txt` and links the `_color` library built with the `COLOR_OCTOMAP_SERVER` macro; the macro is enabled at `OctomapServer.h` line 74, so **you do not need to edit CMake yourself**. If you don't need color, set `colored_map` to `false` and keep using the normal `octomap_server_node`; the error disappears as well.

2. Rebuild and source the environment:

   ```bash
   catkin_make --pkg octomap_server
   source devel/setup.bash
   roslaunch octomap.launch
   ```

   If necessary, carefully uninstall the system package: `sudo apt-get remove ros-melodic-octomap-server`.

3. Make sure the input point cloud (remapped to `/SG_SLAM/Local_Point_Clouds` by default) carries an RGB field.

`octree is empty / Nothing to publish` is a normal message — you need to run the SG-SLAM main program first to publish point clouds. See the CSDN blogs [109003337](https://blog.csdn.net/weixin_44694952/article/details/109003337) and [106803808](https://blog.csdn.net/qq_34131212/article/details/106803808).

> Related issues: [#14](https://github.com/silencht/SG-SLAM/issues/14), [#25](https://github.com/silencht/SG-SLAM/issues/25), [#36](https://github.com/silencht/SG-SLAM/issues/36)

### Q: rviz TF error: can't transform between the `/map` and `map` frames (TF/TF2 migration, common on Ubuntu 20)

After ROS migrated from TF to TF2, leading-slash `frame_id`s are no longer supported, so `/map` and `map` don't match (Ubuntu 18 with `/map` does not reproduce it). The error looks like `Error transforming pose from frame '/map' to frame 'map'`.

**Solution:** change all `/map` in the source/config to `map` (remove the leading slash). Places to change (based on commit `69f5a06`):

- `src/octomap_server/launch/octomap.launch` line 13
- `src/sg-slam/Examples/rgbd_tum.cc` line 207
- `src/sg-slam/Examples/ros_rgbd.cc` line 160
- `src/sg-slam/src/PointcloudMapping.cc` lines 284 / 352 / 387 / 404

Rebuild after the changes, and the three maps (octomap, semantic object metric map, 3D point cloud map) display. Someone confirmed success after making these edits. See [tf2 Migration](http://wiki.ros.org/tf2/Migration).

> Related issues: [#1](https://github.com/silencht/SG-SLAM/issues/1), [#20](https://github.com/silencht/SG-SLAM/issues/20), [#30](https://github.com/silencht/SG-SLAM/issues/30)

### Q: The 3D semantic detection box is positioned incorrectly in rviz

After detecting a chair, the position of the blue 3D box in rviz doesn't line up with the semantic segmentation result. The author did not reach a conclusion; troubleshooting direction:

First confirm which reference frame the 3D box is published in, and compare it with the position of the same object in the octomap — if the 3D box is **aligned** with the object in the octomap and only visually mismatched with the image's semantic segmentation, it's most likely a **perceptual offset** caused by different topic `frame_id`s/TF or a different camera viewpoint, not a localization error.

Suggested details to provide: the dataset/camera used, the `frame_id` of the 3D box topic, and whether the box aligns with the octomap.

> ⚠️ The OP did not reply; the issue was not confirmed resolved. You'll need to troubleshoot yourself.
>
> Related issues: [#6](https://github.com/silencht/SG-SLAM/issues/6)

### Q: The 3D point cloud map shows only part / is sparse; `Global_Point_Clouds` shows nothing at all

First distinguish the two topics:

- `/SG_SLAM/Global_Point_Clouds` — the global point cloud map;
- `/SG_SLAM/Local_Point_Clouds` — the current keyframe's point cloud.

**Common causes of a sparse point cloud:** dynamic objects (e.g. people) are removed; for efficiency only keyframes are processed; low-quality depth from highly reflective objects (glass/water) is removed; high dynamics cause brief keyframe loss.

**If Global shows nothing at all**, check: whether RViz's `Fixed Frame` matches the point cloud's frame, and whether the camera intrinsics/depth scale in `camera.yaml` are correct.

> ⚠️ The author asked for the `camera.yaml`/logs to troubleshoot further, but the user did not reply, so there's no definitive conclusion.
>
> Related issues: [#35](https://github.com/silencht/SG-SLAM/issues/35)

### Q: How to save the point cloud as a `.pcd` file after a run

**Supported, but off by default.** Change the switch in the yaml (e.g. `Examples/astra_pro_camera.yaml`) from `0` to `1`:

```yaml
PointCloudMapping.is_global_pc_reconstruction: 1
```

Once enabled, **on normal exit** (`ros_rgbd` after `ros::spin()` ends, `rgbd_tum` after finishing the sequence) the program takes `GetPointCloudMapper()->get_globalMap()`, and if it's non-empty calls `pcl::io::savePCDFileBinary` to write `global_pcd.pcd` into the current working directory (see `ros_rgbd.cc` lines 82-90 and `rgbd_tum.cc` lines 164-170).

⚠️ It only writes out on normal exit; a `Ctrl+C` force-kill or a crash may not produce the file.

The octomap semantic map (`.bt`/`.ot`) can be saved with `octomap_server`'s `octomap_saver` (`octomap_saver <mapfile.[bt|ot]>`), or set `PointCloudMapping.is_octo_semantic_map_construction` to `1`.

> Related issues: [#46](https://github.com/silencht/SG-SLAM/issues/46)

---

## 4. 🎯 Object Detection Network & Models

### Q: Which objects can be detected / is a GPU required / why don't `person` and `pottedplant` show a 3D semantic box

1. **Detectable classes:** the ncnn MobileNetV3-SSDLite (VOC) model is used, with 20 classes: `aeroplane`, `bicycle`, `bird`, `boat`, `bottle`, `bus`, `car`, `cat`, `chair`, `cow`, `diningtable`, `dog`, `horse`, `motorbike`, `person`, `pottedplant`, `sheep`, `sofa`, `train`, `tvmonitor`. The class definitions are in `class_names` in `Detector2D.cc`, and the model files are `./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.param/.bin` (from the official ncnn examples). For other classes, switch to another ncnn example or train your own.

2. **GPU is not required:** ncnn can run on CPU or GPU; GPU is faster, CPU is a bit slower. It runs without a GPU.

3. **`person` not showing a 3D box is by design, not a bug:** `person` is a highly dynamic object and is actively removed during mapping (`PointcloudMapping.cc` removes points in dynamic regions based on `isInDynamicRegion`, etc.), so its 3D box is not generated/shown. The relevant code is concentrated in `Detector2D.cc`, `Detector3D.cc`, and `PointcloudMapping.cc` under `src/sg-slam/src`.

> Related issues: [#5](https://github.com/silencht/SG-SLAM/issues/5), [#8](https://github.com/silencht/SG-SLAM/issues/8)

### Q: Can I replace it with my own trained detection model (e.g. YOLO)?

Yes — it's essentially ncnn application integration. Steps:

1. Convert your trained model into ncnn's `.param`/`.bin`;
2. Following the [official ncnn examples](https://github.com/Tencent/ncnn/tree/master/examples), modify `src/sg-slam/src/Detector2D.cc`: load the model/params to run inference, and pass the detection results to sg-slam correctly. Usually no other files need to change.

If after switching to YOLO you see **redundant semantic labels, abnormal box sizes, or unclean octree filtering**, the author judged the main cause to be errors in the **parsing and passing** of detection results in `Detector2D.cc` (class, box coordinate/size mapping), and secondarily the thresholds `detection_confidence_threshold` and `dynamic_detection_confidence_threshold`.

> Related issues: [#23](https://github.com/silencht/SG-SLAM/issues/23)

### Q: No semantic map / no detection boxes in the OpenCV window, reporting `find_blob_index_by_name failed`

The root cause is **ncnn model loading failure**: the log has `fopen ./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.param/.bin failed`, so the network has no `input`/`detection_out` node and reports `find_blob_index_by_name failed`.

The model path is a **relative path with respect to the program's execution directory** — even if the file exists, it won't be found if you start from the wrong working directory.

**Solution:** start the main program from the correct working directory — `cd` into the project directory first, then run:

```bash
cd your_sg-slam_path/src/sg-slam/
# then run the command
```

so that the relative path `./Thirdparty/ncnn_model/...` can be located.

**Troubleshooting tip:** first get the official ncnn `examples`/`benchmark` running to rule out a library problem, then focus on the model file's load path (it's relative to the execution directory, not merely whether the file exists).

> Related issues: [#39](https://github.com/silencht/SG-SLAM/issues/39)

### Q: `Detector3D` computes an object/cluster size of `-inf` (common for small objects)

The root cause is that **an empty point cloud cluster is passed to `pcl::getMinMax3D`**, and PCL returns `FLT_MAX`/`-FLT_MAX`, making the size `-inf` and the centroid wrong. This happens frequently for small objects like `bottle`, `keyboard`, `mouse` whose valid points are too few and get entirely removed by filtering/clustering.

**Mitigation suggestions (based on source analysis, not an author-confirmed conclusion):**

1. Add an empty check before computing `minPt`/`maxPt` at `Detector3D.cc` lines 133-134:

   ```cpp
   if (object_cloud_cluster->empty()) return false;
   ```

2. When small objects are often discarded, relax the parameters: reduce the statistical filter strength (increase `StddevMulThresh` or decrease `MeanK`), decrease `EuclideanClusterMinSize`, increase `EuclideanClusterTolerance`;
3. Confirm the distance falls within the valid depth range `0.5~5.0m` (`camera_valid_depth_Min/Max`, see `Detector3D.h` lines 72-73).

> ⚠️ This issue is still open and the author has not replied; the above are source-based suggestions.
>
> Related issues: [#45](https://github.com/silencht/SG-SLAM/issues/45)

---

## 5. 📷 Custom Cameras & Datasets

### Q: The TUM dataset is missing `association.txt` / no response when running (you need `associate.py` for timestamp sync)

The TUM RGB-D dataset download **does not include** the RGB-depth association file; you must generate it with the official script, otherwise the program reads no paired frames, the octree stays empty, rviz shows nothing, and the script appears unresponsive.

Download `associate.py` from the [TUM tools page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools), and run it in the sequence directory:

```bash
python associate.py rgb.txt depth.txt > associations.txt
```

Specify the generated file in the run command and it will render normally. Also confirm the executable/dataset paths in the run script are correct (you can bypass the script and test each item manually). The new version does not need `transform.launch`.

> Related issues: [#4](https://github.com/silencht/SG-SLAM/issues/4), [#44](https://github.com/silencht/SG-SLAM/issues/44)

### Q: Running with a camera (Astra Pro / LeTMC520) crashes or produces no point cloud / semantic map (topics, depth topic, intrinsic calibration)

Configuration and troubleshooting:

1. **Topic alignment:** the camera must publish RGB (`/camera/rgb/image_raw`) and depth image topics; the subscription code is in `message_filters::Subscriber` in `Examples/ros_rgbd.cc`. Make sure the published names match the subscribed names and that you can see the image streams in rviz. Modify the intrinsics in `Examples/astra_pro_camera.yaml` for your hardware and start with `run_astra_pro_camera.sh`.

2. **Intrinsic calibration:** no point cloud / semantic map is often due to incorrect intrinsic calibration — normally `cx`≈image width/2 and `cy`≈image height/2 (about 320/240 for 640×480); a calibrated `cy`≈182 is clearly wrong. Also check `DepthMapFactor`. Calibrate with [ROS camera_calibration](http://wiki.ros.org/camera_calibration); if the results differ a lot each time, the capture procedure is not rigorous.

3. **Depth topic mismatch:** on the LeTMC520, the author changed the subscribed `/camera/depth_registered/image_raw` to `/camera/depth/image_raw` (around `ros_rgbd.cc` line 74) and it returned to normal; subscribe to whichever the camera actually publishes.

4. **Locating a crash:** use `gdb ./Examples/sg_slam_ros_rgbd` to locate it, and use `cv::imshow`/`std::cout` to confirm RGB/depth actually flow into the mapping thread; you can also try adding/removing `-march=native` on `g2o` and `sg-slam`.

   ⚠️ In #9's case, gdb showed the crash in `ros::CallbackQueue::callAvailable` (`libroscpp.so`), a ROS environment problem that recovered after changing the operating system.

See the [LeTMC520 configuration doc](https://github.com/silencht/SLAM_Note/blob/master/system_environment/LeTMC520_Camera.md).

> Related issues: [#9](https://github.com/silencht/SG-SLAM/issues/9), [#22](https://github.com/silencht/SG-SLAM/issues/22)

### Q: Which files to modify to run a custom dataset with a Realsense D435i / where the semantic code is / VTK-PCL versions

**Two approaches:**

1. Record in TUM RGB-D format and use `Examples/rgbd_tum.cc`;
2. **(Recommended)** record a ROS bag that publishes two topics, RGB and Depth. For Realsense you must use the Depth **aligned to** the RGB, then modify `Examples/ros_rgbd.cc` according to the bag's actual topic names.

Running `ros_rgbd` and getting `Usage: ... path_to_vocabulary path_to_settings` means the command is **missing the vocabulary argument** — just add the vocabulary path as prompted.

The **semantic-related code** is in `Detector2D.cc`, `Detector3D.cc`, and `PointcloudMapping.cc` under `src/sg-slam/src`.

**Dependency versions:** the author used `libvtk 6.3` and `libpcl 1.8.1` (the original VTK 7.1 + PCL 1.1.0 reported errors; switching to the author's versions is recommended).

> Related issues: [#37](https://github.com/silencht/SG-SLAM/issues/37)

### Q: An octomap/point cloud map built from a self-recorded D435i/TUM dataset looks poor and objects are indistinguishable

The thread reached no technical conclusion (the author asked for a screen recording and terminal logs, then moved to email: `silencht@qq.com`). Actionable advice:

Provide a screen recording of the run and the complete terminal logs, and check the common pitfalls of a self-recorded dataset:

- Whether the RGB and depth timestamps are strictly associate-aligned;
- Whether the depth unit and `DepthMapFactor` match;
- Whether the camera intrinsics `fx/fy/cx/cy` and distortion are calibrated for the actual D435i and filled into the yaml;
- Whether the depth is already aligned to color (`aligned_depth`).

Analysis of mapping failure due to tracking loss is in the [Runtime Crashes section](#2-runtime-crashes--abnormal-exits).

> ⚠️ This issue did not receive a reproducible public answer in the comments.
>
> Related issues: [#42](https://github.com/silencht/SG-SLAM/issues/42)

---

## 6. 🧩 Algorithm & Code Details

### Q: Is the `(*it_cur, *it_pre)` argument order in `CheckEpiLineDistToRmDynamicPoint` — reversed from the paper — a bug?

**Not a bug; no change needed.** For real-time performance, the optical flow is computed backward from the current frame to the previous frame (`Frame.cc:445`: `cv::calcOpticalFlowPyrLK(imGray, imGrayPre, Curpoint, Prepoint, ...)`, where the first image is the current frame). Therefore the fundamental matrix `FundMat` is constructed in the order "current-frame feature, previous-frame feature", and the epipolar distance check follows the same order, so the argument order is `(current frame, previous frame)`.

The paper describes it in temporal order from the previous frame to the current frame, i.e. the opposite direction, but the two formulations are each self-consistent and yield the same result. To rewrite it in the paper's direction, you'd need to swap the optical-flow direction, the fundamental-matrix input order, and this function's arguments **all at once** — you cannot change just this one spot.

> Related issues: [#28](https://github.com/silencht/SG-SLAM/issues/28)

### Q: Where is the per-frame processing time measured, and where can I view it?

The timing code is spread across the tracking-related source files (corresponding to commit `a7046f0`):

- `src/sg-slam/src/Frame.cc` around line 165;
- `src/sg-slam/src/Tracking.cc` around line 235.

Timing points record/aggregate the per-frame time; other similar timing code can be found in these two files the same way.

> Related issues: [#33](https://github.com/silencht/SG-SLAM/issues/33)

---

## 7. 📊 Evaluation & Accuracy

### Q: How to reproduce the trajectory-error comparison figures in the paper (which evaluation tool)

The paper's trajectory-error data and figures **were not produced with evo**, but with the [official TUM RGB-D evaluation tools](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools#evaluation). Workflow:

1. After running a sequence, `CameraTrajectory.txt` and `KeyFrameTrajectory.txt` are generated in the run directory;
2. Use `evaluate_ate.py` and `evaluate_rpe.py` to compare these two estimated trajectories against the dataset's own `groundtruth.txt` to obtain the error data and figures.

The author did not use the [evo](https://github.com/MichaelGrupp/evo) repository (even though its plots may look nicer).

> Related issues: [#2](https://github.com/silencht/SG-SLAM/issues/2)

### Q: The ATE.rmse result is unstable and fluctuates randomly across runs — is that normal?

This is a phenomenon the author knows about but has not fully resolved: for the same dataset (e.g. `fr3_walking_xyz`), ATE.rmse fluctuates randomly across runs, roughly in the range `0.05m ~ 0.33m`.

The author's suspected causes: some parameter settings, insufficient detection-model accuracy leading to poor detection on some frames, and degeneracy of the epipolar geometry constraint. Envisioned improvement directions: switch to a better detection model, or add extra constraints (geometric constraints, IMU priors) in a controlled experiment — but this was shelved due to the time cost. If you need stable accuracy, you can try the directions above.

(Aside: the octomap config file is `src/octomap_server/launch/octomap.launch`.)

> ⚠️ The author did not fully pin down the root cause of this issue.
>
> Related issues: [#3](https://github.com/silencht/SG-SLAM/issues/3)

---

## 8. 💡 Other Conceptual Questions

### Q: How robust is ORB-SLAM2, and can it be upgraded to ORB-SLAM3 or another architecture?

The author's personal view (a conceptual discussion):

1. **Robustness** — although ORB-SLAM2's paper accuracy reaches SOTA, in practice it is very prone to loss (LOST) due to camera hardware (e.g. small FOV), weak texture, high dynamics, and especially large rotations; ORB-SLAM3 improves things by introducing an IMU but still has many issues, and overall can only be considered at an academically usable level.

2. **Upgrading to ORB-SLAM3** — the difficulty depends on how familiar you are with the code. The author thinks it isn't very hard but still involves a fair amount of work, and did not provide specific steps.

> Related issues: [#27](https://github.com/silencht/SG-SLAM/issues/27)
