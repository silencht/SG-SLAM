# SG-SLAM 常见问题解答（FAQ）

> [English](./FAQ.md) · **简体中文**

本文整理自项目全部 GitHub Issues，涵盖编译依赖、运行崩溃、地图显示、目标检测、自定义相机、算法细节与评估精度等常见问题。

## 目录

- [1. 🔧 编译与依赖](#1-编译与依赖)
- [2. 💥 运行崩溃与异常退出](#2-运行崩溃与异常退出)
- [3. 🗺️ 八叉树 / 点云地图显示](#3-八叉树--点云地图显示)
- [4. 🎯 目标检测网络与模型](#4-目标检测网络与模型)
- [5. 📷 自定义相机与数据集](#5-自定义相机与数据集)
- [6. 🧩 算法与代码细节](#6-算法与代码细节)
- [7. 📊 评估与精度](#7-评估与精度)
- [8. 💡 其他概念问题](#8-其他概念问题)

---

## 1. 🔧 编译与依赖

### Q: 第三方库未编译成功 / 找不到可执行文件（`libg2o.so` 缺失、`Sim3Solver` undefined reference、`sg_slam_tum` not found）

这几类报错的共同本质是 **sg-slam 主工程并未真正链接/生成成功**，而不是 PCL 版本等表面原因。请按以下顺序排查：

1. **先确保第三方库编译通过。** 在工程根目录运行第三方库构建脚本：

   ```bash
   ./ThirdpartyBuild.sh
   ```

   该脚本负责编译 `DBoW2`、`g2o`、`ncnn`。其中 `g2o` 最常见的失败原因是找不到 `eigen3` —— 安装好 `eigen3` 后即可生成 `Thirdparty/g2o/lib/libg2o.so`。否则 `catkin_make` 会报：

   ```
   No rule to make target '.../Thirdparty/g2o/lib/libg2o.so', needed by 'libsg-slam.so'. Stop.
   ```

   注意：PCL 的 `std::auto_ptr` 只是 **deprecated warning**，不是错误，与失败无关。

2. **链接期报一批 `ORB_SLAM2::Sim3Solver` undefined reference**（如 `GetEstimatedTranslation`、`SetRansacParameters`、`iterate`、构造函数等）。`Sim3Solver.cc` 本已编入 `libsg-slam.so`，**无需手动改 CMake**。这多为 `build`/`devel` 增量缓存不一致导致，清缓存重编即可：

   ```bash
   rm -rf build/ devel/
   catkin_make
   ```

   可先用 `ldd libsg-slam.so` 查看是否有 `not found` 的依赖。

3. **运行时报 `./Examples/sg_slam_tum: not found`** 说明该目标其实没编出来。重跑几次编译命令，或仔细复查该阶段被忽略的报错，直到生成可执行文件：

   ```bash
   catkin_make --pkg sg-slam
   ```

作者参考环境：`libpcl-dev 1.8.1`、`eigen 3.3.4`。

> 相关 issue: [#10](https://github.com/silencht/SG-SLAM/issues/10), [#19](https://github.com/silencht/SG-SLAM/issues/19), [#32](https://github.com/silencht/SG-SLAM/issues/32)

### Q: ncnn 相关编译问题（找不到 `ncnnConfig.cmake` / 库路径，gcc 版本导致 `_mm256_set_m128` 未声明）

ncnn 编译失败通常有两类原因：**CMake 定位不到配置文件**，或 **编译器版本过低**。

1. **路径问题** —— CMake 找不到 `ncnnConfig.cmake`（或 `ncnn-config.cmake`）。确认所填路径下确实存在该文件，然后任选一种方式：
   - 在 CMake 里直接写该配置文件的绝对路径；
   - 在终端 `export` ncnn 路径；
   - 把 ncnn 编译产物整体拷到系统标准检索路径，例如把 `Thirdparty/ncnn/build/install/` 下的 `bin`、`lib`、`include` 三个目录拷到 `/usr/local/lib/ncnn`：

     ```bash
     sudo cp -r .../Thirdparty/ncnn/build/install/{bin,lib,include} /usr/local/lib/ncnn/
     ```

     这样 `find_package` 即可找到。

2. **报 `'_mm256_set_m128' was not declared in this scope`**（多见于 `shufflechannel_x86_avx512.cpp` 等 x86 AVX 层）。该 intrinsic 从 gcc 8 才引入，`gcc 7.5.0` 不支持。升级到 gcc/g++ 8 及以上后重编 `Thirdparty/ncnn`：

   ```bash
   sudo apt install gcc-9 g++-9
   sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90
   sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90
   ```

**排查提示：** 可先跑通 ncnn 官方 `examples`/`benchmark`，确认库本身正常。

> 相关 issue: [#11](https://github.com/silencht/SG-SLAM/issues/11), [#29](https://github.com/silencht/SG-SLAM/issues/29), [#38](https://github.com/silencht/SG-SLAM/issues/38)

### Q: 高版本 GCC 下 `std::map` 与 `Eigen::aligned_allocator` 类型不匹配的静态断言（`KeyFrameAndPose`）

报错 `std::map must have the same value_type as its allocator` 出现在 `LoopClosing`、`Optimizer` 的 `KeyFrameAndPose` typedef 处。根因是高版本 GCC（9.x/11.x）的 libstdc++ 新增了 `static_assert`，要求 allocator 的 `value_type` 与 map 的 `value_type`（即 `std::pair<const Key, T>`）完全一致，而原代码 `Eigen::aligned_allocator` 的 pair 第一个模板参数漏了 `const`。

**修复：** 给 pair 第一个类型加 `const`。以仓库 `src/sg-slam/include/LoopClosing.h` 第 49-50 行为准：

```cpp
typedef map<KeyFrame*, g2o::Sim3, std::less<KeyFrame*>,
            Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
```

等价写法为 `std::pair<KeyFrame* const, g2o::Sim3>`。`Optimizer.cc` 中同名定义按同样方式修改。参考 [知乎文章](https://zhuanlan.zhihu.com/p/218019316)。编译基于旧版 ORB-SLAM2 的工程（如 DynaSLAM）遇到同类错误处理方式一致。

⚠️ 注意：#40、#41 在解决编译问题后还报了 **运行时段错误**（构建地图时反复重置、系统被重置无法跟踪、八叉树为空），但评论区未定位到确切根因（用户提交的调试截图在 GitHub 上无法显示，讨论中断），该运行崩溃部分无最终结论。

> 相关 issue: [#12](https://github.com/silencht/SG-SLAM/issues/12), [#40](https://github.com/silencht/SG-SLAM/issues/40), [#41](https://github.com/silencht/SG-SLAM/issues/41)

### Q: 较新 PCL 导致的智能指针不匹配：`boost::make_shared` 无法赋值给 `pcl::Ptr`

新版 PCL（如 1.12）把 `pcl::PointCloud::Ptr` 定义为 `std::shared_ptr`，而 `PointcloudMapping.cc` 用 `boost::make_shared` 返回 `boost::shared_ptr`，二者不能赋值，报 `no match for 'operator='`。

**解决：** 把 `src/PointcloudMapping.cc` 构造函数中所有 `boost::make_shared` 改为 `std::make_shared`（约 23-29 行的 `semantic_objMap`、`localMap`、`temp_globalMap`、`globalMap` 等几处）：

```cpp
semantic_objMap = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
localMap        = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
temp_globalMap  = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
globalMap       = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
```

改后即可通过编译（已在 PCL 1.12 下确认有效）。

> 相关 issue: [#16](https://github.com/silencht/SG-SLAM/issues/16)

### Q: OpenCV 与 cv_bridge 版本冲突导致 `cv::FileStorage::open` 段错误

机器同时装了 OpenCV 4.2 和 3.3.1，SG-SLAM 链接 3.3.1 而 `cv_bridge` 链接 4.2。运行到 `System.cc` 第 62 行 `cv::FileStorage::open` 读取 yaml 时崩溃（报错像是文件打不开，其实 yaml 路径/内容都正常，这是版本冲突造成的假象）。

**解决：** 把 SG-SLAM 与 `cv_bridge` 统一到同一 OpenCV 版本（本例统一为 3.3.1）后即可运行。

定位手段：

```bash
gdb ./Examples/sg_slam_tum
run <path_to_vocabulary> <path_to_settings> <path_to_sequence> <path_to_association>
bt
```

`CMakeLists.txt` 开 Debug 模式重编可得更清晰的堆栈。

（该 issue 的另一半 —— rviz 地图不显示 —— 见八叉树/点云章节的 `/map` TF 问题。）

> 相关 issue: [#20](https://github.com/silencht/SG-SLAM/issues/20)

### Q: `catkin_make --pkg octomap_server` 报 `OctomapServerConfig has not been declared`

`dynamic_reconfigure` 未能生成 `OctomapServerConfig` 头文件，根因是 `.cfg` 文件缺可执行权限，且/或用了非系统 Python。

**解决：**

```bash
chmod +x src/octomap_server/cfg/*.cfg
```

若默认 `python` 被 conda 接管，切回系统 Python（`deactivate` conda 环境，或让 `.cfg` 首行 shebang 指向系统 python），然后清理重编：

```bash
rm -rf build devel
catkin_make
```

生成 `OctomapServerConfig.h` 后即可通过编译。

> 相关 issue: [#18](https://github.com/silencht/SG-SLAM/issues/18)

### Q: Pangolin v0.5 编译失败：`AVFMT_RAWPICTURE` 未声明

Pangolin v0.5 的 ffmpeg 驱动（`src/video/drivers/ffmpeg.cpp:564`）引用了已被新版 FFmpeg 移除的 `AVFMT_RAWPICTURE` 宏。

**两种解决方式（任选其一）：**

1. **直接改用 Pangolin master/最新分支编译**（实测可通过，SG-SLAM 可正常使用，不强制必须用 v0.5）；
2. 若坚持用 v0.5，参考 [该博客](https://blog.csdn.net/Robert_Q/article/details/121690089) 规避该 FFmpeg 报错。

> 相关 issue: [#43](https://github.com/silencht/SG-SLAM/issues/43)

### Q: VSCode Debug 报 `std::bad_alloc`（并非真的内存不足）

以 Debug 模式重编后，在 `System.cc` 加载 ORB 词典 `new ORBVocabulary()` 处抛 `std::bad_alloc`，但机器仍有大量空闲内存 —— 这是 **VSCode 的 C++/调试环境配置有误**，不是内存不够。

**解决：** 重新校对 `launch.json`/`tasks.json`，注意区分 VSCode 原生 debug 与 CMake Tools 插件 debug 两套逻辑，不要混用。参考 [VSCode C++ 环境配置](https://zhuanlan.zhihu.com/p/554248108) 与 [两套 debug 逻辑说明](https://www.zhihu.com/question/471176365/answer/2386359912)。

补充：作者一般用 Release 运行（Debug 太慢），中间变量多用打印查看。

> 相关 issue: [#26](https://github.com/silencht/SG-SLAM/issues/26)

---

## 2. 💥 运行崩溃与异常退出

### Q: 运行数据集时段错误/窗口闪退，gdb 指向 g2o `linearizeOplus`（`-march=native` 编译标志不一致 / Eigen 内存对齐）

这是 ORB-SLAM2 系列的经典问题：主工程与各库（`g2o`、`DBoW2`、`Eigen`、`Pangolin`、`PCL`）编译时的 `-march=native` 优化标志不一致，导致在 `g2o::EdgeSE3ProjectXYZOnlyPose::linearizeOplus()` 处内存对齐/指令集不匹配而段错误（常在 `New map created with ... points` 之后崩溃）。

**解决：** 让所有相关 `CMakeLists.txt` 的 `-march=native` 保持一致（**全加或全删**）。关键坑在于 apt 装的 `libpcl-dev` 默认 **不带** `-march=native`，因此最稳妥的做法是把以下 `CMakeLists.txt` 里的 `-march=native` **全部删除**：

- `./eigen/CMakeLists.txt`
- `./Pangolin/CMakeLists.txt`
- `./SG-SLAM/src/sg-slam/CMakeLists.txt`
- `./SG-SLAM/src/sg-slam/Thirdparty/g2o/CMakeLists.txt`
- `./SG-SLAM/src/sg-slam/Thirdparty/DBoW2/CMakeLists.txt`

用 `ldd libsg-slam.so | grep pcl` 确认实际链接的 PCL。改后 **必须清空各 `build` 目录的 cmake 缓存再完整重编**。参考 [ORB_SLAM2 PR #844](https://github.com/raulmur/ORB_SLAM2/pull/844)、g2o PR #314、rtabmap issue #637/#685。

> 相关 issue: [#21](https://github.com/silencht/SG-SLAM/issues/21), [#24](https://github.com/silencht/SG-SLAM/issues/24), [#31](https://github.com/silencht/SG-SLAM/issues/31)

### Q: 换其他数据集运行时出现段错误（用 gdb 定位）

SG-SLAM 并非只能跑 `walking_xyz`，其它动态序列同样支持。换序列后段错误多为 **数据集路径/associate 关联文件配置不对** 或个别帧越界。

通用排查：

```bash
gdb --args ./可执行程序 参数...
run
# 崩溃后：
bt
```

用 `bt` 查看调用栈即可锁定出错代码行。提问者据此排查后已解决（未公开具体改动）。

> 相关 issue: [#13](https://github.com/silencht/SG-SLAM/issues/13)

### Q: 跟踪丢失（track lost）/ 无窗口无响应 / 缺少 tf 报 `A is singular`

跟踪一旦丢失，系统就不再发布位姿 tf，八叉树建图因缺 tf 而失败并伴随 `A is singular, this shouldn't happen`。理论上 track 不应丢特征点，可先按 README 调整 `TUM3.yaml` 中影响跟踪鲁棒性的参数。

跟踪丢失的常见原因：

- 设备算力不足，后端计算慢拖累前端；
- 场景高度动态，正确匹配的特征点过少；
- 弱纹理 / 小 FOV / 重复纹理；
- 光照过亮、过暗或突变；
- 相机运动过于剧烈（大旋转、抖动、遮挡）；
- 深度相机对水、玻璃等物体扫描不佳；
- 前期建图规模太小，后续来不及纠正即挂掉。

**自建数据集建议：** 放慢相机运动、保证充足纹理与稳定光照、避免快速大旋转与遮挡。对「无窗口/无响应」需提供更完整的运行日志才能定位。

> 相关 issue: [#7](https://github.com/silencht/SG-SLAM/issues/7), [#34](https://github.com/silencht/SG-SLAM/issues/34)

### Q: 程序运行到 PCL 统计滤波/欧式聚类处卡死无报错

在 `Detector3D::DetectOne` 中执行 `mSor_Detect3D.filter(...)`（统计滤波）或 `ec.extract(...)`（欧式聚类）时卡住不报错。作者未确认根因，以下为讨论性排查建议：

1. **确认真实阻塞位置。** 程序为多线程，卡死点可能是其他线程出问题拖住了当前线程，建议加打印/断点确认真实阻塞位置；
2. **调整 yaml 中的 PCL 参数：**
   - 统计滤波：`setMeanK`（`Detect3D_Sor_MeanK_`）、`setStddevMulThresh`（`Detect3D_Sor_StddevMulThresh_`）；
   - 体素滤波：`setLeafSize`（`Detect3D_Voxel_LeafSize_`）；
   - 欧式聚类：`setClusterTolerance`、`setMinClusterSize`、`setMaxClusterSize`。

   参数不当（聚类容差过小、最大簇尺寸过大）会使耗时极长，表现为「卡死」。

> ⚠️ 该问题无确切根因结论，需自行排查。
>
> 相关 issue: [#15](https://github.com/silencht/SG-SLAM/issues/15)

---

## 3. 🗺️ 八叉树 / 点云地图显示

### Q: rviz 中什么都不显示 / octomap 报 `Nothing to publish, octree is empty`

octree 为空/无显示通常是两方面原因：**显示端未订阅话题**，或 **主程序没真正在发布点云**。排查步骤：

1. **加载工程 rviz 配置，不要直接开 `rviz`：**

   ```bash
   rviz -d .../src/sg-slam/Examples/rvizconfig.rviz
   ```

   或在 rviz 的 `File` 菜单里手动加载该文件。需要订阅的话题包括 `/occupied_cells_vis_array`、`/SG_SLAM/Semantic_Objects`、`/octomap_point_cloud_centers` 等；八叉树用 `Add → By topic → occupied_cells_vis_array` 下的 `MarkerArray`。

2. **octree 为空多因 SG-SLAM 没跑起来。** 检查 `run_tum_walking_xyz.sh` 里数据集、`ORBvoc`、yaml 路径（相对路径有问题先改绝对路径）。TUM 数据集 **必须先用 `associate.py` 做 RGB/深度时间戳同步**，否则读不到配对帧、octree 为空。

3. **逐一确认数据：**

   ```bash
   rostopic list
   rostopic echo <话题名>
   ```

   若点云话题能 `echo` 到、八叉树话题 `echo` 不到，说明点云未被 `octomap_server` 累积进八叉树。

4. **报 `/map` 与 `map` 转换失败** 见本章 TF 问题。新版本 **无需 `transform.launch`**（tf 变换逻辑已内置进程序）。

`Uninitialized quaternion, assuming identity` 等 WARN 可暂时忽略。作者最终建议在 octomap 代码里加打印辅助定位（#17 未彻底解决）。

> 相关 issue: [#1](https://github.com/silencht/SG-SLAM/issues/1), [#4](https://github.com/silencht/SG-SLAM/issues/4), [#17](https://github.com/silencht/SG-SLAM/issues/17)

### Q: 八叉树无颜色 / 报 `Colored map requested... please define COLOR_OCTOMAP_SERVER`

launch 里设了 `colored_map=true`，却启动了不带颜色的 `octomap_server_node`；或加载的是 apt 系统包而非工程自带（已开 `COLOR_OCTOMAP_SERVER` 宏）的彩色版本。

**解决：**

1. 把 `src/octomap_server/launch/octomap.launch` 第 11 行的节点类型改为彩色版本，并保留 `colored_map=true`：

   ```xml
   <node pkg="octomap_server" type="octomap_color_server_node" name="octomap_server">
       <param name="colored_map" type="bool" value="true"/>
   ```

   `octomap_color_server_node` 已在 `CMakeLists.txt` 中定义并链接带 `COLOR_OCTOMAP_SERVER` 宏的 `_color` 库，`OctomapServer.h` 第 74 行已启用该宏，**无需自己改 CMake**。不需要颜色则把 `colored_map` 设为 `false`，继续用普通 `octomap_server_node`，报错同样消失。

2. 重新编译并加载环境：

   ```bash
   catkin_make --pkg octomap_server
   source devel/setup.bash
   roslaunch octomap.launch
   ```

   必要时谨慎卸载系统包：`sudo apt-get remove ros-melodic-octomap-server`。

3. 确保输入点云（默认 remap 到 `/SG_SLAM/Local_Point_Clouds`）带 RGB 字段。

`octree is empty / Nothing to publish` 属正常提示，需先运行 SG-SLAM 主程序发布点云。参考 CSDN 博客 [109003337](https://blog.csdn.net/weixin_44694952/article/details/109003337)、[106803808](https://blog.csdn.net/qq_34131212/article/details/106803808)。

> 相关 issue: [#14](https://github.com/silencht/SG-SLAM/issues/14), [#25](https://github.com/silencht/SG-SLAM/issues/25), [#36](https://github.com/silencht/SG-SLAM/issues/36)

### Q: rviz TF 报错：无法在 `/map` 与 `map` 坐标系之间转换（TF/TF2 迁移，多见于 Ubuntu 20）

ROS 从 TF 迁移到 TF2 后，不再兼容带前导斜杠的 `frame_id`，`/map` 与 `map` 无法匹配（Ubuntu 18 用 `/map` 不复现）。报错形如 `Error transforming pose from frame '/map' to frame 'map'`。

**解决：** 把源码/配置里所有 `/map` 统一改为 `map`（去掉前导斜杠）。需改位置（基于 commit `69f5a06`）：

- `src/octomap_server/launch/octomap.launch` 第 13 行
- `src/sg-slam/Examples/rgbd_tum.cc` 第 207 行
- `src/sg-slam/Examples/ros_rgbd.cc` 第 160 行
- `src/sg-slam/src/PointcloudMapping.cc` 第 284 / 352 / 387 / 404 行

改完重编，三种地图（八叉树、语义对象度量地图、三维点云地图）即可显示。有反馈者据此修改后成功。参考 [tf2 Migration](http://wiki.ros.org/tf2/Migration)。

> 相关 issue: [#1](https://github.com/silencht/SG-SLAM/issues/1), [#20](https://github.com/silencht/SG-SLAM/issues/20), [#30](https://github.com/silencht/SG-SLAM/issues/30)

### Q: 语义三维检测框在 rviz 中的位置与实际不符

检测到 chair 后，rviz 中的蓝色三维框位置与语义分割结果对不上。作者未定论，排查思路：

先确认三维框相对哪个参考系发布，并与 octomap 中同一物体位置对比 —— 若三维框与 octomap 中物体 **对齐**、只是与图像语义分割视觉上不重合，多为不同话题 `frame_id`/TF 或相机视角不一致导致的 **观感偏差**，而非定位错误。

建议补充：使用的数据集/相机、三维框话题的 `frame_id`、以及框与 octomap 是否对齐等信息。

> ⚠️ OP 未再回复，问题未确认解决，需自行排查。
>
> 相关 issue: [#6](https://github.com/silencht/SG-SLAM/issues/6)

### Q: 三维点云地图只显示部分/稀疏，`Global_Point_Clouds` 完全不显示

先区分两个话题：

- `/SG_SLAM/Global_Point_Clouds` —— 全局点云地图；
- `/SG_SLAM/Local_Point_Clouds` —— 当前关键帧点云。

**点云稀疏的常见原因：** 动态对象（如人）被剔除；出于效率只处理关键帧；高反射物体（玻璃/水）的低质量深度被剔除；高动态导致关键帧短暂丢失。

**Global 完全不显示** 可检查：RViz 中 `Fixed Frame` 与点云 frame 是否一致、`camera.yaml` 相机内参/深度尺度是否正确。

> ⚠️ 作者索要 `camera.yaml`/日志进一步排查，但用户未回复，未给确切结论。
>
> 相关 issue: [#35](https://github.com/silencht/SG-SLAM/issues/35)

### Q: 运行结束后如何保存点云 `.pcd` 文件

**支持，但默认关闭。** 把 yaml（如 `Examples/astra_pro_camera.yaml`）中的开关由 `0` 改为 `1`：

```yaml
PointCloudMapping.is_global_pc_reconstruction: 1
```

开启后，程序 **正常退出时**（`ros_rgbd` 在 `ros::spin()` 结束后、`rgbd_tum` 跑完序列后）会取 `GetPointCloudMapper()->get_globalMap()`，非空则调用 `pcl::io::savePCDFileBinary` 写出 `global_pcd.pcd` 到当前工作目录（代码见 `ros_rgbd.cc` 82-90 行、`rgbd_tum.cc` 164-170 行）。

⚠️ 仅正常退出时写出，`Ctrl+C` 强杀或崩溃可能不生成。

八叉树语义地图（`.bt`/`.ot`）可用 `octomap_server` 的 `octomap_saver` 保存（`octomap_saver <mapfile.[bt|ot]>`），或置 `PointCloudMapping.is_octo_semantic_map_construction` 为 `1`。

> 相关 issue: [#46](https://github.com/silencht/SG-SLAM/issues/46)

---

## 4. 🎯 目标检测网络与模型

### Q: 能检测哪些目标 / 是否必须用 GPU / 为何 person、pottedplant 不显示三维语义框

1. **可检测类别：** 使用 ncnn 的 MobileNetV3-SSDLite（VOC）模型，共 20 类：`aeroplane`、`bicycle`、`bird`、`boat`、`bottle`、`bus`、`car`、`cat`、`chair`、`cow`、`diningtable`、`dog`、`horse`、`motorbike`、`person`、`pottedplant`、`sheep`、`sofa`、`train`、`tvmonitor`。类别定义见 `Detector2D.cc` 的 `class_names`，模型文件为 `./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.param/.bin`（源自 ncnn 官方示例）。需其他类别可换 ncnn 其他示例或自训练。

2. **GPU 非必需：** ncnn 可跑 CPU 或 GPU，GPU 更快、CPU 稍慢。没有 GPU 也能运行。

3. **person 不显示三维框是设计行为，非 bug：** person 属高度动态物体，在建图阶段被主动剔除（`PointcloudMapping.cc` 依据 `isInDynamicRegion` 等对动态区域点剔除），故不生成/显示其三维框。相关代码集中在 `src/sg-slam/src` 的 `Detector2D.cc`、`Detector3D.cc`、`PointcloudMapping.cc`。

> 相关 issue: [#5](https://github.com/silencht/SG-SLAM/issues/5), [#8](https://github.com/silencht/SG-SLAM/issues/8)

### Q: 能否替换成自己训练的检测模型（如 YOLO）

可以，本质是 ncnn 应用集成。步骤：

1. 把训练好的模型转成 ncnn 的 `.param`/`.bin`；
2. 参照 [ncnn 官方 examples](https://github.com/Tencent/ncnn/tree/master/examples) 修改 `src/sg-slam/src/Detector2D.cc`：加载模型/参数做推理，并把检测结果正确传给 sg-slam，通常无需改其他文件。

换 YOLO 后若出现 **语义标签冗余、目标框尺寸异常、八叉树过滤不干净**，作者判断主要是 `Detector2D.cc` 中检测结果的 **解析与传递**（类别、框坐标/尺寸映射）有误，其次排查 `detection_confidence_threshold`、`dynamic_detection_confidence_threshold` 等阈值。

> 相关 issue: [#23](https://github.com/silencht/SG-SLAM/issues/23)

### Q: 无语义地图 / OpenCV 窗口无检测框，报 `find_blob_index_by_name failed`

根因是 **ncnn 模型加载失败**：日志有 `fopen ./Thirdparty/ncnn_model/mobilenetv3_ssdlite_voc.param/.bin failed`，进而网络无 `input`/`detection_out` 节点报 `find_blob_index_by_name failed`。

模型路径是 **相对于程序执行路径的相对路径**，文件即使存在，在错误工作目录启动也找不到。

**解决：** 在正确工作目录启动主程序 —— 先 `cd` 到工程目录再运行：

```bash
cd your_sg-slam_path/src/sg-slam/
# 再执行运行命令
```

使相对路径 `./Thirdparty/ncnn_model/...` 能被定位。

**排查提示：** 可先跑通 ncnn 官方 `examples`/`benchmark` 排除库本身问题，再重点查模型文件的加载路径（是相对执行路径，而非文件是否存在）。

> 相关 issue: [#39](https://github.com/silencht/SG-SLAM/issues/39)

### Q: `Detector3D` 计算出的物体/聚类尺寸为 `-inf`（小物体常见）

根因是 **空点云聚类被送入 `pcl::getMinMax3D`**，PCL 返回 `FLT_MAX`/`-FLT_MAX`，导致 size 为 `-inf`、质心错误。`bottle`、`keyboard`、`mouse` 等小物体的有效点太少、被滤波/聚类全部剔除时频发。

**规避建议（基于源码分析，非作者确认结论）：**

1. 在 `Detector3D.cc` 第 133-134 行计算 `minPt`/`maxPt` 前加判空：

   ```cpp
   if (object_cloud_cluster->empty()) return false;
   ```

2. 小物体常被丢弃时放宽参数：减小统计滤波强度（增大 `StddevMulThresh` 或减小 `MeanK`）、减小 `EuclideanClusterMinSize`、增大 `EuclideanClusterTolerance`；
3. 确认距离落在有效深度区间 `0.5~5.0m`（`camera_valid_depth_Min/Max`，见 `Detector3D.h` 72-73 行）。

> ⚠️ 该 issue 仍 open、作者未回复，以上为源码建议。
>
> 相关 issue: [#45](https://github.com/silencht/SG-SLAM/issues/45)

---

## 5. 📷 自定义相机与数据集

### Q: TUM 数据集缺少 `association.txt` / 运行无反应（需用 `associate.py` 做时间戳同步）

TUM RGB-D 数据集下载包 **不自带** RGB 与深度的关联文件，需用官方脚本自行生成，否则程序读不到配对帧、octree 为空、rviz 无显示、脚本看似无反应。

到 [TUM 工具页](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools) 下载 `associate.py`，在序列目录执行：

```bash
python associate.py rgb.txt depth.txt > associations.txt
```

把生成的文件在运行命令中指定即可正常出图。另需确认 run 脚本里可执行文件/数据集路径正确（可绕过脚本手动逐项测试）。新版本无需 `transform.launch`。

> 相关 issue: [#4](https://github.com/silencht/SG-SLAM/issues/4), [#44](https://github.com/silencht/SG-SLAM/issues/44)

### Q: 使用相机运行（Astra Pro / LeTMC520）闪退或无点云/语义地图（话题、深度话题、内参标定）

配置与排查：

1. **话题对齐：** 相机需发布 RGB（`/camera/rgb/image_raw`）与深度图话题，订阅代码在 `Examples/ros_rgbd.cc` 的 `message_filters::Subscriber`，确保发布名与订阅名一致并能在 rviz 看到图像流。按硬件修改 `Examples/astra_pro_camera.yaml` 内参，用 `run_astra_pro_camera.sh` 启动。

2. **内参标定：** 无点云/语义地图常因内参标定错误 —— 正常 `cx`≈图宽/2、`cy`≈图高/2（640×480 时约 320/240），若标定出 `cy`≈182 明显异常；同时核对 `DepthMapFactor`。标定用 [ROS camera_calibration](http://wiki.ros.org/camera_calibration)，结果每次差异大说明采集不规范。

3. **深度话题不匹配：** 作者在 LeTMC520 上把订阅的 `/camera/depth_registered/image_raw` 改为 `/camera/depth/image_raw`（约 `ros_rgbd.cc` 第 74 行）后恢复正常；相机只发哪个就订哪个。

4. **闪退定位：** 用 `gdb ./Examples/sg_slam_ros_rgbd` 定位，并用 `cv::imshow`/`std::cout` 确认 RGB/深度真正流入建图线程；可试在 `g2o` 与 `sg-slam` 增删 `-march=native`。

   ⚠️ #9 本例中 gdb 显示崩溃在 `ros::CallbackQueue::callAvailable`（`libroscpp.so`），属 ROS 环境问题，更换操作系统后即恢复。

参考 [LeTMC520 配置文档](https://github.com/silencht/SLAM_Note/blob/master/system_environment/LeTMC520_Camera.md)。

> 相关 issue: [#9](https://github.com/silencht/SG-SLAM/issues/9), [#22](https://github.com/silencht/SG-SLAM/issues/22)

### Q: 用 Realsense D435i 跑自制数据集需改哪些文件 / 语义代码位置 / VTK-PCL 版本

**两种方式：**

1. 按 TUM RGB-D 格式录制，用 `Examples/rgbd_tum.cc`；
2. **（推荐）** 录 ROS bag 发布 RGB 与 Depth 两路话题，Realsense 必须用与 RGB **对齐后的** Depth，再按 bag 实际话题名改 `Examples/ros_rgbd.cc`。

运行 `ros_rgbd` 报 `Usage: ... path_to_vocabulary path_to_settings` 是命令 **缺少 vocabulary 参数**，按提示补上词袋路径即可。

**语义相关代码** 位于 `src/sg-slam/src` 的 `Detector2D.cc`、`Detector3D.cc`、`PointcloudMapping.cc`。

**依赖版本：** 作者用 `libvtk 6.3`、`libpcl 1.8.1`（原用 VTK 7.1 + PCL 1.1.0 报错，建议改为作者版本）。

> 相关 issue: [#37](https://github.com/silencht/SG-SLAM/issues/37)

### Q: 自录 D435i/TUM 数据集建立的八叉树/点云地图效果差、无法分辨物体

帖内未给技术结论（作者索要运行录屏与终端日志后转邮件沟通：`silencht@qq.com`）。可操作建议：

先提供运行录屏与完整终端日志，并检查自录数据集常见坑：

- RGB 与 depth 时间戳是否严格 associate 对齐；
- 深度单位与 `DepthMapFactor` 是否匹配；
- 相机内参 `fx/fy/cx/cy` 与畸变是否按 D435i 实际标定填入 yaml；
- 深度是否已对齐到彩色（`aligned_depth`）。

跟踪丢失导致建图失败的相关分析见 [运行崩溃章节](#2-运行崩溃与异常退出)。

> ⚠️ 该问题在评论区未得到可复现的公开解答。
>
> 相关 issue: [#42](https://github.com/silencht/SG-SLAM/issues/42)

---

## 6. 🧩 算法与代码细节

### Q: `CheckEpiLineDistToRmDynamicPoint` 传参顺序 `(*it_cur, *it_pre)` 与论文相反是否为 bug

**不是 bug，无需修改。** 代码为兼顾实时性，光流是从当前帧反推到前一帧（`Frame.cc:445` 的 `cv::calcOpticalFlowPyrLK(imGray, imGrayPre, Curpoint, Prepoint, ...)`，第一个图像是当前帧），因此基础矩阵 `FundMat` 的构造顺序即为「当前帧特征、前一帧特征」，极线距离检查沿用同一顺序，故传参是 `(当前帧, 前一帧)`。

论文按时间顺序从前一帧到当前帧描述，方向相反，但两套逻辑各自自洽、结果一致。若要按论文方向改写，需 **同时** 对调光流推算方向、基础矩阵输入顺序和该函数传参，不能只改这一处。

> 相关 issue: [#28](https://github.com/silencht/SG-SLAM/issues/28)

### Q: 每帧处理时间的统计代码在哪里实现与查看

计时代码分散在跟踪相关源文件（对应 commit `a7046f0`）：

- `src/sg-slam/src/Frame.cc` 第 165 行附近；
- `src/sg-slam/src/Tracking.cc` 第 235 行附近。

通过计时点记录/统计单帧耗时，其他类似耗时统计可按同样方式在这两个文件中查找。

> 相关 issue: [#33](https://github.com/silencht/SG-SLAM/issues/33)

---

## 7. 📊 评估与精度

### Q: 如何复现论文中的轨迹误差对比图（用什么评估工具）

论文的轨迹误差数据与图 **并非用 evo**，而是 [TUM RGB-D 官方评估工具](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/tools#evaluation)。流程：

1. 跑完序列后在运行目录生成 `CameraTrajectory.txt` 和 `KeyFrameTrajectory.txt`；
2. 用 `evaluate_ate.py` 和 `evaluate_rpe.py` 将这两个估计轨迹与数据集自带的 `groundtruth.txt` 对比，即可得误差数据与图。

作者未使用 [evo](https://github.com/MichaelGrupp/evo) 仓库（尽管其出图可能更美观）。

> 相关 issue: [#2](https://github.com/silencht/SG-SLAM/issues/2)

### Q: ATE.rmse 结果不稳定、多次运行随机波动是否正常

属作者已知但未彻底解决的现象：同一数据集（如 `fr3_walking_xyz`）多次运行间 ATE.rmse 会在约 `0.05m ~ 0.33m` 随机波动。

作者推测原因：部分参数设置、目标检测模型精度不足致个别帧检测不佳、对极几何约束退化。设想改进方向：换更好的检测模型，或增加额外约束（几何约束、IMU 先验）做控制变量实验，但因耗时搁置。追求稳定精度可尝试上述方向。

（附：octomap 配置文件即 `src/octomap_server/launch/octomap.launch`。）

> ⚠️ 该问题作者未彻底定位根因。
>
> 相关 issue: [#3](https://github.com/silencht/SG-SLAM/issues/3)

---

## 8. 💡 其他概念问题

### Q: ORB-SLAM2 的鲁棒性如何、能否升级到 ORB-SLAM3 或其他架构

作者个人看法（概念性讨论）：

1. **鲁棒性** —— ORB-SLAM2 论文精度虽达 SOTA，但实际受相机硬件（如 FOV 小）、弱纹理、高动态、尤其大旋转等影响非常容易丢失（LOST）；ORB-SLAM3 引入 IMU 后有所改善但仍有很多问题，整体只能说达到学术界可用水平。

2. **升级到 ORB-SLAM3** —— 难度取决于对代码的熟悉程度，作者认为难度不高但仍有一定工作量，未给具体步骤。

> 相关 issue: [#27](https://github.com/silencht/SG-SLAM/issues/27)