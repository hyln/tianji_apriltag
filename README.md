# Tianji AprilTag

独立 Python 包，订阅已由 `tianji_arm_teleop:hik-pc-cameras` 启动的相机，默认只处理 6（左腕）、7（右腕）。不打开、重启或抢占相机。

## Git 管理

本目录使用独立 Git 仓库，主仓库 `tianji_infra_group` 忽略整个目录。源码、Harbor 配置、相机标定、参考 PDF 和现有 `records/` 测量记录均纳入版本管理；Python 缓存、虚拟环境和构建输出不提交。

在本目录运行 `git status`、`git add`、`git commit` 管理改动。远程仓库尚未配置，可在确定地址后使用 `git remote add origin <仓库地址>` 关联。

当前启动脚本依赖同级的 `tianji_arm_teleop/config/global_config.sh`，crop 功能依赖同级 `tianji_data_convert`。独立 Git 管理保留现有目录布局，不改变运行时依赖。

## Tag 规格与计算

打印文件：[AprilTag_18_19_A4_45mm.pdf](AprilTag_18_19_A4_45mm.pdf)，A4 页面，裁切框 **45 × 100 mm**。它沿用原 PDF 的 tag36h11 图案：ID 18 的黑框边长 34 mm，ID 19 为 24 mm。两者中心水平对齐，竖直相差 **55.0 mm**。两个 Tag 的编码图案保持原样，右侧裁切区外的 1:1 示意图标出对应中心点与 55 mm 间距。打印时只裁灰色虚线内的干净 Tag；300 dpi 渲染后两个 Tag 都能被 OpenCV 识别。PDF 绘图指令给出的中心坐标差为 155.9055 pt，即 `155.9055 / 72 × 25.4 = 55.0 mm`。45 mm 裁切框给上方 34 mm Tag 留下左右各 5.5 mm 白边。

| ID | 黑框外边长 | 0.3 m 时约占像素 | 0.5 m | 1 m |
|---|---|---|---|---|
| 18 | 34 mm | 94–95 px | 56–57 px | 28–29 px |
| 19 | 24 mm | 66–67 px | 40 px | 20 px |

使用仓库焦距 830–841 px，正对镜头时近似 `边长像素 = 焦距像素 × 实际边长 / 深度`。表格仅是成像大小估算，并非检测距离保证；倾斜、模糊、曝光和遮挡会影响结果。先在 0.3–0.6 m 验证。如再次打印，请按 Actual size / 100% 打印，并测量 PDF 的 50 mm 检查线；4 mm 白边不计入 tag 尺寸。当前 34/24 mm 是用户提供的实物尺寸，以此计算距离。

识别使用 OpenCV AprilTag 36h11 字典和亚像素角点。先根据包内固定的鱼眼内参去畸变，再用 IPPE_SQUARE 计算两个平面位姿候选，选正深度且像素重投影误差最小者。近正视平面可能有姿态歧义，未做时间滤波、双相机外参标定或机器人基座坐标变换。

## 启动

先启动现有 Harbor 相机任务，再执行：

```bash
cd /home/seen-e-embodied/tianji_infra_group/tianji_apriltag
bash run.sh
```

或让 Harbor 搜索本包目录，运行 `apriltag-6-7`：

`Harbor:local:/home/seen-e-embodied/tianji_infra_group/tianji_apriltag:apriltag-6-7`

## 网页 UI

启动识别任务后，在 Harbor 中打开 `apriltag` 面板，或浏览器访问 http://127.0.0.1:23846/ 。网页同时显示 6、7 号相机的标注画面、Tag ID、距离、深度、XYZ、重投影误差，以及同帧两个 Tag 的三维中心距和连线中点深度。等待相机、断流和处理异常分别提示；断流时标记上次画面和上次结果。暂停按钮仅暂停网页画面刷新，识别仍继续。

直接启动可用 `--port 23846 --host 127.0.0.1` 调整监听，`--no-ui` 关闭网页。Harbor 启动时读取 `HARBOR_WEBVIEW_*` 的端口与监听设置。UI 使用 Python 标准库 HTTP 服务，不需要前端构建或额外 Web 依赖。

### OpenPI crop 原生窗口

运行 Harbor 任务 `openpi-crop-view`，或执行 `bash run_crop_view.sh`。OpenCV 窗口并排显示相机 6、7 的 224×224 crop，按 `Q` 或 `Esc` 关闭。它直接调用 `tianji_data_convert/common/wrist_undistortion.py` 的 `DirectCropper` 和 `shared_camera_intrinsics_5view224.json`，因此处理与 OpenPI 输入一致：先在原始像素尺度做 equidistant 去畸变，再分别裁取 cam6 `[488,332,224,224]`、cam7 `[488,388,224,224]`，不做 resize。

当前 crop 视角中心可用实时 ID 18、19 逐台记录。标定板放在相机 7 下时运行 `PYTHONPATH=.:.. /usr/bin/python3 -m tianji_apriltag.record_crop_center --camera 7`；移到相机 6 后改为 `--camera 6`。程序要求所选相机在同一帧识别到两张 Tag，用两者位姿建立纸面，将该相机的 OpenPI crop 中心射线与纸面求交。结果按相机分别写入 `records/openpi_crop_view_center_camera_<编号>_tag18_19.json`，不会互相覆盖。记录包含 crop/去畸变/原图坐标、相机系三维交点、深度，以及相对两 Tag 中点的偏移。

如果只需记录大 Tag（ID 18）在 OpenPI crop 中的位置，运行 `PYTHONPATH=.:.. /usr/bin/python3 -m tianji_apriltag.record_tag_crop_position --camera 6`，相机 7 则改为 `--camera 7`。结果写入 `records/openpi_crop_tag_18_position_camera_<编号>.json`，包含原图中心、去畸变中心、224 crop 坐标、相对 crop 中心的像素偏移以及是否位于 crop 内。

运行脚本加载现有 global_config，保持相同 ROS_DOMAIN_ID 与 DDS 配置。使用系统 Python 和 ROS Humble，不要求 pip 安装；系统依赖为 python3-opencv（含 aruco）、python3-numpy、ros-humble-cv-bridge、rclpy 和消息包。默认每相机最高 10 Hz，参数 `--cameras 6,7 --hz 10`。

输出：

- `/apriltag/camera_6/detections`、`/apriltag/camera_7/detections`：std_msgs/String JSON，每帧含原始时间戳、frame_id、标定版本、tag ID、尺寸、角点和 center_px；匹配固定标定时含 translation_m、rotation_vector_rad、distance_m、reprojection_error_px；同一相机同一帧都检测到 18、19 且位姿有效时含 tag_center_distance_m、reference_center_distance_m（0.055）、tag_pair_midpoint_m、tag_pair_midpoint_depth_m 和 tag_pair_midpoint_px。未检测到时 tags 为空，位姿失败时给出 pose_error。
- `/apriltag/camera_6/annotated`、`/apriltag/camera_7/annotated`：sensor_msgs/Image，在每个检测到的 Tag 中心画白边红色十字；同时检测到两个 Tag 时用黑边黄色线段直接连接两个中心，位姿有效时用黄点标记三维连线中点的像素投影，并在旁边标出该点的深度与中心距。供网页和 rqt_image_view 查看。

三维中心距采用 `||t19 − t18||`，中点采用 `(t18 + t19) / 2`；中点深度是其在相机 optical frame 下的 Z 值（沿光轴距离）。只用同一相机同一帧的两个 Tag 位姿；任一 Tag 缺失或位姿失败时不输出，且不跨相机比较。纸面平整且按 100% 打印时应接近 PDF 的 55.0 mm，可用差值检查测量结果。

相机坐标为 optical frame：x 向右、y 向下、z 向前。Tag 坐标使用 OpenCV 解码角点顺序：x 向右、y 向上、z 向外；旋转向量使用 Rodrigues 表示，单位弧度。translation_m 是 Tag 中心在相机坐标系的位置；distance_m 为欧氏距离，z 分量为深度。

## 固定的 6、7 号相机内参

参数在 [calibration.json](tianji_apriltag/calibration.json)，对应 HIK 任务输出的 **1200×1000 原图**。程序直接使用它计算位姿，不依赖相机驱动发布的 CameraInfo。

| 相机 | fx | fy | cx（ROI 坐标） | cy | 畸变模型 |
| --- | ---: | ---: | ---: | ---: | --- |
| 6 | 830.644450 | 830.673536 | 600.667426 | 565.281205 | equidistant，4 系数 |
| 7 | 841.063792 | 840.968748 | 604.834889 | 560.105799 | equidistant，4 系数 |

来源：`tianji_arm_teleop` 原有 6/7 相机配置中的各自 fx、fy、cx、cy，水平主点减去 MFS 里的 `OffsetX=116`。畸变模型及前四个系数与 `tianji_data_convert/common/wrist_undistortion.py` 和其五视角配置一致。原 YAML 的 3×3 内参字段实际写了 12 个数；它呈 3×4 投影矩阵排列。原 YAML 的鱼眼畸变多一个末尾零，现按已有去畸变程序只取前四个。五视角训练配置使用的是三台相机的**近似共用** K，这里为测距采用 6/7 各自的原始 K。

这些值继承现有标定，不是重新实测。图像分辨率不符时直接报告错误，避免错用参数；建议用已知距离检验最终精度。原图画面保持不变，角点使用 OpenCV 鱼眼去畸变与 IPPE 求位姿；没有重复对图像去畸变。

## 验证

```bash
pdftoppm -scale-to 1600 -png -singlefile AprilTag_18_19_A4_45mm.pdf /tmp/apriltag_pdf
PYTHONPATH=. /usr/bin/python3 -m unittest discover -s tests -v
```

PDF 实际识别、两台固定内参的模拟鱼眼投影恢复平移、错误标定和分辨率拒绝测试均已在系统 OpenCV 4.5.4 和 OpenCV 4.13.0 通过。

算法参考：[OpenCV ArUco / AprilTag](https://docs.opencv.org/4.7.0/d5/dae/tutorial_aruco_detection.html)、[AprilRobotics 位姿说明](https://github.com/AprilRobotics/apriltag/blob/master/README.md)。
