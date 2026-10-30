# SPGA Web Lab — 发布到 GitHub Pages

## 1. 网页已经做好

无需安装 Three.js、Node.js，也无需 Python 服务器。网页只用 HTML/CSS/JavaScript；打开网站即可操作。用户画的轮廓、轨迹和 Warping 都在用户浏览器本地计算。

## 2. 发布步骤

1. 在 GitHub 创建一个新的 **Public** 仓库，推荐名字 `spga-web-lab`。
2. 把下载 ZIP 解压后，**文件夹内部**所有文件上传到仓库根目录。特别确认仓库根目录有 `index.html`。
3. 在仓库中选择 `Settings` → `Pages`。
4. 在 `Build and deployment` 中选择 `Deploy from a branch`。
5. Branch 选 `main`，Folder 选 `/(root)`，点击 `Save`。
6. 发布完成后，网站地址通常是 `https://你的GitHub用户名.github.io/spga-web-lab/`。

你的 GitHub 仓库连接当前尚未授权，因此这里提供的是**可以发布的源码**，不是已经上线的公开网址。

## 3. 推荐操作顺序

- `Surface`：Hill / Saddle 或 Draw 2D profile；拖动相机旋转/缩放，在 Move、Rotate 模式可以改变曲面位置和旋转。
- `Guide & Nominal`：调 Cut angle 与 Plane offset，看到黄色 Guide；手绘左到右的 Nominal 轨迹，纵轴代表从 Guide 横向偏移。
- `Tool & Warping`：选择棒长、Tilt 角度、Cone 半角；先 Generate Tilted，再 Warp。
- 使用下方 Play / Pose slider 查看变化，右侧可看到最大穿透、发生碰撞的姿态数量。
- `Export JSON`：导出曲面、手绘轨迹、Tilted 和 Warped A0/A1 等数据。

## 4. 与论文 M2 的区别（重要）

这是用于访问、展示和人机交互的 **Web Lab 版本**，并不等同于锁定的 Python SPGA-M2 算法。为了保证 GitHub Pages 可独立运行，浏览器中使用了连续高度场法线、离散棒碰撞采样和锥内方向搜索来演示 Warp；没有直接执行 M2 的 Geometry/Conic/Ribbon 广义力平衡，结果不能作为论文的正式实验数据。

若以后要把真正的 M2 运行结果放到这个网页上，需要另外运行 Python 后端，或完整移植并验证求解器。
