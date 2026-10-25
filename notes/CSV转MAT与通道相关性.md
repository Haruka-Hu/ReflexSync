# EMG / IMU CSV 转 MAT 与通道相关性

Python 图形上位机。支持多文件拖放、文件选择、批量 MAT 转换、数据预览以及 EMG 通道相关性热力图。转换不需要 MATLAB，不连接采集硬件。

## 启动

在项目根目录执行，Windows 和 Mac 使用同一个命令：

```sh
pixi run csv-mat
```

也可启动时传入一个或多个 CSV 路径：

```sh
pixi run csv-mat "/path/to/emg48_capture.csv" "/path/to/imu_capture.csv"
```

入口为 `tools/csv_mat_gui.py`，转换与分析逻辑为 `tools/csv_mat_core.py`。使用现有 `pixi.toml` 中的 Python、NumPy、SciPy、Matplotlib 和 PySide6，不再打包为 `.app` 或 `.exe`。

## 使用

1. 拖入一个或多个 CSV，或点击“选择 CSV 文件”。点击列表中的文件，查看类型、采样率和前 30 行。
2. 点击“转换当前文件”或“批量转换”。默认写入每个 CSV 旁的 `csv_mat_exports`，也可选择统一输出目录。已有同名结果自动添加数字后缀。原 CSV 不变。
3. 对 EMG 切换到“EMG 通道相关性”。默认勾选存在变化的通道；可手动取消饱和通道，或使用全选/全不选。
4. 选择相对记录起点的时间区间，终点 0 表示记录末尾。区间为 `[起点, 终点)`。采样率 0 表示按设备时间自动检测，缺少时间列时需手动填写 Hz。
5. 默认使用原始值。可选 4 阶 Butterworth 零相位带通（初值 20–450 Hz）和 50/60 Hz 陷波（Q=30），或陷波及谐波。谐波严格小于带通上限（未带通时小于 Nyquist）。先对整段滤波，再取区间；记录边缘存在滤波边界效应。
6. 点击“计算通道相关性”，得到保留正负号、未整流、零延迟的 Pearson 相关矩阵。平均 r 和平均绝对 r 都只统计上三角非对角通道对。通道多时点击“放大热力图”查看；导出 PNG 的分辨率随通道数增加，不受窗口缩放影响。
7. “导出分析”创建独立结果目录，保存 `analysis.mat`、`correlation.csv` 和 `correlation.png`。更改文件、通道或参数后需重新计算，旧结果不会冒充新结果。

## 支持的数据

- EMG：`emg48_capture_*.csv` 一类宽表，通道名为 `ch01…ch48`，也支持其他 `ch` 加数字的大小写列名和通道数量。识别依据是列名。
- IMU：`imu_capture_*.csv`，保留 `source/source_id`、`timestamp_us`、`sample_seq`、`batch_seq`、`rx_flags`、`x/y/z/vector_norm` 等全部实际存在的列。按 `source/source_id` 分流估计采样率。
- 加速度：支持 `accel.csv` / `accel_raw.csv` 的 `x/y/z` 和 `time_unwrapped_us`。
- 其他有表头 CSV：按通用表转换，不启用 EMG 分析。旧版把 EMG 存在 `source=EMG,x/y/z` 的混合长表可完整转换，但需先整理为通道宽表才能做相关性。
- 支持 UTF-8（含 BOM）与 GB18030，以及逗号、分号、Tab 分隔。重复/空列名、行列数错误、仅表头文件会明确报错。混合文本列作为文本保留，空数值作为 NaN 保留，不自动清洗。

## MAT 变量

MAT 使用压缩的 MATLAB Level 5 格式，数值向量按列保存，可直接 `load`。参见 [SciPy savemat 格式说明](https://docs.scipy.org/doc/scipy/reference/generated/scipy.io.savemat.html)。单个变量接近 2 GB 的超大文件不适用此格式，请先分段。

|变量|含义|
|---|---|
|`columns`|原始列构成的 struct；整数保留 int64/uint64 精度，文本为 cell 数组|
|`column_names` / `column_keys`|CSV 原列名与 MATLAB 合法字段名一一对应，解决空格、中文和重名冲突|
|`emg` / `channel_names`|EMG 样本 × 通道矩阵及通道名，保留全部槽位，包括零通道|
|`imu_xyz`|IMU 原始行顺序的 N×3 矩阵，不混合重采样|
|`time_s`|每个设备流相对其首行的秒数，仍与 CSV 原行对应；各流的零点独立，不能据此声称跨设备同步|
|`streams`|每流名称、MATLAB 从 1 开始的行索引、流内时间和估计采样率|
|`fs_hz`|单流 EMG 的估计采样率；无法估计时 NaN|
|`source_file` / `source_sha256`|输入路径与读取时 SHA-256|
|`warnings` / `units`|数据提示与原始单位说明，不擅自换算 counts 到物理量|

时间列优先 `time_unwrapped_us`，其次 `timestamp_us`、`time_s`、`t_s`。对微秒时间列识别 32 位回绕。原始列、重复记录和异常记录均保留，转换阶段不会排序、插值、补点或删行。

MATLAB 示例：

```matlab
S = load('emg48_capture_20261023_135942.mat');
plot(S.time_s, double(S.emg(:, 1)));
xlabel('Time (s)'); ylabel('CSV value');
raw_timestamp = S.columns.timestamp_us;
A = load('analysis.mat');
imagesc(A.correlation, [-1, 1]); colorbar;
xticks(1:numel(A.channel_names)); xticklabels(A.channel_names);
```

`analysis.mat` 另含实际分析通道、排除的恒定通道、相关矩阵、处理后的区间数据、时间、样本数、采样率、滤波说明、请求区间、统计值和来源哈希。

## 数据质量

原始相关性不会删除异常样本；所选数据含 NaN/Inf 会报错。自动选通道会跳过全零及恒定通道，分析时也会按所选时间区间再检查。ADS1298 正负满量程值会在日志提示，由使用者决定是否取消该通道。

时间重复、倒退会阻止分析。时间间隔偏离 `1/fs` 超过 5% 时提示抖动；超过 10%，或 `sample_seq` 不连续时，滤波分析会拒绝直接处理。10% 以内的时间戳抖动按样本顺序和采样率近似等间隔滤波，不做重采样，并记录这一假设。原始相关性仍保留原行。没有时间列时，手动采样率只建立假定时间轴，不能证明采样连续。相关性本身不能单独判定硬件故障、生理正常或运动单位可分解性。

批量中单文件失败不会阻止其他文件，日志逐个报告结果。数据读取、数值分析和 MAT 压缩在后台线程执行；处理期间需等待完成后关闭窗口。

## 验证

`pixi run python -m unittest discover -s tests -p test_csv_mat.py -v` 检查 MAT 回读、整数精度、多设备分流、计时器回绕、已知正负相关、恒定通道、NaN、间断和滤波参数。`pixi run python tests/csv_mat_gui_smoke.py` 进行离屏 GUI 操作检查。已有真实数据回读记录保存在 `outputs/desktop_csv_mat_validation`。
