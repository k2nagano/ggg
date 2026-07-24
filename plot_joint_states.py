import sys
from datetime import datetime, timezone, timedelta
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState


def extract_data_from_bag(bag_path: str):
    """MCAPファイルから必要な2つのトピックデータを抽出する"""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    # 必要な2つのトピックのみフィルタリング
    target_topics = [
        "/joint_states",
        "/joint_trajectory_controller/controller_state",
    ]
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    joint_states_data = {"time_ns": [], "datetime": [], "positions": [], "names": None}
    ref_data = {"time_ns": [], "datetime": [], "pos_index2": []}

    # ローカルタイムゾーン（システムの設定）を取得
    local_tz = datetime.now().astimezone().tzinfo

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        # ナノ秒スタンプから Python datetime オブジェクトを変換
        dt_obj = datetime.fromtimestamp(timestamp_ns / 1e9, tz=local_tz)

        if topic == "/joint_states":
            msg = deserialize_message(data, JointState)
            if joint_states_data["names"] is None and len(msg.name) > 0:
                joint_states_data["names"] = list(msg.name)
            
            if len(msg.position) > 0:
                joint_states_data["time_ns"].append(timestamp_ns)
                joint_states_data["datetime"].append(dt_obj)
                joint_states_data["positions"].append(list(msg.position))

        elif topic == "/joint_trajectory_controller/controller_state":
            msg = deserialize_message(data, JointTrajectoryControllerState)
            if hasattr(msg, "reference") and len(msg.reference.positions) > 2:
                ref_data["time_ns"].append(timestamp_ns)
                ref_data["datetime"].append(dt_obj)
                ref_data["pos_index2"].append(msg.reference.positions[2])

    return joint_states_data, ref_data


def detect_motion_time_range(
    ref_time_ns, ref_values, velocity_threshold=1e-4, margin_sec=0.5
):
    """
    reference/positions[2] の変化率(速度)から、
    フラットから動き始めて再びフラットになるまでの開始・終了時刻(datetime)を自動検出する
    """
    if len(ref_time_ns) < 2:
        return None, None

    ref_times_sec = np.array(ref_time_ns) / 1e9
    ref_values = np.array(ref_values)

    dt = np.diff(ref_times_sec)
    dv = np.diff(ref_values)
    
    dt[dt == 0] = 1e-9  # ゼロ割対策
    velocities = np.abs(dv / dt)

    moving_indices = np.where(velocities > velocity_threshold)[0]

    if len(moving_indices) == 0:
        print("警告: reference/positions[2] に有意な変化が検出されませんでした。全区間を描画します。")
        return ref_time_ns[0], ref_time_ns[-1]

    start_idx = moving_indices[0]
    end_idx = moving_indices[-1] + 1

    # マージン（0.5秒）を考慮してナノ秒で範囲を確定
    margin_ns = int(margin_sec * 1e9)
    start_ns = max(ref_time_ns[0], ref_time_ns[start_idx] - margin_ns)
    end_ns = min(ref_time_ns[-1], ref_time_ns[end_idx] + margin_ns)

    return start_ns, end_ns


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_joint_states.py <path_to_mcap_or_bag_folder>")
        sys.exit(1)

    bag_path = sys.argv[1]
    print(f"Reading bag: {bag_path} ...")
    
    js_data, ref_data = extract_data_from_bag(bag_path)

    if not js_data["datetime"]:
        print("エラー: /joint_states データが見つかりませんでした。")
        sys.exit(1)

    js_dts = js_data["datetime"]
    js_times_sec = np.array(js_data["time_ns"]) / 1e9
    js_positions = np.array(js_data["positions"])

    # 1. 範囲検出（controller_state基準）
    if ref_data["time_ns"]:
        start_ns, end_ns = detect_motion_time_range(
            ref_data["time_ns"], ref_data["pos_index2"]
        )
    else:
        print("警告: controller_state が存在しないため全時間を対象とします。")
        start_ns, end_ns = js_data["time_ns"][0], js_data["time_ns"][-1]

    local_tz = datetime.now().astimezone().tzinfo
    dt_start = datetime.fromtimestamp(start_ns / 1e9, tz=local_tz)
    dt_end = datetime.fromtimestamp(end_ns / 1e9, tz=local_tz)

    print(f"描画切り出し区間: {dt_start.strftime('%Y/%m/%d %H:%M:%S')} 〜 {dt_end.strftime('%Y/%m/%d %H:%M:%S')}")

    # 2. 配信周期 (dt [ms]) の計算と、中間点時刻の生成
    dt_series_sec = np.diff(js_times_sec)
    dt_series_ms = dt_series_sec * 1000.0

    # diff によるズレを埋めるため、連続する2つの datetime の中間時刻を算出
    dt_dts = [
        t1 + (t2 - t1) / 2
        for t1, t2 in zip(js_dts[:-1], js_dts[1:])
    ]

    # 3. グラフ描画 (2段構成)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # ---- 上段: Joint Positions [rad] ----
    num_joints = js_positions.shape[1]
    joint_names = js_data["names"] if js_data["names"] else [f"joint_{i}" for i in range(num_joints)]

    for i in range(num_joints):
        ax1.plot(js_dts, js_positions[:, i], label=joint_names[i])

    ax1.set_ylabel("Position [rad]")
    ax1.set_title("/joint_states Positions")
    ax1.grid(True)
    #ax1.legend(loc="upper right", bbox_to_anchor=(1.15, 1.0))
    ax1.legend()

    # ---- 下段: 配信間隔 / 周期 ----
    ax2.plot(
        dt_dts,
        dt_series_ms,
        color="royalblue",
        linestyle="-",
        marker="o",
        markersize=4,
        label="Message Interval (dt)",
    )

    if len(dt_series_sec) > 0:
        mean_dt_ms = np.mean(dt_series_ms)
        mean_hz = 1.0 / np.mean(dt_series_sec) if np.mean(dt_series_sec) > 0 else 0

        ax2.axhline(
            y=mean_dt_ms,
            color="red",
            linestyle="--",
            linewidth=1.5,
            label=f"Mean Interval ({mean_dt_ms:.2f} ms)",
        )

        ax2.annotate(
            f"Mean: {mean_hz:.1f} Hz ({mean_dt_ms:.2f} ms)",
            xy=(0.02, 0.85),
            xycoords="axes fraction",
            fontsize=10,
            bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8),
        )

    ax2.set_xlabel("Time from command start [s]")
    ax2.set_ylabel("Interval [ms]")
    ax2.set_title("/joint_states Publishing Interval")
    ax2.grid(True)
    ax2.legend(loc="upper right")

    # ---- X軸の日時フォーマット設定 (YYYY/mm/dd HH:MM:SS) ----
    date_formatter = mdates.DateFormatter("%Y/%m/%d %H:%M:%S")
    ax2.xaxis.set_major_formatter(date_formatter)

    # ラベルが重ならないように斜め回転
    fig.autofmt_xdate()

    # X軸の表示範囲を設定
    ax1.set_xlim(dt_start, dt_end)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
