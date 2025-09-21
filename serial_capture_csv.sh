#!/usr/bin/env bash
set -euo pipefail

# Serial CSV Auto Capture
# 指定シリアルポートから受信した 8列CSV 行のみを抽出して、
# 指定ディレクトリに日時付きファイル名で保存します。
#
# 使い方:
#   ./serial_capture_csv.sh [SAVE_DIR] [UART_PORT] [BAUD]
# 例:
#   ./serial_capture_csv.sh ~/STM32Workspace/micromouse_log_visualizer/logs /dev/ttyUSB0 115200
#   ./serial_capture_csv.sh                      # 省略時は既定値を使用
#
# 終了: Ctrl+C

DEFAULT_SAVE_DIR="$HOME/STM32Workspace/micromouse_log_visualizer/logs"
DEFAULT_UART_PORT="/dev/ttyUSB0"
DEFAULT_BAUD="115200"

SAVE_DIR=${1:-$DEFAULT_SAVE_DIR}
UART_PORT=${2:-$DEFAULT_UART_PORT}
BAUD_RATE=${3:-$DEFAULT_BAUD}

mkdir -p "$SAVE_DIR"

if [[ ! -e "$UART_PORT" ]]; then
  echo "[ERROR] UART port not found: $UART_PORT" >&2
  exit 1
fi

# シリアルポート設定
stty -F "$UART_PORT" "$BAUD_RATE" cs8 -cstopb -parenb -ixon -ixoff -crtscts -echo -echoe -echok -echoctl -echoke raw

cat <<EOF
=== Serial CSV Auto Capture ===
Port       : $UART_PORT
Baud       : $BAUD_RATE
Save Dir   : $SAVE_DIR

このウィンドウを閉じるか、Ctrl+C で保存を終了します。
CSV判定: 8列 (time_ms, param1..param7) 数値のみの行だけを保存します。
ファイル分割: 最初のCSV行、または time_ms が前回より小さくなった時に新しいCSVファイルを自動作成します。
================================
EOF

# 受信 → CSV判定 → 出力
# 改行に \r が混じるため除去し、8列数値のみをファイル保存＋同時に画面表示します。
# 数値判定は整数/小数/負号を許可します。
stdbuf -oL cat "$UART_PORT" \
  | stdbuf -oL tr -d '\r' \
  | stdbuf -oL awk -F',' -v SAVE_DIR="$SAVE_DIR" '
    function isnum(x){ return x ~ /^-?[0-9]+(\.[0-9]+)?$/ }
    function newfile(){
      TIMESTAMP = strftime("%Y%m%d_%H%M%S");
      current_out = SAVE_DIR "/stm32_log_" TIMESTAMP ".csv";
      print "\n[INFO] New capture file: " current_out > "/dev/stderr"; fflush("/dev/stderr");
    }
    BEGIN {
      prev_ts = -1;
      current_out = "";
    }
    NF==8 && $1 ~ /^[0-9]+$/ && isnum($2) && isnum($3) && isnum($4) && isnum($5) && isnum($6) && isnum($7) && isnum($8) {
      ts = $1 + 0;
      if (current_out == "" || (prev_ts >= 0 && ts < prev_ts)) {
        newfile();
      }
      print $0; fflush();
      print $0 >> current_out; fflush(current_out);
      prev_ts = ts;
    }
  '

echo "\n[INFO] Capture finished. Saved files under: $SAVE_DIR"
