#!/bin/bash

# STM32 UART Flash Script for HM_Nightfall-mini_v1.1
# Usage: ./flash_uart.sh [UART_PORT] [BAUD_RATE]

# デフォルト設定
DEFAULT_PORT="/dev/ttyUSB0"
DEFAULT_BAUD="115200"
PROGRAMMER_CLI="/home/xfa273/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
BIN_FILE_DEBUG="build/Debug/HM_Nightfall-mini_v1.1.bin"
BIN_FILE_RELEASE="build/Release/HM_Nightfall-mini_v1.1.bin"
if [ -f "$BIN_FILE_DEBUG" ] && [ -f "$BIN_FILE_RELEASE" ]; then
    TS_DEBUG=$(stat -c %Y "$BIN_FILE_DEBUG" 2>/dev/null || echo 0)
    TS_RELEASE=$(stat -c %Y "$BIN_FILE_RELEASE" 2>/dev/null || echo 0)
    if [ "$TS_DEBUG" -ge "$TS_RELEASE" ]; then
        BIN_FILE="$BIN_FILE_DEBUG"
    else
        BIN_FILE="$BIN_FILE_RELEASE"
    fi
elif [ -f "$BIN_FILE_DEBUG" ]; then
    BIN_FILE="$BIN_FILE_DEBUG"
elif [ -f "$BIN_FILE_RELEASE" ]; then
    BIN_FILE="$BIN_FILE_RELEASE"
else
    BIN_FILE="$BIN_FILE_DEBUG"
fi
FLASH_ADDRESS="0x08000000"

# パラメータ設定
UART_PORT=${1:-$DEFAULT_PORT}
BAUD_RATE=${2:-$DEFAULT_BAUD}

echo "=== STM32 UART Flash Script ==="
echo "Port: $UART_PORT"
echo "Baud Rate: $BAUD_RATE"
echo "Binary File: $BIN_FILE"
echo "Flash Address: $FLASH_ADDRESS"
echo ""

# binファイルの存在確認
if [ ! -f "$BIN_FILE" ]; then
    echo "Error: Binary file not found: $BIN_FILE"
    echo "Please build the project first: cmake --build build/Debug"
    exit 1
fi

# CubeProgrammer CLIの存在確認
if [ ! -f "$PROGRAMMER_CLI" ]; then
    echo "Error: STM32_Programmer_CLI not found: $PROGRAMMER_CLI"
    exit 1
fi

echo "Starting UART flash programming..."
echo ""

# UART経由でのフラッシュ書き込み実行
# タイムアウト付きで実行（60秒）
timeout 60s $PROGRAMMER_CLI \
  -c port=$UART_PORT br=$BAUD_RATE \
  -d $BIN_FILE $FLASH_ADDRESS

PROG_EXIT_CODE=$?

echo ""
echo "Programming exit code: $PROG_EXIT_CODE"

# UARTポートを使用しているプロセスがあれば強制終了
echo "Ensuring UART port is released..."
fuser -k $UART_PORT 2>/dev/null || true

# 短時間待機してポートの完全開放を確認
sleep 1

if [ $PROG_EXIT_CODE -eq 0 ]; then
    echo "✅ Flash programming completed successfully!"
    echo "✅ UART port $UART_PORT has been released."
    echo "💡 To monitor serial output, use: ./serial_minicom.sh"
else
    echo "❌ Flash programming failed or timed out (exit code: $PROG_EXIT_CODE)"
    echo "✅ UART port $UART_PORT has been released."
fi
