# Mac(Windsurf)向け 引き継ぎ: Nightfall-mini ビルド/RAM overflow

## 目的

Mac(M4 MacBook Air)側の Windsurf 内 AI が、Nightfall-mini のビルド不具合（RAM overflow）を自走で切り分け・修正できるように、必要情報をまとめます。

## 重要: ビルド対象の混同について

このワークスペースには 2 つのフォルダがあり、ビルド対象を間違えると状況が変わります。

- `HM_Nightfall-mini_v1/`
  - `master` を指していることが多い
  - Linux 側では `origin/master [behind 20]` になっている状態を確認
- `HM_Nightfall-mini_v1.1/`
  - Linux 側では `migration-2025-12-24` を追従している状態を確認
  - 実機運用は「-mini は v1.1 を使用していた」ため **こちらが実質の対象**

注意:

- `HM_Nightfall-mini_v1.1/` の `CMakeLists.txt` で `CMAKE_PROJECT_NAME` を `HM_Nightfall-mini_v1.1` にしておくと、生成物名も v1.1 になり、v1 と混同しにくくなります。
  - 例: `HM_Nightfall-mini_v1.1.elf/.map` が生成される

## MCU/メモリ前提

- MCU: STM32F405RG 系
- FLASH: 1024KB
- RAM: 128KB
- CCMRAM: 64KB

Linux 側の `HM_Nightfall-mini_v1.1/STM32F405XX_FLASH.ld` で以下を確認:

- `RAM ORIGIN = 0x20000000, LENGTH = 128K`
- `CCMRAM ORIGIN = 0x10000000, LENGTH = 64K`

## Mac での推奨ビルド手順（Ninja 想定）

### 1) ツールチェーン

- `arm-none-eabi-gcc` が入っていること
- `cmake` と `ninja` が入っていること

### 2) CMake configure（ビルドディレクトリを分ける）

generator 不一致（以前 Makefiles で作った build を Ninja で再利用等）を避けるため、ビルド dir を分けます。

例（Debug）:

```bash
cmake -S . -B build/Debug -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
```

例（MinSizeRel）:

```bash
cmake -S . -B build/MinSizeRel -G Ninja \
  -DCMAKE_BUILD_TYPE=MinSizeRel \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
```

### 3) build

```bash
cmake --build build/Debug -j
```

## 現象: RAM overflow（Linux で再現済み）

Linux 側で `HM_Nightfall-mini_v1.1/` を Ninja でビルドしたところ、Debug/MinSizeRel ともにリンクで失敗しました。

エラー（代表）:

- `section .bss will not fit in region RAM`
- `region RAM overflowed by 44264 bytes`（前後数バイト差あり）

メモリ使用量（--print-memory-usage）:

- RAM: `175336 B / 128 KB (133.77%)`
- CCMRAM: `0 B / 64 KB (0%)`
- FLASH: Debug で `145528 B`（MinSizeRel で `99860 B`）

重要:

- 最適化を変えても `.bss` 由来のため、RAM はほぼ改善しません。
- つまり「巨大な静的領域（グローバル/static 配列）」が主因です。

## 原因（.map から判明）

Linux 側で `.map` を解析した結果、`.bss` の大半を以下の配列が消費しています。

### 1) solver.c の static ワーク領域

`HM_Nightfall-mini_v1.1/Core/Src/solver.c`:

- `static NodeCost g_nodes[MAZE_SIZE][MAZE_SIZE];`
  - `.bss.g_nodes` = **28672 bytes**
- `static Pos2D g_path_buf[MAZE_SIZE * MAZE_SIZE * 2];`
  - **16384 bytes** 相当（Pos2D=8bytes, 要素数 2048）
- `static Pos2D best_path_buf[MAZE_SIZE * MAZE_SIZE];`（solver.c 内で static）
  - **8192 bytes** 相当（要素数 1024）

### 2) main.c 由来の大きめ配列

`map/smap/route/path/visited` 等の配列が積み上がっています。

補足:

- `MAZE_SIZE` は `Core/Inc/params.h` で **32**。
- `.map` では CCMRAM が 0 使用のため、CCMRAM へ逃がす余地があります。

## Mac 側での解析手順（.elf が生成されなくても OK）

リンクに失敗しても `.map` は生成されることがあります。

### `.map`から大きい RAM 要因を抽出（Python）

`.map` が `build/Debug/HM_Nightfall-mini_v1.1.map` にある前提:

```bash
python3 - <<'PY'
import re
from pathlib import Path
p=Path('build/Debug/HM_Nightfall-mini_v1.1.map')
lines=p.read_text(errors='ignore').splitlines()
pat=re.compile(r'^\s+\.(bss[^\s]*)\s+0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s*(.*)$')
items=[]
for line in lines:
    m=pat.match(line)
    if not m:
        continue
    sect=m.group(1)
    size=int(m.group(2),16)
    rest=m.group(3).strip()
    items.append((size,sect,rest))
items.sort(reverse=True)
for size,sect,rest in items[:40]:
    print(f"{size:8d} {sect:20s} {rest}")
PY
```

## 対処方針（優先順）

### 方針 A: 配列サイズ/型を削減（おすすめ）

- `NodeCost` が float/enum/int で肥大化しています
- `g_path_buf/best_path_buf` の `Pos2D` も `int` で 8byte 消費

案:

- `Pos2D` を `int8_t` or `uint8_t` に（迷路サイズ 32 なら十分）
- `NodeCost` の `dist` を float から固定小数（`uint16_t` 等）へ
- `g_path_buf` の容量を下げる（本当に `*2` が必要か確認）

### 方針 B: 大きい配列を CCMRAM へ配置

CCMRAM(64KB)が未使用なので、`.bss` から `.ccmram` へ逃がすと通る可能性が高いです。

ただし注意:

- 現状の `startup_stm32f405xx.s` は `.ccmram` を 0 クリアしていない可能性があります（文字列検索で `_sccmram/_eccmram` が見つからない）。
- `.ccmram` に置いた変数を「初期値 0 前提」で使うなら、スタートアップ側修正か、起動後に明示初期化が必要です。

### 方針 C: MAZE_SIZE を下げる（暫定）

- 最短でビルドを通す応急処置
- 実機の目的に合うか要検討

## 追加の注意（Mac でハマりがちな点）

- generator 不一致: 既存 build ディレクトリが Makefiles 生成だと Ninja で configure できない
  - build ディレクトリを分ける/削除して作り直す
- `CMAKE_PROJECT_NAME`が v1.1 でも `HM_Nightfall-mini_v1` のままなので生成物名で混乱しやすい
- リンカスクリプトが意図通りか確認
  - `cmake/gcc-arm-none-eabi.cmake` の `-T "${CMAKE_SOURCE_DIR}/STM32F405XX_FLASH.ld"`

## AI への次アクション（推奨）

1. `HM_Nightfall-mini_v1.1/` を対象に Debug ビルドを再現
2. `.map` から `.bss`上位を確定（`g_nodes/g_path_buf/best_path_buf`）
3. 方針 A（型縮小）を第一候補として RAM 削減
4. それでも厳しければ方針 B（CCMRAM へ配置 + 初期化整備）

---

作成日: 2025-12-27
