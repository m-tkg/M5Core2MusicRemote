#!/usr/bin/env python3
"""TTF を LovyanGFX/M5GFX の VLWfont が読める .vlw フォーマットに変換する。

VLW のバイナリ仕様(Processing の PFont.java 由来、ビッグエンディアン):
  ファイルヘッダ (24 bytes):
    glyphCount, version(=11), size, 0, ascent, descent  各 int32
  グリフヘッダ (グリフ数ぶん連続、各 28 bytes):
    unicode, height, width, setWidth, topExtent, leftExtent, 0(padding)  各 int32
  ビットマップ (全グリフヘッダの後に、グリフ順に連結):
    width*height bytes (1 byte/pixel, 8bit グレースケール、行優先)

M5GFX (lgfx_fonts.cpp VLWfont::loadFont/drawChar) の実装を直接確認して
フィールド順序・符号の意味(topExtent=ベースラインからグリフ上端までの距離、
leftExtent=カーソル位置からグリフ左端までのオフセット)を検証済み。

使い方:
  python3 tools/ttf_to_vlw.py <font.ttf> <output.vlw> <pixel_size> [--charset-file FILE]
"""
import argparse
import struct
import sys

from PIL import ImageFont


def extract_full_charset(ttf_path: str) -> list[int]:
    """フォントの cmap から実際に描画可能な Unicode コードポイント一覧を得る。
    Hangul/Thai/Arabic 等 (LINE Seed が対応する他言語) は対象外にして
    ラテン・かな・漢字・全角記号・キリル/ギリシャの範囲に絞る。
    """
    with open(ttf_path, "rb") as f:
        data = f.read()
    num_tables = struct.unpack(">H", data[4:6])[0]
    tables = {}
    for i in range(num_tables):
        offset = 12 + i * 16
        tag = data[offset : offset + 4].decode("ascii", errors="replace")
        toff, tlen = struct.unpack(">II", data[offset + 8 : offset + 16])
        tables[tag] = (toff, tlen)

    coff, _ = tables["cmap"]
    _, num_subtables = struct.unpack(">HH", data[coff : coff + 4])
    best = None
    for i in range(num_subtables):
        ro = coff + 4 + i * 8
        platform_id, encoding_id, offset = struct.unpack(">HHI", data[ro : ro + 8])
        if platform_id == 3 and encoding_id == 10:
            best = coff + offset
    if best is None:
        raise RuntimeError("format12 (3,10) cmap subtable not found")

    off = best
    fmt, _reserved, _length, _lang, n_groups = struct.unpack(">HHIII", data[off : off + 16])
    assert fmt == 12
    all_chars = set()
    for g in range(n_groups):
        go = off + 16 + g * 12
        start_char, end_char, _start_glyph = struct.unpack(">III", data[go : go + 12])
        for c in range(start_char, end_char + 1):
            all_chars.add(c)

    ranges = [
        (0x20, 0x7E),      # ASCII printable
        (0xA0, 0xFF),      # Latin-1 supplement
        (0x100, 0x2AF),    # Latin extended A/B
        (0x370, 0x3FF),    # Greek
        (0x400, 0x4FF),    # Cyrillic
        (0x3000, 0x303F),  # CJK 記号・句読点
        (0x3040, 0x30FF),  # ひらがな・カタカナ
        (0x3400, 0x4DBF),  # CJK 拡張A
        (0x4E00, 0x9FFF),  # CJK 統合漢字
        (0xFF00, 0xFFEF),  # 全角形
        (0x2000, 0x206F),  # 一般句読点 (ダッシュ・省略記号など)
    ]
    result = set()
    for lo, hi in ranges:
        for c in range(lo, hi + 1):
            if c in all_chars:
                result.add(c)
    return sorted(result)


def build_vlw(ttf_path: str, out_path: str, pixel_size: int, codepoints: list[int]) -> None:
    font = ImageFont.truetype(ttf_path, pixel_size)
    ascent, descent = font.getmetrics()

    glyph_headers = []  # (codepoint, height, width, setWidth, topExtent, leftExtent)
    glyph_bitmaps = []  # bytes

    skipped = []
    for cp in codepoints:
        ch = chr(cp)
        try:
            bbox = font.getbbox(ch)
        except Exception:
            skipped.append(cp)
            continue

        try:
            adv = font.getlength(ch)
        except Exception:
            adv = 0
        set_width = int(round(adv))

        if bbox is None:
            glyph_headers.append((cp, 0, 0, set_width, 0, 0))
            glyph_bitmaps.append(b"")
            continue

        left, top, right, bottom = bbox
        mask = font.getmask(ch, mode="L")
        w, h = mask.size
        if w == 0 or h == 0:
            glyph_headers.append((cp, 0, 0, set_width, 0, 0))
            glyph_bitmaps.append(b"")
            continue

        top_extent = ascent - top
        left_extent = left
        glyph_headers.append((cp, h, w, set_width, top_extent, left_extent))
        glyph_bitmaps.append(bytes(mask))

    glyph_count = len(glyph_headers)
    with open(out_path, "wb") as f:
        f.write(struct.pack(">iiiiii", glyph_count, 11, pixel_size, 0, ascent, descent))
        for cp, h, w, set_width, top_extent, left_extent in glyph_headers:
            f.write(struct.pack(">iiiiiii", cp, h, w, set_width, top_extent, left_extent, 0))
        for bm in glyph_bitmaps:
            f.write(bm)

    total_bitmap_bytes = sum(len(b) for b in glyph_bitmaps)
    print(
        f"[{out_path}] glyphs={glyph_count} size={pixel_size}px "
        f"ascent={ascent} descent={descent} "
        f"header_bytes={24 + 28*glyph_count} bitmap_bytes={total_bitmap_bytes} "
        f"total={24 + 28*glyph_count + total_bitmap_bytes} skipped={len(skipped)}"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ttf_path")
    ap.add_argument("out_path")
    ap.add_argument("pixel_size", type=int)
    args = ap.parse_args()

    codepoints = extract_full_charset(args.ttf_path)
    print(f"charset size: {len(codepoints)} codepoints", file=sys.stderr)
    build_vlw(args.ttf_path, args.out_path, args.pixel_size, codepoints)


if __name__ == "__main__":
    main()
