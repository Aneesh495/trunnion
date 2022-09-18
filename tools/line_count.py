#!/usr/bin/env python3
"""
Source Code Line Count and Metrics Summary
Analyzes raw source lines, blank lines, and comment lines across subsystems.

Copyright (c) 2026 Aneesh Krishna Parthasarathy
Licensed under the ISC License.
"""

import os
import sys

EXCLUDE_DIRS = {".git", "bin", "build", "__pycache__", ".zig-cache"}

def analyze_file(fpath):
    total = 0
    blank = 0
    comment = 0
    code = 0
    ext = os.path.splitext(fpath)[1].lower()
    
    with open(fpath, "r", encoding="utf-8", errors="replace") as f:
        in_block_comment = False
        for line in f:
            total += 1
            s = line.strip()
            if not s:
                blank += 1
                continue
            if ext in {".fs", ".fth"}:
                if s.startswith("\\"):
                    comment += 1
                elif s.startswith("(") and s.endswith(")"):
                    comment += 1
                elif s.startswith("(*"):
                    in_block_comment = True
                    comment += 1
                elif in_block_comment:
                    comment += 1
                    if "*)" in s:
                        in_block_comment = False
                else:
                    code += 1
            elif ext in {".c", ".h"}:
                if s.startswith("//"):
                    comment += 1
                elif s.startswith("/*"):
                    in_block_comment = True
                    comment += 1
                elif in_block_comment:
                    comment += 1
                    if "*/" in s:
                        in_block_comment = False
                else:
                    code += 1
            elif ext == ".py":
                if s.startswith("#"):
                    comment += 1
                else:
                    code += 1
            else:
                code += 1
    return total, blank, comment, code

def main():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    metrics_by_ext = {}
    
    for r, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in EXCLUDE_DIRS]
        for fname in files:
            ext = os.path.splitext(fname)[1].lower()
            if not ext:
                continue
            fpath = os.path.join(r, fname)
            t, b, c, cd = analyze_file(fpath)
            if ext not in metrics_by_ext:
                metrics_by_ext[ext] = {"total": 0, "blank": 0, "comment": 0, "code": 0, "files": 0}
            metrics_by_ext[ext]["total"] += t
            metrics_by_ext[ext]["blank"] += b
            metrics_by_ext[ext]["comment"] += c
            metrics_by_ext[ext]["code"] += cd
            metrics_by_ext[ext]["files"] += 1

    print("================================================================================")
    print("                      TRUNNION CODEBASE SCALE AND METRICS                       ")
    print("================================================================================")
    print(f"{'Extension':<12} {'Files':<8} {'Code':<12} {'Comments':<12} {'Blank':<10} {'Total':<10}")
    print("--------------------------------------------------------------------------------")
    grand_total = 0
    grand_code = 0
    grand_forth = 0
    for ext, m in sorted(metrics_by_ext.items(), key=lambda x: -x[1]["total"]):
        print(f"{ext:<12} {m['files']:<8} {m['code']:<12} {m['comment']:<12} {m['blank']:<10} {m['total']:<10}")
        grand_total += m["total"]
        grand_code += m["code"]
        if ext in {".fs", ".fth"}:
            grand_forth += m["total"]
    print("================================================================================")
    print(f"Total Forth (.fs) Raw Lines : {grand_forth}")
    print(f"Total Project Code Lines    : {grand_code}")
    print(f"Total Project Raw Lines     : {grand_total}")
    print("================================================================================")

if __name__ == "__main__":
    main()
