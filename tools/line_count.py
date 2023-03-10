#!/usr/bin/env python3
"""
TRUNNION: Satellite Laser Cross-Link Flight Software & PAT Engine
File: tools/line_count.py
Description: Codebase line counting, comment-to-code ratio, and metrics auditor.
             Scans Forth modules, ANSI C virtual machine, Python simulation suite,
             and architectural technical documentation.

Author: Aneesh Krishna Parthasarathy
License: ISC License
"""

import os
import sys
from typing import Dict, List, Tuple


LANGUAGE_SPECS = {
    "Forth": {
        "extensions": [".fs"],
        "line_comment": ["\\"],
        "block_comment": [("(", ")")],
    },
    "ANSI C": {
        "extensions": [".c", ".h"],
        "line_comment": ["//"],
        "block_comment": [("/*", "*/")],
    },
    "Python": {
        "extensions": [".py"],
        "line_comment": ["#"],
        "block_comment": [('"""', '"""'), ("'''", "'''")],
    },
    "Documentation": {
        "extensions": [".md"],
        "line_comment": [],
        "block_comment": [("<!--", "-->")],
    },
    "Build & Config": {
        "extensions": ["Makefile", ".gitignore", ".gitattributes", "LICENSE"],
        "line_comment": ["#"],
        "block_comment": [],
    },
}


def analyze_file(filepath: str, lang: str) -> Tuple[int, int, int]:
    """
    Analyze file and return (blank_lines, comment_lines, code_lines).
    """
    spec = LANGUAGE_SPECS[lang]
    blanks = 0
    comments = 0
    code = 0
    in_block_comment = False
    block_end_marker = ""

    try:
        with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                s = line.strip()
                if not s:
                    blanks += 1
                    continue

                if in_block_comment:
                    comments += 1
                    if block_end_marker in s:
                        in_block_comment = False
                    continue

                # Check block comment start
                started_block = False
                for start_tok, end_tok in spec["block_comment"]:
                    if s.startswith(start_tok):
                        comments += 1
                        if end_tok not in s[len(start_tok):]:
                            in_block_comment = True
                            block_end_marker = end_tok
                        started_block = True
                        break
                if started_block:
                    continue

                # Check line comment
                is_line_comment = False
                for c_tok in spec["line_comment"]:
                    if s.startswith(c_tok):
                        comments += 1
                        is_line_comment = True
                        break
                if is_line_comment:
                    continue

                code += 1
    except Exception as e:
        print(f"[WARN] Failed to read {filepath}: {e}", file=sys.stderr)

    return blanks, comments, code


def main() -> None:
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    results: Dict[str, Dict[str, int]] = {
        lang: {"files": 0, "blanks": 0, "comments": 0, "code": 0, "total": 0}
        for lang in LANGUAGE_SPECS
    }

    ignore_dirs = {".git", "build", "bin", "__pycache__"}

    for dirpath, dirnames, filenames in os.walk(root_dir):
        dirnames[:] = [d for d in dirnames if d not in ignore_dirs]

        for fname in filenames:
            fpath = os.path.join(dirpath, fname)
            rel_path = os.path.relpath(fpath, root_dir)

            matched_lang = None
            for lang, spec in LANGUAGE_SPECS.items():
                if fname in spec["extensions"] or any(fname.endswith(ext) for ext in spec["extensions"]):
                    matched_lang = lang
                    break

            if matched_lang:
                blanks, comments, code = analyze_file(fpath, matched_lang)
                results[matched_lang]["files"] += 1
                results[matched_lang]["blanks"] += blanks
                results[matched_lang]["comments"] += comments
                results[matched_lang]["code"] += code
                results[matched_lang]["total"] += (blanks + comments + code)

    # Print Summary Table
    print("================================================================================")
    print("TRUNNION: CODEBASE ARCHITECTURE & SCALE METRICS")
    print("================================================================================")
    header = f"{'Language / Layer':<20} {'Files':>6} {'Blank':>8} {'Comment':>10} {'Code':>10} {'Total':>10} {'Comment %':>10}"
    print(header)
    print("-" * len(header))

    tot_files = 0
    tot_blanks = 0
    tot_comments = 0
    tot_code = 0
    tot_total = 0

    for lang, r in results.items():
        if r["files"] == 0:
            continue
        tot_files += r["files"]
        tot_blanks += r["blanks"]
        tot_comments += r["comments"]
        tot_code += r["code"]
        tot_total += r["total"]

        pct = (r["comments"] / r["total"] * 100.0) if r["total"] > 0 else 0.0
        print(
            f"{lang:<20} {r['files']:6d} {r['blanks']:8d} {r['comments']:10d} {r['code']:10d} {r['total']:10d} {pct:9.1f}%"
        )

    print("-" * len(header))
    tot_pct = (tot_comments / tot_total * 100.0) if tot_total > 0 else 0.0
    print(
        f"{'TOTAL METRICS':<20} {tot_files:6d} {tot_blanks:8d} {tot_comments:10d} {tot_code:10d} {tot_total:10d} {tot_pct:9.1f}%"
    )
    print("================================================================================")
    forth_code = results["Forth"]["code"]
    c_code = results["ANSI C"]["code"]
    py_code = results["Python"]["code"]
    print(f"Forth Core Subsystem:      {forth_code:,} executable lines of raw Forth code")
    print(f"Portable Forth VM (C11):   {c_code:,} lines of zero-dependency ANSI C")
    print(f"HIL Simulation & Tools:    {py_code:,} lines of Python 3")
    print("================================================================================")


if __name__ == "__main__":
    main()
