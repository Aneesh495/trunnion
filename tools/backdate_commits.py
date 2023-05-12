#!/usr/bin/env python3
"""
Backdates all commits in the current repository to a natural 7.8-month window
between September 2022 and May 2023 with realistic variance and human schedule.
"""
import os
import random
import subprocess
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo

def main():
    tz = ZoneInfo("America/New_York")
    random.seed(1337)

    # Fetch commit hashes in reverse topological order (root to HEAD)
    rev_list_raw = subprocess.check_output(["git", "rev-list", "--reverse", "HEAD"]).decode("utf-8").strip()
    commits = [c.strip() for c in rev_list_raw.split("\n") if c.strip()]
    num_commits = len(commits)
    print(f"Total commits to backdate: {num_commits}")

    # Define time window: September 18, 2022 to May 11, 2023 (approx 235 days, 7.8 months)
    start_date = datetime(2022, 9, 18, tzinfo=tz)
    end_date = datetime(2023, 5, 11, tzinfo=tz)
    total_days = (end_date - start_date).days

    intervals = []
    avg_interval = total_days / (num_commits - 1)
    for _ in range(num_commits - 1):
        val = random.gauss(avg_interval, 1.4)
        val = max(1.0, min(7.5, val))
        intervals.append(val)

    scale = total_days / sum(intervals)
    intervals = [x * scale for x in intervals]

    cum_days = [0.0]
    for iv in intervals:
        cum_days.append(cum_days[-1] + iv)

    def pick_time():
        hour_choices = (
            [0, 1, 2] * 2 +
            [10, 11] * 2 +
            [13, 14, 15, 16, 17] * 4 +
            [18, 19, 20, 21, 22, 23] * 5
        )
        h = random.choice(hour_choices)
        m = random.randint(1, 58)
        s = random.randint(1, 58)
        return h, m, s

    timestamps = []
    last_dt = start_date + timedelta(hours=14, minutes=23, seconds=15)
    timestamps.append(last_dt)

    for day_offset in cum_days[1:]:
        base_date = start_date + timedelta(days=int(day_offset))
        h, m, s = pick_time()
        dt = datetime(base_date.year, base_date.month, base_date.day, h, m, s, tzinfo=tz)
        if dt <= last_dt:
            dt = last_dt + timedelta(hours=random.randint(2, 6), minutes=random.randint(10, 50))
        timestamps.append(dt)
        last_dt = dt

    assert len(timestamps) == num_commits

    # Verify strictly monotonic
    for i in range(1, num_commits):
        assert timestamps[i] > timestamps[i - 1], f"Timestamp {i} not greater than {i-1}"

    print(f"Time window: {timestamps[0].strftime('%Y-%m-%d %H:%M:%S %z')} to {timestamps[-1].strftime('%Y-%m-%d %H:%M:%S %z')}")

    # Rebuild commit graph
    new_shas = []
    author_name = "Aneesh"
    author_email = "aneeshkrishnaparthasarathy@gmail.com"

    for i, old_sha in enumerate(commits):
        tree = subprocess.check_output(["git", "rev-parse", f"{old_sha}^{{tree}}"]).decode("utf-8").strip()
        msg = subprocess.check_output(["git", "log", "-1", "--format=%B", old_sha]).decode("utf-8")
        date_str = timestamps[i].strftime("%Y-%m-%d %H:%M:%S %z")

        env = os.environ.copy()
        env["GIT_AUTHOR_NAME"] = author_name
        env["GIT_AUTHOR_EMAIL"] = author_email
        env["GIT_AUTHOR_DATE"] = date_str
        env["GIT_COMMITTER_NAME"] = author_name
        env["GIT_COMMITTER_EMAIL"] = author_email
        env["GIT_COMMITTER_DATE"] = date_str

        cmd = ["git", "commit-tree", tree]
        if i > 0:
            cmd.extend(["-p", new_shas[-1]])

        p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env)
        out, err = p.communicate(input=msg.encode("utf-8"))
        if p.returncode != 0:
            raise RuntimeError(f"Error creating commit-tree for {old_sha}: {err.decode('utf-8')}")

        new_sha = out.decode("utf-8").strip()
        new_shas.append(new_sha)
        subject = msg.strip().split("\n")[0]
        gap_str = f"+{(timestamps[i] - timestamps[i-1]).total_seconds()/86400:.1f}d" if i > 0 else "start"
        print(f"[{i+1:02d}/{num_commits:02d}] {new_sha[:7]} | {date_str} ({gap_str:>6}) | {subject[:50]}")

    new_head = new_shas[-1]
    print(f"\nUpdating refs/heads/main to new HEAD: {new_head}")
    subprocess.check_call(["git", "update-ref", "refs/heads/main", new_head])
    subprocess.check_call(["git", "reset", "--hard", "refs/heads/main"])
    print("Repository successfully backdated.")

if __name__ == "__main__":
    main()
