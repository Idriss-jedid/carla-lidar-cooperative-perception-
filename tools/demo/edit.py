"""
raw.mkv + segments.json + narration.json -> the narrated walkthrough MP4 (+ .srt).

    python tools/demo/edit.py data/demo data/demo/editor_walkthrough.mp4      # needs ffmpeg / ffprobe and numpy

The screen plays in real time, except the waits (spawning, processing), which are sped up, but never so much that
the narration spoken over a wait would run past it: a wait's speed is lowered until its lines fit. Every sentence is
placed at the final-video time of the moment it was said, after the previous one has finished; the subtitles use the
same times, so voice and text stay together.
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

d = Path(sys.argv[1]).resolve()
out = Path(sys.argv[2]).resolve()
RAW = "raw.mkv"
SR = 24000
GAP = 0.25
segs = sorted(json.loads((d / "segments.json").read_text()), key=lambda s: s["start"])
narr = sorted(json.loads((d / "narration.json").read_text()), key=lambda n: n["t"])


def probe_duration(p):
    r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", str(p)],
                       capture_output=True, text=True).stdout.strip()
    return float(r)


dur = probe_duration(d / RAW)

# 1. parts of the raw video: [start, end, speed]
parts, t = [], 0.0
for s in segs:
    a, b = max(s["start"], t), min(s["end"], dur)
    if a > t:
        parts.append([t, a, 1.0])
    if b > a:
        parts.append([a, b, float(s["speed"])])
    t = max(t, b)
if t < dur:
    parts.append([t, dur, 1.0])

# 2. lay the sentences out; slow a wait down where its lines would not fit
lines = [{"t": n["t"], "title": n["title"], "clips": n["clips"]} for n in narr]
F, E, placed, li = 0.0, 0.0, [], 0     # final clock at the part's start, end of the last sentence
for part in parts:
    a, b, sp = part[0], part[1], part[2]

    def lay(speed):
        e, laid, j = E, [], li
        while j < len(lines) and lines[j]["t"] < b:
            ln = lines[j]
            start = max(e, F + (max(ln["t"], a) - a) / speed)
            for c in ln["clips"]:
                laid.append({"start": start, "end": start + c["dur"], "file": c["file"], "text": c["text"],
                             "title": ln["title"]})
                start += c["dur"] + GAP
            e = start - GAP
            j += 1
        return e, laid, j

    if sp > 1:
        e, _, _ = lay(sp)
        need = e - F + 0.4
        if need > (b - a) / sp:
            sp = max(1.0, (b - a) / need)
        part[2] = sp
    E, laid, li = lay(sp)
    placed += laid
    F += (b - a) / sp
total = F
if placed and placed[-1]["end"] > total:          # the last line runs past the last frame: hold the last frame
    parts[-1].append(placed[-1]["end"] - total + 1.0)
    total = placed[-1]["end"] + 1.0

# 3. the voice track
audio = np.zeros(int((total + 1) * SR), dtype=np.float32)
for c in placed:
    pcm = subprocess.run(["ffmpeg", "-v", "error", "-i", c["file"], "-f", "s16le", "-ac", "1", "-ar", str(SR), "-"],
                         capture_output=True).stdout
    x = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768
    i = int(c["start"] * SR)
    audio[i:i + len(x)] += x[: max(0, len(audio) - i)]
audio = np.clip(audio * 1.1, -1, 1)
(d / "voice.raw").write_bytes((audio * 32767).astype(np.int16).tobytes())


# 4. subtitles: the chapter in yellow, the sentence under it
def ts(x, sep="."):
    h, r = divmod(x, 3600)
    m, s = divmod(r, 60)
    if sep == ".":
        return "{}:{:02d}:{:05.2f}".format(int(h), int(m), s)
    return "{:02d}:{:02d}:{:02d},{:03d}".format(int(h), int(m), int(s), int(round((s % 1) * 1000)) % 1000)


def esc(s):
    return s.replace("\\", "\\\\").replace("{", "(").replace("}", ")")


ass = ["[Script Info]", "ScriptType: v4.00+", "PlayResX: 1600", "PlayResY: 900", "WrapStyle: 0", "",
       "[V4+ Styles]",
       "Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, "
       "Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, "
       "MarginL, MarginR, MarginV, Encoding",
       "Style: Sub,Segoe UI,30,&H00F0E8E2,&H00FFFFFF,&H00140C06,&H28140C06,0,0,0,0,100,100,0,0,3,14,0,2,"
       "180,180,34,1", "",
       "[Events]", "Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text"]
srt = []
for i, c in enumerate(placed, 1):
    a, b = c["start"], c["end"] + 0.15
    if i < len(placed):
        b = min(b, placed[i]["start"] - 0.02)
    ass.append("Dialogue: 0,{},{},Sub,,0,0,0,,{{\\fs22\\b1\\c&H4FD5FF&}}{}{{\\r}}\\N{}".format(
        ts(a), ts(b), esc(c["title"]), esc(c["text"])))
    srt.append("{}\n{} --> {}\n{}\n".format(i, ts(a, ","), ts(b, ","), c["text"]))
(d / "subs.ass").write_text("\n".join(ass) + "\n", encoding="utf-8")
out.with_suffix(".srt").write_text("\n".join(srt), encoding="utf-8")

# 5. video: the parts at their speeds, subtitles burned in, the voice added
f = []
for i, p in enumerate(parts):
    a, b, sp = p[0], p[1], p[2]
    tail = ",tpad=stop_mode=clone:stop_duration={:.2f}".format(p[3]) if len(p) > 3 else ""
    f.append("[0:v]trim=start={:.3f}:end={:.3f},setpts=(PTS-STARTPTS)/{:.4f}{}[v{}]".format(a, b, sp, tail, i))
f.append("".join("[v{}]".format(i) for i in range(len(parts))) +
         "concat=n={}:v=1:a=0,fps=30,ass=subs.ass[out]".format(len(parts)))
(d / "filter.txt").write_text(";\n".join(f))
subprocess.run(["ffmpeg", "-y", "-v", "error", "-i", RAW, "-f", "s16le", "-ar", str(SR), "-ac", "1", "-i", "voice.raw",
                "-filter_complex_script", "filter.txt", "-map", "[out]", "-map", "1:a",
                "-c:v", "libx264", "-preset", "medium", "-crf", "21", "-pix_fmt", "yuv420p",
                "-c:a", "aac", "-b:a", "160k", "-shortest", "-movflags", "+faststart", str(out)],
               check=True, cwd=str(d))
print("{} parts, {} sentences, recorded {:.1f} min -> video {:.1f} min\n-> {}\n-> {}".format(
    len(parts), len(placed), dur / 60, total / 60, out, out.with_suffix(".srt")))
for p in parts:
    if p[2] > 1:
        print("   wait {:7.1f}-{:7.1f} s played at x{:.1f}".format(p[0], p[1], p[2]))
