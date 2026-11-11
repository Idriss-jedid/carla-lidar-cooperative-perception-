# Narrated walkthrough of the editor

A video of the scenario editor at work, driven live by a script against a running CARLA: the paper's Figure 2 scene
placed car by car, the paper's settings, Start, Pause, Record, the processing, the BEV of every CAV. Nothing is
recorded from the screen and nothing plays on the speakers, so the PC stays usable meanwhile.

```bash
# 1. the voice: one MP3 per sentence of narration.py (edge-tts, Microsoft's online voices; any Python >= 3.8)
python tools/demo/tts.py data/demo

# 2. the recording: CARLA 0.9.12 running Town03 (e.g. CarlaUE4.exe -RenderOffScreen), project environment
python tools/demo/record.py data/demo

# 3. the edit: voice, subtitles, waits sped up (ffmpeg, numpy)
python tools/demo/edit.py data/demo data/demo/editor_walkthrough.mp4
```

| File | What it does |
|---|---|
| `narration.py` | every spoken line, in order; numbers that change from run to run are only shown on screen |
| `tts.py` | sentence → MP3 with retries (the service often answers "no audio" at random); `tts/manifest.json` gives each clip's length |
| `record.py` | opens the editor transparent and off-screen; a director moves an invisible pointer, clicks the map and the buttons, presses keys, waits for CARLA, and says each line for as long as its clip lasts. A second thread takes the window's own pixels with `PrintWindow` 15 times per second and draws the pointer, click ripples, highlights, the figure and CARLA's camera view (a camera behind `cav_0`) on top → `raw.mkv`, `narration.json`, `segments.json`, `steps.log` |
| `edit.py` | speeds up the waits (spawning, processing) as long as the narration still fits, lays each clip at its time, burns the subtitles, encodes the MP4 and writes the `.srt` |
| `winshot.py` | `PrintWindow` capture of a Tk window and its menu bar |
| `cards.py` | frame size and fonts |

`DEMO_DRY=1 python tools/demo/record.py data/demo_dry` rehearses without the voice (sentence lengths estimated).
The run writes `config/spawn_plans/paper_figure2.json`, `config/experiments/custom/paper_figure2.yaml` and a run
folder in `data/runs/`, like pressing Start and Record by hand.
