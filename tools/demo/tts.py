"""
Synthesize every sentence of narration.py with edge-tts (Microsoft's online voices) into <out>/tts/.

    python tools/demo/tts.py data/demo            # needs the edge-tts package and ffprobe

Writes one MP3 per sentence (file name = hash of voice + sentence, so unchanged sentences are not sent again) and
<out>/tts/manifest.json: {sentence: {"file": ..., "dur": seconds}}. Runs in any Python >= 3.8 with edge-tts; the
recording itself runs in the CARLA environment and only reads the manifest.
"""
import asyncio
import hashlib
import json
import os
import re
import subprocess
import sys

import edge_tts

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from narration import LINES  # noqa: E402

VOICE = "en-US-AndrewNeural"


def sentences(text):
    return [s.strip() for s in re.split(r"(?<=[.!?])\s+(?=[A-Z])", text.strip()) if s.strip()]


def duration(path):
    return float(subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0",
                                 path], capture_output=True, text=True).stdout)


async def synth_all(items, folder, attempts=12):
    """One request at a time; the service often answers "no audio" at random, so each sentence is retried."""
    done = 0
    for sentence, path in items:
        if not (os.path.isfile(path) and os.path.getsize(path) > 0):
            for attempt in range(attempts):
                try:
                    await asyncio.wait_for(edge_tts.Communicate(sentence, VOICE).save(path), 60)
                    break
                except Exception as exc:
                    if os.path.isfile(path):
                        os.remove(path)
                    if attempt == attempts - 1:
                        raise RuntimeError("no audio for: {} ({})".format(sentence, exc))
                    await asyncio.sleep(3 + 2 * attempt)
        done += 1
        print("{}/{}".format(done, len(items)), flush=True)


def main():
    folder = os.path.join(sys.argv[1] if len(sys.argv) > 1 else "data/demo", "tts")
    os.makedirs(folder, exist_ok=True)
    items = []
    for _, _, text in LINES:
        for s in sentences(text):
            name = hashlib.sha1("{}|{}".format(VOICE, s).encode()).hexdigest()[:16] + ".mp3"
            items.append((s, os.path.join(folder, name)))
    asyncio.run(synth_all(items, folder))
    manifest = {s: {"file": os.path.abspath(p), "dur": round(duration(p), 3)} for s, p in items}
    with open(os.path.join(folder, "manifest.json"), "w", encoding="utf-8") as fh:
        json.dump({"voice": VOICE, "sentences": manifest}, fh, indent=1)
    total = sum(v["dur"] for v in manifest.values())
    print("{} sentences, {:.1f} min of speech -> {}".format(len(manifest), total / 60, folder))


if __name__ == "__main__":
    main()
