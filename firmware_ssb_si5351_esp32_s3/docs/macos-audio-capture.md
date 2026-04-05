# Record macOS system audio

macOS does not expose speaker output as an FFmpeg input directly. Use BlackHole
as a loopback device and a Multi-Output Device so the same audio reaches both
the MacBook speakers and the recorder.

## One-time setup

```sh
brew install ffmpeg
brew install --cask blackhole-2ch
```

Restart the relevant audio applications after installing BlackHole. In **Audio
MIDI Setup**:

1. Create a Multi-Output Device.
2. Enable both **MacBook Speakers** and **BlackHole 2ch**.
3. Make MacBook Speakers the clock source and enable drift correction for
   BlackHole if Audio MIDI Setup offers it.
4. Choose the Multi-Output Device as the macOS sound output.

BlackHole's official setup guide is at
<https://github.com/ExistentialAudio/BlackHole/wiki/Getting-Started%3A-Creating-a-Multi-Output-Device>.

## Record

List AVFoundation devices and verify BlackHole's name:

```sh
scripts/record-macos-system-audio.sh --list
```

Record until `q` is pressed in FFmpeg:

```sh
scripts/record-macos-system-audio.sh output.wav
```

Record exactly 30 seconds:

```sh
scripts/record-macos-system-audio.sh output.wav 30
```

Override the device name if the listing differs:

```sh
AUDIO_DEVICE='BlackHole 2ch' scripts/record-macos-system-audio.sh output.wav
```

The script stores 48 kHz stereo, 24-bit PCM by default. This avoids adding a
lossy codec while retaining plenty of measurement headroom.

After recording, restore **MacBook Speakers** as the normal sound output, then
follow [`airspy-audio-analysis.md`](airspy-audio-analysis.md).
