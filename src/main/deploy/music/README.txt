Place your CTRE Orchestra song file here as:

song.chrp

The robot code loads this path by default:

music/song.chrp

You can change it in `frc.robot26.RobotContainer` via `ORCHESTRA_FILE`.

Loudness notes:
- Phoenix Orchestra does not expose a direct Java "volume" API.
- Make the song louder in your source MIDI/CHRP generation workflow
	(increase note velocity / gain before exporting `.chrp`).
- Add more supported motor instruments for a fuller output.

Current wiring on this robot:
- Orchestra is added to all swerve Talons (drive + steer on FL/FR/BL/BR).
