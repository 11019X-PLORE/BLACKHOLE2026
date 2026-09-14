# Shoot_template_2026

Team 11019 FRC 2026 robot code.

## Branches

- **main** — full codebase, including the autonomous routines (PathPlanner/Choreo).
- **demo** — exhibition build for manual driving only. All autonomous code has been
  removed, and manual drive/turn speed is limited by two percentages adjustable from
  the Elastic dashboard ("Drive Speed %" / "Turn Speed %"), persisted on the roboRIO
  so the last-used values survive a reboot. Import the `elastic-layout*` file on that
  branch into Elastic to get the dashboard widgets.

## Build

Open this folder in WPILib VS Code, or run:

```powershell
.\gradlew.bat build
```

On macOS/Linux with the WPILib JDK:

```bash
JAVA_HOME=~/wpilib/2026/jdk sh gradlew build
```

WPILib 2026.2.1 and Java 17.
