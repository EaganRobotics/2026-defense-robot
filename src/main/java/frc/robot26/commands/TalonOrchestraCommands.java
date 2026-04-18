package frc.robot26.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class TalonOrchestraCommands {
  private TalonOrchestraCommands() {}

  public static Command play(Orchestra orchestra, String musicFile) {
    return Commands.runOnce(
            () -> {
              if (orchestra == null) {
                return;
              }

              orchestra.stop();
              StatusCode loadStatus = orchestra.loadMusic(musicFile);
              if (loadStatus.isOK()) {
                orchestra.play();
              } else {
                System.out.println(
                    "[Orchestra] Failed to load '"
                        + musicFile
                        + "': "
                        + loadStatus.getName()
                        + " - "
                        + loadStatus.getDescription());
              }
            })
        .ignoringDisable(true)
        .withName("Orchestra.play");
  }

  public static Command stop(Orchestra orchestra) {
    return Commands.runOnce(
            () -> {
              if (orchestra != null) {
                orchestra.stop();
              }
            })
        .ignoringDisable(true)
        .withName("Orchestra.stop");
  }
}
