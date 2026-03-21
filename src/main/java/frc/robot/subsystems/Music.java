// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.traits.CommonDevice;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.util.ArrayList;

public class Music extends SubsystemBase {
  private static Orchestra orchestra = new Orchestra();
  private ArrayList<String> tracks = new ArrayList<String>();

  private final SendableChooser<String> songChooser = new SendableChooser<>();
  private String currentTrack = "";
  private boolean isPlaying = false;

  final File folder = new File("/home/lvuser/deploy/music");

  /** Creates a new Music. */
  public Music() {
    // Safely load tracks from the deploy folder (avoids NPE in sim/replay)
    File[] files = folder.listFiles();
    if (files != null) {
      for (File song : files) {
        String name = song.getName();
        if (name.endsWith(".chrp")) {
          tracks.add(name.replace(".chrp", ""));
        }
      }
    }

    // Populate the song chooser dropdown
    songChooser.setDefaultOption("-- None --", "");
    for (String track : tracks) {
      songChooser.addOption(track, track);
    }

    // Publish widgets to Elastic
    SmartDashboard.putData("Music/Song Select", songChooser);
    SmartDashboard.putBoolean("Music/Playing", false);
    SmartDashboard.putString("Music/Current Track", "None");
  }

  public static void addMotor(CommonDevice motor) {
    orchestra.addInstrument(motor);
  }

  private void play() {
    orchestra.play();
    isPlaying = true;
  }

  private void stop() {
    orchestra.stop();
    isPlaying = false;
  }

  private void loadMusic(String song) {
    orchestra.loadMusic(String.format("music/%s.chrp", song));
    currentTrack = song;
    SmartDashboard.putString("Music/Current Track", song);
  }

  @Override
  public void periodic() {
    // Read the play/stop toggle from Elastic
    boolean wantsPlaying = SmartDashboard.getBoolean("Music/Playing", false);

    // Read the selected song from the dropdown
    String selectedSong = songChooser.getSelected();
    if (selectedSong == null) {
      selectedSong = "";
    }

    // If the song selection changed, load the new track
    if (!selectedSong.isEmpty() && !selectedSong.equals(currentTrack)) {
      stop();
      loadMusic(selectedSong);
      // Auto-play on song change if the toggle is on
      if (wantsPlaying) {
        play();
      }
    }

    // Handle play/stop toggle changes
    if (wantsPlaying && !isPlaying && !currentTrack.isEmpty()) {
      play();
    } else if (!wantsPlaying && isPlaying) {
      stop();
    }

    // Keep the dashboard in sync with actual state
    SmartDashboard.putBoolean("Music/Playing", isPlaying);
  }
}
