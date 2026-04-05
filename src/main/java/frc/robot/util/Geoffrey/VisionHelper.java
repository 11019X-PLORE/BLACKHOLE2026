// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Geoffrey;

/** Add your docs here. */
public class VisionHelper {
  /**
   * Calculates the maximum area among all detected tags in pixels.
   *
   * @param rawDetections The double array from the "rawdetections" NetworkTable entry.
   * @return The area of the largest tag (in pixels^2), or 0.0 if no tags found.
   */
  public static double getMaxTagArea(double[] rawDetections, int num_nonCornerEntries) {
    if (rawDetections == null || rawDetections.length == 0) {
      return 0.0;
    }

    double maxArea = 0.0;

    // Each tag detection has 12 entries in the array
    for (int i = 0; i < rawDetections.length; i += 8 + num_nonCornerEntries) {
      // Safety check for array bounds
      if (i + num_nonCornerEntries + 7 >= rawDetections.length) break;

      // Extract corner coordinates (indices 4 through 11)
      double x0 = rawDetections[i + num_nonCornerEntries];
      double y0 = rawDetections[i + num_nonCornerEntries + 1];
      double x1 = rawDetections[i + num_nonCornerEntries + 2];
      double y1 = rawDetections[i + num_nonCornerEntries + 3];
      double x2 = rawDetections[i + num_nonCornerEntries + 4];
      double y2 = rawDetections[i + num_nonCornerEntries + 5];
      double x3 = rawDetections[i + num_nonCornerEntries + 6];
      double y3 = rawDetections[i + num_nonCornerEntries + 7];

      // Shoelace Formula for a 4-point polygon:
      // Area = 0.5 * |(x0y1 + x1y2 + x2y3 + x3y0) - (y0x1 + y1x2 + y2x3 + y3x0)|
      double area =
          0.5
              * Math.abs(
                  (x0 * y1 + x1 * y2 + x2 * y3 + x3 * y0)
                      - (y0 * x1 + y1 * x2 + y2 * x3 + y3 * x0));

      if (area > maxArea) {
        maxArea = area;
      }
    }

    return maxArea;
  }
}
