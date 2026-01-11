package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

public class Path {
    //private final ArrayList<Pose2d>
    private final Pose2d start;
    private final Pose2d end;

    public Path(Pose2d iStart, Pose2d iEnd) {
        start = iStart;
        end = iEnd;
    }

    public boolean atStart(Pose2d pose) {
        return FieldUtils.atPose(pose, start);
    }

    public boolean atEnd(Pose2d pose) {
        return FieldUtils.atPose(pose, end);
    }
}
