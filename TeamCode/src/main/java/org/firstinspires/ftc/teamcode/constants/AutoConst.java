package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;

public class AutoConst {

    static double gridMod = Math.PI * 3/2;

    //// Pose list ////
    public static Pose leftInitPose         = new Pose(32.5, 8.5,Math.PI + gridMod);
    public static Pose rightInitPose        = new Pose(85,  8.5, 0 + gridMod);

    public static Pose subPark              = new Pose(45,   60,  Math.PI + gridMod);
    public static Pose humanPark            = new Pose(110,  10,  Math.PI * 3/2 + gridMod);

    public static Pose highBasketPose       = new Pose(14,   14,  Math.PI * 7/4 + gridMod);
    public static Pose highBasketPoseOffset = new Pose(17,   17,  Math.PI * 7/4 + gridMod);

    public static Pose lowBasketPose        = new Pose(15,   15,  Math.PI * 7/4 + gridMod);

    public static Pose rightSamplePose      = new Pose(22.5, 30,  Math.PI * 3/2 + gridMod);
    public static Pose midSamplePose        = new Pose(12.5, 30,  Math.PI * 3/2 + gridMod);
    public static Pose leftSamplePose       = new Pose(19,   44,  Math.PI + gridMod);
    public static Pose partnerSamplePose    = new Pose(55,   15,  0 + gridMod);


    //// Sample Locations ////
    public static int SAMPLE_RIGHT      = 1;
    public static int SAMPLE_MID        = 2;
    public static int SAMPLE_LEFT       = 3;
    public static int SAMPLE_PARTNER    = 4;

}
