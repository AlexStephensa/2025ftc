package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;

public class AutoConst {

    public static double gridMod = Math.PI * 3/2;

    //// Pose list ////
    public static Pose leftInitPose         = new Pose(32.5, 8.5,Math.PI + gridMod);
    public static Pose rightInitPose        = new Pose(85,  8.5, 0 + gridMod);

    public static Pose subPark              = new Pose(50,   58,  Math.PI + gridMod);
    public static Pose humanPark            = new Pose(110,  10,  Math.PI * 3/2 + gridMod);

    public static Pose highBasketPose       = new Pose(13,   13,  Math.PI * 7/4 + gridMod);
    public static Pose highBasketPoseOffset = new Pose(18,   18,  Math.PI * 7/4 + gridMod);

    public static Pose lowBasketPose        = new Pose(15,   15,  Math.PI * 7/4 + gridMod);

    //// Sample Locations ////
    public static int SAMPLE_RIGHT              = 1;
    public static Pose rightSamplePose          = new Pose(24, 30,  Math.PI * 3/2 + gridMod);
    public static Pose rightSamplePoseOffset    = new Pose(24, 20,  Math.PI * 3/2 + gridMod);

    public static int SAMPLE_MID                = 2;
    public static Pose midSamplePose            = new Pose(12.5, 30,  Math.PI * 3/2 + gridMod);
    public static Pose midSamplePoseOffset      = new Pose(12.5, 20,  Math.PI * 3/2 + gridMod);

    public static int SAMPLE_LEFT               = 3;
    public static Pose leftSamplePose           = new Pose(14,   44,  Math.PI + gridMod);
    public static Pose leftSamplePoseOffset     = new Pose(19,   44,  Math.PI + gridMod);

    public static int SAMPLE_PARTNER            = 4;
    public static Pose partnerSamplePose        = new Pose(55,   15,  0 + gridMod);
    public static Pose partnerSamplePoseOffset  = new Pose(47,   15,  0 + gridMod);

}
