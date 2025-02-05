package org.firstinspires.ftc.teamcode.constants;

import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;

public class AutoConst {

    //// Pose list ////
    public static Pose initPose = new Pose(32.5, 8.5, Math.PI/2);
    public static Pose subPark = new Pose(0, 0, Math.PI * 3/2);
    public static Pose humanPark = new Pose(110, 10, 0);

    public static Pose highBasketPose = new Pose(16, 16, (Math.PI * 5/4));
    public static Pose lowBasketPose = new Pose(15, 15, (Math.PI * 5/4));

    public static Pose rightSamplePose = new Pose(0, 0, 0);
    public static Pose midSamplePose = new Pose(0, 0, 0);
    public static Pose leftSamplePose = new Pose(0, 0, 0);
    public static Pose partnerSamplePose = new Pose(0, 0, 0);


    //// Sample Locations ////
    public static int SAMPLE_RIGHT = 1;
    public static int SAMPLE_MID = 2;
    public static int SAMPLE_LEFT = 3;
    public static int SAMPLE_PARTNER = 4;



}
