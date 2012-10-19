package com.itseez.peopledemo;

import android.graphics.Color;

public enum BodyPartLabel {
    LFOOT(Color.rgb(50, 34, 22)),
    LLEG(Color.rgb(24, 247, 196)),
    LKNEE(Color.rgb(33, 207, 189)),
    LTHIGH(Color.rgb(254, 194, 127)),
    RFOOT(Color.rgb(88, 115, 175)),
    RLEG(Color.rgb(158, 91, 64)),
    RKNEE(Color.rgb(14, 90, 2)),
    RTHIGH(Color.rgb(100, 156, 227)),
    RHIPS(Color.rgb(243, 167, 17)),
    LHIPS(Color.rgb(145, 194, 184)),
    NECK(Color.rgb(234, 171, 147)),
    RARM(Color.rgb(220, 112, 93)),
    RELBOW(Color.rgb(93, 132, 163)),
    RFOREARM(Color.rgb(122, 4, 85)),
    RHAND(Color.rgb(75, 168, 46)),
    LARM(Color.rgb(15, 5, 108)),
    LELBOW(Color.rgb(180, 125, 107)),
    LFOREARM(Color.rgb(157, 77, 167)),
    LHAND(Color.rgb(214, 89, 73)),
    FACELB(Color.rgb(52, 183, 58)),
    FACERB(Color.rgb(54, 155, 75)),
    FACELT(Color.rgb(249, 61, 187)),
    FACERT(Color.rgb(143, 57, 11)),
    RCHEST(Color.rgb(246, 198, 0)),
    LCHEST(Color.rgb(202, 177, 251)),
    LSHOULDER(Color.rgb(229, 115, 80)),
    RSHOULDER(Color.rgb(159, 185, 1)),
    GROUND_PLANE(Color.rgb(186, 213, 229)),
    CEILING(Color.rgb(82, 47, 144)),
    BACKGROUND(Color.argb(0, 140, 69, 139)),
    RESERVED(Color.rgb(189, 115, 117)),
    NO_LABEL(Color.rgb(80, 57, 150));

    public final int color;

    BodyPartLabel(int color) {
        this.color = color;
    }
}
