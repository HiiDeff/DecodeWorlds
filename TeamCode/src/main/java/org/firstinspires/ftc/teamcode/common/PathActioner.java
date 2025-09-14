package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public interface PathActioner {
    public PathChain createPath(PathBuilder builder);
}
