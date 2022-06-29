using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carousel
{

namespace MotionMatching{

public abstract class PoseProviderBase : PlayerControllerBase
{

    abstract public void ResetToIdle();
}
public abstract class MMPoseProvider : PoseProviderBase {

    public PoseState poseState;
    public MotionMatching mm;


    virtual public Quaternion GetGlobalRotation(int boneIdx)
    {
        return poseState.fkRotationBuffer[boneIdx];
    }

    virtual public Vector3 GetGlobalPosition(int boneIdx)
    {
        return poseState.fkPositionBuffer[boneIdx];
    }
    virtual public Quaternion GetGlobalRotation(HumanBodyBones bone)
    {
        var boneIdx = mm.database.boneIndexMap[bone];
        return poseState.fkRotationBuffer[boneIdx];
    }

    virtual public Vector3 GetGlobalPosition(HumanBodyBones bone)
    {
        var boneIdx = mm.database.boneIndexMap[bone];
        return poseState.fkPositionBuffer[boneIdx];
    }

    virtual public bool GetGlobalPosition(HumanBodyBones bone, out Vector3 p)
    {
        p = Vector3.zero;
        if (!mm.database.boneIndexMap.ContainsKey(bone)) return false;
        var boneIdx = mm.database.boneIndexMap[bone];
        p = poseState.fkPositionBuffer[boneIdx];
        return true;
    }

    virtual public bool GetGlobalRotation(HumanBodyBones bone, out Quaternion q)
    {
        q = Quaternion.identity;
        if (!mm.database.boneIndexMap.ContainsKey(bone)) return false;
        var boneIdx = mm.database.boneIndexMap[bone];
        q = poseState.fkRotationBuffer[boneIdx];
        return true;
    }

    public static void simulationRotationsUpdate(
    ref Quaternion rotations,
    ref Vector3 avs,
    Quaternion desiredRotation,
    float halflife, float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;

        var inv_q = Quaternion.Inverse(desiredRotation);

        var delta_q = rotations * inv_q;
        if (delta_q.w < 0)
        {
            delta_q.w *= -1;
        }

        Vector3 j0 = Quat.to_scaled_angle_axis(delta_q);
        Vector3 j1 = avs + j0 * y;

        float eydt = Utils.fast_negexpf(y * dt);

        rotations = Quat.from_scaled_angle_axis(eydt * (j0 + j1 * dt)) * desiredRotation;
        avs = eydt * (avs - j1 * y * dt);
    }

    // Taken from https://theorangeduck.com/page/spring-roll-call#controllers

    public static void simulationPositionsUpdate(
    ref Vector3 position,
    ref Vector3 velocity,
    ref Vector3 acceleration,
    Vector3 desired_velocity,
   float halflife,
    float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;
        Vector3 j0 = velocity - desired_velocity;
        Vector3 j1 = acceleration + j0 * y;
        float eydt = Utils.fast_negexpf(y * dt);

        Vector3 position_prev = position;
        position = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + desired_velocity * dt + position_prev;
        velocity = eydt * (j0 + j1 * dt) + desired_velocity;
        acceleration = eydt * (acceleration - j1 * y * dt);
    }



    // Taken from https://theorangeduck.com/page/spring-roll-call#controllers
    public static void simulationPositionsUpdate(
        ref List<Vector3> position,
        ref List<Vector3> velocity,
        ref List<Vector3> acceleration,
        List<Vector3> desired_velocity,
        int index, float halflife,
        float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;
        Vector3 j0 = velocity[index] - desired_velocity[index];
        Vector3 j1 = acceleration[index] + j0 * y;
        float eydt = Utils.fast_negexpf(y * dt);

        Vector3 position_prev = position[index];
        position[index] = eydt * (((-j1) / (y * y)) + ((-j0 - j1 * dt) / y)) +
            (j1 / (y * y)) + j0 / y + desired_velocity[index] * dt + position_prev;
        velocity[index] = eydt * (j0 + j1 * dt) + desired_velocity[index];
        acceleration[index] = eydt * (acceleration[index] - j1 * y * dt);
    }




    public static void simulationRotationsUpdate(
    ref List<Quaternion> rotations,
    ref List<Vector3> avs,
    List<Quaternion> desiredRotation,
    int index, float halflife, float dt)
    {
        float y = Utils.halflife_to_damping(halflife) / 2.0f;

        var inv_q = Quaternion.Inverse(desiredRotation[index]);

        var delta_q = rotations[index] * inv_q;
        if (delta_q.w < 0)
        {
            delta_q.w *= -1;
        }

        Vector3 j0 = Quat.to_scaled_angle_axis(delta_q);
        Vector3 j1 = avs[index] + j0 * y;

        float eydt = Utils.fast_negexpf(y * dt);

        rotations[index] = Quat.from_scaled_angle_axis(eydt * (j0 + j1 * dt)) * desiredRotation[index];
        avs[index] = eydt * (avs[index] - j1 * y * dt);
    }


};

}
}