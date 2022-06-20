using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Carousel
{

namespace MotionMatching{
    
[Serializable]
public class PoseState
{

    public Vector3 simulationPosition;
    public Vector3 simulationVelocity;
    public Vector3 simulationAcceleration;
    public Quaternion simulationRotation;
    public Vector3 simulationAV;
    public int nBones;
    public int[] boneParents;
    public Vector3[] bonePositions;
    public Quaternion[] boneRotations;
    public Vector3[] fkPositionBuffer;
    public Quaternion[] fkRotationBuffer;
    public float yOffset = 0;
    public PoseState(int nBones, int[] boneParents, float yOffset =0)
    {
        this.yOffset = yOffset;
        simulationPosition = new Vector3(0, yOffset, 0);
        this.boneParents = boneParents;
        this.nBones = nBones;
        bonePositions = new Vector3[nBones];
        boneRotations = new Quaternion[nBones];
        fkPositionBuffer = new Vector3[nBones];
        fkRotationBuffer = new Quaternion[nBones];
        simulationRotation = Quaternion.identity;
    }

    public PoseState(PoseState other)
    {
        this.yOffset = other.yOffset;
        simulationPosition = other.simulationPosition;
        simulationRotation = other.simulationRotation;
        simulationVelocity = other.simulationVelocity;
        simulationAcceleration = other.simulationAcceleration;
        this.boneParents = other.boneParents;
        this.nBones = other.nBones;
        bonePositions = new Vector3[nBones];
        boneRotations = new Quaternion[nBones];
        fkPositionBuffer = new Vector3[nBones];
        fkRotationBuffer = new Quaternion[nBones];
        for(int i =0; i < nBones; i++) {
            bonePositions[i] = other.bonePositions[i];
            boneRotations[i] = other.boneRotations[i];
            fkPositionBuffer[i] = other.fkPositionBuffer[i];
            fkRotationBuffer[i] = other.fkRotationBuffer[i];
        }
    }


    public void SetState(MMDatabase db, int frameIdx, bool useSim = true, bool setSimVelocity = false)
    {
        int i = 0;
        if (useSim)
        {
            bonePositions[0] = simulationPosition;
            boneRotations[0] = simulationRotation;
            i = 1;
        }
        for (; i < nBones; i++)
        {
            bonePositions[i] = db.bonePositions[frameIdx, i];
            boneRotations[i] = db.boneRotations[frameIdx, i];
        }
        if (useSim && setSimVelocity)
        {
            simulationVelocity = db.boneVelocities[frameIdx, 0];
            simulationAV = db.boneAngularVelocities[frameIdx, 0];
        }
        UpdateFKBuffer();
    }

    public void Interpolate(MMDatabase db, int frameIdx, bool useSim = true, bool setSimVelocity = false)
    {
        int i = 0;
        if (useSim)
        {
            bonePositions[0] = simulationPosition;
            boneRotations[0] = simulationRotation;
            i = 1;
        }
        for (; i < nBones; i++)
        {
            bonePositions[i] = db.bonePositions[frameIdx, i];
            boneRotations[i] = db.boneRotations[frameIdx, i];
        }
        if (useSim && setSimVelocity)
        {
            simulationVelocity = db.boneVelocities[frameIdx, 0];
            simulationAV = db.boneAngularVelocities[frameIdx, 0];
        }
        UpdateFKBuffer();
    }

    void UpdateFKBuffer()
    {
        for (int i = 0; i < nBones; i++)
        {
            ForwardKinematics(out fkPositionBuffer[i], out fkRotationBuffer[i], i);
        }
    }

    public void ForwardKinematics(out Vector3 pos, out Quaternion rot, int boneIdx)
    {

        if (boneParents[boneIdx] != -1)
        {
            Vector3 parentPos; Quaternion parentRot;
            ForwardKinematics(out parentPos, out parentRot, boneParents[boneIdx]);
            pos = parentRot * bonePositions[boneIdx] + parentPos;
            rot = parentRot * boneRotations[boneIdx];
        }
        else
        {
            pos = bonePositions[boneIdx];
            rot = boneRotations[boneIdx];
        }
    }

    public void Draw(float visScale)
    {
        for (int boneIdx = 0; boneIdx < nBones; boneIdx++)
        {
            var p = bonePositions[boneIdx];
            Vector3 pos; Quaternion rot;
            ForwardKinematics(out pos, out rot, boneIdx);
            Gizmos.DrawSphere(pos, visScale);
        }
    }

    public void Reset()
    {
        simulationPosition = new Vector3(0, yOffset, 0);
        simulationRotation = Quaternion.identity;
        simulationVelocity = Vector3.zero;
        simulationAV = Vector3.zero;
        simulationAcceleration = Vector3.zero;
    }


}


}
}