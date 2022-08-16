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
    public bool useInterpolation = true;
    public bool moveWithVelo;
    public TargetLocomotionController tLC;
    public float maxDegreesPerSecond = 90f;

    public Vector3 simulationPosition;
    public Vector3 simulationVelocity;
    public Vector3 simulationAcceleration;
    public Quaternion simulationRotation;
    public Vector3 simulationAV;
    public Vector3 motionVelocity;
    public Vector3 motionAV;
    public Quaternion motionRotation;
    public int nBones;
    public int[] boneParents;
    public Vector3[] bonePositions;
    public Quaternion[] boneRotations;
    public Vector3[] fkPositionBuffer;
    public Quaternion[] fkRotationBuffer;
    public bool[] fkCalculated;
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
        fkCalculated = new bool[nBones];
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
            fkCalculated[i] = other.fkCalculated[i];
        }
    }

    
    public void SetState(MMDatabase db, int frameIdx, bool useSim = true, bool setSimVelocity = false)
    {
            int i = 0;
            if (useSim)
            {
                //Vector3 vv = (simulationPosition - bonePositions[0]) * a;
                bonePositions[0] = simulationPosition;
                boneRotations[0] = simulationRotation;
                i = 1;
            }
            i = useSim ? 1 : 0;

            float maxAngle = float.MinValue;
                float a = -1;
            if (useInterpolation)
            {
                for (; i < nBones; i++)
                {

                        float angle = Quaternion.Angle(boneRotations[i], db.boneRotations[frameIdx, i]);
                    if(maxAngle < angle)
                        {
                            maxAngle = angle;
                            
                            a = Mathf.Min(1, (maxDegreesPerSecond * Time.fixedDeltaTime) / angle);
                        }
                }
            }

            if (moveWithVelo)
                tLC.velocityScale = Mathf.Min(2, Mathf.Max(0,a-0.1f) * 10);

            i = useSim ? 1 : 0;

            for (; i < nBones; i++)
            {
                if (useInterpolation)
                {
                    float angle = Quaternion.Angle(boneRotations[i], db.boneRotations[frameIdx, i]);
                    float rotSpeed = angle / maxAngle * maxDegreesPerSecond * Time.fixedDeltaTime;

                    bonePositions[i] = db.bonePositions[frameIdx, i];
                    boneRotations[i] = Quaternion.RotateTowards(boneRotations[i], db.boneRotations[frameIdx, i], rotSpeed); ;// db.boneRotations[frameIdx, i];
                }
                else
                {
                    bonePositions[i] = db.bonePositions[frameIdx, i];
                    boneRotations[i] = db.boneRotations[frameIdx, i];
                }
       
            }


            //if (moveWithVelo && useInterpolation && useSim)
            //{
            //    Debug.Log("A: " + a);
            //    Vector3 vv = (bonePositions[0] - simulationPosition) * (1-a);
            //    bonePositions[0] = simulationPosition +  vv;//simulationPosition;
            //    boneRotations[0] = simulationRotation;

            //}

            int frameIdxMinusOne = frameIdx == 0 ? 0 : frameIdx - 1;
            Vector3 v = Vector3.ProjectOnPlane(db.boneVelocities[frameIdxMinusOne, 1], Vector3.up);
            
            Debug.DrawRay(bonePositions[0], boneRotations[0] * Vector3.forward, Color.red);
            Debug.DrawRay(bonePositions[0], v, Color.magenta);

            motionRotation = db.boneRotations[frameIdx, 0];
            motionVelocity = Quaternion.Inverse(motionRotation) * db.boneVelocities[frameIdx, 0];
            motionAV = Quaternion.Inverse(motionRotation) * db.boneAngularVelocities[frameIdx, 0];
            motionAV = -motionAV;

            if (useSim && setSimVelocity)
            {
                simulationVelocity = motionVelocity;
                simulationAV = motionAV;
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
            fkCalculated[i] = false;
        }
            
        for (int i = 0; i < nBones; i++)
        {  
            if(!fkCalculated[i]){
                ForwardKinematics(out fkPositionBuffer[i], out fkRotationBuffer[i], i);
                fkCalculated[i] = true;
            }
            
        }
    }

    public void ForwardKinematics(out Vector3 pos, out Quaternion rot, int boneIdx)
    {

        if (boneParents[boneIdx] != -1)
        {
            Vector3 parentPos; Quaternion parentRot;
            var boneParentIdx = boneParents[boneIdx];
            if(!fkCalculated[boneParentIdx]){
                ForwardKinematics(out parentPos, out parentRot, boneParentIdx);
                
                fkCalculated[boneParentIdx] = true;
            }else{
                parentPos = bonePositions[boneParentIdx];
                parentRot = fkRotationBuffer[boneParentIdx];
            }
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
            //var p = bonePositions[boneIdx];
            //Vector3 pos; Quaternion rot;
            //ForwardKinematics(out pos, out rot, boneIdx);
            Vector3 pos = fkPositionBuffer[boneIdx];
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