using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Diagnostics;

namespace Carousel
{

namespace MotionMatching{
public enum MMDatabaseVersion
{
    HOLDEN,
    DANCE
}
public enum MMDatabaseType
{
    Trajectory  = 0,
    Pose = 1,
    Music =  2,
    Annotation = 3
}

public enum MMFeatureType
{
    Position,
    Velocity,
    TrajectoryPositions,
    TrajectoryDirections,
    Phase
}

public enum MMFileFormat
{
    Binary,
    Numpy
}

public enum Bones
{
    Entity = 0,
    Hips = 1,
    LeftUpLeg = 2,
    LeftLeg = 3,
    LeftFoot = 4,
    LeftToe = 5,
    RightUpLeg = 6,
    RightLeg = 7,
    RightFoot = 8,
    RightToe = 9,
    Spine = 10,
    Spine1 = 11,
    Spine2 = 12,
    Neck = 13,
    Head = 14,
    LeftShoulder = 15,
    LeftArm = 16,
    LeftForeArm = 17,
    LeftHand = 18,
    RightShoulder = 19,
    RightArm = 20,
    RightForeArm = 21,
    RightHand = 22,
};




[Serializable]
public class MMFeature{
    public HumanBodyBones bone;
    public int boneIdx;
    public MMFeatureType type;
    public float weight;
}

[Serializable]
public class MMSettings
{
    public int ignoreRangeEnd;
    public MMDatabaseVersion version;
    public MMDatabaseType databaseType;
    public List<MMFeature> features = new List<MMFeature>() { new MMFeature() { weight = 0.75f, bone = HumanBodyBones.LeftFoot, type= MMFeatureType.Position},
                                                         new MMFeature() { weight = 1, bone = HumanBodyBones.LeftFoot, type= MMFeatureType.Velocity},
                                                          new MMFeature() { weight = 0.75f, bone = HumanBodyBones.RightFoot, type= MMFeatureType.Position},
                                                           new MMFeature() { weight = 1, bone = HumanBodyBones.RightFoot, type= MMFeatureType.Velocity},
                                                           new MMFeature() { weight = 1, bone = HumanBodyBones.Hips, type= MMFeatureType.Velocity},
                                                           new MMFeature() { weight = 1, bone = HumanBodyBones.LastBone, type= MMFeatureType.TrajectoryPositions},
                                                           new MMFeature() { weight = 1.25f, bone = HumanBodyBones.LastBone, type= MMFeatureType.TrajectoryDirections},
                                                             };
    public MMFileFormat format;
    
}



public class SearchResult
{
    int size;
    public SortedList<float, int> sortedList;
    public SearchResult(int size)
    {
        this.size = size;
        sortedList = new SortedList<float, int>();
    }

    public void Update(int frameIdx, float cost)
    {
        if (sortedList.ContainsKey(cost)) return;
        if (sortedList.Count == size)
        {
            if (sortedList.Keys[sortedList.Count-1] > cost)
            {
                sortedList.Add(cost, frameIdx);
                sortedList.RemoveAt(size - 1);
            }
        }
        else
        {
            sortedList.Add(cost, frameIdx);
            
        }

    }
    public int GetBestIndex() { 
        return sortedList.Values[0];
    }

    public int GetRandomIndex()
    {
        int idx = UnityEngine.Random.Range(0, sortedList.Count);
        return sortedList.Values[idx];
    }

    public int GetFurthestIndex(int currentIdx)
    {
        int bestIdx = 0;
        int bestDistance = 0;
        for (int i = 0; i< sortedList.Count; i++)
        {
            int distance = Math.Abs(sortedList.Values[i] - currentIdx);
            if (distance >= bestDistance)
            {
                bestDistance = distance;
                bestIdx = i;
            }
        }
        return sortedList.Values[bestIdx];
    }

    public int GetRandomIndex(int currentIdx) {
        float[] distances = new float[sortedList.Count];
        float sum = 0;
        for (int i = 0; i < sortedList.Count; i++)
        {
            distances[i] = Math.Abs(sortedList.Values[i] - currentIdx);
            sum += distances[i];
        }
        float r = UnityEngine.Random.Range(0.0f, 1.0f);
        //chose the index based on range defined by cumulative sum
        float cumulativeSum = 0;
        int bestIdx = 0;
        for (int i = 0; i < sortedList.Count; i++)
        {
            cumulativeSum += distances[i] / sum;
            if(r <= cumulativeSum)
            {
                bestIdx = i;
                break;
            }
        }
        //UnityEngine.Debug.Log("idx:" + bestIdx.ToString() + " d: " +  distances[bestIdx].ToString()+" r:" + r.ToString());
        return sortedList.Values[bestIdx];
    }
}

[Serializable]
public class MMDatabase 
{

    public float fps = 60f;
    public int nFrames;
    public int nBones;
    public int nMusicDims;
    public int nAnnotations;
    public Vector3[,] bonePositions; //offsets relative to parent
    public Vector3[,] boneVelocities; //velocites relative to parent coordinate system
    public Quaternion[,] boneRotations; //rotations relative to parent coordinate system
    public Vector3[,] boneAngularVelocities;
    public int[] boneParents;
    public int[] rangeStart;
    public int[] rangeStop;
    public bool[,] contactStates;
    public float[] phaseData;
    public List<string> annotationKeys;
    public List<string> annotationValues;
    public int[,] annotationMatrix;



    public float[,] features;
    public float[] featuresMean;
    public float[] featuresScale;
    public int nFeatures;
    public MMSettings settings;
    public List<string> boneNames = new List<string>{ "Entity", "Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToe",
                                                      "RightUpLeg", "RightLeg", "RightFoot",  "RightToe", "Spine", "Spine1",  "Spine2",  "Neck", "Head", "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand", "RightShoulder", "RightArm", "RightForeArm", "RightHand" };
    public Dictionary<HumanBodyBones, string> boneMap = new Dictionary<HumanBodyBones, string>() { { HumanBodyBones.LastBone, "Entity" },
                                                                                                    { HumanBodyBones.LeftFoot, "LeftFoot" },
                                                                                                     { HumanBodyBones.RightFoot, "RightFoot" },
                                                                                                    { HumanBodyBones.LeftHand, "LeftHand" },
                                                                                                     { HumanBodyBones.RightHand, "RightHand" },
                                                                                                     { HumanBodyBones.Hips, "Hips" },};
    public Dictionary<HumanBodyBones, int> boneIndexMap;
    public List<HumanBodyBones> boneArray;

    public bool useConstraint;
    int[] constraint;

    public MMDatabase(MMSettings _settings)
    {
        settings = _settings;
    }
    
    public void SetAnnotationConstraint(int[] constraint){
        this.constraint = constraint;
        useConstraint = true;
    }

    public void RemoveAnnotationConstraint(){
        this.constraint = constraint;
        useConstraint = false;
    }

    public void GetAnnotationConstraint(ref List<int> constraint, int frameIdx){
        for (int i = 0; i < nAnnotations; i++){
            constraint[i] = annotationMatrix[i, frameIdx];
        }
    }

    public void CopyFeatureUnnormalized(ref float[] result, ref int offset, int size, int frameIdx)
    {
        for(int i = 0; i < size; i++)
        {
            result[offset+i] = (features[frameIdx, offset +i]  * featuresScale[offset + i]) + featuresMean[offset + i];
        }
        offset += size;
    }

    public float[] normalizeFeature(float[] query, int offset =0)
    {
        float[] queryNormalized = new float[nFeatures];
        for (int i = offset; i < nFeatures; i++)
        {
            queryNormalized[i] = (query[i] - featuresMean[i]) / featuresScale[i];
        }
        return queryNormalized;
    }

    public void Search(float[] query, ref SearchResult result)
    {
        float[] queryNormalized = normalizeFeature(query);
        for (int rIdx = 0; rIdx < rangeStart.Length; rIdx++)
        {
            int frameIdx = rangeStart[rIdx];
            int rangeEnd = rangeStop[rIdx] - settings.ignoreRangeEnd;
            while (frameIdx < rangeEnd)
            {
                float cost = GetDistance(queryNormalized, frameIdx);
                result.Update(frameIdx, cost);
                frameIdx++;
            }
        }
    }

    public int Search(float[] query, int bestIdx = -1)
    {
        float[] queryNormalized = normalizeFeature(query);

        float bestCost = Mathf.Infinity;
        float initialCost = Mathf.Infinity;
        if (bestIdx > -1)
        {
            bestCost = GetDistance(queryNormalized, bestIdx);
            initialCost = bestCost;
        }
        for (int rIdx = 0; rIdx < rangeStart.Length; rIdx++)
        {
            int frameIdx = rangeStart[rIdx];
            int rangeEnd = rangeStop[rIdx] - settings.ignoreRangeEnd;
            while (frameIdx < rangeEnd)
            {
                float cost = Mathf.Infinity;
                if (useConstraint){
                    if (checkConstraint(frameIdx)){
                        cost = GetDistance(queryNormalized, frameIdx);
                    }
                }else{
                    cost = GetDistance(queryNormalized, frameIdx);
                }
                if (cost < bestCost)
                {
                    bestIdx = frameIdx;
                    bestCost = cost;
                }
                frameIdx++;
            }
        }
        //Debug.Log("best cost"+ bestCost.ToString() + " < " +  initialCost.ToString());
        return bestIdx;
    }

    public bool checkConstraint(int index){
        int distance = 0;
        for (int i = 0; i < nAnnotations; i++)
        {
            distance += constraint[i] - annotationMatrix[i, index];
        }
        return distance == 0;
    }

    public int SearchOld(float[] query, int bestIdx = -1)
    {
        float[] queryNormalized = normalizeFeature(query);
        float bestCost = Mathf.Infinity;
        float initialCost = Mathf.Infinity;
        if (bestIdx > -1)
        {
            bestCost = GetDistance(queryNormalized, bestIdx);
            initialCost = bestCost;
        }
        for (int frameIdx = 0; frameIdx < nFrames; frameIdx++)
        {
            float cost = GetDistance(queryNormalized, frameIdx);
            if(cost < bestCost)
            {
                bestIdx = frameIdx;
                bestCost = cost;
            }
        }
        //Debug.Log("best cost"+ bestCost.ToString() + " < " +  initialCost.ToString());
        return bestIdx;
    }

    public float GetDistance(float[] queryNormalized, int index)
    {
        float cost = 0.0f;
        float temp;
        for (int i = 0; i < nFeatures; i++)
        {
            temp = queryNormalized[i] - features[index, i];
            cost += temp*temp;
        }
        return cost;
    }

    public void ComputeFeatures()
    {

        nFeatures = 0;
        foreach(var f in settings.features)
        {if (f.type == MMFeatureType.Phase)
            {
                nFeatures += 1;
            }
            else if (f.type == MMFeatureType.TrajectoryPositions || f.type == MMFeatureType.TrajectoryDirections)
            {
                nFeatures += 6;
            }
            else { 
                nFeatures += 3;
            }
        }

        features = new float[nFrames, nFeatures];
        featuresMean = new float[nFeatures];
        featuresScale = new float[nFeatures];
        int offset = 0;
        foreach (var f in settings.features)
        {
            int boneIdx = f.boneIdx;
            if (settings.version != MMDatabaseVersion.HOLDEN)
                boneIdx = boneIndexMap[f.bone];
            //UnityEngine.Debug.Log(f.bone.ToString()+" "+boneIdx.ToString());
            switch (f.type) {
                case MMFeatureType.Phase:
                    CopyPhaseFeature(ref offset, f.weight);
                    break;
                case MMFeatureType.Position:
                    ComputeBonePositionFeature(ref offset, boneIdx, f.weight);
                    break;
                case MMFeatureType.Velocity:
                    ComputeBoneVelocityFeature(ref offset, boneIdx, f.weight);
                    break;
                case MMFeatureType.TrajectoryPositions:
                    ComputeTrajectoryPositionFeature(ref offset, boneIdx, f.weight);
                    break;
                case MMFeatureType.TrajectoryDirections:
                    ComputeTrajectoryDirectionFeature(ref offset, boneIdx, f.weight);
                    break;
            }
            
        }

    }



    // hard coded for left foot right foot and hips
    public void GetFeatureTrajectory(PoseState state, int frameIdx, ref List<Vector3> featureTrajectoryPos)
    {
        int offset = 15;// skip to the  joint features
        float x1 = (features[frameIdx, offset + 0] * featuresScale[offset + 0]) + featuresMean[offset + 0];
        float z1 = (features[frameIdx, offset + 1] * featuresScale[offset + 1]) + featuresMean[offset + 1];

        float x2 = (features[frameIdx, offset + 2] * featuresScale[offset + 2]) + featuresMean[offset + 2];
        float z2 = (features[frameIdx, offset + 3] * featuresScale[offset + 3]) + featuresMean[offset + 3];

        float x3 = (features[frameIdx, offset + 4] * featuresScale[offset + 4]) + featuresMean[offset + 4];
        float z3 = (features[frameIdx, offset + 5] * featuresScale[offset + 5]) + featuresMean[offset + 5];
        featureTrajectoryPos[0] = state.bonePositions[0] + state.boneRotations[0] * new Vector3(x1, 0, z1);
        featureTrajectoryPos[1] = state.bonePositions[0] + state.boneRotations[0] * new Vector3(x2, 0, z2);
        featureTrajectoryPos[2] = state.bonePositions[0] + state.boneRotations[0] * new Vector3(x3, 0, z3);
    }

    // When we add an offset to a frame in the database there is a chance
    // it will go out of the relevant range so here we can clamp it to 
    // the last frame of that range.
    public int trajectoryIndexClamp(int frame, int offset)
    {
        for (int i = 0; i < rangeStart.Length; i++)
        {
            if (frame >= rangeStart[i] && frame < rangeStop[i])
            {
                return Mathf.Clamp(frame + offset, rangeStart[i], rangeStop[i] - 1);
            }
        }
        if(frame < 0) frame = 0;
        if(frame > nFrames) frame = nFrames-1;
        return frame;
    }

    void ComputeTrajectoryPositionFeature(ref int offset, int boneIdx, float weight)
    {
        for (int i = 0; i < nFrames; i++)
        {
            int t0 = trajectoryIndexClamp(i, 20);
            int t1 = trajectoryIndexClamp(i, 40);
            int t2 = trajectoryIndexClamp(i, 60);
            var rootInvRot = Quaternion.Inverse(boneRotations[i, boneIdx]);
            var delta0 = rootInvRot * (bonePositions[t0, boneIdx] - bonePositions[i, boneIdx]);
            var delta1 = rootInvRot * (bonePositions[t1, boneIdx] - bonePositions[i, boneIdx]);
            var delta2 = rootInvRot * (bonePositions[t2, boneIdx] - bonePositions[i, boneIdx]);
        
            features[i, offset + 0] = delta0.x;
            features[i, offset + 1] = delta0.z;
            features[i, offset + 2] = delta1.x;
            features[i, offset + 3] = delta1.z;
            features[i, offset + 4] = delta2.x;
            features[i, offset + 5] = delta2.z;
        }

        normalizeFeature(offset, 6, weight);
        offset += 6;
    }

    void ComputeTrajectoryDirectionFeature(ref int offset, int boneIdx, float weight)
    {
        for (int i = 0; i < nFrames; i++)
        {
            int t0 = trajectoryIndexClamp(i, 20);
            int t1 = trajectoryIndexClamp(i, 40);
            int t2 = trajectoryIndexClamp(i, 60);
            var rootInvRot = Quaternion.Inverse(boneRotations[i, boneIdx]);
            var delta0 = rootInvRot * (boneRotations[t0, boneIdx] * new Vector3(0, 0, 1));
            var delta1 = rootInvRot * (boneRotations[t1, boneIdx] * new Vector3(0, 0, 1));
            var delta2 = rootInvRot * (boneRotations[t2, boneIdx] * new Vector3(0, 0, 1));
            features[i, offset + 0] = delta0.x;
            features[i, offset + 1] = delta0.z;
            features[i, offset + 2] = delta1.x;
            features[i, offset + 3] = delta1.z;
            features[i, offset + 4] = delta2.x;
            features[i, offset + 5] = delta2.z;
        }

        normalizeFeature(offset, 6, weight);
        offset += 6;
    }

    void CopyPhaseFeature(ref int offset, float weight)
    {

        for (int i =0; i < nFrames; i++)
        {
            features[i, offset] = phaseData[i];
        }

        normalizeFeature(offset, 1, weight);
        offset += 1;
    }

    void ComputeBonePositionFeature(ref int offset, int boneIdx, float weight)
    {

        for (int i =0; i < nFrames; i++)
        {
            Vector3 pos; Quaternion rot;
            ForwardKinematics(out pos, out rot, i, boneIdx);
            var relPos = pos - bonePositions[i, 0];
            var invRootRot = Quaternion.Inverse(boneRotations[i, 0]);
            relPos = invRootRot * relPos;
            features[i, offset + 0] = relPos.x;
            features[i, offset + 1] = relPos.y;
            features[i, offset + 2] = relPos.z;
        }

        normalizeFeature(offset, 3, weight);
        offset += 3;
    }

    void ComputeBoneVelocityFeature(ref int offset, int boneIdx, float weight)
    {
        for (int i = 0; i < nFrames; i++)
        {
            Vector3 pos; Vector3 vel;  Quaternion rot; Vector3 av;
            ForwardKinematicsVelocity(out pos, out vel, out rot, out av,  i, boneIdx);
            var invRootRot = Quaternion.Inverse(boneRotations[i, 0]);
            vel = invRootRot * vel;
            features[i, offset + 0] = vel.x;
            features[i, offset + 1] = vel.y;
            features[i, offset + 2] = vel.z;
        }

        normalizeFeature(offset, 3, weight);
        offset += 3;
    }

    void normalizeFeature(int offset, int size, float weight)
    {
        // First compute what is essentially the mean 
        // value for each feature dimension
        float[] vars = new float[size];
        for (int j = 0; j < size; j++)
        {
            featuresMean[offset + j] = 0.0f;
            vars[j] = 0.0f;
        }
        for (int i = 0; i < nFrames; i++)
        {
            for (int j = 0; j < size; j++)
            {
                featuresMean[offset + j] += features[i, offset + j] / nFrames;
            }
        }
        // Now compute the variance of each feature dimension
        float  temp;
        for (int i = 0; i < nFrames; i++)
        {
            for (int j = 0; j < size; j++)
            {
                temp = (features[i, offset + j] - featuresMean[offset + j]);
                vars[j] += (temp*temp)/ nFrames;
            }
        }
        // We compute the overall std of the feature as the average
        // std across all dimensions
        float std = 0.0f;
        for (int j = 0; j < size; j++)
        {
            std += Mathf.Sqrt(vars[j]) / size;
        }
        for (int j = 0; j < size; j++)
        {
            featuresScale[offset + j] = std / weight;
        }

        // Using the offset and scale we can then normalize the features
        for (int i = 0; i < nFrames; i++)
        {
            for (int j = 0; j < size; j++)
            {
                features[i, offset + j] = (features[i, offset + j] - featuresMean[offset + j]) / featuresScale[offset + j];
            }
        }

    }

    void normalizeFeature2(int offset, int size, float weight)
    {
        // First compute what is essentially the mean 
        // value and var for each feature dimension
        float[] vars = new float[size];
        for (int j = 0; j < size; j++)
        {
            featuresMean[offset + j] = 0.0f;
            vars[j] = 0.0f;

            for (int i = 0; i < nFrames; i++)
            {
                featuresMean[offset + j] += features[i, offset + j] / nFrames;
            }
            for (int i = 0; i < nFrames; i++)
            {
                float temp = (features[i, offset + j] - featuresMean[offset + j]);
                vars[j] += (temp * temp) / nFrames;
            }
        }

        // We compute the overall std of the feature as the average
        // std across all dimensions
        float std = 0.0f;
        for (int j = 0; j < size; j++)
        {
            std += Mathf.Sqrt(vars[j]) / size;
        }
        for (int j = 0; j < size; j++)
        {
            featuresScale[offset + j] = std / weight;
        }
        // Using the offset and scale we can then normalize the features
        for (int i = 0; i < nFrames; i++)
        {
            for (int j = 0; j < size; j++)
            {
                features[i, offset + j] = (features[i, offset + j] - featuresMean[offset + j]) / featuresScale[offset + j];
            }
        }
    }

    public Vector3 GetBonePosition(int frameIdx, int boneIdx)
    {
        return bonePositions[frameIdx, boneIdx];
    }
    public Quaternion GetBoneRotation(int frameIdx, int boneIdx)
    {
        return boneRotations[frameIdx, boneIdx];
    }
    public void ForwardKinematics(out Vector3 pos, out Quaternion rot, int frameIdx, int boneIdx)
    {

        if (boneParents[boneIdx] != -1)
        {
            Vector3 parentPos; Quaternion parentRot;
            ForwardKinematics(out parentPos, out parentRot, frameIdx, boneParents[boneIdx]);
            pos = parentRot*GetBonePosition(frameIdx, boneIdx) + parentPos;
            rot = parentRot*GetBoneRotation(frameIdx, boneIdx);
        }
        else
        {
            pos = GetBonePosition(frameIdx, boneIdx);
            rot = GetBoneRotation(frameIdx, boneIdx);
        }
    }


    public void ForwardKinematicsVelocity(out Vector3 pos, out Vector3 vel, out Quaternion rot, out Vector3 av, int frameIdx, int boneIdx)
    {

        if (boneParents[boneIdx] != -1)
        {
            Vector3 parentPos; Vector3 parentVel; Quaternion parentRot; Vector3 parentAV;
            ForwardKinematicsVelocity(out parentPos, out parentVel, out parentRot, out parentAV, frameIdx, boneParents[boneIdx]);
            pos = parentRot * bonePositions[frameIdx, boneIdx] + parentPos;
            vel = parentVel + (parentRot * boneVelocities[frameIdx, boneIdx]) + Vector3.Cross(parentAV, parentRot * bonePositions[frameIdx, boneIdx]);
            rot = parentRot * boneRotations[frameIdx, boneIdx];
            av = parentRot * (boneAngularVelocities[frameIdx, boneIdx] + parentAV);
        }
        else
        {
            pos = bonePositions[frameIdx, boneIdx];
            vel = boneVelocities[frameIdx, boneIdx];
            rot = boneRotations[frameIdx, boneIdx];
            av = boneAngularVelocities[frameIdx, boneIdx];


        }
    }


}
}
}