using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

namespace Carousel
{

namespace MotionMatching{

public class MotionMatching : MonoBehaviour
{
    [SerializeField] public MMDatabase database;
    public string filename;
    public MMSettings settings;
    public bool initialized = false;
    public PoseState initialState;

    public bool AssertIndex(int idx)
    {
        return  !(database == null || idx < 0 || idx > database.nFrames);
    }
    public PoseState Load()
    {
        database = new MMDatabase(settings);

        // var fullPath = Path.Combine(Application.streamingAssetsPath, filename);
        //database.Load(fullPath);
        //state = new MotionState(database.nBones, database.boneParents);
        //state.SetState(database, frameIdx);

        database.LoadResource(filename);
        initialized = true;
        initialState = new PoseState(database.nBones, database.boneParents);
        initialState.SetState(database, 0);
        return initialState;
    }

    public void ComputeFeatures()
    {
        database.ComputeFeatures();
        Debug.Log("caclulated features" + database.features.Length.ToString() +" "+ database.nFeatures.ToString());
    }

    public int FindTransition(PoseState state, int frameIdx, List<Vector3> trajectoryPos, List<Quaternion> trajectoryRot)
    {
        float[] query = ComputeQuery(state, frameIdx, trajectoryPos, trajectoryRot);
        return database.Search(query, frameIdx);
    }

    public int FindTransition(PoseState state, int frameIdx)
    {
        float[] query = ComputeQuery(frameIdx);

        return database.Search(query, frameIdx);
    }

    public void FindTransition(PoseState state, int frameIdx, ref SearchResult result)
    {
        float[] query = ComputeQuery(frameIdx);

        database.Search(query, ref result);
    }

    public void FindTransition(PoseState state, int frameIdx, List<Vector3> trajectoryPos, List<Quaternion> trajectoryRot, ref SearchResult result)
    {
        float[] query = ComputeQuery(state, frameIdx, trajectoryPos, trajectoryRot);
        database.Search(query, ref result);
    }

public float[] ComputeQuery(int frameIdx)
    {
        var query = new float[database.nFeatures];
        int offset = 0;
        foreach (var f in settings.features)
        {
            switch (f.type)
            {
                case MMFeatureType.Position:
                    database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx);
                    break;
                case MMFeatureType.Velocity:
                    database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx);
                    break;
            }

        }

        /*int offset = 0;
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // left foot pos
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // right foot pos
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // left foot vel
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // right foot vel
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // hip vel*/
        return query;
    }
    public float[] ComputeQuery(PoseState state, int frameIdx, List<Vector3> trajectoryPos, List<Quaternion> trajectoryRot)
    {
        var query = new float[database.nFeatures];
        int offset = 0;
        foreach (var f in settings.features)
        {
            switch (f.type)
            {
                case MMFeatureType.Position:
                    database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx);
                    break;
                case MMFeatureType.Velocity:
                    database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx);
                    break;
                case MMFeatureType.TrajectoryPositions:
                    computeTrajectoryPositionFeature(state, (int)f.boneIdx, trajectoryPos, trajectoryRot, ref query, ref offset);
                    break;
                case MMFeatureType.TrajectoryDirections:
                    computeTrajectoryDirectionFeature(state, (int)f.boneIdx, trajectoryPos, trajectoryRot, ref query, ref offset);
                    break;
            }

        }
        /*
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // left foot pos
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // right foot pos
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // left foot vel
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // right foot vel
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // hip vel
        if (settings.databaseType == MMDatabaseType.Trajectory)
        {
            computeTrajectoryPositionFeature(state, 0, trajectoryPos, trajectoryRot, ref query, ref offset);
            computeTrajectoryDirectionFeature(state, 0, trajectoryPos, trajectoryRot, ref query, ref offset);
        }
        */
        return query;
    }

    public int TestSearch(int frameIdx)
    {
        float[] query = new float[database.nFeatures];
        int offset = 0;
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // left foot pos
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // right foot pos
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // left foot vel
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // right foot vel
        database.CopyFeatureUnnormalized(ref query, ref offset, 3, frameIdx); // hip vel
        database.CopyFeatureUnnormalized(ref query, ref offset, 6, frameIdx); // trajectory pos
        database.CopyFeatureUnnormalized(ref query, ref offset, 6, frameIdx); // trajectory vel
        return database.Search(query, frameIdx);
    }

    public void GetFeatureTrajectory(PoseState state, int frameIdx, ref List<Vector3> trajectory)
    {

        database.GetFeatureTrajectory(state, frameIdx, ref trajectory);
    }


    void computeTrajectoryPositionFeature(PoseState state, int boneIdx, List<Vector3> trajectoryPos, List<Quaternion> trajectoryRot, ref float[] query, ref int offset)
    {
        var rootPosition = state.bonePositions[boneIdx];
        var rootInvRot = Quaternion.Inverse(state.boneRotations[boneIdx]);
        var delta0 = rootInvRot * (trajectoryPos[0] - rootPosition);
        var delta1 = rootInvRot * (trajectoryPos[1] - rootPosition);
        var delta2 = rootInvRot * (trajectoryPos[2] - rootPosition);
        query[offset + 0] = delta0.x;
        query[offset + 1] = delta0.z;
        query[offset + 2] = delta1.x;
        query[offset + 3] = delta1.z;
        query[offset + 4] = delta2.x;
        query[offset + 5] = delta2.z;
        offset += 6;
    }

    void computeTrajectoryDirectionFeature(PoseState state, int boneIdx, List<Vector3> trajectoryPos, List<Quaternion> trajectoryRot, ref float[] query, ref int offset)
    {
       
        var rootInvRot = Quaternion.Inverse(state.boneRotations[boneIdx]);
        var delta0 = rootInvRot * (trajectoryRot[0] * new Vector3(0, 0, 1));
        var delta1 = rootInvRot * (trajectoryRot[1] * new Vector3(0, 0, 1));
        var delta2 = rootInvRot * (trajectoryRot[2] * new Vector3(0, 0, 1));
        query[offset + 0] = delta0.x;
        query[offset + 1] = delta0.z;
        query[offset + 2] = delta1.x;
        query[offset + 3] = delta1.z;
        query[offset + 4] = delta2.x;
        query[offset + 5] = delta2.z;
        offset += 6;
    }


    public int trajectoryIndexClamp(int frame, int offset)
    {
        return database.trajectoryIndexClamp(frame, offset);
    }


    public void DrawPose(int fIdx, float visScale)
    {
        for (int boneIdx = 0; boneIdx < database.nBones; boneIdx++)
        {
            var p = database.GetBonePosition(fIdx, boneIdx);
            Vector3 pos; Quaternion rot;
            database.ForwardKinematics(out pos, out rot, fIdx, boneIdx);
            Gizmos.DrawSphere(pos, visScale);
        }
    }


}  
}
}
