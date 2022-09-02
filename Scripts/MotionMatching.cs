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
        return !(database == null || idx < 0 || idx > database.nFrames);
    }
    public PoseState Load()
    {
        // var fullPath = Path.Combine(Application.streamingAssetsPath, filename);
        //database.Load(fullPath);
        //state = new MotionState(database.nBones, database.boneParents);
        //state.SetState(database, frameIdx);
        if (settings.format == MMFileFormat.Binary){
            var loader = new MMDatabaseBinaryLoader(settings);
            database = loader.LoadResource(filename);
        }else{
            var loader = new MMDatabaseNumbyLoader(settings);
            database = loader.LoadResource(filename);

        }
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
    
    public void SetAnnotationConstraint(int[] constraint){
        database.SetAnnotationConstraint(constraint);
    }

    public void RemoveAnnotationConstraint(){
        database.RemoveAnnotationConstraint();
    }

    public void GetAnnotationConstraint(ref List<int> constraint, int frameIdx){
        database.GetAnnotationConstraint(ref constraint, frameIdx);
    }

    public int FindTransition(PoseState state, int frameIdx, List<Vector3> trajectoryPos, List<Quaternion> trajectoryRot)
    {
        float[] query = database.ComputeQuery(state, frameIdx, trajectoryPos, trajectoryRot);
        return database.Search(query, frameIdx);
    }

    public int FindTransition(PoseState state, int frameIdx)
    {
        float[] query = database.ComputeQuery(frameIdx);

        return database.Search(query, frameIdx);
    }

    public void FindTransition(PoseState state, int frameIdx, ref SearchResult result)
    {
        float[] query = database.ComputeQuery(frameIdx);

        database.Search(query, ref result);
    }

    public void FindTransition(PoseState state, int frameIdx, List<Vector3> trajectoryPos, List<Quaternion> trajectoryRot, ref SearchResult result)
    {
        float[] query = database.ComputeQuery(state, frameIdx, trajectoryPos, trajectoryRot);
        database.Search(query, ref result);
    }

    public int TestSearch(int frameIdx)
    {
        return database.TestSearch(frameIdx);
    }

    public void GetFeatureTrajectory(PoseState state, int frameIdx, ref List<Vector3> trajectory)
    {

        database.GetFeatureTrajectory(state, frameIdx, ref trajectory);
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
