using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace Carousel
{

namespace MotionMatching{
    
public class TestPrediction : MonoBehaviour
{

    public MotionMatching mm;

    public int frameIdx = 0;
    public float visScale = 0.1f;
    List<Vector3> featureTrajectoryPos = new List<Vector3>() { new Vector3(0, 0, 0), new Vector3(0, 0, 0), new Vector3(0, 0, 0)};
    PoseState poseState;
    private void Start()
    {
        poseState = mm.Load();
        mm.ComputeFeatures();
    }
    void drawTrajectory(List<Vector3> trajectory)
    {
        for (int i = 0; i < trajectory.Count; i++)
        {
            Gizmos.DrawSphere(trajectory[i], visScale);
            // Gizmos.DrawMesh(PrimitiveType.Cube, trajectoryPos[i], trajectoryRot[i]);
        }
    }

    public void TestSearch()
    {
        int resultIdx = mm.TestSearch(frameIdx);
        Debug.Log("result " + resultIdx);
    }


    public void OnDrawGizmos()
    {
        if (!mm.initialized) return;
        Gizmos.color = Color.yellow;
        poseState.SetState(mm.database, frameIdx, false);
        mm.GetFeatureTrajectory(poseState, frameIdx, ref featureTrajectoryPos);
        float sum = 0;
        for (int i = 0; i < featureTrajectoryPos.Count; i++)
        {
            sum += Vector3.SqrMagnitude(featureTrajectoryPos[i]);
            Debug.Log("trajectory" + i.ToString() + " " +  featureTrajectoryPos[i].ToString());
        }
        Debug.Log("trajectory sum" + sum.ToString());
        drawTrajectory(featureTrajectoryPos);
        Gizmos.color = Color.green;
        mm.DrawPose(frameIdx, visScale);
    }

}
}
}