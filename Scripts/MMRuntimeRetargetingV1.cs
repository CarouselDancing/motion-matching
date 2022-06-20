
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;


namespace Carousel
{

namespace MotionMatching{
public class MMRuntimeRetargetingV1 : MonoBehaviour
{
    [Serializable]
    public class RetargetingMap
    {
        public Bones src;
        public string dst;
        public int childIdx;
        public Vector3 srcUp;
        public Vector3 srcSide;
        public Vector3 dstUp;
        public Vector3 dstSide;
        public Transform dstT;
        public bool active = true;

    }
    public MMPoseProvider src;
    private List<Transform> _transforms;
    private List<Transform> _srcTransforms;
    public List<RetargetingMap> retargetingMap;
    public float visLength = 10;
    public string rootName;
    public Vector3 rootOffset;
    public bool bringToLocal = true;
    public bool mirror = false;
    public bool showSrcGizmos = false;
    public bool showDstGizmos = false;
    public bool showAxisFeature = false;
    RetargetingMap rootMap;
    bool initialized = false;


    void Awake()
    {
        SetTransforms();
    }

    public void SetTransforms()
    {
        _transforms = GetComponentsInChildren<Transform>().ToList();

        for (int i = 0; i < retargetingMap.Count; i++)
        {

            if (retargetingMap[i].dst == rootName)
            {

                retargetingMap[i].dstT = transform;
                rootMap = retargetingMap[i];
            }
            else
            {

                var dstT = _transforms.First(x => x.name == retargetingMap[i].dst);
                retargetingMap[i].dstT = dstT;
            }

        }
        initialized = true;
    }


    // Update is called once per frame
    void FixedUpdate()
    {

        //transform.position = src.GetGlobalPosition(Bones.Entity);
        if (!initialized) SetTransforms();
        OrientToTarget();
    }
    public Quaternion RetargetRotation(Quaternion srcRotation, RetargetingMap m)
    {

        var globalSrcUp = srcRotation * m.srcUp;
        globalSrcUp = globalSrcUp.normalized;
        var globalSrcSide = srcRotation * m.srcSide;
        globalSrcSide = globalSrcSide.normalized;
        var rotation1 = Quaternion.FromToRotation(m.dstUp, globalSrcUp).normalized;
        var dstSide = rotation1 * m.dstSide;
        var rotation2 = Quaternion.FromToRotation(dstSide.normalized, globalSrcSide).normalized;
        var rotation = Quaternion.Normalize(rotation2 * rotation1);
        var dstUp = rotation * m.dstUp;
        var rotation3 = Quaternion.FromToRotation(dstUp.normalized, globalSrcUp).normalized;
        rotation = Quaternion.Normalize(rotation3 * rotation);
        return rotation;
    }

    public void OrientToTarget()
    {
        foreach (var m in retargetingMap)
        {
            if (!m.active) continue;
            var srcRotation = src.GetGlobalRotation((int)m.src);
            var srcPosition = src.GetGlobalPosition((int)m.src);
            var dstT = m.dstT;
            var rotation = RetargetRotation(srcRotation, m);
            if (bringToLocal)
            {
                var invRot = Quaternion.identity;
                if (dstT.parent != null)
                {
                    invRot = Quaternion.Inverse(dstT.parent.rotation);
                }
                dstT.localRotation = invRot * rotation.normalized;
            }
            else
            {
                dstT.rotation = rotation;
                dstT.position = srcPosition;
            }

            if (m.dst == rootName)
            {
                dstT.position = srcPosition;
                dstT.localPosition += rootOffset;
            }
        }

    }
    public void DrawTwoAxisRotationXZ(Transform t)
    {
        var m = Matrix4x4.TRS(Vector3.zero, t.rotation, Vector3.one);
        var x = new Vector3(m[0, 0], m[1, 0], m[2, 0]).normalized * visLength;
        var z = new Vector3(m[0, 2], m[1, 2], m[2, 2]).normalized * visLength;

        Debug.DrawLine(t.position, t.position + x, Color.green);
        Debug.DrawLine(t.position, t.position + z, Color.red);
    }

    void OnDrawGizmos()
    {
        if (!showSrcGizmos && !showDstGizmos && !showAxisFeature) return;
        foreach (var m in retargetingMap)
        {
            if (m.dstT != null)
            {
                if (showSrcGizmos)
                {
                    var srcRotation = src.GetGlobalRotation((int)m.src);
                    var srcPosition = src.GetGlobalPosition((int)m.src);
                    var globalSrcUp = srcRotation * m.srcUp * visLength;
                    var globalSrcSide = srcRotation * m.srcSide * visLength;
                    Debug.DrawLine(srcPosition, srcPosition + globalSrcUp, Color.green);
                    Debug.DrawLine(srcPosition, srcPosition + globalSrcSide, Color.red);
                }
                if (showDstGizmos)
                {
                    var globalDstUp = m.dstT.rotation * m.dstUp * visLength;
                    var globalDstSide = m.dstT.rotation * m.dstSide * visLength;
                    Debug.DrawLine(m.dstT.position, m.dstT.position + globalDstUp, Color.green);
                    Debug.DrawLine(m.dstT.position, m.dstT.position + globalDstSide, Color.red);
                }
                if (showAxisFeature)
                {
                    DrawTwoAxisRotationXZ(m.dstT);
                }
            }
        }
    }

    public void ResetToIdle()
    {
        src.ResetToIdle();
        OrientToTarget();
    }
}
}
}
