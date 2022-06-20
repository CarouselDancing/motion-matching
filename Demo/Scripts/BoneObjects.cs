using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
namespace Carousel
{

namespace MotionMatching{
    
public class BoneObjects : MonoBehaviour
{
    public MMPoseProvider poseProvider;
    public List<Transform> bones;
    public GameObject bonePrefab;

    // Update is called once per frame
    void Update()
    {
        if (bones.Count == 0 && poseProvider.mm.initialized)
        {
            CreateMarkers();
        }
        Vector3 position;
        for (int i = 0; i < bones.Count; i++)
        {
            var bone = poseProvider.mm.database.boneArray[i];
            //poseProvider.GetGlobalPosition(bone, out position);
            position = poseProvider.GetGlobalPosition(i);
            bones[i].transform.position = position;
        }
    }

    void CreateMarkers()
    {
        bones = new List<Transform>();
        int count = 0;
        foreach (var label in poseProvider.mm.database.boneNames)
        {
            var o = GameObject.Instantiate(bonePrefab);
            o.name = count.ToString()+ ":"+label;
            bones.Add(o.GetComponent<Transform>());
            count++;
        }
    }


}
}
}