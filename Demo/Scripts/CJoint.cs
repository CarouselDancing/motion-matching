using System.Collections.Generic;
using UnityEngine;
using System;

namespace CustomAnimation {

    public class CJoint
    {
        public string name;
        public List<CJoint> children = new List<CJoint>();
		public CJoint parent = null;
		public string relativePath;
		public int indexInJointSequence;
		public Quaternion rotation;
		public Vector3 position;
        public Transform transform;

        public JointDesc findJointDesc(JointDesc[] descs, string name){
            JointDesc r = null;
            foreach(JointDesc j in descs){
                if (j.name == name){
                    r = j;
                    break;
                }
            }
            return r;
        }

   

        public void setPose(CKeyframe frame, Dictionary<string, int> indexMap)
        {
            if (indexMap.ContainsKey(name) && transform != null)
            {
                var r = frame.rotations[indexMap[name]];
                transform.localRotation = new Quaternion(r.x, r.y, r.z, r.w);
            }
            else
            {

               if(transform !=null)
                    Debug.Log("index map does not contain " + name);
                //foreach(var k in indexMap.Keys)Debug.Log(k);
            }
            foreach (CJoint c in children)
            {
                c.setPose(frame, indexMap);
            }

        }
        /// <summary>
        ///  Depth first search for transform with target name
        /// </summary>
        /// <param name="parent"></param>
        /// <param name="targetName"></param>
        /// <returns></returns>
        public Transform targetSearchDF(Transform parent, string targetName, int level = 0, bool debug = false)
        {

            for (int i = 0; i < parent.childCount; i++)
            {
                Transform child_t = parent.GetChild(i);

                if (child_t.name == targetName)
                {
                    transform = child_t;
                    return transform;
                }
                else
                {
                    if (debug) Debug.Log("Go down one level " + child_t.name + " " + level);
                    Transform temp = targetSearchDF(child_t, targetName, level++, debug);
                    if (temp != null)
                    {
                        return temp;
                    }
                }
            }
            return null;
        }


        /// <summary>
        /// Assigns recursively a joint struct to a transform of the skeleton in the scene based on a mapping defined in a JointDesc instance.
        /// </summary>
        /// <param name="jointDesc">Contains name of the joint in the source and target skeleton</param>
        /// <param name="skeletonDesc">Has joint sequence property that is needed to look up the JointDesc structures of the child joints.</param>
        /// <param name="parent">"Root transform of the skeleton in the scene"</param>
        public void assignJointTransformFromDesc(JointDesc jointDesc, SkeletonDesc skeletonDesc, Transform parent)
        {

            name = jointDesc.name;
            if (jointDesc.targetName != "none")
            {
                transform = targetSearchDF(parent, jointDesc.targetName, 0, false);
                if (transform != null)
                {
                    Debug.Log("Assigned " + name + " to " + transform.name);

                }
                else
                {
                    Debug.Log("Could not assign " + name);
                }
            }
            else // skip joints without target
            {
                Debug.Log("Ignore " + name);
            }


            foreach (string name in jointDesc.children)
            {
                JointDesc childDesc = findJointDesc(skeletonDesc.jointDescs, name);
                if(childDesc == null)continue;

                CJoint childJoint = new CJoint();
                childJoint.assignJointTransformFromDesc(childDesc, skeletonDesc, parent);
                children.Add(childJoint);
            }
        }

        public GameObject createGameObjectfromDesc(JointDesc jointDesc, SkeletonDesc skeletonDesc, GameObject parent, float scale = 0.1f, Material material = null)
        {
            name = jointDesc.name;
            float length = 1;
            if (jointDesc.children.Length > 0)
            {
                var cName = jointDesc.children[0];
                var cIdx = Array.IndexOf(skeletonDesc.jointSequence, cName);
                var t = skeletonDesc.referencePose.translations[cIdx];
                Debug.Log(name + t.magnitude.ToString());
                length = t.magnitude*5;
            }

            if (name.StartsWith("Bip"))
            {
                scale = 0.02f;
            }
            GameObject jointObj = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            Mesh mesh = jointObj.GetComponent<MeshFilter>().mesh;
            List<Vector3> newVertices = new List<Vector3>();
            var r = Quaternion.Euler(0, 0, 90);
            for (int i = 0; i < mesh.vertices.Length; i++)
            {
                var v = mesh.vertices[i];
                var p = new Vector3(v.x * scale, length * scale * v.y, v.z * scale);
                if(length != 1) p.y += (length * scale)* 3/4;
                p = r * p;
                newVertices.Add(p);
            }
            jointObj.GetComponent<MeshFilter>().mesh.vertices = newVertices.ToArray();
            jointObj.GetComponent<MeshFilter>().mesh.RecalculateNormals();
            if (material != null) jointObj.GetComponent<MeshRenderer>().material = material;

            //GameObject b = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            //b.transform.parent = parent.transform;
           // b.transform.localScale = new Vector3(scale, scale, scale);
            jointObj.transform.parent = parent.transform;
            jointObj.name = name;
          
            if (jointObj != null)
            {
                transform = jointObj.transform;

                foreach (string name in jointDesc.children)
                {
              
                    JointDesc childDesc = findJointDesc(skeletonDesc.jointDescs, name);
                    if(childDesc == null)continue;

                    CJoint childJoint = new CJoint();
                    childJoint.createGameObjectfromDesc(childDesc, skeletonDesc, jointObj, scale, material);
                    children.Add(childJoint);
                }
            }
            return jointObj;
        }


        public void getJointAngles(List<Vector3> angles)
        {
            Vector3 localEulerAngles = transform.localEulerAngles;
            angles.Add(localEulerAngles);
            foreach (CJoint childJoint in children)
            {
                if (childJoint.transform != null)
                {
                    childJoint.getJointAngles(angles);
                }
            }
        }

        public void getJointPositions(List<Vector3> positions)
        {
            Vector3 jointPosition = transform.position;
            foreach (CJoint childJoint in children)
            {
                if (childJoint.transform != null)
                {
                    Vector3 childPosition = childJoint.transform.position;
                    positions.Add(jointPosition);
                    positions.Add(childPosition);
                    childJoint.getJointPositions(positions);
                    positions.Add(childPosition);
                    positions.Add(jointPosition);
                }
            }
        }
    }
}