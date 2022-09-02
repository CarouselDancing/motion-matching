using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using System.Diagnostics;


namespace Carousel.MotionMatching{

class MMDatabaseBinaryLoader{
    public MMSettings settings;

    public MMDatabaseBinaryLoader(MMSettings settings){
        this.settings = settings;
    }

    public MMDatabase LoadResource(string fileName){
        
        MMDatabase db;
        TextAsset asset = Resources.Load(fileName) as TextAsset;
        Stream stream = new MemoryStream(asset.bytes);
        using (var reader = new BinaryReader(stream))
        {
            db = Load(reader);
        }
        return db;

    }

    public MMDatabase Load(string fileName)
    {
        MMDatabase db;
        var stopwatch = new Stopwatch();
        stopwatch.Start();
        UnityEngine.Debug.Log("Load" + fileName);
        using (var stream = File.Open(fileName, FileMode.Open, FileAccess.Read, FileShare.Read ))
        {
            using (var reader = new BinaryReader(stream))
            {
                db = Load(reader);

            }
        }
        stopwatch.Stop();
        var deltaTime = stopwatch.ElapsedMilliseconds;

        UnityEngine.Debug.Log("finished loading " + deltaTime.ToString());
        return db;
    }

    public MMDatabase Load(BinaryReader reader)
    {
        var db = new MMDatabase(settings);
        db.bonePositions = LoadPositions(reader);
        db.nFrames = db.bonePositions.GetLength(0);
        db.nBones = db.bonePositions.GetLength(1);
        db.boneVelocities = LoadPositions(reader);
        db.boneRotations = LoadRotations(reader);
        db.boneAngularVelocities = LoadPositions(reader);
        db.boneParents = LoadArray1D(reader);
        db.rangeStart = LoadArray1D(reader);
        db.rangeStop = LoadArray1D(reader);
        db.contactStates = LoadArray2D(reader);

        db.boneIndexMap = new Dictionary<HumanBodyBones, int>();
        db.boneMap = new Dictionary<HumanBodyBones, string>();
        if (settings.version == MMDatabaseVersion.DANCE)
        {
            LoadBoneData(reader, ref db);
        }
        else
        {
            foreach (var bone in db.boneMap.Keys)
            {
                var boneName = db.boneMap[bone];
                db.boneIndexMap[bone] = db.boneNames.IndexOf(boneName);
            }
        }
        foreach (var f in db.settings.features)
        {
            f.boneIdx = db.boneNames.IndexOf(db.boneMap[f.bone]);
        }

        if(settings.databaseType == MMDatabaseType.Music){
            db.phaseData = LoadPhaseData(reader);
        }
        else if(settings.databaseType == MMDatabaseType.Annotation){
            db.phaseData = LoadPhaseData(reader);
            LoadAnnotationData(reader, ref db);
        }

        return db;
    }


    
    bool[,] LoadArray2D(BinaryReader reader)
    {
        int nFrames = reader.ReadInt32();
        int nFeatures = reader.ReadInt32();
        bool[,] array = new bool[nFrames, nFeatures];
        for (int i = 0; i < nFrames; i++)
        {
            for (int j = 0; j < nFeatures; j++)
            {
                array[i, j] = reader.ReadBoolean();//8 bit
            }
        }
        return array;
    }

    Vector3[,] LoadPositions(BinaryReader reader)
    {
        int nFrames = reader.ReadInt32();
        int nBones = reader.ReadInt32();

        UnityEngine.Debug.Log("Load" + nFrames.ToString() +" " + nBones.ToString());
        Vector3[,] array = new Vector3[nFrames, nBones];
        for (int i = 0; i < nFrames; i++)
        {
            for (int j = 0; j < nBones; j++)
            {
                array[i, j].x = -reader.ReadSingle();
                array[i, j].y = reader.ReadSingle();
                array[i, j].z = reader.ReadSingle();
            }
        }
        return array;
    }

    Quaternion[,] LoadRotations(BinaryReader reader)
    {
        int nFrames = reader.ReadInt32();
        int nBones = reader.ReadInt32();

        UnityEngine.Debug.Log("Load" + nFrames.ToString() + " " + nBones.ToString());
        Quaternion[,] boneRotations = new Quaternion[nFrames,nBones];
        for (int i = 0; i < nFrames; i++)
        {
            for (int j = 0; j < nBones; j++)
            {
                boneRotations[i, j].w = -reader.ReadSingle();
                boneRotations[i, j].x = -reader.ReadSingle();
                boneRotations[i, j].y = reader.ReadSingle();
                boneRotations[i, j].z = reader.ReadSingle();
            }
        }
        return boneRotations;
    }

    float[] LoadPhaseData(BinaryReader reader)
    {
        int nFrames = reader.ReadInt32();
        int nPhaseDims = reader.ReadInt32();
        UnityEngine.Debug.Log("Load" + nFrames.ToString());
        float[] floatArray = new float[nFrames];
        for (int i = 0; i < nFrames; i++){
            floatArray[i] = reader.ReadSingle();
        }
        return floatArray;
    }

    void LoadAnnotationData(BinaryReader reader, ref MMDatabase db){
        
        db.annotationKeys = LoadNames(reader);
        db.annotationValues = LoadNames(reader);
        int nFrames = reader.ReadInt32();
        db.nAnnotations = reader.ReadInt32();
        db.annotationMatrix = new int[db.nFrames, db.nAnnotations];
        for (int i = 0; i < db.nFrames; i++)
        {
            for (int j = 0; j < db.nAnnotations; j++)
            {
                db.annotationMatrix[i, j] = reader.ReadInt32();
            }
        }
    }

    int[] LoadArray1D(BinaryReader reader)
    {
        int nBones = reader.ReadInt32();
        int[] array = new int[ nBones];
        for (int i = 0; i <  nBones; i++)
        {
            array[i] = reader.ReadInt32();
            
        }
        return array;
    }
    
    List<string> LoadNames(BinaryReader reader)
    {
        int strLength = reader.ReadInt32();
        var byteArray = reader.ReadBytes(strLength);
        string nameStr = System.Text.Encoding.UTF8.GetString(byteArray);
        return new List<string>(nameStr.Split(','));
    }


    public void LoadBoneData(BinaryReader reader, ref MMDatabase db){
        db.boneNames = LoadNames(reader);
        int[] boneMapArray = LoadArray1D(reader);
        db.boneArray = new List<HumanBodyBones>();
        for (int i = 0; i < db.boneNames.Count; i++)
        {
            var bone = (HumanBodyBones)boneMapArray[i];
            db.boneArray.Add(bone);
            var name = db.boneNames[i];
            db.boneMap[bone] = name;
            db.boneIndexMap[bone] = i;
        }
    }


}

}