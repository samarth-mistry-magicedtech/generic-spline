using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;

[ExecuteAlways]
[RequireComponent(typeof(SplineContainer))]
[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshRenderer))]
public class GenericCable : MonoBehaviour
{
   [Header("Cable Endpoints")]
   [SerializeField] private Transform startPoint;
   [SerializeField] private Transform endPoint;

   [SerializeField] private bool useChildEndpoint = false;
   [SerializeField] private string childName = "Tip";

   [Header("Cable Sag / Dip")]
   [Range(0f, 1f)] public float sagStrength = 0.3f;
   public AnimationCurve sagProfile = AnimationCurve.EaseInOut(0, 0, 1, 1);
   public Vector3 dipAxis = Vector3.down;
   public Transform dipReference;
   public bool useReferenceDipAxis = false;

   [Header("Obstacle Avoidance")]
   public bool useObstacleAvoidance = true;
   public LayerMask obstacleLayers = ~0;
   public bool ignoreCableLayer = true;
   [Range(0.01f, 0.3f)] public float clearanceRadius = 0.03f;

   [Header("Sampling Quality")]
   [Min(8)] public int segments = 64;

   [Header("Cable Mesh")]
   [Min(0.001f)] public float cableRadius = 0.005f;
   [Range(3, 32)] public int radialSegments = 8;
   public Material cableMaterial;

   [Header("Auto-Update Rules")]
   public bool updateInEditor = true;
   public bool updateInPlayMode = true;

   private SplineContainer container;
   private Mesh cableMesh;
   private MeshFilter meshFilter;
   private MeshRenderer meshRenderer;

   private Vector3 lastStart, lastEnd;
   private bool cached = false;

   void Awake()
   {
       Init();
       UpdateCable();
   }

   void Reset()
   {
       Init();
       UpdateCable();
   }

   void Init()
   {
       container = GetComponent<SplineContainer>();
       meshFilter = GetComponent<MeshFilter>();
       meshRenderer = GetComponent<MeshRenderer>();

       SetupMesh();
       AssignCableLayer();
   }

   void SetupMesh()
   {
       if (meshFilter.sharedMesh == null)
       {
           cableMesh = new Mesh();
           cableMesh.name = "Cable Mesh";
           meshFilter.sharedMesh = cableMesh;
       }
       else
       {
           cableMesh = meshFilter.sharedMesh;
       }

       if (meshRenderer != null && cableMaterial != null)
           meshRenderer.sharedMaterial = cableMaterial;
   }

   void AssignCableLayer()
   {
       int cableLayer = LayerMask.NameToLayer("Cable");
       if (cableLayer == -1 && ignoreCableLayer)
       {
           Debug.LogWarning("[GenericCable] No 'Cable' layer found. Consider creating one to avoid self-detection.");
       }
       else if (ignoreCableLayer)
       {
           gameObject.layer = cableLayer;
       }
   }

#if UNITY_EDITOR
   void OnValidate()
   {
       if (!Application.isPlaying && updateInEditor)
           UpdateCable();
   }

   void Update()
   {
       if (!Application.isPlaying && updateInEditor)
           UpdateCable();
   }
#endif

   void LateUpdate()
   {
       if (!Application.isPlaying || !updateInPlayMode) return;
       if (startPoint == null || endPoint == null) return;

       if (!cached ||
           (lastStart - startPoint.position).sqrMagnitude > 1e-6f ||
           (lastEnd - endPoint.position).sqrMagnitude > 1e-6f)
       {
           UpdateCable();
           lastStart = startPoint.position;
           lastEnd = endPoint.position;
           cached = true;
       }
   }

   Transform ResolveEndpoint(Transform t)
   {
       if (!useChildEndpoint || t == null) return t;

       Transform child = t.Find(childName);
       return child != null ? child : t;
   }

   public void UpdateCable()
   {
       if (startPoint == null)
       {
           Debug.LogWarning("[GenericCable] Missing startPoint.");
           return;
       }
       if (endPoint == null)
       {
           Debug.LogWarning("[GenericCable] Missing endPoint.");
           return;
       }

       Transform A = ResolveEndpoint(startPoint);
       Transform B = ResolveEndpoint(endPoint);

       Vector3 p0 = A.position;
       Vector3 p3 = B.position;

       float distance = Vector3.Distance(p0, p3);
       if (distance < 0.001f) return;

       Vector3 dipDir = useReferenceDipAxis && dipReference
           ? dipReference.TransformDirection(dipAxis)
           : dipAxis;

       dipDir.Normalize();

       Vector3 mid = (p0 + p3) * 0.5f - dipDir * (sagStrength * distance);

       if (useObstacleAvoidance)
           ApplySimpleAvoidance(p0, p3, ref mid);

       GenerateSpline(p0, mid, p3);
       SampleAndBuildMesh();
   }

   void ApplySimpleAvoidance(Vector3 a, Vector3 b, ref Vector3 mid)
   {
       int mask = FinalMask();

       if (Physics.Linecast(a, b, out RaycastHit hit, mask, QueryTriggerInteraction.Ignore))
       {
           mid += hit.normal * clearanceRadius * 2f;
       }
   }

   int FinalMask()
   {
       if (!ignoreCableLayer) return obstacleLayers;

       int cableLayer = LayerMask.NameToLayer("Cable");
       if (cableLayer < 0) return obstacleLayers;

       return obstacleLayers & ~(1 << cableLayer);
   }

   void GenerateSpline(Vector3 a, Vector3 m, Vector3 b)
   {
       if (container.Splines.Count == 0)
           container.AddSpline();

       Spline s = container.Splines[0];
       s.Clear();

       s.Add(new BezierKnot((float3)container.transform.InverseTransformPoint(a)));
       s.Add(new BezierKnot((float3)container.transform.InverseTransformPoint(m)));
       s.Add(new BezierKnot((float3)container.transform.InverseTransformPoint(b)));

       s.SetTangentMode(TangentMode.AutoSmooth);
   }

   void SampleAndBuildMesh()
   {
       int count = segments + 1;
       Vector3[] samples = new Vector3[count];

       Spline s = container.Splines[0];

       for (int i = 0; i < count; i++)
       {
           float t = i / (float)(count - 1);
           float3 pos = SplineUtility.EvaluatePosition(s, t);
           samples[i] = container.transform.TransformPoint((Vector3)pos);

           if (sagProfile != null)
               samples[i] -= dipAxis.normalized * sagProfile.Evaluate(t) * (sagStrength * 0.1f);
       }

       BuildMesh(samples);
   }

   void BuildMesh(Vector3[] pts)
   {
       if (pts.Length < 2) return;

       int rings = pts.Length;
       int ringVerts = radialSegments;

       int vertexCount = rings * ringVerts;
       int triangleCount = (rings - 1) * ringVerts * 2;

       Vector3[] verts = new Vector3[vertexCount];
       Vector3[] norms = new Vector3[vertexCount];
       Vector2[] uvs = new Vector2[vertexCount];
       int[] tris = new int[triangleCount * 3];

       for (int i = 0; i < rings; i++)
       {
           Vector3 p = transform.InverseTransformPoint(pts[i]);
           Vector3 tangent =
               (i == rings - 1 ? pts[i] - pts[i - 1] : pts[i + 1] - pts[i]).normalized;

           Vector3 n = Vector3.up;
           if (Mathf.Abs(Vector3.Dot(n, tangent)) > 0.9f) n = Vector3.right;
           Vector3 b = Vector3.Cross(tangent, n).normalized;
           n = Vector3.Cross(b, tangent).normalized;

           float v = i / (float)(rings - 1);

           for (int j = 0; j < ringVerts; j++)
           {
               float ang = (j / (float)ringVerts) * Mathf.PI * 2f;
               float u = j / (float)ringVerts;

               Vector3 off = Mathf.Cos(ang) * n + Mathf.Sin(ang) * b;

               int id = i * ringVerts + j;
               verts[id] = p + off * cableRadius;
               norms[id] = off.normalized;
               uvs[id] = new Vector2(u, v);
           }
       }

       int idx = 0;
       for (int i = 0; i < rings - 1; i++)
       {
           for (int j = 0; j < ringVerts; j++)
           {
               int a = i * ringVerts + j;
               int b = i * ringVerts + (j + 1) % ringVerts;
               int c = (i + 1) * ringVerts + j;
               int d = (i + 1) * ringVerts + (j + 1) % ringVerts;

               tris[idx++] = a; tris[idx++] = c; tris[idx++] = d;
               tris[idx++] = a; tris[idx++] = d; tris[idx++] = b;
           }
       }

       cableMesh.Clear();
       cableMesh.SetVertices(verts);
       cableMesh.SetNormals(norms);
       cableMesh.SetUVs(0, uvs);
       cableMesh.SetTriangles(tris, 0);
       cableMesh.RecalculateBounds();
   }

   void OnDrawGizmos()
   {
       if (container == null || container.Splines.Count == 0) return;

       Gizmos.color = Color.yellow;
       Spline s = container.Splines[0];

       Vector3 prev = SplineUtility.EvaluatePosition(s, 0);
       prev = container.transform.TransformPoint(prev);

       for (int i = 1; i <= 32; i++)
       {
           float t = i / 32f;
           Vector3 pos = SplineUtility.EvaluatePosition(s, t);
           pos = container.transform.TransformPoint(pos);
           Gizmos.DrawLine(prev, pos);
           prev = pos;
       }
   }
}