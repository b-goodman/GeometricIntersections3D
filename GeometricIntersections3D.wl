(* ::Package:: *)

BeginPackage["GeometricIntersections3D`"];

loadExampleData::usage = "loadExampleData[] returns exemplar data from git repository";

intersectTriangleBox::usage = "intersectTriangleBox[boxBounds, triangleVertices] returns True if a triangle with vertices v_ix,v_iy,v_iz intersects with cuboid with points {{x_min,y_min,z_min},{x_max,y_max,z_max}}.";

intersectRayBox::usage = "intersectRayBox[boxBounds,rayOrigin,source] returns True if specified ray intersects with cuboid given by {{min_x,min_y,min_z},{max_x,max_y,max_z}}.";

intersectRayTriangle::usage = "intersectRayTriangle=[vertex0,vertex1,vertex2,rayPoint0,rayPoint1]";

intersectBoxBox::usage = "intersectBoxBox[box1,box2] returns True if specified Cuboids intersect.";

PrimitiveIntersectionQ3D::usage = "PrimitiveIntersectionQ3D[obj1,obj2] returns True if graphical primitives obj1, obj2 intersect, False otherwise.";

cuboidSubdivide::usage = "cuboidSubdivide[cuboidBounds] subdivides the cuboid {{min_x,min_y,min_z},{max_x,max_y,max_z}} into 8 equal cuboids, returning their {min,max} boundss respectivly.";

newBVH::usage = "newBVH[cuboidPartitions,polyPoints] returns a new instance of a BVH object representing a list of polygons bound by a box.";

addLevelBVH::usage = "addLevelBVH[BVH] returns a BVH object with its bounding volumes subdivided and added to the existing BVH tree.";

finalizeBVH::usage = "finalizeBVH[BVH] returns a BVH object with its bounding volumes linked to the polygons of its representative 3D model.";

intersectionRayBVH::usage = "intersectionRayBVH[BVHObj,rayOrigin,rayDest] returns True if a ray from p0 to p1 intersects with any of the polygons in a BVH indexed 3D model.";

sceneConstructor::usage = "sceneConstructor[BVHobj] Initiates the GUI scene constructor."

newScene::usage = "newScene[BVHobj,lightingPath,frameCount,rayRefinement] returns a new instance of a scene object.";

renderScene::usage = "renderScene[sceneObj] returns a scene object with ray-tracing applied to all frames within the specified scene object";

viewSceneFrame::usage = "viewSceneFrame[sceneObj,frameIndex] returns a Graphics3D rendition of a single specified frame from a processed scene object.";

solarPositionPts::usage = "solarPositionPts[] returns the cartesian-transformed solar position from sunrise to sunset for the current time in 30 min intervals";
solarPositionPts::usage = "solarPositionPts[date] specifies a date"; 
solarPositionPts::usage = "solarPositionPts[tSpec] specifies a sample interval";
solarPositionPts::usage = "solarPositionPts[date,tSpec]";



Begin["`Private`"];

(* ----------  Example Data  ----------- *)
(* load demo material *)
loadExampleData[]:=With[{
directoryPath="https://raw.githubusercontent.com/b-goodman/GeometricIntersections3D/master/Demo"
},
With[{
modelRegion=Import[
directoryPath<>"/houseModel4.dae",
"MeshRegion"
],
polyPoints=Delete[0]@Import[
directoryPath<>"/houseModel4.dae",
"PolygonObjects"
],
bvh=Uncompress[Import[directoryPath<>"/BVH.txt"]],
scene=Uncompress[Import[directoryPath<>"/scene.txt"]]
},

Return[<|
"Model"-><|
"PolygonObjects"->polyPoints,
"Region"->modelRegion,
"CuboidPartition"->{Delete[0]@BoundingRegion[modelRegion,"MinCuboid"]}
|>,
"BVH"-> bvh,
"Scene"->scene
|>];
]
];


(* -----------  BVH -----------  *)
(* CUBOID SUBDIVISION *)
axialShiftLat[axis_,subDiv_,edgeLengths_]:=Block[{shiftSubDiv},
shiftSubDiv=Transpose[subDiv];
shiftSubDiv[[axis]]+=edgeLengths[[axis]]/2;
Return@Transpose[shiftSubDiv]
];

axialShiftDiag[subDiv_,edgeLengths_]:=Block[{shiftSubDiv},
shiftSubDiv=Transpose[subDiv];
shiftSubDiv[[1]]+=edgeLengths[[1]]/2;
shiftSubDiv[[2]]+=edgeLengths[[2]]/2;
Return[Transpose[shiftSubDiv]]
];

planeSubdivide[subDiv_,edgeLengths_]:=
Reap[
Sow[subDiv];
(* shift x *)
Sow@axialShiftLat[1,subDiv,edgeLengths];
(* shift y *)
Sow@axialShiftLat[2,subDiv,edgeLengths];
(* shift x,y *)
Sow@axialShiftDiag[subDiv,edgeLengths];
]//Last//Flatten[#,1]&;

subDivide[cuboidBounds_]:=Flatten[Last[Reap[
Block[{edgeLengths,cuboidOrigin,subDiv},
(* paramaters *)

edgeLengths=Flatten[Differences/@Transpose[cuboidBounds]];
cuboidOrigin=cuboidBounds[[1]];
(* initial *)
subDiv=Transpose@Table[{cuboidOrigin[[i]],cuboidOrigin[[i]]+(edgeLengths[[i]]/2)},{i,1,3}];
Sow@planeSubdivide[subDiv,edgeLengths];
(* shift z *)
subDiv[[1,3]]+=edgeLengths[[3]]/2;
subDiv[[2,3]]+=edgeLengths[[3]]/2;
(* repeat *)
Sow@planeSubdivide[subDiv,edgeLengths];
]
]],2];

cuboidSubdivide[cuboidBounds_]:=If[TensorRank[cuboidBounds]==2,subDivide[cuboidBounds],Flatten[(subDivide/@cuboidBounds),1]];

cullIntersectingPartitions=Compile[{
{cuboidPartitions,_Real,3},
{polyPoints,_Real,3}
},
Select[cuboidPartitions,Function[{partitions},MemberQ[ParallelMap[Quiet@intersectTriangleBox[partitions,#]&,polyPoints],True]]],
CompilationTarget->"C"
];

(* BVH Generation *)
(* 1.  Begin tree.  Initial AABB is root.  Subdivide root AABB and link returns to root *) 
newBVH[cuboidPartitions_,polyPoints_]:=Block[{
newLevel,edges
},
newLevel=Quiet[cullIntersectingPartitions[cuboidSubdivide[cuboidPartitions],polyPoints]];
edges=cuboidPartitions\[DirectedEdge]#&/@newLevel;
Return[<|
"Tree"->TreeGraph[edges],
"PolygonObjects"->polyPoints
|>];
];

(* 2. Each new subdivision acts as root.  For each, subdivide further and remove any non-intersecting boxes.  Link back to parent box as directed edge *)
addLevelBVH[BVH_]:=Block[{
tree=BVH["Tree"],polyPoints=BVH["PolygonObjects"],returnEdges
},
Module[{
subEdges=Map[
Function[{levelComponent},levelComponent\[DirectedEdge]#&/@Quiet@cullIntersectingPartitions[cuboidSubdivide[levelComponent],polyPoints]],
Pick[VertexList[tree],VertexOutDegree[tree],0]]
},
returnEdges=ConstantArray[0,Length[subEdges]];
Do[returnEdges[[i]]=EdgeAdd[tree,subEdges[[i]]],{i,1,Length[subEdges],1}];
];
returnEdges=DeleteDuplicates[Flatten[Join[EdgeList/@returnEdges]]];
Return[<|
"Tree"->TreeGraph[returnEdges],
"PolygonObjects"->polyPoints
|>]
];

(* 3. For each outermost subdivision (leaf box), find intersecting polygons.  Link to intersecting box via directed edge.  Append to graph *)
finalizeBVH[BVH_]:=Block[{
(* all leaf boxes for BVH *)
leafBoxes=Select[
VertexList[BVH["Tree"]],
VertexOutDegree[BVH["Tree"],#]==0&
],
(* setup temp association *)
temp=<||>,
(* block varaibles *)
leafPolygons,
leafPolygonsEdges
},
(* For each BVH leaf box *)
Do[
(* 3.1. intersecitng polygons for specified BVH leaf box *)
leafPolygons=Select[
BVH["PolygonObjects"],
Quiet@intersectTriangleBox[leafBoxes[[i]],#]==True&
];
(* 3.2. associate each specified BVH leaf box to its intersecting polygon(s) *)
AppendTo[temp,leafBoxes[[i]]->leafPolygons],
{i,1,Length[leafBoxes],1}
];
Return[<|
"Tree"->BVH["Tree"],
"LeafObjects"->temp,
"PolygonObjects"->BVH["PolygonObjects"]
|>]
];

(* Search BVH for intersecting boxes *)

(* select peripheral out-components of root box that intersect with ray *)
intersectingSubBoxes[BVHObj_,initialVertex_,rayOrigin_,raySource_]:=Select[Rest[VertexOutComponent[BVHObj["Tree"],{initialVertex},1]],intersectRayBox[#,rayOrigin,raySource]==True&];

(* for root box intersecting rays, find which leaf box(es) intersect with ray *)
BVHLeafBoxIntersection[BVHObj_,rayInt_,rayDest_]:=Block[{v0},
(*initialize search *)v0=intersectingSubBoxes[BVHObj,VertexList[BVHObj["Tree"]][[1]],rayInt,rayDest];
(* breadth search *)
If[v0=={},Return[v0],
While[
(* check that vertex isn't a polygon - true if !0.  Check that intersection isn't empty *)
AllTrue[VertexOutDegree[BVHObj["Tree"],#]&/@v0,#=!=0&],
v0=Flatten[intersectingSubBoxes[BVHObj,#,rayInt,rayDest]&/@v0,1];
If[v0==={},Break[]]
];
Return[v0];
]
];

(* test intersection between ray and object polygon via BVH search *)
intersectionRayBVH[BVHObj_,rayOrigin_,rayDest_]:=With[{
intersectionLeafBoxes=BVHLeafBoxIntersection[BVHObj,rayOrigin,rayDest]
},
Block[{i},If[intersectionLeafBoxes=!={},
Return[Catch[For[i=1,i<Length[#],i++,
Function[{thowQ},If[thowQ,Throw[thowQ]]][intersectRayTriangle[#[[1]],#[[2]],#[[3]],rayOrigin,rayDest]&@#[[i]]]
]&[DeleteDuplicates[Flatten[Lookup[BVHObj["LeafObjects"],intersectionLeafBoxes],1]]]]===True],
Return[False]
]]
];

(* ---------- RAY TRACER ------------ *)
(* generate scene obj and render lighting for all source positions *)
(*newScene[BVHobj_,lightingPath_,frameCount_,rayRefinement_]:=With[{
sourecPositions=lightingPath/@Range[0,1,N[1/frameCount]],
projectionPlane=Catenate[Table[{i,j,0},{i,-400,900,rayRefinement},{j,-400,900,rayRefinement}]]
},
Return[<|
"BVH"->BVHobj,
"SourcePositions"->sourecPositions,
"FrameCount"->frameCount,
"Refinement"->rayRefinement,
"ProjectionPoints"->projectionPlane,
"FrameData"-><||>
|>]
];*)

solarPositionPts0[date_:DateValue[Now,{"Year","Month","Day"}],tSpec_:{30,"Minute"}]:=
Evaluate[CoordinateTransformData["Spherical"->"Cartesian","Mapping",{1,\[Pi]/2-(#2 Degree),2Pi-(#1 Degree)}]]&@@@(Function[{series},Map[QuantityMagnitude,series["Values"],{2}]]@SunPosition[DateRange[Sunrise[#],Sunset[#],tSpec]&[DateObject[date]]]);

solarPositionPts[]:=solarPositionPts0[DateValue[Now,{"Year","Month","Day"}],{30,"Minute"}];
solarPositionPts[date_]:=solarPositionPts0[date,{30,"Minute"}]/;Length[date]==3;
solarPositionPts[tSpec_]:=solarPositionPts0[DateValue[Now,{"Year","Month","Day"}],tSpec]/;Length[tSpec]==2;
solarPositionPts[date_,tSpec_]:=solarPositionPts0[date,tSpec];

(* scene constructors *)
newSceneDiscretePath[BVHobj_,lightingPath_,rayRefinement_,planeSpec_]:=With[{
frameCount=Length[lightingPath]
},
Return[<|
"BVH"->BVHobj,
"SourcePositions"->lightingPath,
"FrameCount"->frameCount,
"Refinement"->rayRefinement,
"ProjectionPoints"->planeSpec,
"FrameData"-><||>
|>]
];

newSceneCtsPath[BVHobj_,lightingPath_,frameCount_,rayRefinement_,planeSpec_]:=With[{
sourecPositions=BSplineFunction[Delete[0]@lightingPath]/@Range[0,1,N[1/frameCount]],
projectionPlane=planeSpec
},
Return[<|
"BVH"->BVHobj,
"SourcePositions"->sourecPositions,
"FrameCount"->frameCount,
"Refinement"->rayRefinement,
"ProjectionPoints"->projectionPlane,
"FrameData"-><||>
|>]
];

(* *******Update public export******** *)
newScene[BVHobj_,lightingPath_:BSplineFunction,frameCount_:Integer,rayRefinement_:Integer,planeSpec_:List]:=newSceneCtsPath[BVHobj,lightingPath,frameCount,rayRefinement,planeSpec]/;Head[lightingPath]==BSplineCurve;
newScene[BVHobj_,lightingPath_:List,rayRefinement_:Integer,planeSpec_:List]:=newSceneDiscretePath[BVHobj,lightingPath,rayRefinement,planeSpec]/;Head[lightingPath]==List;


(* GUI Scene Constructor *)
sceneConstructor[BVHobj_]:=DynamicModule[{
(* spec. Defaults *)
customLightPathQ=False,
customLightPathSpec=HoldForm[BSplineCurve[solarPositionPts[]*1000]],
planeSpecX=0,
planeSpecY=0,
planeSpecH=0,
rayRefinementSpec=50,
frameCountSpec=20
},
(* config. Projection Plane *)
projectionPlane[x_,y_,height_]:=Block[{
rootBB=First[VertexList[BVHobj["Tree"]]]
},
rootBB[[1,3]]=height;
rootBB[[2,3]]=height;
rootBB+={-{x,y,0},{x,y,0}};
Return[rootBB]
];
previewProjectionPlane[x_,y_,height_]:=Module[{
projPlanePreview=projectionPlane[x,y,height],
model=Polygon[BVHobj["PolygonObjects"]]
},
Row[{
Graphics3D[{
model,
Cuboid[projPlanePreview]
},ImageSize->Medium],
Deploy@Graphics3D[{
model,
Cuboid[projPlanePreview]
},ViewPoint->Above,ImageSize->Medium,Boxed->False]
}]
];
(* config. Lighting *)
defaultLightingPath[x_,y_,h_]:=Module[{
defaultHeigth=First[VertexList[BVHobj["Tree"]]][[2,3]]*2,
midHeight=(First[VertexList[BVHobj["Tree"]]][[2,3]]*2)+((First[VertexList[BVHobj["Tree"]]][[2,3]]*2)*0.1),
p0=First[projectionPlane[x,y,h]][[1;;2]],
p1=Last[projectionPlane[x,y,h]][[1;;2]]
},
BSplineCurve[{
Flatten[Join[{p0,defaultHeigth}]],
Flatten[Join[{RegionCentroid[Line[{p0,p1}]],defaultHeigth+midHeight}]],
Flatten[Join[{p1,defaultHeigth}]]
}]
];
previewLightingPath0[planeSpecX_,planeSpecY_,planeSpecH_,frameCountSpec_]:=Graphics3D[{
defaultLightingPath[planeSpecX,planeSpecY,planeSpecH],
Point[(BSplineFunction[Delete[0]@defaultLightingPath[planeSpecX,planeSpecY,planeSpecH]]/@Range[0,1,N[1/frameCountSpec]])],
Polygon[BVHobj["PolygonObjects"]]
},ImageSize->Medium];
previewLightingPathCustomSpline[lightingPathSpline_:BSplineCurve,frameCountSpec_:Integer]:=Graphics3D[{
lightingPathSpline,
Polygon[BVHobj["PolygonObjects"]],
Point[(BSplineFunction[Delete[0]@ReleaseHold[lightingPathSpline]]/@Range[0,1,N[1/frameCountSpec]])]
},ImageSize->Medium];
previewLightingPathCustomPts[lightingPathPts_:List]:=Graphics3D[{
Point[lightingPathPts],
Polygon[BVHobj["PolygonObjects"]]
},ImageSize->Medium];
previewLightingPath[frameCountSpec_:Integer]:=previewLightingPath0[planeSpecX,planeSpecY,planeSpecH,frameCountSpec];
previewLightingPath[lightingPathSpline_:BSplineCurve,frameCountSpec_:Integer]:=previewLightingPathCustomSpline[lightingPathSpline,frameCountSpec]/;Head[lightingPathSpline]==BSplineCurve;
previewLightingPath[lightingPathPts_:List,frameCountSpec_:Integer]:=previewLightingPathCustomPts[lightingPathPts]/;Head[lightingPathPts]==List;
(* scene constructor *)
lightingPath[]:=ReleaseHold[customLightPathSpec]/;customLightPathQ;
lightingPath[]:=defaultLightingPath[scene,planeSpecX,planeSpecY,planeSpecH]/;customLightPathQ==False;
generateProjectionPlane[planeSpecX_,planeSpecY_,planeSpecH_,rayRefinementSpec_]:=With[{
xBounds=First/@(Most/@projectionPlane[planeSpecX,planeSpecY,planeSpecH]),
yBounds=Last/@(Most/@projectionPlane[planeSpecX,planeSpecY,planeSpecH])
},
Catenate[Table[{x,y,planeSpecH},{x,xBounds[[1]],xBounds[[2]],rayRefinementSpec},{y,yBounds[[1]],yBounds[[2]],rayRefinementSpec}]]
];
generateScene[]:=newScene[BVHobj,lightingPath[],frameCountSpec,rayRefinementSpec,generateProjectionPlane[scene,planeSpecX,planeSpecY,planeSpecH,rayRefinementSpec]]/;Head[lightingPath[]]==BSplineCurve;
generateScene[]:=newScene[BVHobj,lightingPath[],rayRefinementSpec,generateProjectionPlane[scene,planeSpecX,planeSpecY,planeSpecH,rayRefinementSpec]]/;Head[lightingPath[]]==List
(* Create and show GUI *)
CreateDialog[
Column[{
TabView[{
"Shadow Plane"->Manipulate[
previewProjectionPlane[scene,planeSpecX,planeSpecY,planeSpecH],
{{planeSpecX,0,"X"},0,500,1},
{{planeSpecY,0,"Y"},0,500,1},
{{planeSpecH,0,"Height"},-500,500,1},
LocalizeVariables->False
],
"Light Source"->Column[{
Column[{
Row[{TextCell["Specify Light Path"],Checkbox[Dynamic[customLightPathQ],{False,True}],InputField[Dynamic[customLightPathSpec],FieldSize->{25, 1},Enabled->Dynamic[customLightPathQ]]},Spacer[5]],
Row[{
Row[{TextCell["Ray Refinement: "],InputField[Dynamic[rayRefinementSpec],Number,FieldSize->{3,1}],Spacer[10]}],
Row[{TextCell["Frames: "],InputField[Dynamic[frameCountSpec],Number,Enabled->Dynamic[(Head[ReleaseHold[customLightPathSpec]]=!=List)||!customLightPathQ],FieldSize->{3,1}],Spacer[10]}]
},Spacer[10]]
}],
Dynamic@If[
customLightPathQ,
previewLightingPath[scene,ReleaseHold@customLightPathSpec,frameCountSpec],
previewLightingPath[scene,frameCountSpec]
]
}]
}],
Button["Generate Scene",CreateDocument[ExpressionCell[generateScene[],"Input"]],ImageSize->{130,30}]
}]
]
];


(* BVH Accelerated Shadow Mapper *)
renderShadowPts[sceneObj_,i_]:=With[{
pts=Select[
sceneObj["ProjectionPoints"],
Quiet@intersectionRayBVH[sceneObj["BVH"],#,sceneObj["SourcePositions"][[i]]]==True&
]
},
Return[({##,##+{sceneObj["Refinement"],sceneObj["Refinement"],0}}&/@pts)]
];

getGroundPts[sceneObj_,shadowTiles_]:=Return[
Complement[{##,##+{sceneObj["Refinement"],sceneObj["Refinement"],0}}&/@sceneObj["ProjectionPoints"],shadowTiles]
];

compileFrameData[frameIndex_,shadowTiles_,sourcePosition_,groundPts_]:=Return[
frameIndex-><|
"ShadowPts"->shadowTiles,
"SourcePosition"-> sourcePosition,
"GroundPts"-> groundPts
|>
];

renderSceneFrame[sceneObj_,frameIndex_]:=With[{
shadowPts=renderShadowPts[sceneObj,frameIndex]
},
With[{
groundPts=getGroundPts[sceneObj,shadowPts]
},
Return[
compileFrameData[frameIndex,shadowPts,sceneObj["SourcePositions"][[frameIndex]],groundPts]
];
]
];


(* Shadow mapped scene constructor *)
renderScene[sceneObj_]:=Module[{
sceneTemp=sceneObj
},
Block[{temp=<||>},
Monitor[
Do[
AppendTo[temp,renderSceneFrame[sceneObj,frameIndex]],
{frameIndex,1,sceneObj["FrameCount"],1}
],
Row[{ProgressIndicator[Dynamic[frameIndex],{1,sceneObj["FrameCount"]}]," ",frameIndex,"/",sceneObj["FrameCount"]}]
];
sceneTemp["FrameData"]=temp
];
Return[sceneTemp];
];


(* Scene output *)
(* Single Frames *)

(*viewSceneFrame[sceneObj_,frameIndex_]:=Graphics3D[{
{Yellow,PointSize[0.03],Point[sceneObj["FrameData"][frameIndex]["SourcePosition"]]},
Polygon[sceneObj["BVH"]["PolygonObjects"]],
{GrayLevel[.5],Cuboid/@sceneObj["FrameData"][frameIndex]["ShadowPts"]},
{Green,Cuboid/@sceneObj["FrameData"][frameIndex]["GroundPts"]}
},Lighting->{{White,sceneObj["FrameData"][frameIndex]["SourcePosition"]}}]*)


(* *******Update public export******** *)
Options[viewSceneFrame]={DrawSource->False};
viewSceneFrame[sceneObj_,frameIndex_,opts:OptionsPattern[]]:=Graphics3D[{
(* source *)
If[
OptionValue[
viewSceneFrame,
Evaluate[FilterRules[{opts}, Options[viewSceneFrame]]],
DrawSource
]==True,
{Yellow,PointSize[0.03],Point[sceneObj["FrameData"][frameIndex]["SourcePosition"]]},
{}
],
(* 3D Model *)
Polygon[sceneObj["BVH"]["PolygonObjects"]],
(* Shadow *)
{GrayLevel[.5],Cuboid/@sceneObj["FrameData"][frameIndex]["ShadowPts"]},
(* Projection surface *)
{Green,Cuboid/@sceneObj["FrameData"][frameIndex]["GroundPts"]}
},
(* Model Lighting *)
Lighting->{{White,sceneObj["FrameData"][frameIndex]["SourcePosition"]}},
Evaluate[FilterRules[{opts}, {Options[Graphics3D],Except[Options[viewSceneFrame]]}]]
]



(* -----------  INTERSECTIONS ----------- *)

(* RAY-TRIANGLE *)
intersectRayTriangle=Compile[
{
{vertex0,_Real,1},
{vertex1,_Real,1},
{vertex2,_Real,1},
{rayPoint0,_Real,1},
{rayPoint1,_Real,1}
},
(*triangle edge vectors (u,v) and plane normal (n) *)
With[{
u = vertex1-vertex0,
v=vertex2-vertex0,
epsilon = 0.00000001
},
With[{
n=Cross[u,v]
},
(* ray direction vector (dir) *)
With[{
dir = rayPoint1 - rayPoint0,
w0 = rayPoint0 - vertex0
},
With[{
a=-Dot[n,w0],
b=Dot[n,dir]
},
With[{
r=a/b
},
(*if ray parallel to triangle plane *)
If[Abs[b]<epsilon,
If[a==0,
(* ray lies in triangle plane *)
Return[False],
(* ray disjoint from plane *)
Return[False]]
];
(* get intersect point of ray with triangle plane *)
If[r < 0, 
(* no intersection *)
Return[False]
];
(* define intersection point of ray with triangle plane*)
With[{
intersection = rayPoint0 + r * dir
},
(* is r inside triangle? *)
With[{
uu = Dot[u,u],
uv = Dot[u,v],
vv=Dot[v,v],
w = intersection - vertex0
},
With[{
wu = Dot[w,u],
wv = Dot[w,v]
},
With[{
d= uv * uv - uu * vv
},
With[{
s=(uv*wv - vv * wu) /d
},
If[ s < 0 || s> 1 ,(* point is outside *)Return[False]];
With[{
t=(uv*wu - uu*wv)/d
},
If[t<0|| (s+t)>1,(* point is outside *)Return[False]]]
]
]
]
]
]
]
]
]
];
(*otherwise, point is inside triangle *)
Return[True]
],
CompilationTarget->"C"
];

(* RAY-BOX *)
ClearAll[intersectRayBox];
intersectRayBox[boxBounds_,rayOrigin_,source_]:=Block[{
boundsMin=boxBounds[[1]],
boundsMax=boxBounds[[2]],
dir=source-rayOrigin,
returnValue=False,
tMin,tMax,tyMin,tyMax,tzMin,tzMax
},
If[dir[[1]] >= 0,
tMin=(boundsMin[[1]] - rayOrigin[[1]])/dir[[1]];
tMax = (boundsMax[[1]] - rayOrigin[[1]])/dir[[1]],
(*else*)
tMin=(boundsMax[[1]] - rayOrigin[[1]])/dir[[1]];
tMax = (boundsMin[[1]] - rayOrigin[[1]])/dir[[1]]
];
If[dir[[2]]>=0,
tyMin=(boundsMin[[2]]-rayOrigin[[2]])/dir[[2]];
tyMax=(boundsMax[[2]]-rayOrigin[[2]])/dir[[2]],
(*else*)
tyMin=(boundsMax[[2]]-rayOrigin[[2]])/dir[[2]];
tyMax=(boundsMin[[2]]-rayOrigin[[2]])/dir[[2]]
];
If[(tMin > tyMax)||(tyMin >tMax),Return[returnValue]];
If[tyMin>tMin,
tMin = tyMin
];
If[tyMax < tMax,
tMax = tyMax
];
If[dir[[3]] >= 0,
tzMin = (boundsMin[[3]] - rayOrigin[[3]])/dir[[3]];
tzMax = (boundsMax[[3]] - rayOrigin[[3]])/dir[[3]],
(* else*)
tzMin = (boundsMax[[3]] - rayOrigin[[3]])/dir[[3]];
tzMax = (boundsMin[[3]] - rayOrigin[[3]])/dir[[3]]
];
If[(tMin>tzMax)||(tzMin>tMax),returnValue=False,returnValue=True];
Return[returnValue];
];


(* TRIANGLE-BOX *)
ClearAll[intersectTriangleBox0];
intersectTriangleBox0 = Compile[{
    {c, _Real, 1},
    {e, _Real, 1},
    {v, _Real, 2}
    },
   With[{
     v0 = {v[[1, 1]] - c[[1]], v[[1, 2]] - c[[2]], v[[1, 3]] - c[[3]]},
     v1 = {v[[2, 1]] - c[[1]], v[[2, 2]] - c[[2]], v[[2, 3]] - c[[3]]},
     v2 = {v[[3, 1]] - c[[1]], v[[3, 2]] - c[[2]], v[[3, 3]] - c[[3]]}
     },
    With[{
      f0 = {v1[[1]] - v0[[1]], v1[[2]] - v0[[2]], v1[[3]] - v0[[3]]},
      f1 = {v2[[1]] - v1[[1]], v2[[2]] - v1[[2]], v2[[3]] - v1[[3]]}, 
      f2 = {v0[[1]] - v2[[1]], v0[[2]] - v2[[2]], v0[[3]] - v2[[3]]}
      },
     returnValue = False;
     
     a00l = Sqrt[f0[[3]]^2 + f0[[2]]^2];
     a00y = -f0[[3]]/a00l; 
     a00z = f0[[2]]/a00l;
     a00r = e[[2]]*Abs[a00y] + e[[3]]*Abs[a00z];
      a00p0 = v0[[2]]*a00y + v0[[3]]*a00z; 
     a00p1 = v1[[2]]*a00y + v1[[3]]*a00z; 
     a00p2 = v2[[2]]*a00y + v2[[3]]*a00z; 
     a00min = Min[a00p0, a00p1, a00p2]; 
     a00max = Max[a00p0, a00p1, a00p2];
     If[a00min > a00r, Return[returnValue]];
     If[a00max < -a00r , Return [False],
      If[a00min < -a00r, 
       distance = -(a00max + a00r),
       distance = a00r - a00min
       ];
      axis = {0, a00y, a00z}
      ];
     
     a01l = Sqrt[f1[[3]]^2 + f1[[2]]^2];
     a01y = -f1[[3]]/a01l;
     a01z = f1[[2]]/a01l; a01r = e[[2]]*Abs[a01y] + e[[3]]*Abs[a01z]; 
     a01p0 = v0[[2]]*a01y + v0[[3]]*a01z; 
     a01p1 = v1[[2]]*a01y + v1[[3]]*a01z; 
     a01p2 = v2[[2]]*a01y + v2[[3]]*a01z; 
     a01min = Min[a01p0, a01p1, a01p2]; 
     a01max = Max[a01p0, a01p1, a01p2];
     If[a01min > a01r, Return[returnValue]]; 
     If[a01max < -a01r, Return[returnValue],
      If[a01min < -a01r, 
       newDistance = -(a01max + a01r),
       newDistance = a01r - a01min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {0, a01y, a01z}
       ]
      ];
     
     a02l = Sqrt[f2[[3]]^2 + f2[[2]]^2]; a02y = -f2[[3]]/a02l; 
     a02z = f2[[2]]/a02l; a02r = e[[2]]*Abs[a02y] + e[[3]]*Abs[a02z]; 
     a02p0 = v0[[2]]*a02y + v0[[3]]*a02z; 
     a02p1 = v1[[2]]*a02y + v1[[3]]*a02z; 
     a02p2 = v2[[2]]*a02y + v2[[3]]*a02z; 
     a02min = Min[a02p0, a02p1, a02p2]; 
     a02max = Max[a02p0, a02p1, a02p2];
     If[a02min > a02r, Return[returnValue]];
     If[a02max < -a02r, Return[returnValue],
      If[a02min < -a02r,
       newDistance = -(a02max + a02r),
       newDistance = a02r - a02min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {0, a02y, a02z}
       ]
      ];
     
     a10l = Sqrt[f0[[3]]^2 + f0[[1]]^2]; a10x = f0[[3]]/a10l; 
     a10z = -f0[[1]]/a10l; a10r = e[[1]]*Abs[a10x] + e[[3]]*Abs[a10z];
      a10p0 = v0[[1]]*a10x + v0[[3]]*a10z; 
     a10p1 = v1[[1]]*a10x + v1[[3]]*a10z; 
     a10p2 = v2[[1]]*a10x + v2[[3]]*a10z; 
     a10min = Min[a10p0, a10p1, a10p2]; 
     a10max = Max[a10p0, a10p1, a10p2];
     If[a10min > a10r, Return[returnValue]];
     If[a10max < -a10r, Return[returnValue],
      If[a10min < -a10r,
       newDistance = -(a10max + a10r),
       newDistance = a10r - a10min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {a10x, 0, a10z}
       ]
      ];
     
     a11l = Sqrt[f1[[3]]^2 + f1[[1]]^2]; a11x = f1[[3]]/a11l; 
     a11z = -f1[[1]]/a11l; a11r = e[[1]]*Abs[a11x] + e[[3]]*Abs[a11z];
      a11p0 = v0[[1]]*a11x + v0[[3]]*a11z; 
     a11p1 = v1[[1]]*a11x + v1[[3]]*a11z; 
     a11p2 = v2[[1]]*a11x + v2[[3]]*a11z; 
     a11min = Min[a11p0, a11p1, a11p2]; 
     a11max = Max[a11p0, a11p1, a11p2];
     
     If[a11min > a11r, Return[returnValue]]; 
     If[a11max < -a11r, Return[returnValue],
      If[a11min < -a11r,
       newDistance = -(a11max + a11r),
       newDistance = a11r - a11min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {a11x, 0, a11z}
       ]
      ];
     
     a12l = Sqrt[f2[[3]]^2 + f2[[1]]^2]; a12x = f2[[3]]/a12l; 
     a12z = -f2[[1]]/a12l; a12r = e[[1]]*Abs[a12x] + e[[3]]*Abs[a12z];
      a12p0 = v0[[1]]*a12x + v0[[3]]*a12z; 
     a12p1 = v1[[1]]*a12x + v1[[3]]*a12z; 
     a12p2 = v2[[1]]*a12x + v2[[3]]*a12z; 
     a12min = Min[a12p0, a12p1, a12p2]; 
     a12max = Max[a12p0, a12p1, a12p2];
     If[a12min > a12r, Return[returnValue]];
     If[a12max < -a12r, Return[returnValue],
      If[a12min < -a12r,
       newDistance = -(a12max + a12r),
       newDistance = a12r - a12min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {a12x, 0, a12z}
       ]
      ];
     
     a20l = Sqrt[f0[[2]]^2 + f0[[1]]^2]; a20x = -f0[[2]]/a20l; 
     a20y = f0[[1]]/a20l; a20r = e[[1]]*Abs[a20x] + e[[2]]*Abs[a20y]; 
     a20p0 = v0[[1]]*a20x + v0[[2]]*a20y; 
     a20p1 = v1[[1]]*a20x + v1[[2]]*a20y; 
     a20p2 = v2[[1]]*a20x + v2[[2]]*a20y; 
     a20min = Min[a20p0, a20p1, a20p2]; 
     a20max = Max[a20p0, a20p1, a20p2];
     
     If[a20min > a20r, Return[returnValue]]; 
     If[a20max < -a20r, Return[returnValue],
      If[a20min < -a20r,
       newDistance = -(a20max + a20r),
       newDistance = a20r - a20min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {a20x, a20y, 0}
       ]
      ];
     
     a21l = Sqrt[f1[[2]]^2 + f1[[1]]^2]; a21x = -f1[[2]]/a21l; 
     a21y = f1[[1]]/a21l; a21r = e[[1]]*Abs[a21x] + e[[2]]*Abs[a21y]; 
     a21p0 = v0[[1]]*a21x + v0[[2]]*a21y; 
     a21p1 = v1[[1]]*a21x + v1[[2]]*a21y; 
     a21p2 = v2[[1]]*a21x + v2[[2]]*a21y; 
     a21min = Min[a21p0, a21p1, a21p2]; 
     a21max = Max[a21p0, a21p1, a21p2];
     If[a21min > a21r, Return[returnValue]]; 
     If[a21max < -a21r, Return[returnValue],
      If[a21min < -a21r,
       newDistance = -(a21max + a21r),
       newDistance = a21r - a21min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {a21x, a21y, 0}
       ]
      ];
     
     a22l = Sqrt[f2[[2]]^2 + f2[[1]]^2]; a22x = -f2[[2]]/a22l; 
     a22y = f2[[1]]/a22l; a22r = e[[1]]*Abs[a22x] + e[[2]]*Abs[a22y]; 
     a22p0 = v0[[1]]*a22x + v0[[2]]*a22y; 
     a22p1 = v1[[1]]*a22x + v1[[2]]*a22y; 
     a22p2 = v2[[1]]*a22x + v2[[2]]*a22y; 
     a22min = Min[a22p0, a22p1, a22p2]; 
     a22max = Max[a22p0, a22p1, a22p2];
     If[a22min > a22r, Return[returnValue]];
     If[a22max < -a22r, Return[returnValue],
      If[a22min < -a22r,
       newDistance = -(a22max + a22r),
       newDistance = a22r - a22min
       ];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {a22x, a22y, 0}
       ]
      ];
     
     b0max = Max[v0[[1]], v1[[1]], v2[[1]]];
     If[b0max < -e[[1]], Return[returnValue],
       newDistance = -(e[[1]] + b0max);
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {1, 0, 0}
       ]
      ];
     
     b0min = Min[v0[[1]], v1[[1]], v2[[1]]];
     If[b0min > e[[1]], Return[returnValue],
      newDistance = b0min - e[[1]];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {-1, 0, 0}
       ]
      ];
     
     b1max = Max[v0[[2]], v1[[2]], v2[[2]]];
     If[b1max < -e[[2]], Return[returnValue],
      newDistance = -(e[[2]] + b1max);
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {0, 1, 0}
       ]
      ];
     
     b1min = Min[v0[[2]], v1[[2]], v2[[2]]];
     If[b1min > e[[2]], Return[returnValue],
      newDistance = b1min - e[[2]];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {0, -1, 0}
       ]
      ];
     
     b2max = Max[v0[[3]], v1[[3]], v2[[3]]];
     If[b2max < -e[[3]], Return[returnValue],
      newDistance = -(e[[3]] + b2max);
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {0, 0, 1}
       ]
      ];
     
     b2min = Min[v0[[3]], v1[[3]], v2[[3]]];
     If[b2min > e[[3]], Return[returnValue],
      newDistance = b2min - e[[3]];
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {0, 0, -1}
       ]
      ];
     
     pn = {
       -f0[[3]]*f1[[2]] + f0[[2]]*f1[[3]],
       f0[[3]]*f1[[1]] - f0[[1]]*f1[[3]],
       -f0[[2]]*f1[[1]] + f0[[1]]*f1[[2]]
       };
     
     pnl = Sqrt[pn[[1]]^2 + pn[[2]]^2 + pn[[3]]^2]; pn[[1]] /= pnl; 
     pn[[2]] /= pnl; pn[[3]] /= pnl; 
     pd = v0[[1]]*pn[[1]] + v0[[2]]*pn[[2]] + v0[[3]]*pn[[3]]; 
     pr = e[[1]]*Abs[pn[[1]]] + e[[2]]*Abs[pn[[2]]] + 
       e[[3]]*Abs[pn[[3]]];
     If[Abs[pd] > pr, Return[returnValue],
      newDistance = -pr - pd;
      If[Abs[newDistance] < Abs[distance],
       distance = newDistance;
       axis = {pn[[1]], pn[[2]], pn[[3]]}
       ]
      ];
     
     distance *= -1;
     translation = {0, 0, 0};
     translation[[1]] = axis[[1]]*distance; 
     translation[[2]] = axis[[2]]*distance; 
     translation[[3]] = axis[[3]]*distance; returnValue = True;
     Return[returnValue]
     ]
    ], CompilationTarget -> "C", Parallelization -> True
   ];


boxHalfLength[boxBounds_] := 
  Flatten[Differences /@ (Transpose@boxBounds)]/2;

boxCenter[boxBounds_] := RegionCentroid[Cuboid @@ boxBounds];

intersectTriangleBox[boxBounds_, triangleVertices_] := 
  intersectTriangleBox0[boxCenter[boxBounds], 
   boxHalfLength[boxBounds], triangleVertices];
   
   
   
(* BOX-BOX *)
   
intersectBoxBox[box1_,box2_]:=Block[{
b1Min=box1[[1]],b2Min=box2[[1]],
b1Max=box1[[2]],b2Max=box2[[2]]
},
(b1Min[[1]]<=b2Max[[1]]&&b1Max[[1]]>=b2Min[[1]])&&(b1Min[[2]]<=b2Max[[2]]&&b1Max[[2]]>=b2Min[[2]])&&(b1Min[[3]]<=b2Max[[3]]&&b1Max[[3]]>=b2Min[[3]])
];

(* LINE-LINE *)
intersectionLineLine[line1_,line2_]:=Solve[{x,y,z}\[Element]Line[line1]&&{x,y,z}\[Element]Line[line2],{x,y,z}]=!={};




(* Export Function *)
(* RAY-TRIANGLE *)
PrimitiveIntersectionQ3D[obj1_,obj2_]:=Quiet@intersectRayTriangle[(Delete[0]@obj1)[[1]],(Delete[0]@obj1)[[2]],(Delete[0]@obj1)[[3]],(Delete[0]@obj2)[[1]],(Delete[0]@obj2)[[2]]]/;(Head[obj1]==Triangle&&Head[obj2]==Line);
PrimitiveIntersectionQ3D[obj1_,obj2_]:=Quiet@intersectRayTriangle[(Delete[0]@obj2)[[1]],(Delete[0]@obj2)[[2]],(Delete[0]@obj2)[[3]],(Delete[0]@obj1)[[1]],(Delete[0]@obj1)[[2]]]/;(Head[obj2]==Triangle&&Head[obj1]==Line);

(* CUBOID-LINE *)
PrimitiveIntersectionQ3D[obj1_,obj2_]:=Quiet@intersectRayBox[Delete[0]@obj1,(Delete[0]@obj2)[[1]],(Delete[0]@obj2)[[2]]]/;(Head[obj1]==Cuboid&&Head[obj2]==Line);
PrimitiveIntersectionQ3D[obj1_,obj2_]:=Quiet@intersectRayBox[Delete[0]@obj2,(Delete[0]@obj1)[[1]],(Delete[0]@obj1)[[2]]]/;(Head[obj2]==Cuboid&&Head[obj1]==Line);

(* CUBOID-TRIANGLE *)
PrimitiveIntersectionQ3D[obj1_,obj2_]:=Quiet@intersectTriangleBox[Delete[0]@obj1, Delete[0]@obj2]/;(Head[obj1]==Cuboid&&Head[obj2]==Triangle);
PrimitiveIntersectionQ3D[obj1_,obj2_]:=Quiet@intersectTriangleBox[Delete[0]@obj2,Delete[0]@obj1]/;(Head[obj2]==Cuboid&&Head[obj1]==Triangle);

(* CUBOID-CUBOID *)
PrimitiveIntersectionQ3D[obj1_,obj2_]:=intersectBoxBox[Delete[0]@obj1,Delete[0]@obj2]/;(Head[obj1]==Cuboid&&Head[obj2]==Cuboid);

(*LINE-LINE*)
PrimitiveIntersectionQ3D[obj1_,obj2_]:=intersectionLineLine[Delete[0]@obj1,Delete[0]@obj2]/;(Head[obj1]==Line&&Head[obj2]==Line);


End[];

Column[{
With[{
testCuboid=Cuboid[{{0,0,0},{3,3,3}}],
testLine=Line[{{1,3,2},{2,1,3}}],
testTriangle=Triangle[{{0,0,0},{0,1,1},{1,2,2}}]
},
Row[{
Graphics3D[{
{Hue[0,0,0,0],testCuboid},testLine,testTriangle
},Boxed->False,ImageSize->Medium],
TableForm[Flatten/@Transpose[{Map[Head,#,{2}],PrimitiveIntersectionQ3D@@@#}],TableHeadings->{None,{"Obj1","Obj2","Intersects"}}]&[Permutations[{testCuboid,testLine,testTriangle},{2}]]
}]
],
With[{
cuboidTestCases=Permutations[{{Red,Cuboid[{{0,0,0},{5,5,5}}]},{Green,Cuboid[{{5,5,5},{6,6,6}}]},{Blue,Cuboid[{{2,2,2},{7,4,4}}]}},{2}]
},
Row[{
Graphics3D[cuboidTestCases,ImageSize->Medium],
Framed@TableForm[Flatten/@Transpose[{Map[First,cuboidTestCases,{2}],(PrimitiveIntersectionQ3D@@(Last/@#))&/@cuboidTestCases}]]
},Spacer[5]]
]
}]

EndPackage[];








