#pragma once

// For more information on binding and using Zilch APIs, visit: http://zilch.digipen.edu/
// For auto binding specifically, visit: http://zilch.digipen.edu/home/AutomaticBinding.html

class ClothSpringSystem;

DeclareEnum3(IntegrationMethod, SemiImplicitEuler, Verlet, Jakobsen);

//-----------------------------------------------------------------------------ClothSpring
class ClothSpring
{
public:
  int mIndexA;
  int mIndexB;
  Real mRestLength;
  Real mStiffness;
  Real mDamping;
};

//-----------------------------------------------------------------------------ClothFace
class ClothFace
{
public:
  int mIndexA;
  int mIndexB;
  int mIndexC;
};

//-----------------------------------------------------------------------------ClothAnchor
class ClothAnchor
{
public:
  int mIndex;
  Zilch::HandleOf<Cog> mAnchor;
  Real3 mLocalPosition;
};

//-----------------------------------------------------------------------------ClothSpringSystemEvent
class ClothSpringSystemEvent : public ZeroEngine::ZilchEvent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);

  Zilch::HandleOf<ClothSpringSystem> mClothSystem;
  Real mDt;
};

//-----------------------------------------------------------------------------ClothSpringSystem
class ClothSpringSystem : public ZeroEngine::ZilchComponent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);

  ClothSpringSystem();
  ~ClothSpringSystem();

  void Initialize(ZeroEngine::CogInitializer* initializer);

  void OnLogicUpdate(ZeroEngine::UpdateEvent* event);

  void IterateBasicTimestep(Real dt);
  void IterateJakobsenTimestep(Real dt);

  void UpdateAnchors(Real dt);
  void CalcuateGlobalForces(Real dt);
  void CalculateSpringForces(Real dt);
  void SatisfyConstraintsJakobsen(Real dt);
  void Integrate(Real dt);
  void IntegrateEuler(Real dt);
  void IntegrateVerlet(Real dt);
  void ClearForces();
  void UploadToMesh(ZeroEngine::Mesh* mesh);
  void UploadToMeshWorld(ZeroEngine::Mesh* mesh, Array<Real3>& normals);
  void UploadToMeshLocal(ZeroEngine::Mesh* mesh, Array<Real3>& normals);

  int GetParticleCount();
  void AddParticle(Real3Param position, Real invMass);

  Real3 GetParticlePosition(int index);
  void SetParticlePosition(int index, Real3Param position);
  Real GetParticleInvMass(int index);
  void SetParticleInvMass(int index, Real invMass);
  void ApplyParticleForce(int index, Real3Param force);

  int GetSpringCount();
  void AddSpring(int indexA, int indexB, Real stiffness, Real damping);
  Real GetRestLength(int index);
  void SetRestLength(int index, Real restLength);

  int GetFaceCount();
  void AddFace(int indexA, int indexB, int indexC);
  Integer3 GetFaceIndices(int index);
  int GetFaceIndexA(int index);
  int GetFaceIndexB(int index);
  int GetFaceIndexC(int index);

  void AddAnchor(int index, Cog* anchorCog);
  void ClearAnchors();

  void ClearSystem();
  void ClearParticles();
  void ClearSprings();
  void ClearFaces();
  void ClearUvs();

  void SetMesh(ZeroEngine::Mesh* mesh);
  void AddUv(Real2Param uv);


  Array<Real3> mPositions;
  Array<Real3> mOldPositions;
  Array<Real3> mVelocities;
  Array<Real3> mForces;
  Array<Real> mInvMasses;

  Array<ClothSpring> mSprings;
  Array<ClothFace> mFaces;
  Array<ClothAnchor> mAnchors;
  int mSubDivisions;
  int mIterations;

  Array<Real2> mUvs;
  Zilch::HandleOf<ZeroEngine::Mesh> mMesh;
  IntegrationMethod::Enum mIntegrationMethod;

  bool mUploadInLocalSpace;
};

//-----------------------------------------------------------------------------ClothWind
class ClothWind : public ZeroEngine::ZilchComponent
{
public:
  ZilchDeclareType(Zilch::TypeCopyMode::ReferenceType);

  ClothWind();
  ~ClothWind();

  void Initialize(ZeroEngine::CogInitializer* initializer);

  void OnComputeLocalClothForces(ClothSpringSystemEvent* event);
  void OnComputeSpaceClothForces(ClothSpringSystemEvent* event);

  void ComputeClothForces(ClothSpringSystem* system);

  bool mActive;
  bool mSpaceForce;
  Real3 mWindDirection;
  Real mWindStrength;
};
