#ifndef SKINPLUGIN_SKINNING_COMPONENT_HPP_
#define SKINPLUGIN_SKINNING_COMPONENT_HPP_

#include <SkinningPluginMacros.hpp>

#include <Core/Animation/Handle/HandleWeight.hpp>
#include <Core/Animation/Pose/Pose.hpp>
#include <Core/Animation/Skinning/SkinningData.hpp>
#include <Core/File/HandleData.hpp>
#include <Core/Math/DualQuaternion.hpp>
#include <Core/Mesh/TopologicalTriMesh/TopologicalMesh.hpp>
#include <Core/Mesh/TriangleMesh.hpp>

#include <Core/Algorithm/Subdivision/CatmullClarkSubdivider.hpp>
#include <Core/Algorithm/Subdivision/LoopSubdivider.hpp>
#include <Engine/Component/Component.hpp>
#include <Engine/Managers/ComponentMessenger/ComponentMessenger.hpp>

namespace SkinningPlugin {
class SKIN_PLUGIN_API SkinningComponent : public Ra::Engine::Component {
  public:
    enum SkinningType {
        LBS = 0, // Linear Blend Skinning
        DQS,     // Dual Quaternion Skinning
        COR      // Center of Rotation skinning
    };

    SkinningComponent( const std::string& name, SkinningType type, Ra::Engine::Entity* entity ) :
        Component( name, entity ),
        m_skinningType( type ),
        m_isReady( false ) {}
    virtual ~SkinningComponent() {}

    virtual void initialize() override { setupSkinning(); }

    void skin();
    void endSkinning();
    void setupSkinning();

    void setSkinningType( SkinningType type );
    inline SkinningType getSkinningType() const { return m_skinningType; }

    virtual void handleWeightsLoading( const Ra::Asset::HandleData* data );

    const Ra::Core::Skinning::RefData* getRefData() const { return &m_refData; }
    const Ra::Core::Skinning::FrameData* getFrameData() const { return &m_frameData; }
    const Ra::Core::AlignedStdVector<Ra::Core::DualQuaternion>* getDQ() const { return &m_DQ; }

  public:
    void setupIO( const std::string& id );
    void setupSkinningType( SkinningType type );
    void setContentsName( const std::string name );

  private:
    //    using Subdivider = Ra::Core::CatmullClarkSubdivider;
    using Subdivider = Ra::Core::LoopSubdivider;

    std::string m_contentsName;

    // Skinning data
    Ra::Core::Skinning::RefData m_refData;
    Ra::Core::Skinning::FrameData m_frameData;

    // #################### topo data
    Ra::Core::TopologicalMesh m_topoMeshCoarse;
    Ra::Core::TopologicalMesh m_topoMeshFull;
    Ra::Core::TriangleMesh m_meshFull;
    std::vector<Ra::Core::TopologicalMesh::VertexHandle> m_mapI2HCoarse;
    std::vector<Ra::Core::TopologicalMesh::VertexHandle> m_mapI2HFull;
    std::vector<Subdivider::S_OPS> m_subdivNewOps;
    std::vector<Subdivider::S_OPS> m_subdivOldOps;
    // ####################

    Ra::Engine::ComponentMessenger::CallbackTypes<Ra::Core::Animation::Skeleton>::Getter
        m_skeletonGetter;
    Ra::Engine::ComponentMessenger::CallbackTypes<std::vector<Ra::Core::Index>>::Getter
        m_duplicateTableGetter;

    Ra::Engine::ComponentMessenger::CallbackTypes<Ra::Core::Index>::Getter m_roIdxGetter;

    Ra::Engine::ComponentMessenger::CallbackTypes<Ra::Core::Vector3Array>::ReadWrite
        m_verticesWriter;
    Ra::Engine::ComponentMessenger::CallbackTypes<Ra::Core::Vector3Array>::ReadWrite
        m_normalsWriter;

    Ra::Core::AlignedStdVector<Ra::Core::DualQuaternion> m_DQ;

    SkinningType m_skinningType;
    bool m_isReady;
};
} // namespace SkinningPlugin

#endif //  SKINPLUGIN_SKINNING_COMPONENT_HPP_
