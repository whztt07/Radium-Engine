#include <MeshPaintSystem.hpp>

#include <Core/String/StringUtils.hpp>
#include <Core/Tasks/Task.hpp>
#include <Core/Tasks/TaskQueue.hpp>
#include <Core/File/FileData.hpp>
#include <Core/File/GeometryData.hpp>

#include <Engine/RadiumEngine.hpp>
#include <Engine/Entity/Entity.hpp>
#include <Engine/FrameInfo.hpp>
#include <Engine/Renderer/RenderTechnique/RenderTechnique.hpp>
#include <Engine/Managers/ComponentMessenger/ComponentMessenger.hpp>

#include <MeshPaintComponent.hpp>

namespace MeshPaintPlugin
{
    MeshPaintSystem::MeshPaintSystem()
        : Ra::Engine::System()
    {
    }

    MeshPaintSystem::~MeshPaintSystem()
    {
    }

    void MeshPaintSystem::handleAssetLoading( Ra::Engine::Entity* entity, const Ra::Asset::FileData* fileData )
    {
        auto geomData = fileData->getGeometryData();

        uint id = 0;

        for ( const auto& data : geomData )
        {
            std::string componentName = "MP_" + entity->getName() + std::to_string( id++ );
            MeshPaintComponent *comp = new MeshPaintComponent( componentName );
            comp->setDataId( data->getName() );
            entity->addComponent( comp );
            registerComponent( entity, comp );
        }
    }

    void MeshPaintSystem::generateTasks( Ra::Core::TaskQueue* taskQueue, const Ra::Engine::FrameInfo& frameInfo )
    {
        // Do nothing, as this system only displays painted meshes.
    }

    void MeshPaintSystem::startPaintMesh( bool start )
    {
        // switch rendered mesh
        for ( auto& compEntry : this->m_components )
        {
            auto MPcomp = static_cast<MeshPaintComponent*>( compEntry.second );
            MPcomp->startPaint( start );
        }
    }

    void MeshPaintSystem::paintMesh( const Ra::Engine::Renderer::PickingResult& picking, const Ra::Core::Color& color )
    {
        for ( auto& compEntry : this->m_components )
        {
            auto MPcomp = static_cast<MeshPaintComponent*>( compEntry.second );
            MPcomp->paintMesh( picking, color );
        }
    }

} // namespace FancyMeshPlugin
