#include <glbinding/Binding.h>
#include <glbinding/ContextInfo.h>
#include <glbinding/Version.h>
// Do not import namespace to prevent glbinding/QTOpenGL collision
#include <glbinding/gl/gl.h>

#include <globjects/globjects.h>

#include <Engine/RadiumEngine.hpp>

#include <GuiBase/Viewer/Viewer.hpp>

#include <iostream>

#include <QTimer>
#include <QMouseEvent>
#include <QPainter>

#include <Core/String/StringUtils.hpp>
#include <Core/Log/Log.hpp>
#include <Core/Math/ColorPresets.hpp>
#include <Core/Math/Math.hpp>
#include <Core/Containers/MakeShared.hpp>
#include <Core/Image/stb_image_write.h>

#include <Engine/Component/Component.hpp>

#include <Engine/Managers/EntityManager/EntityManager.hpp>
#include <Engine/Managers/SystemDisplay/SystemDisplay.hpp>

#include <Engine/Renderer/Camera/Camera.hpp>
#include <Engine/Renderer/Light/DirLight.hpp>
#include <Engine/Renderer/Renderer.hpp>
#include <Engine/Renderer/Renderers/ExperimentalRenderer.hpp>
#include <Engine/Renderer/Renderers/ForwardRenderer.hpp>
#include <Engine/Renderer/RenderTechnique/ShaderProgramManager.hpp>

#include <GuiBase/Viewer/TrackballCamera.hpp>

#include <GuiBase/Utils/Keyboard.hpp>
#include <GuiBase/Utils/KeyMappingManager.hpp>
#include <GuiBase/Utils/PickingManager.hpp>

namespace Ra
{
    Gui::Viewer::Viewer( QWidget* parent )
        : QOpenGLWidget( parent )
//        , m_renderers(3)
        , m_pickingManager( nullptr )
        , m_isBrushPickingEnabled( false )
        , m_brushRadius( 10 )
        , m_gizmoManager(new GizmoManager(this))
        , m_renderThread( nullptr )
    {
        // Allow Viewer to receive events
        setFocusPolicy( Qt::StrongFocus );
        setMinimumSize( QSize( 800, 600 ) );

        m_camera.reset( new Gui::TrackballCamera( width(), height() ) );

        m_pickingManager = new PickingManager();
        /// Intercept events to properly lock the renderer when it is compositing.
    }

    Gui::Viewer::~Viewer(){}


    int Gui::Viewer::addRenderer(std::unique_ptr<Engine::Renderer> e){
        m_renderers.push_back(std::move(e));
        return m_renderers.size()-1;
    }

    void Gui::Viewer::initializeGL()
    {
        //glbinding::Binding::initialize(false);
        // no need to initalize glbinding. globjects (magically) do this internally.
        globjects::init(globjects::Shader::IncludeImplementation::Fallback);

        LOG( logINFO ) << "*** Radium Engine Viewer ***";
        LOG( logINFO ) << "Renderer (glbinding) : " << glbinding::ContextInfo::renderer();
        LOG( logINFO ) << "Vendor   (glbinding) : " << glbinding::ContextInfo::vendor();
        LOG( logINFO ) << "OpenGL   (glbinding) : " << glbinding::ContextInfo::version().toString();
        LOG( logINFO ) << "GLSL                 : " << gl::glGetString(gl::GLenum(GL_SHADING_LANGUAGE_VERSION));

        // FIXME(Charly): Renderer type should not be changed here
        // m_renderers.resize( 3 );
        // FIXME (Mathias): width and height might be wrong the first time ResizeGL is called (see QOpenGLWidget doc). This may cause problem on Retina display under MacOsX (and this happens)
        m_renderers.push_back(std::unique_ptr<Engine::Renderer>(new Engine::ForwardRenderer( width(), height()))); // Forward

//        m_renderers[1].reset( nullptr ); // deferred
        // m_renderers[2].reset( new Engine::ExperimentalRenderer( width(), height() ) ); // experimental

        for ( auto& renderer : m_renderers )
        {
            if (renderer)
            {
                renderer->initialize();
            }
        }

        m_currentRenderer = m_renderers[0].get();

        auto light = Ra::Core::make_shared<Engine::DirectionalLight>();

        for ( auto& renderer : m_renderers )
        {
            if (renderer)
            {
                renderer->addLight( light );
            }
        }

        m_camera->attachLight( light );
        emit rendererReady();
    }

    Gui::CameraInterface* Gui::Viewer::getCameraInterface()
    {
        return m_camera.get();
    }

    Gui::GizmoManager* Gui::Viewer::getGizmoManager()
    {
        return m_gizmoManager;
    }

    const Engine::Renderer* Gui::Viewer::getRenderer() const
    {
        return m_currentRenderer;
    }

    Engine::Renderer* Gui::Viewer::getRenderer()
    {
        return m_currentRenderer;
    }

    Gui::PickingManager* Gui::Viewer::getPickingManager()
    {
        return m_pickingManager;
    }

    void Gui::Viewer::onAboutToCompose()
    {
        // This slot function is called from the main thread as part of the event loop
        // when the GUI is about to update. We have to wait for the rendering to finish.
        m_currentRenderer->lockRendering();
    }

    void Gui::Viewer::onFrameSwapped()
    {
        // This slot is called from the main thread as part of the event loop when the
        // GUI has finished displaying the rendered image, so we unlock the renderer.
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::onAboutToResize()
    {
        // Like swap buffers, resizing is a blocking operation and we have to wait for the rendering
        // to finish before resizing.
        m_currentRenderer->lockRendering();
    }

    void Gui::Viewer::onResized()
    {
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::resizeGL( int width, int height )
    {
        // FIXME (Mathias) : Problem of glarea dimension on OsX Retina Display (half the size)
        // Renderer should have been locked by previous events.
        m_camera->resizeViewport( width, height );
        m_currentRenderer->resize( width, height );
    }

    Engine::Renderer::PickingMode Gui::Viewer::getPickingMode() const
    {
        auto keyMap = Gui::KeyMappingManager::getInstance();
        if( Gui::isKeyPressed( keyMap->getKeyFromAction( Gui::KeyMappingManager::FEATUREPICKING_VERTEX ) ) )
        {
            return m_isBrushPickingEnabled ? Engine::Renderer::C_VERTEX : Engine::Renderer::VERTEX;
        }
        if( Gui::isKeyPressed( keyMap->getKeyFromAction( Gui::KeyMappingManager::FEATUREPICKING_EDGE ) ) )
        {
            return m_isBrushPickingEnabled ? Engine::Renderer::C_EDGE : Engine::Renderer::EDGE;
        }
        if( Gui::isKeyPressed( keyMap->getKeyFromAction( Gui::KeyMappingManager::FEATUREPICKING_TRIANGLE ) ) )
        {
            return m_isBrushPickingEnabled ? Engine::Renderer::C_TRIANGLE : Engine::Renderer::TRIANGLE;
        }
        return Engine::Renderer::RO;
    }

    void Gui::Viewer::mousePressEvent( QMouseEvent* event )
    {
        auto keyMap = Gui::KeyMappingManager::getInstance();
        if ( keyMap->actionTriggered( event, Gui::KeyMappingManager::VIEWER_BUTTON_CAST_RAY_QUERY ) &&
             isKeyPressed( keyMap->getKeyFromAction(Gui::KeyMappingManager::VIEWER_RAYCAST_QUERY ) ) )
        {
            LOG( logINFO ) << "Raycast query launched";
            Core::Ray r = m_camera->getCamera()->getRayFromScreen(Core::Vector2(event->x(), event->y()));
            RA_DISPLAY_POINT(r.origin(), Core::Colors::Cyan(), 0.1f);
            RA_DISPLAY_RAY(r, Core::Colors::Yellow());
            auto ents = Engine::RadiumEngine::getInstance()->getEntityManager()->getEntities();
            for (auto e : ents)
            {
                e->rayCastQuery(r);
            }
        }
        else if ( keyMap->getKeyFromAction(Gui::KeyMappingManager::TRACKBALLCAMERA_MANIPULATION) == event->button() )
        {
            m_camera->handleMousePressEvent(event);
        }
        else if ( keyMap->actionTriggered( event, Gui::KeyMappingManager::GIZMOMANAGER_MANIPULATION ) )
        {
            m_currentRenderer->addPickingRequest({ Core::Vector2(event->x(), height() - event->y()),
                                                   Core::MouseButton::RA_MOUSE_LEFT_BUTTON,
                                                   Engine::Renderer::RO });
            if (m_gizmoManager != nullptr)
            {
                m_gizmoManager->handleMousePressEvent(event);
            }
        }
        else if ( keyMap->actionTriggered( event, Gui::KeyMappingManager::VIEWER_BUTTON_PICKING_QUERY ) )
        {
            // Check picking
            Engine::Renderer::PickingQuery query  = { Core::Vector2(event->x(), height() - event->y()),
                                                      Core::MouseButton::RA_MOUSE_RIGHT_BUTTON,
                                                      getPickingMode() };
            m_currentRenderer->addPickingRequest(query);
        }

        QOpenGLWidget::mousePressEvent(event);
    }

    void Gui::Viewer::mouseReleaseEvent( QMouseEvent* event )
    {
        m_camera->handleMouseReleaseEvent( event );
        if (m_gizmoManager != nullptr)
        {
            m_gizmoManager->handleMouseReleaseEvent(event);
        }

        QOpenGLWidget::mouseReleaseEvent(event);
    }

    void Gui::Viewer::mouseMoveEvent( QMouseEvent* event )
    {
        m_camera->handleMouseMoveEvent( event );
        if (m_gizmoManager != nullptr)
        {
            m_gizmoManager->handleMouseMoveEvent(event);
        }
        m_currentRenderer->setMousePosition(Ra::Core::Vector2(event->x(), event->y()));
        if ( (int(event->buttons()) | int(event->modifiers())) == Gui::KeyMappingManager::getInstance()->getKeyFromAction( Gui::KeyMappingManager::VIEWER_BUTTON_PICKING_QUERY ) )
        {
            // Check picking
            Engine::Renderer::PickingQuery query  = { Core::Vector2(event->x(), (height() - event->y())),
                                                      Core::MouseButton::RA_MOUSE_RIGHT_BUTTON,
                                                      getPickingMode() };
            m_currentRenderer->addPickingRequest(query);
        }

        QOpenGLWidget::mouseMoveEvent(event);
    }

    void Gui::Viewer::wheelEvent( QWheelEvent* event )
    {
        m_camera->handleWheelEvent(event);
        if (m_isBrushPickingEnabled && isKeyPressed(Qt::Key_Shift))
        {
            m_brushRadius += (event->angleDelta().y() * 0.01 + event->angleDelta().x() * 0.01) > 0 ? 5 : -5 ;
            m_brushRadius = std::max( m_brushRadius, Scalar(5) );
            m_currentRenderer->setBrushRadius( m_brushRadius );
        }
        else
        {
            m_camera->handleWheelEvent(event);
        }

        QOpenGLWidget::wheelEvent(event);
    }

    void Gui::Viewer::keyPressEvent( QKeyEvent* event )
    {
        keyPressed(event->key());
        m_camera->handleKeyPressEvent( event );

        QOpenGLWidget::keyPressEvent(event);
    }

    void Gui::Viewer::keyReleaseEvent( QKeyEvent* event )
    {
        keyReleased(event->key());
        m_camera->handleKeyReleaseEvent( event );

        auto keyMap = Gui::KeyMappingManager::getInstance();
        if ( keyMap->actionTriggered( event, Gui::KeyMappingManager::VIEWER_TOGGLE_WIREFRAME )
             && !event->isAutoRepeat() )
        {
            m_currentRenderer->toggleWireframe();
        }
        if ( keyMap->actionTriggered( event, Gui::KeyMappingManager::FEATUREPICKING_MULTI_CIRCLE )
             && event->modifiers() == Qt::NoModifier && !event->isAutoRepeat() )
        {
            m_isBrushPickingEnabled = !m_isBrushPickingEnabled;
            m_currentRenderer->setBrushRadius( m_isBrushPickingEnabled ? m_brushRadius : 0 );
            emit toggleBrushPicking( m_isBrushPickingEnabled );
        }

        QOpenGLWidget::keyReleaseEvent(event);
    }

    void Gui::Viewer::reloadShaders()
    {
        // FIXME : check thread-saefty of this.
        m_currentRenderer->lockRendering();
        makeCurrent();
        m_currentRenderer->reloadShaders();
        doneCurrent();
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::displayTexture( const QString &tex )
    {
        m_currentRenderer->lockRendering();
        m_currentRenderer->displayTexture( tex.toStdString() );
        m_currentRenderer->unlockRendering();
    }

    void Gui::Viewer::changeRenderer( int index )
    {
        if (m_renderers[index]) {
            // NOTE(Charly): This is probably buggy since it has not been tested.
            LOG( logWARNING ) << "Changing renderers might be buggy since it has not been tested.";
            m_currentRenderer->lockRendering();
            m_currentRenderer = m_renderers[index].get();
            m_currentRenderer->initialize();
            m_currentRenderer->resize( width(), height() );
            m_currentRenderer->unlockRendering();
        }
    }

    // Asynchronous rendering implementation

    void Gui::Viewer::startRendering( const Scalar dt )
    {
        makeCurrent();

        // Move camera if needed. Disabled for now as it takes too long (see issue #69)
        //m_camera->update( dt );

        Engine::RenderData data;
        data.dt = dt;
        data.projMatrix = m_camera->getProjMatrix();
        data.viewMatrix = m_camera->getViewMatrix();

        m_currentRenderer->render( data );
    }

    void Gui::Viewer::waitForRendering()
    {
    }

    void Gui::Viewer::handleFileLoading( const std::string& file )
    {
        for ( auto& renderer : m_renderers )
        {
            if (renderer)
            {
                renderer->handleFileLoading( file );
            }
        }
    }

    void Gui::Viewer::processPicking()
    {
        CORE_ASSERT( m_currentRenderer->getPickingQueries().size() == m_currentRenderer->getPickingResults().size(),
                    "There should be one result per query." );

        for (uint i = 0 ; i < m_currentRenderer->getPickingQueries().size(); ++i)
        {
            const Engine::Renderer::PickingQuery& query  = m_currentRenderer->getPickingQueries()[i];
            if ( query.m_button == Core::MouseButton::RA_MOUSE_LEFT_BUTTON)
            {
                emit leftClickPicking(m_currentRenderer->getPickingResults()[i].m_roIdx);
            }
            else if (query.m_button == Core::MouseButton::RA_MOUSE_RIGHT_BUTTON)
            {
                const auto& result = m_currentRenderer->getPickingResults()[i];
                m_pickingManager->setCurrent( result );
                emit rightClickPicking( result );
            }
        }
    }

    void Gui::Viewer::fitCameraToScene( const Core::Aabb& aabb )
    {
        if (!aabb.isEmpty())
        {
            m_camera->fitScene(aabb);
        }
    }

    void Gui::Viewer::loadCamera(std::istream &in)
    {
        m_camera->load(in);
    }

    void Gui::Viewer::saveCamera(std::ostream& out) const
    {
        m_camera->save(out);
    }


    std::vector<std::string> Gui::Viewer::getRenderersName() const
    {
        std::vector<std::string> ret;

        for ( const auto& renderer : m_renderers )
        {
            if (renderer)
            {
                ret.push_back( renderer->getRendererName() );
            }
        }

        return ret;
    }

    void Gui::Viewer::grabFrame( const std::string& filename )
    {
        makeCurrent();

        uint w, h;
        uchar* writtenPixels = m_currentRenderer->grabFrame(w, h);

        std::string ext = Core::StringUtils::getFileExt(filename);

        if (ext == "bmp")
        {
            stbi_write_bmp(filename.c_str(), w, h, 4, writtenPixels);
        }
        else if (ext == "png")
        {
            stbi_write_png(filename.c_str(), w, h, 4, writtenPixels, w * 4 * sizeof(uchar));
        }
        else
        {
            LOG(logWARNING) << "Cannot write frame to "<<filename<<" : unsupported extension";
        }


        delete[] writtenPixels;

    }

    void Gui::Viewer::enablePostProcess(int enabled)
    {
        m_currentRenderer->enablePostProcess(enabled);
    }

    void Gui::Viewer::enableDebugDraw(int enabled)
    {
        m_currentRenderer->enableDebugDraw(enabled);
    }

    void Gui::Viewer::resetCamera()
    {
        m_camera.reset( new Gui::TrackballCamera( width(), height() ) );
    }
} // namespace Ra
