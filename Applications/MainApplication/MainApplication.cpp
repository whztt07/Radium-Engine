#include <MainApplication.hpp>

#include <Core/CoreMacros.hpp>

#include <fstream>

#include <QTimer>
#include <QDir>
#include <QPluginLoader>
#include <QCommandLineParser>
#include <QOpenGLContext>

#include <Core/Log/Log.hpp>
#include <Core/String/StringUtils.hpp>
#include <Core/Mesh/MeshUtils.hpp>
#include <Core/Math/LinearAlgebra.hpp>
#include <Core/Math/ColorPresets.hpp>
#include <Core/Tasks/Task.hpp>
#include <Core/Tasks/TaskQueue.hpp>
#include <Core/String/StringUtils.hpp>
#include <Core/Utils/Version.hpp>

#include <Engine/RadiumEngine.hpp>
#include <Engine/Entity/Entity.hpp>

#include <Engine/Managers/SystemDisplay/SystemDisplay.hpp>
#include <Engine/Managers/EntityManager/EntityManager.hpp>

#include <Engine/Renderer/Renderer.hpp>
#include <Engine/Renderer/RenderObject/RenderObjectManager.hpp>
#include <Engine/Renderer/RenderObject/RenderObject.hpp>
#include <Engine/Renderer/Mesh/Mesh.hpp>
#include <Engine/Renderer/RenderTechnique/ShaderConfigFactory.hpp>

#include <PluginBase/RadiumPluginInterface.hpp>

#include <Gui/MainWindow.hpp>

#include <GuiBase/Utils/KeyMappingManager.hpp>



// global variables for config


// IS display
bool g_show_anat = false;
bool g_run_anat = false ;
bool g_is_trans = false ;
bool g_show_subdiv = false ;
bool g_anim_autoplay = false;
bool g_force_IS = true;
bool g_export_meshes = false;
bool g_no_IS = false;

Scalar g_animation_speed = 1.f;

// Physics parameters

uint   g_num_particles = 30; // Number of particles per muscle
Scalar g_tendon_proportion = 0.1;  // percentage of the muscle length which is considered tendon

Scalar g_massFactor = 0.5f; // multiplicator of all masses.

Scalar g_stiffness = 0.60f;          // Stiffness of the spring constraints (0.2f);
Scalar g_rest_length= 0.02f;        // Rest length fraction of the spring constraint ( 0.02f)
Scalar g_tendon_stiffness = 1.f;   // Stiffness of spring for tendon sections (1.f)
Scalar g_tendon_rest_length = -1.f; // Rest length of srping of tendon section (-1.f)

Scalar g_damping = 1.f; // damping

Scalar g_collision_stiffness = 0.2f; // Stiffness of the collision vs implicit constraint (10.f)
Scalar g_keep_inside_stiffness = 1.5f; // Stiffness of the collision vs skin constraint (10.5f)
Scalar g_iso_inside = 1.f; // Target iso of the inside constraint (0.01f)

//Scalar g_physics_time_step = 0.001f; // Duration of the physics time step
//Scalar g_physics_time_step = 0.005f; // Duration of the physics time step
uint g_physics_iter = 10;  // number of physics steps per frame
uint g_framerate = 30;

const uint * g_frame_num_ptr;

std::string g_anat_file = "";
std::string g_phys_file = "";
std::string g_kf_file = "";
std::string g_export_folder="";

bool g_debug_physics;
namespace Ra
{
    BaseApplication::BaseApplication( int argc, char** argv, QString applicationName, QString organizationName)
        : QApplication( argc, argv )
        , m_mainWindow( nullptr )
        , m_engine( nullptr )
        , m_taskQueue( nullptr )
        , m_viewer( nullptr )
        , m_frameTimer( new QTimer( this ) )
        , m_frameCounter( 0 )
        , m_numFrames( 0 )
        , m_realFrameRate( false )
        , m_exportFolderName("")
        , m_recordFrames( false )
        , m_recordTimings( false )
        , m_recordGraph( false )
        , m_recordMeshes( false )
        , m_isAboutToQuit( false )
    {

        g_frame_num_ptr = & m_frameCounter;
        // Set application and organization names in order to ensure uniform
        // QSettings configurations.
        // \see http://doc.qt.io/qt-5/qsettings.html#QSettings-4
        QCoreApplication::setOrganizationName(organizationName);
        QCoreApplication::setApplicationName(applicationName);

        m_targetFPS = 30; // Default
        std::string pluginsPath = "Plugins";

        QCommandLineParser parser;
        parser.setApplicationDescription("Radium Engine RPZ, TMTC");
        parser.addHelpOption();
        parser.addVersionOption();

        QCommandLineOption fpsOpt(QStringList{"r", "framerate", "fps"}, "Control the application framerate, 0 to disable it (and run as fast as possible).", "number", "30");
        QCommandLineOption maxThreadsOpt(QStringList{"m", "maxthreads", "max-threads"}, "Control the maximum number of threads. 0 will set to the number of cores available", "number", "0");
        QCommandLineOption numFramesOpt(QStringList{"n", "numframes"}, "Run for a fixed number of frames.", "number", "0");
        QCommandLineOption pluginOpt(QStringList{"p", "plugins", "pluginsPath"}, "Set the path to the plugin dlls.", "folder", "Plugins");
        QCommandLineOption pluginLoadOpt(QStringList{"l", "load", "loadPlugin"}, "Only load plugin with the given name (filename without the extension). If this option is not used, all plugins in the plugins folder will be loaded. ", "name");
        QCommandLineOption pluginIgnoreOpt(QStringList{"i", "ignore", "ignorePlugin"}, "Ignore plugins with the given name. If the name appears within both load and ignore options, it will be ignored.", "name");
        QCommandLineOption fileOpt(QStringList{"f", "file", "scene"}, "Open a scene file at startup.", "file name", "foo.bar");
        QCommandLineOption camOpt(QStringList{"c", "camera", "cam"}, "Open a camera file at startup", "file name", "foo.bar");
        QCommandLineOption exportOpt(QStringList{"e", "export", "exportFolder"}, "Name of the output folder of the application.", "folder name", "Numerical date and time");
        QCommandLineOption timingsOpt(QStringList{"t", "timings", "exportTimings"}, "Export the timings of each frame", "", "");



        QCommandLineOption showAnatOpt(QStringList{"showanat"}, "show anatomy","","");
        QCommandLineOption runAnatOpt(QStringList{"runanat"}, "run anatomy","","");
        QCommandLineOption transISOpt(QStringList{"trans"}, "transparent skin","","");
        QCommandLineOption subdivOpt(QStringList{"subdiv"}, "run subdivision","","");
        QCommandLineOption autoplayOpt(QStringList{"autoplay"}, "autoplay animation","","");
        QCommandLineOption animSpeedOpt(QStringList{"animspeed"}, "autoplay animation","scalar","1.0");
        QCommandLineOption physicsIterOp(QStringList{"timestep"}, "physics iterations","int","10");
        QCommandLineOption numParticlesOpt(QStringList{"numparticles"}, "number of particles","integer","30");


        QCommandLineOption noISOpt(QStringList{"nois"}, "disable implicit skinning","","");

        QCommandLineOption saveMeshes(QStringList{"savemeshes"}, "save meshes","", "");
        QCommandLineOption saveFrames(QStringList{"saveframes"}, "save frames","", "");


        QCommandLineOption anatFile(QStringList{"anatfile"}, "anatomical data file","file name", "foo.txt");
        QCommandLineOption kfFile(QStringList{"kffile"}, "keyframe data file","file name", "foo.txt");
        QCommandLineOption physFile(QStringList{"phyfile"}, "physics data file","file name", "foo.txt");



        parser.addOptions({fpsOpt, pluginOpt, pluginLoadOpt, pluginIgnoreOpt, fileOpt, camOpt, maxThreadsOpt, numFramesOpt, exportOpt,timingsOpt });
        parser.addOptions({ showAnatOpt, runAnatOpt, transISOpt, subdivOpt, saveMeshes, saveFrames, autoplayOpt, animSpeedOpt, physicsIterOp, numParticlesOpt, noISOpt });
        parser.addOptions({anatFile, kfFile, physFile});
        parser.process(*this);

        if (parser.isSet(fpsOpt))       m_targetFPS = parser.value(fpsOpt).toUInt();
        g_framerate = m_targetFPS;
        if (parser.isSet(pluginOpt))    pluginsPath = parser.value(pluginOpt).toStdString();
        if (parser.isSet(numFramesOpt)) m_numFrames = parser.value(numFramesOpt).toUInt();
        if (parser.isSet(maxThreadsOpt)) m_maxThreads = parser.value(maxThreadsOpt).toUInt();
        if (parser.isSet(timingsOpt))    setRecordTimings(true);


        if (parser.isSet(showAnatOpt))  g_show_anat     = true;
        if (parser.isSet(runAnatOpt))   g_run_anat      = true;
        if (parser.isSet(transISOpt))   g_is_trans      = true;
        if (parser.isSet(subdivOpt))    g_show_subdiv   = true;
        if (parser.isSet(autoplayOpt))  g_anim_autoplay = true;
        if (parser.isSet(animSpeedOpt)) g_animation_speed = parser.value(animSpeedOpt).toDouble();
        if (parser.isSet(physicsIterOp))  g_physics_iter = parser.value(physicsIterOp).toInt();
        if (parser.isSet(numParticlesOpt)) g_num_particles = parser.value(numParticlesOpt).toInt();


        if (parser.isSet(noISOpt)) g_no_IS = true;

        if (parser.isSet(saveMeshes))   m_recordMeshes  = g_export_meshes = true;
        if (parser.isSet(saveFrames))   m_recordFrames  = true;

        if (parser.isSet(anatFile)) g_anat_file = parser.value(anatFile).toStdString();
        if (parser.isSet(kfFile)) g_kf_file = parser.value(kfFile).toStdString();
        if (parser.isSet(physFile)) g_phys_file = parser.value(physFile).toStdString();


        if (parser.isSet(exportOpt))
        {
            m_exportFolderName = parser.value(exportOpt).toStdString();
        }
        // If the option was not set or if it was set to an empty string
        // use default date folder

        if (m_exportFolderName.empty())
        {
            std::time_t startTime = std::time(nullptr);
            std::tm* startTm = std::localtime(&startTime);
            Ra::Core::StringUtils::stringPrintf(m_exportFolderName, "%4u%02u%02u-%02u%02u",
                                            1900 + startTm->tm_year,
                                            startTm->tm_mon+1,
                                            startTm->tm_mday,
                                            startTm->tm_hour,
                                            startTm->tm_min);

        }
        g_export_folder = m_exportFolderName;
        QDir().mkdir(m_exportFolderName.c_str());

        // Boilerplate print.
        LOG( logINFO ) << "*** Radium Engine Main App  ***";
        std::stringstream config;
#if defined (CORE_DEBUG)
        config << "Debug Build ";
#else
        config << "Release Build ";
#endif
#if defined (CORE_ENABLE_ASSERT)
        config<< "(with asserts) --";
#else
        config<<" --";
#endif

#if defined (ARCH_X86)
        config << " 32 bits x86";
#elif defined (ARCH_X64)
        config << " 64 bits x64";
#endif
        LOG( logINFO ) << config.str();

        config.str( std::string() );
        config << "Floating point format : ";
#if defined(CORE_USE_DOUBLE)
        config << "double precision";
#else
        config << "single precision" ;
#endif

        LOG( logINFO ) << config.str();

        config.str( std::string() );
        config<<"core build: "<<Version::compiler<<" - "<<Version::compileDate<<" "<<Version::compileTime;


        LOG( logINFO ) << config.str();

        std::stringstream cline;
        cline << "Command line arguments :";
        if (argc < 2)
        {
            cline << " (none)";
        }
        for (uint i = 1; i < argc; ++i)
        {
            cline << " " << argv[i];
        }

        LOG( logINFO ) << cline.str();

        LOG( logINFO ) << "Qt Version: " << qVersion();

        LOG( logINFO ) << "Running at "<<m_targetFPS<< "fps as target";

        // Create default format for Qt.
        QSurfaceFormat format;
        format.setVersion( 4, 4 );
        format.setProfile( QSurfaceFormat::CoreProfile );
        format.setDepthBufferSize( 24 );
        format.setStencilBufferSize( 8 );
        format.setSamples( 16 );
        format.setSwapBehavior( QSurfaceFormat::DoubleBuffer );
        format.setSwapInterval( 0 );
        QSurfaceFormat::setDefaultFormat( format );

        // Create engine
        m_engine.reset(Engine::RadiumEngine::createInstance());
        m_engine->initialize();
        addBasicShaders();

        // Create main window.
        m_mainWindow.reset( new Gui::MainWindow );
        m_mainWindow->show();

        m_viewer = m_mainWindow->getViewer();
        CORE_ASSERT( m_viewer != nullptr, "GUI was not initialized" );
        CORE_ASSERT( m_viewer->context()->isValid(), "OpenGL was not initialized" );

        // Allow all events to be processed (thus the viewer should have
        // initialized the OpenGL context..)
        processEvents();

        // Load plugins
        if ( !loadPlugins( pluginsPath, parser.values(pluginLoadOpt), parser.values(pluginIgnoreOpt) ) )
        {
            LOG( logERROR ) << "An error occurred while trying to load plugins.";
        }
        // Create task queue with N-1 threads (we keep one for rendering).
        uint numThreads =  std::thread::hardware_concurrency() - 1;
        if (m_maxThreads > 0 && m_maxThreads < numThreads)
        {
            numThreads = m_maxThreads;
        }
        m_taskQueue.reset( new Core::TaskQueue(numThreads) );

        // Create the instance of the keymapping manager (should it be done here ?)
        Gui::KeyMappingManager::createInstance();

        createConnections();

        setupScene();
        emit starting();

        // A file has been required, load it.
        const bool doLoadFile = parser.isSet(fileOpt);
        const bool doLoadCam = parser.isSet(camOpt);


        if (doLoadFile)
        {
            loadFile(parser.value(fileOpt), !doLoadCam);
        }

        if (doLoadCam)
        {
            m_mainWindow->loadCameraFromFile(parser.value(camOpt));
        }

        m_lastFrameStart = Core::Timer::Clock::now();
    }

    void BaseApplication::createConnections()
    {
        connect( m_mainWindow.get(), &Gui::MainWindow::closed , this, &BaseApplication::appNeedsToQuit );
    }

    void BaseApplication::setupScene()
    {

        return; // LALALA
        using namespace Engine::DrawPrimitives;

        auto grid = Primitive(Engine::SystemEntity::uiCmp(),
                              Grid( Core::Vector3::Zero(), Core::Vector3::UnitX(),
                                    Core::Vector3::UnitZ(), Core::Colors::Grey(0.6f) ));
        grid->setPickable( false );
        Engine::SystemEntity::uiCmp()->addRenderObject(grid);

        auto frame = Primitive(Engine::SystemEntity::uiCmp(), Frame(Ra::Core::Transform::Identity(), 0.05f));
        frame->setPickable( false );
        Engine::SystemEntity::uiCmp()->addRenderObject(frame);


        auto em =  Ra::Engine::RadiumEngine::getInstance()->getEntityManager();
        Ra::Engine::Entity* e = em->entityExists("Test") ?
            Ra::Engine::RadiumEngine::getInstance()->getEntityManager()->getEntity("Test"):
            Ra::Engine::RadiumEngine::getInstance()->getEntityManager()->createEntity("Test");

        for (auto& c: e->getComponents())
        {
            c->initialize();
        }
    }

    void BaseApplication::loadFile( QString path, bool fitCam )
    {
        path.replace("\\", "/");
        std::string pathStr = path.toLocal8Bit().data();
        LOG(logINFO) << "Loading file " << pathStr << "...";
        bool res = m_engine->loadFile( pathStr );
        CORE_UNUSED( res );
        m_viewer->handleFileLoading( pathStr );
        if (fitCam)
        {
            m_mainWindow->fitCamera();
        }
        QSettings settings;
        settings.setValue("files/load", path);
        emit loadComplete();
    }

    void BaseApplication::framesCountForStatsChanged( uint count )
    {
        m_frameCountBeforeUpdate = count;
    }

    void BaseApplication::addBasicShaders()
    {
        using namespace Ra::Engine;

        ShaderConfiguration bpConfig("BlinnPhong");
        bpConfig.addShader(ShaderType_VERTEX, "Shaders/BlinnPhong.vert.glsl");
        bpConfig.addShader(ShaderType_FRAGMENT, "Shaders/BlinnPhong.frag.glsl");
        ShaderConfigurationFactory::addConfiguration(bpConfig);

        ShaderConfiguration bpwConfig("BlinnPhong_wire");
        bpwConfig.addShader(ShaderType_VERTEX, "Shaders/BlinnPhong_wire.vert.glsl");
        bpwConfig.addShader(ShaderType_FRAGMENT, "Shaders/BlinnPhong_wire.frag.glsl");
        ShaderConfigurationFactory::addConfiguration(bpwConfig);

        ShaderConfiguration pConfig("Plain");
        pConfig.addShader(ShaderType_VERTEX, "Shaders/Plain.vert.glsl");
        pConfig.addShader(ShaderType_FRAGMENT, "Shaders/Plain.frag.glsl");
        ShaderConfigurationFactory::addConfiguration(pConfig);

        ShaderConfiguration lgConfig("LinesGeom");
        lgConfig.addShader(ShaderType_VERTEX, "Shaders/Lines.vert.glsl");
        lgConfig.addShader(ShaderType_FRAGMENT, "Shaders/Lines.frag.glsl");
        lgConfig.addShader(ShaderType_GEOMETRY, "Shaders/Lines.geom.glsl");
        ShaderConfigurationFactory::addConfiguration(lgConfig);

        ShaderConfiguration lConfig("Lines");
        lConfig.addShader(ShaderType_VERTEX, "Shaders/Lines.vert.glsl");
        lConfig.addShader(ShaderType_FRAGMENT, "Shaders/Lines.frag.glsl");
        ShaderConfigurationFactory::addConfiguration(lConfig);

        ShaderConfiguration gdConfig("GradientDisplay");
        lConfig.addShader(ShaderType_VERTEX, "Shaders/GradientDisplay.vert.glsl");
        lConfig.addShader(ShaderType_FRAGMENT, "Shaders/GradientDisplay.frag.glsl");
        ShaderConfigurationFactory::addConfiguration(gdConfig);
    }

    void BaseApplication::radiumFrame()
    {
        FrameTimerData timerData;
        timerData.frameStart = Core::Timer::Clock::now();

        // ----------
        // 0. Compute time since last frame.
        const Scalar dt = m_realFrameRate ?
                    Core::Timer::getIntervalSeconds( m_lastFrameStart, timerData.frameStart ) :
                    1.f / Scalar(m_targetFPS);
        m_lastFrameStart = timerData.frameStart;

        timerData.eventsStart = Core::Timer::Clock::now();
        processEvents();
        timerData.eventsEnd = Core::Timer::Clock::now();

        // ----------
        // 1. Gather user input and dispatch it.

        // Get picking results from last frame and forward it to the selection.
        m_viewer->processPicking();


        // ----------
        // 2. Kickoff rendering
        m_viewer->startRendering( dt );


        timerData.tasksStart = Core::Timer::Clock::now();

        // ----------
        // 3. Run the engine task queue.
        m_engine->getTasks( m_taskQueue.get(), dt );

        if (m_recordGraph) {m_taskQueue->printTaskGraph(std::cout);}

        // Run one frame of tasks
        m_taskQueue->startTasks();
        m_taskQueue->waitForTasks();
        timerData.taskData = m_taskQueue->getTimerData();
        m_taskQueue->flushTaskQueue();

        timerData.tasksEnd = Core::Timer::Clock::now();

        // ----------
        // 4. Wait until frame is fully rendered and display.
        m_viewer->waitForRendering();
        m_viewer->update();

        timerData.renderData = m_viewer->getRenderer()->getTimerData();

        // ----------
        // 5. Synchronize whatever needs synchronisation
        m_engine->endFrameSync();

        // ----------
        // 6. Frame end.
        timerData.frameEnd = Core::Timer::Clock::now();
        timerData.numFrame = m_frameCounter;

        if (m_recordTimings)
        {
            recordTimings( timerData );
        }
        m_timerData.push_back( timerData );

        if (m_recordFrames)
        {
            recordFrame();
        }

        if (m_recordMeshes)
        {
            recordMeshes();
        }

        ++m_frameCounter;

        if (m_numFrames > 0  &&  m_frameCounter > m_numFrames )
        {
            appNeedsToQuit();
        }

        if ( m_frameCounter % m_frameCountBeforeUpdate == 0 )
        {
            emit( updateFrameStats( m_timerData ) );
            m_timerData.clear();
        }

        m_mainWindow->onFrameComplete();
    }

    void BaseApplication::appNeedsToQuit()
    {
        LOG( logDEBUG ) << "About to quit.";
        m_isAboutToQuit = true;
    }

    void BaseApplication::setRealFrameRate(bool on)
    {
       m_realFrameRate = on;
    }
    void BaseApplication::setRecordFrames(bool on)
    {
        m_recordFrames = on;
    }
    void BaseApplication::setRecordMeshes(bool on)
    {
        m_recordMeshes = on;
    }

    void BaseApplication::recordTimings( const FrameTimerData& data )
    {
        std::string filename;
        Ra::Core::StringUtils::stringPrintf(filename, "%s/radiumtimings_%06u.txt", m_exportFolderName.c_str(), m_frameCounter);
        std::ofstream file(filename);
        data.print(file);
    }

    void BaseApplication::recordFrame()
    {
        std::string filename;
        Ra::Core::StringUtils::stringPrintf(filename, "%s/radiumframe_%06u.png", m_exportFolderName.c_str(), m_frameCounter);
        m_viewer->grabFrame(filename);
    }

    void BaseApplication::recordMeshes()
    {
       m_mainWindow->exportAllMeshes( m_exportFolderName );
    }

    BaseApplication::~BaseApplication()
    {
        LOG( logINFO ) << "About to quit... Cleaning RadiumEngine memory";
        emit stopping();
        m_mainWindow->cleanup();
        m_engine->cleanup();
        // This will remove the directory if empty.
        QDir().rmdir( m_exportFolderName.c_str());
    }

    bool BaseApplication::loadPlugins( const std::string& pluginsPath, const QStringList& loadList, const QStringList& ignoreList )
    {
        QDir pluginsDir( qApp->applicationDirPath() );
        LOG( logINFO )<<" *** Loading Plugins ***";
        bool result = pluginsDir.cd( pluginsPath.c_str() );

        if (!result)
        {
            LOG(logERROR) << "Cannot open specified plugins directory "<<pluginsPath;
            return false;
        }

        LOG( logDEBUG )<<"Plugin directory :"<<pluginsDir.absolutePath().toStdString();
        bool res = true;
        uint pluginCpt = 0;

        PluginContext context;
        context.m_engine = m_engine.get();
        context.m_selectionManager = m_mainWindow->getSelectionManager();
        context.m_pickingManager = m_viewer->getPickingManager();

        for (const auto& filename : pluginsDir.entryList(QDir::Files))
        {

            std::string ext = Core::StringUtils::getFileExt( filename.toStdString() );
#if defined( OS_WINDOWS )
            std::string sysDllExt = "dll";
#elif defined( OS_LINUX )
            std::string sysDllExt = "so";
#elif defined( OS_MACOS )
            std::string sysDllExt = "dylib";
#else
            static_assert( false, "System configuration not handled" );
#endif
            if ( ext == sysDllExt )
            {
                std::string basename = Core::StringUtils::getBaseName(filename.toStdString(),false);

                auto stringCmp = [basename](const QString& str) { return str.toStdString() == basename;};

                if (!loadList.empty() && std::find_if(loadList.begin(), loadList.end(),stringCmp ) == loadList.end() )
                {
                    LOG(logDEBUG)<<"Ignoring "<<filename.toStdString()<<" (not on load list)";
                    continue;
                }
                if ( std::find_if (ignoreList.begin(), ignoreList.end(), stringCmp) != ignoreList.end())
                {
                    LOG(logDEBUG)<<"Ignoring "<<filename.toStdString()<<" (on ignore list)";
                    continue;
                }

                QPluginLoader pluginLoader( pluginsDir.absoluteFilePath( filename ) );
                // Force symbol resolution at load time.
                pluginLoader.setLoadHints( QLibrary::ResolveAllSymbolsHint );

                LOG( logINFO ) << "Found plugin " << filename.toStdString();

                QObject* plugin = pluginLoader.instance();
                Plugins::RadiumPluginInterface* loadedPlugin;

                if ( plugin )
                {
                    loadedPlugin = qobject_cast<Plugins::RadiumPluginInterface*>( plugin );
                    if ( loadedPlugin )
                    {
                        ++pluginCpt;
                        loadedPlugin->registerPlugin( context );
                        m_mainWindow->updateUi( loadedPlugin );
                    }
                    else
                    {
                        LOG( logERROR ) << "Something went wrong while trying to cast plugin"
                                        << filename.toStdString();
                        res = false;
                    }
                }
                else
                {
                    LOG( logERROR ) << "Something went wrong while trying to load plugin "
                                    << filename.toStdString() << " : "
                                    << pluginLoader.errorString().toStdString();
                    res = false;
                }
            }
        }

        if (pluginCpt == 0)
        {
            LOG(logINFO) << "No plugin found or loaded.";
        }
        else
        {
            LOG(logINFO) << "Loaded " << pluginCpt << " plugins.";
        }

        return res;
    }

    void BaseApplication::setRecordTimings(bool on)
    {
        m_recordTimings = on;
    }

    void BaseApplication::setRecordGraph(bool on)
    {
        m_recordGraph = on;
    }
}
