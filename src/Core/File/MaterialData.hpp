#ifndef RADIUMENGINE_MATERIALDATA_HPP
#define RADIUMENGINE_MATERIALDATA_HPP
#include <string>
#include <vector>

#include <Core/Containers/VectorArray.hpp>
#include <Core/Math/LinearAlgebra.hpp>
#include <Core/RaCore.hpp>

#include <Core/File/AssetData.hpp>

namespace Ra {
namespace Asset {

/** @brief represent material data loaded by a file loader.
 * Material data must be identified by a unique name.
 * Radium Engine reserves the following names
 *  "AbstractMaterial"  --> unknown material, might serve for error management
 *  "BlinnPhong"        --> standar Blinn-Phong Material
 *
 *  When extending the material system with a loader plugin (or something similar) :
 *      Define your own "Asset" class derived from Ra::Asset::MaterialData with the "type" that
 * identifies it uniquely and corresponding to the external format of your material (in the file you
 * load). Define your own "Engine" class, derived from Ra::Engine::Material.
 *
 *      Make your plugin register the converter function from "Asset" to "Engine" for this material.
 *          This function may be everithing that is of the same type than
 * std::function<RadiumMaterialPtr(AssetMaterialPtr)> Make your plugin register the default
 * technique builder for this material. This will require to write some GLSL shaders and make them
 * accessible from the application search directory. See example in RadiumEngine.cpp
 * (RadiumEngine::initialize()) that defins default BlinnPhong Material.
 *
 *      Make your loader instantiate the right "Asset" MaterialData class while loading material
 * data from a file.
 *
 *      The active system (FancyMesh is the Radium Default), will then automatically use your new
 * material and technique so that the rendering will be fine. When writing your own system, see
 * FancyMesh implementations as an example.
 *
 */
class RA_CORE_API MaterialData : public AssetData {
  public:
    MaterialData( const std::string& name = "", const std::string& type = "AbstractMaterial" );

    virtual ~MaterialData();

    /// Return the name of the Material.
    inline void setName( const std::string& name );

    /// Return the type of the Material.
    inline std::string getType() const;

    /// Set the type of the Material.
    inline void setType( const std::string& type );

    /// Pring stat info to the Debug output.
    virtual void displayInfo() const;

  private:
    /// The type of the Material.
    std::string m_type;
};

} // namespace Asset
} // namespace Ra

#include <Core/File/MaterialData.inl>

#endif // RADIUMENGINE_MATERIALDATA_HPP
